/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for ardupilot version of Gazebo
*/

#include "SIM_Gazebo.h"

#include <stdio.h>
#include <algorithm>
#include <sys/time.h>


#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

Gazebo::Gazebo(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    last_timestamp(0)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // Gazebo keeps sending us packets. Not strictly necessary but
    // useful for debugging
    fprintf(stdout, "Starting SITL Gazebo\n");

}

/*
  decode and send servos
*/
void Gazebo::send_servos(const struct sitl_input &input)
{
    servo_packet pkt;
    // should rename servo_command
    // 16 because struct sitl_input.servos is 16 large in SIM_Aircraft.h
    for (unsigned i = 0; i < 16; ++i)
    {
      pkt.motor_speed[i] = (input.servos[i]-1000) / 1000.0f;
    }
    socket_out.send(&pkt, sizeof(pkt));
}

double getsecondstoday()
{
  struct timeval time;
  gettimeofday(&time, NULL); // Start Time
  return (time.tv_sec) + (time.tv_usec / 1000000.0);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void Gazebo::recv_fdm(const struct sitl_input &input)
{
    unsigned int iterations = 1;
    fdm_packet pkt;
    this->cycle ++;
    if (this->cycle >= iterations)
    {
      this->cycle = 0;
      
      
      double total_wait = 0;
      double total_acting = 0;
      double total_external_acting = 0;
      double max_wait = 0;
      double max_acting = 0;
      double max_external_acting = 0;
      for (unsigned int i = 0; i < iterations; i++)
      {
        total_acting += acting_time[i];
        total_wait += waiting_time[i];
        total_external_acting += external_acting_time[i];
        max_acting = std::max(max_acting, acting_time[i]);
        max_wait = std::max(max_wait, waiting_time[i]);
        max_external_acting = std::max(max_external_acting, external_acting_time[i]);
      }
      
      double average_acting = total_acting / (float)iterations;
      double average_wait = total_wait / (float)iterations;
      double average_external_acting = total_external_acting / (float)iterations;
      double now = getsecondstoday();
      double freq = (float) iterations / (now - cycle_time);
      cycle_time = now;
      double fraction = average_external_acting / (average_acting + average_external_acting + average_wait);
      double max_fraction = max_external_acting / (max_acting + max_external_acting + max_wait);
      double external_total_fraction = total_external_acting / (total_acting + total_external_acting + total_wait);
      fprintf(stdout, "<%d> Samples >> ArdupilotFreq: (%f) Average[max] wait %f[%f], acting: %f[%f], external acting: %f[%f] External blocking: %f %%, Max External blocking: %f %% Sum External_fraction %f %%\n", iterations, freq, average_wait, max_wait, average_acting, max_acting, average_external_acting, max_external_acting, fraction * 100, max_fraction * 100, external_total_fraction * 100);
    }
    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    double now = getsecondstoday();
    this->external_acting_time[this->cycle] = ( now - last_internal_action );
    last_send = now;
    while (socket_in.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        send_servos(input);
    }
    now = getsecondstoday();
    this->waiting_time[this->cycle] = ( now - last_send );
    last_receive = now;


    // get imu stuff
    accel_body = Vector3f(pkt.imu_linear_acceleration_xyz[0],
                          pkt.imu_linear_acceleration_xyz[1],
                          pkt.imu_linear_acceleration_xyz[2]);

    gyro = Vector3f(pkt.imu_angular_velocity_rpy[0],
                    pkt.imu_angular_velocity_rpy[1],
                    pkt.imu_angular_velocity_rpy[2]);

    // compute dcm from imu orientation
    Quaternion quat(pkt.imu_orientation_quat[0],
                    pkt.imu_orientation_quat[1],
                    pkt.imu_orientation_quat[2],
                    pkt.imu_orientation_quat[3]);
    quat.rotation_matrix(dcm);

    double speedN =  pkt.velocity_xyz[0];
    double speedE =  pkt.velocity_xyz[1];
    double speedD =  pkt.velocity_xyz[2];
    velocity_ef = Vector3f(speedN, speedE, speedD);

    // Approximate airspeed as gps velocity
    airspeed = sqrt(speedN * speedN + speedE * speedE + speedD * speedD);
    airspeed_pitot = airspeed;

    position = Vector3f(pkt.position_xyz[0],
                        pkt.position_xyz[1],
                        pkt.position_xyz[2]);


    // auto-adjust to simulation frame rate
    double deltat = pkt.timestamp - last_timestamp;
    time_now_us += deltat * 1.0e6;

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(1.0/deltat);
    }
    last_timestamp = pkt.timestamp;

    //Approimation
    double meters_per_degree = 111319.9;

    // Set the location too
    location.lat = home.lat + degrees(pkt.position_xyz[0] / meters_per_degree ) * 1.0e7;
    location.lng = home.lng + degrees(pkt.position_xyz[1] / meters_per_degree * cos(location.lat) * 1.0e7);
    location.alt = home.alt + pkt.position_xyz[0] * 100;
    // printf("location: %d %d %d\n", location.lat, location.lng, location.alt);

    // update magnetic field
    // Implicitly uses the location
    update_mag_field_bf();

    //Spoof battery voltage and current
    battery_voltage = 12.4;
    battery_current = 2.5;

    /* copied below from iris_ros.py */
    /*
    bearing = to_degrees(atan2(position.y, position.x));
    distance = math.sqrt(self.position.x**2 + self.position.y**2)
    (self.latitude, self.longitude) = util.gps_newpos(
      self.home_latitude, self.home_longitude, bearing, distance)
    self.altitude  = self.home_altitude - self.position.z
    velocity_body = self.dcm.transposed() * self.velocity
    self.accelerometer = self.accel_body.copy()
    */
    now = getsecondstoday();
    this->acting_time[this->cycle] = ( now - last_receive );
    last_internal_action = now;
}

/*
  update the Gazebo simulation by one time step
 */
void Gazebo::update(const struct sitl_input &input)
{
    send_servos(input);
    recv_fdm(input);
    update_position();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
