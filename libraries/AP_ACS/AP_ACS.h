#ifndef AP_ACS_H__
#define AP_ACS_H__
/* Naval Postgraduate School Aerial Combat Swarms module */

#include <AP_Param.h>
#include <GCS_MAVLink.h>
#include <AP_TECS.h>
#include <AP_AHRS_NavEKF.h>

class AP_ACS {
public:
    //TODO: modes should be in their own library so they aren't duplicated here.
    //I wonder if AP_Mission was supposed to do that.  It may need some help.
    typedef enum ACS_FlightMode {
        ACS_NONE          = -1,
        ACS_MANUAL        = 0,
        ACS_CIRCLE        = 1,
        ACS_STABILIZE     = 2,
        ACS_TRAINING      = 3,
        ACS_ACRO          = 4,
        ACS_FLY_BY_WIRE_A = 5,
        ACS_FLY_BY_WIRE_B = 6,
        ACS_CRUISE        = 7,
        ACS_AUTO          = 10,
        ACS_RTL           = 11,
        ACS_LOITER        = 12,
        ACS_GUIDED        = 15,
        ACS_INITIALISING  = 16
    } ACS_FlightMode;

    typedef enum FailsafeState {
        GPS_LONG_FS = 0,
        GPS_SHORT_FS,
        GPS_RECOVERING_FS,
        SEC_CONTROL_FS,
        BATTERY_CURR_FS,
        BATTERY_VOLT_FS,
        GEOFENCE_FS,
        GCS_FS,
        THROTTLE_FS,
        NO_FS,
        NO_COMPANION_COMPUTER_FS
    } FailsafeState;

    // for holding parameters
	static const struct AP_Param::GroupInfo var_info[];

    AP_ACS();

    //returns true if the hearbeat was from a companion computer
    bool handle_heartbeat(mavlink_message_t* msg);

    // essentially returns a bool: if not 0, then DO kill throttle
    AP_Int8 get_kill_throttle();

    // essentially setting a bool: if not 0, then DO kill throttle
    void set_kill_throttle(AP_Int8 kt);

    FailsafeState get_current_fs_state() { return _current_fs_state; }

    //used when recovering from a GPS failsafe state to get back to 
    //the mode we were in before the failsafe triggered
    ACS_FlightMode get_previous_mode() { return _previous_mode; }

    //returns true if everything OK.
    //false if RTL should happen
    bool check(ACS_FlightMode mode, AP_SpdHgtControl::FlightStage flight_stage,
           uint32_t last_heartbeat_ms, uint32_t last_gps_fix_ms);

#if AP_AHRS_NAVEKF_AVAILABLE
    void send_position_attitude_to_payload(AP_AHRS_NavEKF &ahrs,
            mavlink_channel_t chan);
#endif //AP_AHRS_NAVEKF_AVAILABLE

protected:
    //params
    AP_Int8           _watch_heartbeat;
    AP_Int8             _kill_throttle;

    uint32_t _last_computer_heartbeat_ms;

     
    FailsafeState       _current_fs_state;
    
    ACS_FlightMode      _previous_mode;
};

#endif // AP_ACS_H__
