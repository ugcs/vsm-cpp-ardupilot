// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file ardupilot_vehicle.h
 */
#ifndef _ARDUPILOT_VEHICLE_H_
#define _ARDUPILOT_VEHICLE_H_

#include <mavlink_vehicle.h>
#include <adsb_aircraft.h>

#define ARDUPILOT_VERSION(maj, min, patch) ((maj << 24) + (min << 16) + (patch << 8))
#define DISARM_MAGIC_VALUE 21196.0f

/** Vehicle supporting Ardupilot specific flavor of Mavlink. */
class Ardupilot_vehicle : public Mavlink_vehicle {
    DEFINE_COMMON_CLASS(Ardupilot_vehicle, Mavlink_vehicle)

public:
    template<typename... Args>
    Ardupilot_vehicle(
        ugcs::vsm::Mavlink_demuxer::System_id system_id,
        ugcs::vsm::Mavlink_demuxer::Component_id component_id,
        ugcs::vsm::mavlink::MAV_TYPE type,
        ugcs::vsm::Mavlink_stream::Ptr stream,
        ugcs::vsm::Optional<std::string> mission_dump_path,
        const std::string& serial,
        const std::string& model,
        Args &&... args) :
            Mavlink_vehicle(
                system_id,
                component_id,
                Vendor::ARDUPILOT,
                type,
                stream,
                mission_dump_path,
                serial,
                model,
                std::forward<Args>(args)...),
            vehicle_command(*this),
            task_upload(*this)
    {
        Set_autopilot_type("ardupilot");
        switch (Get_vehicle_type()) {
        case ugcs::vsm::proto::VEHICLE_TYPE_FIXED_WING:
            Set_model_name("ArduPlane");
            break;
        case ugcs::vsm::proto::VEHICLE_TYPE_HELICOPTER:
        case ugcs::vsm::proto::VEHICLE_TYPE_MULTICOPTER:
            Set_model_name("ArduCopter");
            break;
        case ugcs::vsm::proto::VEHICLE_TYPE_GROUND:
            Set_model_name("ArduRover");
            break;
        case ugcs::vsm::proto::VEHICLE_TYPE_VTOL:
            Set_model_name("ArduVTOL");
            break;
        }

        /* Consider this as uptime start. */
        recent_connect = std::chrono::steady_clock::now();

        use_mavlink_2 = false;  // Default to mavlink 1 for ardupilot.
    }

    // Constructor for command processor.
    Ardupilot_vehicle(ugcs::vsm::proto::Vehicle_type type);

    virtual void
    On_enable() override;

    virtual void
    On_disable() override;

    virtual void
    Handle_ucs_command(ugcs::vsm::Ucs_request::Ptr ucs_request) override;

    /** Ardupilot specific activity. */
    class Ardupilot_activity : public Activity {
    public:
        /** Constructor based on Ardupilot vehicle class. */
        Ardupilot_activity(Ardupilot_vehicle& ardu_vehicle) :
            Activity(ardu_vehicle),
            ardu_vehicle(ardu_vehicle) {}

        /** Managed Ardupilot vehicle. */
        Ardupilot_vehicle& ardu_vehicle;
    };

    /** Data related to specific payload subsystem */
    struct PayloadSubsystem {
        PayloadSubsystem(uint8_t payloadId) : payloadId(payloadId) {}

        uint8_t payloadId;
        ugcs::vsm::Subsystem::Ptr subsystem;

        ugcs::vsm::Property::Ptr t_payload_status_b64;
        ugcs::vsm::Property::Ptr t_payload_data_b64;
        ugcs::vsm::Property::Ptr t_payload_value_b64;

        ugcs::vsm::Vsm_command::Ptr c_payload_command;
    };

    /** Data related to vehicle command processing. */
    class Vehicle_command_act : public Ardupilot_activity {
    public:
        using Ardupilot_activity::Ardupilot_activity;

        /** Try execute command a vehicle. */
        bool
        Try();

        /** Try execute a verifying command for polyfence */
        bool
        Try_verify_polyfence();

        /** Command ack received. */
        void
        On_command_ack(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::COMMAND_ACK>::Ptr);

        void
        On_mission_ack(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_ACK>::Ptr);

        void
        On_param_value(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr);

        void
        On_point_value(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::apm::MESSAGE_ID::FENCE_POINT,
            ugcs::vsm::mavlink::apm::Extension>::Ptr);

        void
        On_mission_current(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_CURRENT>::Ptr);

        void
        On_param_str_value(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::sph::MESSAGE_ID::PARAM_STR_VALUE,
            ugcs::vsm::mavlink::sph::Extension>::Ptr);

        void
        Send_next_command();

        /** Status text recieved. */
        void
        On_status_text(
                ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::STATUSTEXT>::Ptr);

        /** Enable class and start command execution. */
        void
        Enable();

        /** Disable the activity. */
        virtual void
        On_disable() override;

        /** Schedule timer for retry operation. */
        void
        Schedule_timer();

        /** Schedule timer for verifying operation. */
        void
        Schedule_verify_timer();

        /** Register status text handler. */
        void
        Register_status_text();

        /** Unregister status text handler. */
        void
        Unregister_status_text();

        /** Get the value of custom mode corresponding to reasonable manual mode
         * of the current vehicle type. */
        uint32_t
        Get_custom_manual_mode();

        /** Get the value of custom mode corresponding to guided mode
         * of the current vehicle type. */
        uint32_t
        Get_custom_guided_mode();

        /** Mavlink messages to be sent to execute current command. */
        std::list<ugcs::vsm::mavlink::Payload_base::Ptr> cmd_messages;

        /** Remaining attempts towards vehicle. */
        size_t remaining_attempts = 0;

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;

        /** Current timeout to use when scheduling timer. */
        std::chrono::milliseconds current_timeout;

        /** check if point is inside polygon.
         *  Algorithm was taken from ardupilot cause we have to check if the returning point
         *  for geofence is outside polygon exactly as ardupilot does.
         *  If returning point is outside polygon fence then fence will not work and there is
         *  no way to know about it from ardupilot
         */
        bool
        Is_Outside_Polygon(double check_point_latitude, double check_point_longitude,
                           ugcs::vsm::proto::List_value lats, ugcs::vsm::proto::List_value lngs);

        float command_count = 0; // for progress reporting

    private:
        void
        Process_guided();

        void
        Process_joystick();

        void
        Process_auto();

        void
        Process_manual();

        void
        Process_arm();

        void
        Process_disarm();

        void
        Process_pause(const ugcs::vsm::Property_list& params);

        void
        Process_resume();

        // throws if a parameter is missing.
        void
        Process_waypoint(const ugcs::vsm::Property_list& params);

        // throws if a parameter is missing.
        void
        Process_direct_vehicle_control(const ugcs::vsm::Property_list& params);

        void
        Process_land();

        void
        Process_rth();

        void
        Process_relative_heading(const ugcs::vsm::Property_list& params);

        void
        Process_heading(const ugcs::vsm::Property_list& params);

        void
        Process_takeoff(const ugcs::vsm::Property_list& params);

        void
        Process_reboot();

        void
        Process_mission_clear();

        void
        Process_calibration();

        void
        Process_set_fence(const ugcs::vsm::Property_list& params);

        void
        Process_repeat_servo(const ugcs::vsm::Property_list& params);

        void
        Process_set_servo(const ugcs::vsm::Property_list& params);

        void
        Process_adsb_set_mode(const ugcs::vsm::Property_list& params);

        void
        Process_adsb_set_ident();

        void
        Process_adsb_set_parameter(const ugcs::vsm::Property_list& params);

        // throws if a parameter is invalid (not float).
        void
        Process_write_parameter(const ugcs::vsm::Property_list& params);

        // throws if a parameter is invalid (not float).
        void
        Process_set_poi(const ugcs::vsm::Property_list& params);

        void
        Process_emergency_land();

        // Workaround for set_heading not working if not preceded by WP.
        void
        Generate_wp_from_current_position();

        void
        Process_camera_trigger();

        // Stop camera trigger by time/distance if active.
        void
        Stop_camera_series();

        void
        Process_direct_mount_control(const ugcs::vsm::Property_list& params);

    } vehicle_command;


    /** Data related to task upload processing. */
    class Task_upload: public Ardupilot_activity {
    public:
        Task_upload(Ardupilot_vehicle& vehicle):
            Ardupilot_activity(vehicle),
            task_attributes(vehicle.real_system_id, vehicle.real_component_id)
        {}

        /** Add mission item to prepared actions. Common mission item
         * initialization are made, like sequence number generation.
         */
        void
        Add_mission_item(ugcs::vsm::mavlink::Pld_mission_item_int::Ptr);

        //@{
        /** Prepare methods for different types of actions. These methods
         * create an item in the prepared actions list.
         * @return Created mission item. */

        void
        Prepare_move(const ugcs::vsm::Property_list&, bool is_last = false);

        void
        Prepare_wait(const ugcs::vsm::Property_list&);

        void
        Prepare_takeoff_mission(const ugcs::vsm::Property_list& params);

        void
        Prepare_landing(const ugcs::vsm::Property_list& params);

        void
        Prepare_change_speed(const ugcs::vsm::Property_list&);

        void
        Prepare_set_home(const ugcs::vsm::Property_list& params);

        void
        Prepare_poi(const ugcs::vsm::Property_list& params);

        void
        Prepare_heading(const ugcs::vsm::Property_list& params);

        void
        Prepare_panorama(const ugcs::vsm::Property_list& params);

        void
        Prepare_camera_series_by_distance(const ugcs::vsm::Property_list& params);

        void
        Prepare_camera_series_by_time(const ugcs::vsm::Property_list& params);

        void
        Prepare_camera_trigger(const ugcs::vsm::Property_list& params);

        void
        Prepare_vtol_transition(bool vtol);

        void
        Prepare_wait_until(const ugcs::vsm::Property_list& params);

        void
        Prepare_set_servo(const ugcs::vsm::Property_list& params);

        void
        Prepare_repeat_servo(const ugcs::vsm::Property_list& params);

        void
        Prepare_payload_control(const ugcs::vsm::Property_list& params);

        //@}

        /** Build waypoint mission item based on move action. */
        ugcs::vsm::mavlink::Pld_mission_item_int::Ptr
        Build_wp_mission_item(const ugcs::vsm::Property_list& params);

        /** Build ROI mission item based on given coordinates */
        ugcs::vsm::mavlink::Pld_mission_item_int::Ptr
        Build_roi_mission_item(const ugcs::vsm::Geodetic_tuple& coords);

        /** Build Heading mission item */
        ugcs::vsm::mavlink::Pld_mission_item_int::Ptr
        Build_heading_mission_item(
                float heading,
                float speed = 0.0,
                bool absolute_angle = true,
                bool clockwise = true);

        /** Add proper frame and Z-coordinate to WP
         * based on follow_terrain parameter in Property_list
         */
        void
        Handle_terrain_following(ugcs::vsm::mavlink::Pld_mission_item_int& mi,
                                 const ugcs::vsm::Property_list& params);

        void
        Stop_camera_series();

        void
        Add_camera_trigger_item();

        /** Previous activity is completed, enable class and start task upload. */
        void
        Enable(bool generate_file);

        /** Disable this class and cancel any existing request. */
        virtual void
        On_disable() override;

        /** Prepare task attributes depending on the vehicle type. */
        void
        Prepare_task_attributes(const ugcs::vsm::Property_list& params);

        // Validates the mission
        // Throws Invalid_param_exception
        void
        Preprocess_mission(const ugcs::vsm::proto::Device_command& vsm_cmd);

        /** Prepare the task for uploading to the vehicle. */
        void
        Prepare_mission(const ugcs::vsm::proto::Device_command& vsm_cmd);

        void
        Prepare_set_parameters(const ugcs::vsm::Property_list& params);

        /** Task attributes upload handler. */
        void
        Task_atributes_uploaded(bool success, std::string);

        /** Task attributes upload handler. */
        void
        Upload_parameters(bool success, std::string);

        /** Task attributes upload handler. */
        void
        Task_commands_sent(bool success, std::string = "");

        /** Mission uploaded. */
        void
        Mission_uploaded(bool success, std::string);

        /** Mission downloaded. Calculate the mission_id. */
        void
        Mission_downloaded(bool success, std::string);

        /**
         * Fill coordinates into Mavlink message based on ugcs::vsm::Geodetic_tuple and
         * some other common mission item data structures.
         * @param msg Mavlink message.
         * @param tuple Geodetic tuple.
         * @param heading Vehicle heading.
         */
        void
        Fill_mavlink_mission_item_coords(ugcs::vsm::mavlink::Pld_mission_item_int& msg,
                const ugcs::vsm::Geodetic_tuple& tuple, double heading);

        /**
         * Fill Mavlink mission item common parameters.
         * @param msg Mavlink message.
         */
        void
        Fill_mavlink_mission_item_common(ugcs::vsm::mavlink::Pld_mission_item_int& msg);

        ugcs::vsm::mavlink::Pld_mission_item_int::Ptr
        Create_mission_item();

        ugcs::vsm::mavlink::Pld_mission_item_int::Ptr
        Create_mission_item_with_coords(const ugcs::vsm::Property_list& params);

        ugcs::vsm::mavlink::Pld_command_long::Ptr
        Create_command_long_with_coords(const ugcs::vsm::Property_list& params);

        ugcs::vsm::Geodetic_tuple
        Create_geodetic_tuple(const ugcs::vsm::Property_list& params);

        // Called on each mission item during upload
        void
        On_upload_progress(int seqnum);

        // Called on each mission item during download
        void
        On_download_progress(int seqnum);

        /** Prepared Mavlink actions to be uploaded to the vehicle and built based
         * on the actions from the original request. Original actions could be
         * extended/removed/updated to meet the Mavlink mission protocol
         * requirements. Example is adding of magical "dummy waypoints" and
         * special processing of waypoint zero.
         */
        ugcs::vsm::mavlink::Payload_list prepared_actions;

        /** Task attributes to be written to the vehicle. */
        Write_parameters::List task_attributes;

        /** Previous move action, if any. */
        ugcs::vsm::Optional<ugcs::vsm::Property_list> last_move_params;

        /** Active POI from mission. */
        ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple> current_mission_poi;

        /** Current heading from mission. */
        ugcs::vsm::Optional<float> current_mission_heading;

        /** Does current WP have POI action defined in mission*/
        bool first_mission_poi_set = false;

        /** Mission POI action must be added as it was cancelled by previous actions.*/
        bool restart_mission_poi = false;

        float current_heading;

        // current speed in mission.
        float current_speed = -1;

        // used only when autoheading is on.
        float heading_to_this_wp = 0.0;

        /** CAMERA_SERIES_BY_DISTANCE was activated. */
        bool camera_series_by_dist_active = false,
        /** CAMERA_SERIES_BY_DISTANCE was activated in current waypoint. */
             camera_series_by_dist_active_in_wp = false,
        /** CAMERA_SERIES_BY_TIME was activated. */
             camera_series_by_time_active = false,
        /** CAMERA_SERIES_BY_DISTANCE was activated in current waypoint. */
             camera_series_by_time_active_in_wp = false;

        float altitude_origin = 0;
        float home_altitude = 0;
        bool use_crlf_in_native_route = false;

        int current_route_command_index = -1;  // for command map
        float command_count = 0; // for progress reporting
        float item_count = 0; // for progress reporting
        float param_count = 0; // for progress reporting
        float total_count = 0; // for progress reporting
        // home location
        ugcs::vsm::Property_list hl_params;

        // Index of last move command in mission. Used to force straight turn-type for it.
        int last_move_idx = 0;
    } task_upload;


    void
    On_adsb_state(
        ugcs::vsm::mavlink::Message<
            ugcs::vsm::mavlink::sph::SPH_ADSB_TRANSPONDER_STATE,
            ugcs::vsm::mavlink::sph::Extension>::Ptr message);

    void
    On_autopilot_version(ugcs::vsm::mavlink::Pld_autopilot_version ver);

    void
    On_parameter(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr);

    void
    On_rangefinder(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::apm::MESSAGE_ID::RANGEFINDER,
        ugcs::vsm::mavlink::apm::Extension>::Ptr);

    void
    On_rpm(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::apm::MESSAGE_ID::RPM,
            ugcs::vsm::mavlink::apm::Extension>::Ptr);

    void
    On_battery2(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::apm::MESSAGE_ID::BATTERY2,
            ugcs::vsm::mavlink::apm::Extension>::Ptr);

    void
    On_power_status(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::POWER_STATUS>::Ptr);

    void
    On_efi_status(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::EFI_STATUS>::Ptr);

    void
    On_adsb_vehicle(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::ADSB_VEHICLE>::Ptr);

    void
    On_string_parameter(
        ugcs::vsm::mavlink::Message<
            ugcs::vsm::mavlink::sph::MESSAGE_ID::PARAM_STR_VALUE,
            ugcs::vsm::mavlink::sph::Extension>::Ptr message);

    void
    On_mission_item(ugcs::vsm::mavlink::Pld_mission_item_int);

    void
    On_mission_request(int seq);

    void
    Report_home_location(double lat, double lon, double alt);

    void
    On_home_position(
        ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HOME_POSITION>::Ptr m);


    void
    On_mission_downloaded(bool, const std::string);

    bool
    On_home_location_timer();

    bool
    On_adsb_vehicles_timer();

    void
    On_gimbal_report(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::apm::MESSAGE_ID::GIMBAL_REPORT,
                    ugcs::vsm::mavlink::apm::Extension>::Ptr);

    void
    On_mount_status(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::apm::MESSAGE_ID::MOUNT_STATUS,
                    ugcs::vsm::mavlink::apm::Extension>::Ptr);

protected:
    /** Load parameters from configuration. */
    void
    Configure_common();

private:
    /** Flight modes of the Ardupilot Copter mode. Copied from ArduCopter/defines.h*/
    enum class Copter_flight_mode {
        STABILIZE =     0,  // manual airframe angle with manual throttle
        ACRO =          1,  // manual body-frame angular rate with manual throttle
        ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate cmds
        LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        RTL =           6,  // automatic return to launching point
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        LAND =          9,  // automatic landing with horizontal position control
        OF_LOITER =    10,  // Hold a single location using optical flow sensor.
        DRIFT =        11,  // semi-automous position, yaw and throttle control
        SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        FLIP =         14,  // automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
    };

    // @{
    /** Flight modes of the Ardupilot Plane mode. Copied form ArduPlane/defines.h */
    enum class Plane_flight_mode {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21
    };
    // @}

    // @{
    /** Flight modes of the Ardupilot Rover mode. Copied form APMrover2/defines.h */
    enum class Rover_flight_mode {
        MANUAL       = 0,
        ACRO         = 1,
        STEERING     = 3,
        HOLD         = 4,
        AUTO         = 10,
        RTL          = 11,
        SMART_RTL    = 12,
        GUIDED       = 15,
        INITIALISING = 16
    };
    // @}

    /** Process heartbeat message by setting system status according to it. */
    virtual void
    Process_heartbeat(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr) override;

    virtual void
    Initialize_telemetry() override;

    /** Map custom flight mode from the heartbeat to our flight mode. */
    void
    Update_flight_mode(int);

    const char*
    Get_native_flight_mode_name(int);

    void
    Update_control_mode(int);

    /** Updates current capabilities based on vehicle type. */
    void
    Update_capabilities();

    /** Updates current capability states based on vehicle type. */
    void
    Update_capability_states();

    /** Load parameters from configuration for real vehicle */
    void
    Configure_real_vehicle();

    void
    Get_home_location();

    bool
    Is_home_position_valid();

    void
    Start_rc_override();

    void
    Send_rc_override();

    bool
    Send_rc_override_timer();

    bool
    Is_rc_override_active();

    void
    Stop_rc_override();

    void
    Set_rc_override(int p, int r, int t, int y);

    void
    On_version_processed(bool success, std::string);

    // True if detected ardupilot version is less than given number.
    bool
    Is_version_less_than(uint32_t maj, uint32_t min, uint32_t patch) {
        return (ardupilot_version < (maj << 24) + (min << 16) + (patch << 8));
    }

    /**
     * Minimal waypoint acceptance radius to use.
     */
    constexpr static double ACCEPTANCE_RADIUS_MIN = 1;

    /** Recent connect time of the vehicle. */
    std::chrono::steady_clock::time_point recent_connect;

    /** Index of servo to use for camera trigger. */
    int camera_servo_idx = -1;  // Not configured by default.
    /** PWM value to set for camera trigger. */
    int camera_servo_pwm = -1;
    /** Time to hold camera servo at the specified PWM when triggering. */
    float camera_servo_time = -1;

    /** Time to wait after camera trigger to avoid bluring photos. Camera shot is not instant, so copter may start turning while diaphragm is open*/
    float panorama_post_trigger_delay = -1;

    /** Constrains taken from autopilot parameters */
    struct Mount_Constraints {
        /** MNT_ANGMAX_PAN */
        float mount_max_pan = 18000;
        /** MNT_ANGMAX_ROL */
        float mount_min_pan = -18000;
        /** MNT_ANGMAX_TIL */
        float mount_max_tilt = 18000;
        /** MNT_ANGMIN_PAN */
        float mount_min_tilt = -18000;
        /** MNT_ANGMAX_ROL */
        float mount_max_roll = 18000;
        /** MNT_ANGMIN_ROL */
        float mount_min_roll = -18000;
    } mount_constraints;

    /** check for correct mount configuration, if ok, then control from ugcs will work */
    bool is_mount_control_enabled = false;

    /** By default joystick mode is disabled for planes.
     * Turn on via an entry in conf file. */
    bool enable_joystick_control_for_fixed_wing = false;

    /** if true, then enable GND_ALT_OFFSET calibration during mission upload
     *  if false, set GND_ALT_OFFSET to zero */
    bool set_ground_alt_offset = true;

    /** by default autoheading is turned on */
    bool autoheading = true;

    /** If vehicle does not support ROI for multiple WPts then VSM must
     * generate POI commands for each WP until POI(none) received.
     * Leave it true for now until VSM is able to detect Ardupilot FW version.
     * Pre 3.2 needs POI for each WP
     * 3.2+ will keep pointing to current poi POI until POI(0,0,0) received.*/
    bool auto_generate_mission_poi = true;

    int current_native_flight_mode = -1;

    /** Ardupilot version 3.3.1+ does not have FS_GPS_ENABLE parameter
     * It uses FS_EKF_ACTION for that instead.
     */
    bool use_ekf_action_as_gps_failsafe = false;

    /** Ardupilot version 3.6+ renamed FS_BATT_ to BATT_FS_
     */
    bool use_batt_fs_as_batt_failsafe = false;

    /** Ardupilot version 3.6+ allows to issue DO_SET_CAM_TRIGG_DIST in MAV_CMD_LONG
     * Prior to that the only way to terminate trigger by distance is to reach WP with
     * DO_SET_CAM_TRIGG_DIST(0)
     */
    bool do_set_cam_trig_dist_as_command = false;

    /** Ardupilot version 3.3.1+ requires Home position
     * to be sent as MAV_CMD instead of mission item. */
    bool send_home_position_as_mav_cmd = false;

    /** Ardupilot version 3.4+ allows to adjust raw altitude with GND_ALT_OFFSET
     * parameter. Previous versions either don't have this parameter or have a bug
     */
    bool gnd_alt_offset_allow = false;

    /** Poll for home location until it is nonzero. */
    ugcs::vsm::Timer_processor::Timer::Ptr home_location_timer;

    ugcs::vsm::Geodetic_tuple home_location {0, 0, 0};

    /** Joystick mode support */

    // Timer instance for sending rc_override messages.
    ugcs::vsm::Timer_processor::Timer::Ptr rc_override_timer = nullptr;

    // RC Override message which holds the latest joystick values.
    // Existence of this messages means that vehicle is in joystick mode.
    ugcs::vsm::mavlink::Pld_rc_channels_override::Ptr rc_override = nullptr;

    // Last time we sent the rc_overrride message. Used to limit rate at which
    // the joystick commands are sent to vehicle.
    std::chrono::time_point<std::chrono::steady_clock> rc_override_last_sent;

    // Last time we received joystick message from ucs. Used to fail over to
    // manual mode automatically if no messages received for RC_OVERRIDE_TIMEOUT.
    std::chrono::time_point<std::chrono::steady_clock> direct_vehicle_control_last_received;

    // Timeout when to revert back to manual mode if no joystick messages
    // received from ucs.
    constexpr static std::chrono::milliseconds RC_OVERRIDE_TIMEOUT {3000};

    // Poll for HL each 10 seconds until success.
    constexpr static std::chrono::seconds HL_POLLING_PERIOD {10};

    // Generate CHANGE_SPEED command only if new speed differs from current speed more than this.
    constexpr static float CHANGE_SPEED_TRESHOLD = 0.1;

    // Generate MAV_CMD_CONDITION_YAW command only if new heading differs from current more than this radians.
    constexpr static float CHANGE_HEADING_TRESHOLD = 0.01;

    // Use this if altitude not given in takeoff_command
    constexpr static float DEFAULT_TAKEOFF_ALTITUDE = 5.0;

    // Counter which governs the end of Joystick mode. To ensure ardupilot
    // exits rc_override we need to spam 0,0,0,0 messages.
    size_t rc_override_end_counter = 0;

    // How frequently to send the rc_overrride messages to vehicle.
    constexpr static std::chrono::milliseconds RC_OVERRIDE_PERIOD {200};

    // How many 0,0,0,0 rc_override messages to send to exit joystick mode.
    constexpr static size_t RC_OVERRIDE_END_COUNT = 15;

    ugcs::vsm::Property::Ptr t_adsb_altitude_internal;
    ugcs::vsm::Property::Ptr t_adsb_transponder_mode;
    ugcs::vsm::Property::Ptr t_adsb_altitude;
    ugcs::vsm::Property::Ptr t_adsb_icao;
    ugcs::vsm::Property::Ptr t_adsb_squawk;
    ugcs::vsm::Property::Ptr t_adsb_ident_active;
    ugcs::vsm::Property::Ptr t_adsb_flight;
    ugcs::vsm::Property::Ptr t_adsb_registration;
    ugcs::vsm::Property::Ptr t_adsb_error_xpdr;
    ugcs::vsm::Property::Ptr t_adsb_error_icao;
    ugcs::vsm::Property::Ptr t_adsb_error_gps;
    ugcs::vsm::Property::Ptr t_adsb_error_squitter;
    ugcs::vsm::Property::Ptr t_adsb_error_temperature;

    std::unordered_map<std::string, ugcs::vsm::Property::Ptr> adsb_parameter_map;
    std::unordered_map<std::string, ugcs::vsm::Property::Ptr> adsb_string_parameter_map;

    ugcs::vsm::Property::Ptr t_battery2_voltage = nullptr;
    ugcs::vsm::Property::Ptr t_battery2_current = nullptr;
    ugcs::vsm::Property::Ptr t_rpm1 = nullptr;
    ugcs::vsm::Property::Ptr t_rail_voltage = nullptr;
    ugcs::vsm::Property::Ptr t_rail_servo_voltage = nullptr;

    std::unordered_map<std::string, ugcs::vsm::Property::Ptr> efi_status_telemetry;

    bool is_airborne = false;

    // Workaround the bug in ardupilot which ignores MAV_CMD_CONDITION_YAW command if there is no
    // WP command executed previously. Used to support set_heading just after takeoff.
    bool set_heading_needs_wp = true;

    // workaround for cases when user does not want to have set_speed on each WP.
    bool ignore_speed_in_route = false;

    // disable route download after upload. Download may take too much time for large missions.
    // This effectively disables the "Current WP" feature.
    bool enable_route_download = false;

    // value of vehicle.ardupilot.route_hash_parameter config parameter
    std::string route_hash_parameter;

    // if route_hash_parameter parameter exists in vehicle actualy
    bool route_hash_parameter_detected = false;

    bool Save_hash_on_autopilot() { return (route_hash_parameter.size() > 0) && route_hash_parameter_detected; }

    // Onboard transponder type if configured.
    ugcs::vsm::Optional<int> adsb_transponder_type;

    virtual bool
    Verify_parameter(const std::string& name, float value, ugcs::vsm::mavlink::MAV_PARAM_TYPE& type) override;

    ugcs::vsm::Optional<float> current_alt_offset;

    // Map of ADSB detected aircrafts (icao code -> adsb vehicle)
    std::unordered_map<uint32_t, Adsb_aircraft::Ptr> adsb_vehicles;

    // check regularly and unregister stale adsb vehicles.
    ugcs::vsm::Timer_processor::Timer::Ptr adsb_vehicles_timer;

    // if there is no update for 1 minute consider vehicle out of range.
    constexpr static std::chrono::seconds ADSB_VEHICLE_TIMEOUT {60};

    // Autopilot version
    uint32_t ardupilot_version = 0;
};

#endif /* _ARDUPILOT_VEHICLE_H_ */
