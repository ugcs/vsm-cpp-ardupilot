// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <ardupilot_vehicle.h>

using namespace ugcs::vsm;

constexpr std::chrono::milliseconds Ardupilot_vehicle::RC_OVERRIDE_PERIOD;
constexpr std::chrono::milliseconds Ardupilot_vehicle::RC_OVERRIDE_TIMEOUT;
constexpr std::chrono::seconds Ardupilot_vehicle::HL_POLLING_PERIOD;
constexpr std::chrono::seconds Ardupilot_vehicle::ADSB_VEHICLE_TIMEOUT;

// Constructor for command processor.
Ardupilot_vehicle::Ardupilot_vehicle(proto::Vehicle_type type):
        Mavlink_vehicle(Vendor::ARDUPILOT, "ardupilot", type),
        vehicle_command(*this),
        task_upload(*this)
{
    switch (type) {
    case proto::VEHICLE_TYPE_FIXED_WING:
        Set_model_name("ArduPlane");
        break;
    case proto::VEHICLE_TYPE_HELICOPTER:
    case proto::VEHICLE_TYPE_MULTICOPTER:
        Set_model_name("ArduCopter");
        break;
    case proto::VEHICLE_TYPE_GROUND:
        Set_model_name("ArduRover");
        break;
    case proto::VEHICLE_TYPE_VTOL:
        Set_model_name("ArduVTOL");
        break;
    }
}

void
Ardupilot_vehicle::On_enable()
{
    Configure_common();

    if (device_type == proto::DEVICE_TYPE_VEHICLE_COMMAND_PROCESSOR) {
        // Do not need any other initialization for command_processor.
        // Just register it with UCS.
        Register();
        // Send command availability.
        Commit_to_ucs();
        return;
    }

    Configure_real_vehicle();

    c_mission_clear->Set_available();

    // Get parameter values.
    Mavlink_vehicle::On_enable();

    payload_processor.configure("vehicle.ardupilot.custom_payload");

    read_version.version_handler = Read_version::Make_version_handler(
        &Ardupilot_vehicle::On_autopilot_version,
        Shared_from_this());
    read_version.Set_next_action(
        Read_version::Make_next_action(
                &Ardupilot_vehicle::On_version_processed,
                this));

    // We need to understand only that FS_EKF_ACTION is supported.
    read_version.Enable();

    read_waypoints.item_handler = Read_waypoints::Make_mission_item_handler(
        &Ardupilot_vehicle::On_mission_item,
        Shared_from_this());

    if (enable_route_download) {
        read_waypoints.Set_next_action(
            Write_parameters::Make_next_action(
                &Ardupilot_vehicle::On_mission_downloaded,
                this));
    }

    mission_upload.item_handler = Mission_upload::Make_mission_request_handler(
        &Ardupilot_vehicle::On_mission_request,
        Shared_from_this());

    // Start polling for home location.
    home_location_timer =
        Timer_processor::Get_instance()->Create_timer(
            HL_POLLING_PERIOD,
            Make_callback(
                &Ardupilot_vehicle::On_home_location_timer,
                Shared_from_this()),
            Get_completion_ctx());

    adsb_vehicles_timer =
        Timer_processor::Get_instance()->Create_timer(
            std::chrono::seconds(1),
            Make_callback(
                &Ardupilot_vehicle::On_adsb_vehicles_timer,
                Shared_from_this()),
            Get_completion_ctx());

    common_handlers.Register_mavlink_handler<mavlink::MESSAGE_ID::PARAM_VALUE>(
        &Ardupilot_vehicle::On_parameter,
        this);

    common_handlers.Register_mavlink_handler<mavlink::MESSAGE_ID::ADSB_VEHICLE>(
        &Ardupilot_vehicle::On_adsb_vehicle,
        this);

    common_handlers.Register_mavlink_handler<mavlink::MESSAGE_ID::HOME_POSITION>(
            &Ardupilot_vehicle::On_home_position,
            this);

    common_handlers.Register_mavlink_handler<mavlink::sph::MESSAGE_ID::PARAM_STR_VALUE, mavlink::sph::Extension>(
        &Ardupilot_vehicle::On_string_parameter,
        this);

    common_handlers.Register_mavlink_handler<mavlink::apm::MESSAGE_ID::RANGEFINDER, mavlink::apm::Extension>(
        &Ardupilot_vehicle::On_rangefinder,
        this);

    common_handlers.Register_mavlink_handler<mavlink::sph::SPH_ADSB_TRANSPONDER_STATE, mavlink::sph::Extension>(
        &Ardupilot_vehicle::On_adsb_state,
        this);

    adsb_transponder = Add_subsystem(proto::SUBSYSTEM_TYPE_ADSB_TRANSPONDER);

    c_adsb_set_mode = adsb_transponder->Add_command("adsb_set_mode", false);
    auto  p = c_adsb_set_mode->Add_parameter("mode", proto::FIELD_SEMANTIC_ADSB_MODE);
    p->Add_enum("off", proto::ADSB_MODE_OFF);
    p->Add_enum("on", proto::ADSB_MODE_ON);
    p->Add_enum("stby", proto::ADSB_MODE_STBY);
    p->Add_enum("alt", proto::ADSB_MODE_ALT);

    c_adsb_set_parameter = adsb_transponder->Add_command("set_parameter", false);
    c_adsb_set_parameter->Add_parameter("adsb_icao", Property::VALUE_TYPE_INT);
    c_adsb_set_parameter->Add_parameter("adsb_squawk", proto::FIELD_SEMANTIC_SQUAWK);
    c_adsb_set_parameter->Add_parameter("adsb_registration", proto::FIELD_SEMANTIC_STRING);
    c_adsb_set_parameter->Add_parameter("adsb_flight_id", proto::FIELD_SEMANTIC_STRING);

    c_adsb_set_ident = adsb_transponder->Add_command("adsb_set_ident", false);

    if (Is_copter()) {
        c_set_heading->Set_available(true);
        c_set_relative_heading->Set_available(true);
        // Takeoff for planes requires too much setup/tuning. Enabling it only for copters.
        c_takeoff_command->Set_available(true);
    }

    c_trigger_calibration->Set_available(true);
    c_trigger_reboot->Set_available(true);

    c_write_parameter->Set_available();
    c_write_parameter->Set_enabled();

    c_set_poi->Set_available();
    c_set_poi->Set_enabled();

#define ADD_T(x, y) t_##x = flight_controller->Add_telemetry(#x, y)
    ADD_T(battery2_voltage, proto::FIELD_SEMANTIC_VOLTAGE);
    ADD_T(battery2_current, proto::FIELD_SEMANTIC_CURRENT);
    ADD_T(rpm1, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(rail_voltage, proto::FIELD_SEMANTIC_VOLTAGE);
    ADD_T(rail_servo_voltage, proto::FIELD_SEMANTIC_VOLTAGE);
#undef ADD_T


    common_handlers.Register_mavlink_handler<mavlink::apm::MESSAGE_ID::RPM, mavlink::apm::Extension>(
            &Ardupilot_vehicle::On_rpm,
            this);

    common_handlers.Register_mavlink_handler<mavlink::apm::MESSAGE_ID::BATTERY2, mavlink::apm::Extension>(
            &Ardupilot_vehicle::On_battery2,
            this);

    common_handlers.Register_mavlink_handler<mavlink::MESSAGE_ID::POWER_STATUS>(
            &Ardupilot_vehicle::On_power_status,
            this);

#define ADD_T(x, y) efi_status_telemetry[#x] = flight_controller->Add_telemetry("efi_" + std::string(#x), y)
    ADD_T(health, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(ecu_index, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(rpm, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(fuel_consumed, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(fuel_flow, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(engine_load, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(throttle_position, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(spark_dwell_time, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(barometric_pressure, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(intake_manifold_pressure, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(intake_manifold_temperature, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(cylinder_head_temperature, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(ignition_timing, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(injection_time, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(exhaust_gas_temperature, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(throttle_out, proto::FIELD_SEMANTIC_NUMERIC);
    ADD_T(pt_compensation, proto::FIELD_SEMANTIC_NUMERIC);
#undef ADD_T

    common_handlers.Register_mavlink_handler<mavlink::MESSAGE_ID::EFI_STATUS>(
            &Ardupilot_vehicle::On_efi_status,
            this);



#define ADD_T(x, y) t_##x = adsb_transponder->Add_telemetry(#x, y)
    ADD_T(adsb_altitude_internal, proto::FIELD_SEMANTIC_BOOL);
    ADD_T(adsb_icao, proto::FIELD_SEMANTIC_ICAO);
    ADD_T(adsb_registration, proto::FIELD_SEMANTIC_STRING);
    ADD_T(adsb_flight, proto::FIELD_SEMANTIC_STRING);
#undef ADD_T

#define ADD_T(x, y, z) t_##x = adsb_transponder->Add_telemetry(#x, y, z)
    // Set timeout for telemetry fields so they automatically
    // are set to N/A if no new data received for 5 seconds.
    ADD_T(adsb_altitude, proto::FIELD_SEMANTIC_ALTITUDE_AMSL, 5);
    ADD_T(adsb_ident_active, proto::FIELD_SEMANTIC_BOOL, 5);
    ADD_T(adsb_squawk, proto::FIELD_SEMANTIC_SQUAWK, 5);
    ADD_T(adsb_transponder_mode, proto::FIELD_SEMANTIC_ADSB_MODE, 5);
    t_adsb_transponder_mode->Add_enum("off", proto::ADSB_MODE_OFF);
    t_adsb_transponder_mode->Add_enum("on", proto::ADSB_MODE_ON);
    t_adsb_transponder_mode->Add_enum("stby", proto::ADSB_MODE_STBY);
    t_adsb_transponder_mode->Add_enum("alt", proto::ADSB_MODE_ALT);
    ADD_T(adsb_error_xpdr, proto::FIELD_SEMANTIC_BOOL, 5);
    ADD_T(adsb_error_icao, proto::FIELD_SEMANTIC_BOOL, 5);
    ADD_T(adsb_error_gps, proto::FIELD_SEMANTIC_BOOL, 5);
    ADD_T(adsb_error_squitter, proto::FIELD_SEMANTIC_BOOL, 5);
    ADD_T(adsb_error_temperature, proto::FIELD_SEMANTIC_BOOL, 5);
#undef ADD_T

    adsb_parameter_map.insert({"ADSB_ICAO_ID", t_adsb_icao});
    adsb_parameter_map.insert({"ADSB_ALT_SRC", t_adsb_altitude_internal});

    adsb_string_parameter_map.insert({"ADSB_FLIGHT", t_adsb_flight});
    adsb_string_parameter_map.insert({"ADSB_TAIL", t_adsb_registration});

    if (enable_route_download) {
        // download mission only if explicitly configured and route_hash_parameter not set.
        read_waypoints.Enable();
    }

    // Ardupilot does not send target location for spline WPts.
    // If target location is not received for two seconds, invalidate it.
    t_target_altitude_amsl->Set_timeout(2);
    t_target_altitude_raw->Set_timeout(2);
    t_target_latitude->Set_timeout(2);
    t_target_longitude->Set_timeout(2);

    common_handlers.Register_mavlink_handler<mavlink::apm::MOUNT_STATUS, mavlink::apm::Extension>(
            &Ardupilot_vehicle::On_mount_status,
            this);

    Set_parameters_from_properties("vehicle.ardupilot.parameter");
}

bool
Ardupilot_vehicle::Verify_parameter(const std::string& name, float value, mavlink::MAV_PARAM_TYPE& type)
{
    if (name == "DISARM_DELAY") {
        if (value >= 0 && value <= 127) {
            type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
            return true;
        } else {
            return false;
        }
    }
    if (name == "RTL_ALT_FINAL") {
        if (!Is_copter()) {
            // Parameter supported by copters only.
            return false;
        }
        if (value < 0) {
            // Must be positive.
            return false;
        }
    }

    // There is no logic in ardupilot firmware which takes MAV_PARAM_TYPE into account.
    // So we may use any type.
    // We add this for DDC case when we need to update arbitrary parameter among dozens of drones.
    // Old code is above, may be we remove it in the future.
    type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
    return true;
}

void
Ardupilot_vehicle::On_adsb_state(
    mavlink::Message<
        mavlink::sph::SPH_ADSB_TRANSPONDER_STATE,
        mavlink::sph::Extension>::Ptr message)
{
    if (!adsb_transponder_type) {
        adsb_transponder_type = message->payload->type.Get();
        switch (*adsb_transponder_type) {
        case 2: // BCON
            VEHICLE_LOG_INF(*this, "ADSB transponder detected: BCON");
            c_adsb_set_mode->Set_enabled();
            c_adsb_set_mode->Set_available();
            c_adsb_set_parameter->Set_enabled();
            c_adsb_set_parameter->Set_available();
            read_parameters.Enable({"ADSB_ICAO_ID", "ADSB_ALT_SRC"});
            break;
        case 3: // Sagetech
            VEHICLE_LOG_INF(*this, "ADSB transponder detected: Sagetech");
            c_adsb_set_mode->Set_enabled();
            c_adsb_set_mode->Set_available();
            c_adsb_set_ident->Set_enabled();
            c_adsb_set_ident->Set_available();
            c_adsb_set_parameter->Set_enabled();
            c_adsb_set_parameter->Set_available();
            read_parameters.Enable({"ADSB_ICAO_ID", "ADSB_ALT_SRC"});
            read_string_parameters.Enable({"ADSB_FLIGHT", "ADSB_TAIL"});
            break;
        default:
            VEHICLE_LOG_INF(*this, "No ADSB transponder detected");
            return; // No transponder present.
        }
    }

    if (!message->payload->squawk_code.Is_reset()) {
        t_adsb_squawk->Set_value(message->payload->squawk_code.Get());
    }

    if (!message->payload->ident_active.Is_reset()) {
        t_adsb_ident_active->Set_value(message->payload->ident_active.Get());
    }

    if (!message->payload->altitude.Is_reset()) {
        t_adsb_altitude->Set_value(message->payload->altitude.Get());
    }

    if (!message->payload->error_flags.Is_reset()) {
        int f = message->payload->error_flags.Get();
        t_adsb_error_gps->Set_value(f & 0x01);
        t_adsb_error_icao->Set_value(f & 0x02);
        t_adsb_error_temperature->Set_value(f & 0x04);
        t_adsb_error_squitter->Set_value(f & 0x08);
        t_adsb_error_xpdr->Set_value(f & 0x10);
    }

    if (message->payload->altitude_source_internal.Is_reset()) {
        t_adsb_altitude_internal->Set_value_na();
    } else {
        t_adsb_altitude_internal->Set_value(message->payload->altitude_source_internal.Get());
    }

    if (message->payload->transponder_mode.Is_reset()) {
        t_adsb_transponder_mode->Set_value_na();
    } else {
        switch (message->payload->transponder_mode.Get()) {
        case mavlink::sph::ADSB_TRANSPONDER_MODE_ON:
            t_adsb_transponder_mode->Set_value(proto::ADSB_MODE_ON);
            break;
        case mavlink::sph::ADSB_TRANSPONDER_MODE_ALT:
            t_adsb_transponder_mode->Set_value(proto::ADSB_MODE_ALT);
            break;
        case mavlink::sph::ADSB_TRANSPONDER_MODE_STBY:
            t_adsb_transponder_mode->Set_value(proto::ADSB_MODE_STBY);
            break;
        case mavlink::sph::ADSB_TRANSPONDER_MODE_OFF:
            t_adsb_transponder_mode->Set_value(proto::ADSB_MODE_OFF);
            break;
        default:
            t_adsb_transponder_mode->Set_value_na();
            break;
        }
    }
    Commit_to_ucs();
}


void
Ardupilot_vehicle::Handle_ucs_command(
    Ucs_request::Ptr ucs_request)
{
    if (vehicle_command.ucs_request) {
        Command_failed(ucs_request, "Previous request in progress");
        return;
    }

    if (ucs_request->request.device_commands_size() == 0) {
        Command_failed(ucs_request, "No commands found", proto::STATUS_INVALID_COMMAND);
        return;
    }

    try {
        auto &vsm_cmd = ucs_request->request.device_commands(0);
        auto cmd = Get_command(vsm_cmd.command_id());

        if (cmd == c_mission_upload) {
            VEHICLE_LOG_INF((*this), "COMMAND %s", Dump_command(vsm_cmd).c_str());
            if (read_waypoints.In_progress()) {
                Command_failed(ucs_request, "Mission download in progress");
                return;
            }
            task_upload.Disable("Internal error");
            task_upload.ucs_request = ucs_request;
            task_upload.Enable(false);
            return;
        } else if (cmd == c_get_native_route) {
            VEHICLE_LOG_INF((*this), "COMMAND %s", Dump_command(vsm_cmd).c_str());
            task_upload.Disable("Internal error");
            task_upload.ucs_request = ucs_request;
            task_upload.Enable(true);
            return;
        } else if (payload_processor.isSubsystemCmd(cmd)) {
            payload_processor.Disable("Internal error");
            payload_processor.ucs_request = ucs_request;
            payload_processor.Enable();
            return;
        }

        vehicle_command.Disable("Internal error");
        vehicle_command.ucs_request = ucs_request;
        vehicle_command.Enable();
    }
    catch (Altimeter_required_exception& e) {
        Command_failed(ucs_request, e.what(), proto::STATUS_ALTIMETER_REQUIRED);
    }
    catch (const std::exception& ex) {
        Command_failed(ucs_request, ex.what(), proto::STATUS_INVALID_PARAM);
    }
}

void
Ardupilot_vehicle::On_disable()
{
    vehicle_command.Disable("Vehicle disconnected");
    task_upload.Disable("Vehicle disconnected");
    if (device_type == proto::DEVICE_TYPE_VEHICLE) {
        for (auto it = adsb_vehicles.begin(); it != adsb_vehicles.end();) {
            it->second->Unregister();
            it = adsb_vehicles.erase(it);
        }
        if (rc_override_timer) {
            rc_override_timer->Cancel();
        }
        home_location_timer->Cancel();
        adsb_vehicles_timer->Cancel();
        mission_upload.item_handler = Mission_upload::Mission_request_handler();
        read_waypoints.item_handler = Read_waypoints::Mission_item_handler();
        read_version.version_handler = Read_version::Version_handler();
        Mavlink_vehicle::On_disable();
    }
}

void
Ardupilot_vehicle::On_autopilot_version(mavlink::Pld_autopilot_version ver)
{
    ardupilot_version = ver->flight_sw_version.Get();
    int maj = (ardupilot_version >> 24) & 0xff;
    int min = (ardupilot_version >> 16) & 0xff;
    int patch = (ardupilot_version >> 8) & 0xff;
    int type = (ardupilot_version >> 0) & 0xff;
    VEHICLE_LOG_INF(*this, "Ardupilot version=%d.%d.%d, type=%d", maj, min, patch, type);

    if (!Is_version_less_than(3, 3, 1)) {
        use_ekf_action_as_gps_failsafe = true;
        send_home_position_as_mav_cmd = true;
        auto_generate_mission_poi = false;
    }

    std::unordered_set<std::string> names = {"FENCE_ENABLE"};

    if (!Is_version_less_than(3, 4, 0)) {
        gnd_alt_offset_allow = true;
        if (set_ground_alt_offset) {
            names.emplace("GND_ALT_OFFSET");
        }
    }

    if (!Is_version_less_than(3, 7, 0) &&
        Is_vehicle_type(proto::VEHICLE_TYPE_FIXED_WING))
    {
        // Detect if this is VTOL plane.
        names.emplace("Q_ENABLE");
    } else {
        if (!Is_registered()) {
            Register();
        }
    }

    if (!Is_version_less_than(3, 6, 0) && Is_copter()) {
        use_batt_fs_as_batt_failsafe = true;
        do_set_cam_trig_dist_as_command = true;
    }

    if (camera_servo_idx != -1) {
        names.emplace("SERVO" + std::to_string(camera_servo_idx) + "_FUNCTION");
        if (camera_servo_time == -1) {
            names.emplace("CAM_DURATION");
        }

        if (camera_servo_pwm == -1) {
            names.emplace("CAM_SERVO_ON");
        }
    }
    if (route_hash_parameter.size()) {
        names.emplace(route_hash_parameter);
    }

    if ((ver->capabilities & mavlink::MAV_PROTOCOL_CAPABILITY_MAVLINK2)
        && !mav_stream->Is_mavlink_v2()
        && !use_mavlink_2)
    {
        mav_stream->Set_mavlink_v2(true);
        VEHICLE_LOG_DBG(*this, "Enabled MAVLINK2");
    }

    names.emplace("MNT_ANGMAX_PAN");
    names.emplace("MNT_ANGMAX_ROL");
    names.emplace("MNT_ANGMAX_TIL");
    names.emplace("MNT_ANGMIN_PAN");
    names.emplace("MNT_ANGMIN_ROL");
    names.emplace("MNT_ANGMIN_TIL");
    names.emplace("MNT_DEFLT_MODE");

    read_parameters.Enable(std::move(names));
}

void
Ardupilot_vehicle::On_version_processed(bool success, std::string)
{
    // Support for ardupilot < 3.2.
    if (!success && !Is_registered()) {
        // If there is no answer to AUTOPILOT_VERSION_REQUEST register as is.
        Register();
    }
}

void
Ardupilot_vehicle::On_rangefinder(
        mavlink::Message<mavlink::apm::MESSAGE_ID::RANGEFINDER, mavlink::apm::Extension>::Ptr message)
{
    if (!message->payload->distance.Is_reset()) {
        t_altitude_agl->Set_value(message->payload->distance.Get());
        Commit_to_ucs();
    }
}

void
Ardupilot_vehicle::On_rpm(
        mavlink::Message<mavlink::apm::MESSAGE_ID::RPM, mavlink::apm::Extension>::Ptr message)
{
    t_rpm1->Set_value(message->payload->rpm1.Get());
    Commit_to_ucs();
}

void
Ardupilot_vehicle::On_battery2(
        mavlink::Message<mavlink::apm::MESSAGE_ID::BATTERY2, mavlink::apm::Extension>::Ptr message)
{
    t_battery2_voltage->Set_value(message->payload->voltage.Get() / 1000.0);
    t_battery2_current->Set_value(message->payload->current_battery.Get() / 100.0);
    Commit_to_ucs();
}

void
Ardupilot_vehicle::On_power_status(ugcs::vsm::mavlink::Message<mavlink::MESSAGE_ID::POWER_STATUS>::Ptr message)
{
    t_rail_voltage->Set_value(message->payload->Vcc.Get() / 1000.0);
    t_rail_servo_voltage->Set_value(message->payload->Vservo.Get() / 1000.0);
    Commit_to_ucs();
}

void
Ardupilot_vehicle::On_efi_status(ugcs::vsm::mavlink::Message<mavlink::MESSAGE_ID::EFI_STATUS>::Ptr message)
{
    efi_status_telemetry["health"]->Set_value(message->payload->health.Get());
    efi_status_telemetry["ecu_index"]->Set_value(message->payload->ecu_index.Get());
    efi_status_telemetry["rpm"]->Set_value(message->payload->rpm.Get());
    efi_status_telemetry["fuel_consumed"]->Set_value(message->payload->fuel_consumed.Get());
    efi_status_telemetry["fuel_flow"]->Set_value(message->payload->fuel_flow.Get());
    efi_status_telemetry["engine_load"]->Set_value(message->payload->engine_load.Get());
    efi_status_telemetry["throttle_position"]->Set_value(message->payload->throttle_position.Get());
    efi_status_telemetry["spark_dwell_time"]->Set_value(message->payload->spark_dwell_time.Get());
    efi_status_telemetry["barometric_pressure"]->Set_value(message->payload->barometric_pressure.Get());
    efi_status_telemetry["intake_manifold_pressure"]->Set_value(message->payload->intake_manifold_pressure.Get());
    efi_status_telemetry["intake_manifold_temperature"]->Set_value(message->payload->intake_manifold_temperature.Get());
    efi_status_telemetry["cylinder_head_temperature"]->Set_value(message->payload->cylinder_head_temperature.Get());
    efi_status_telemetry["ignition_timing"]->Set_value(message->payload->ignition_timing.Get());
    efi_status_telemetry["injection_time"]->Set_value(message->payload->injection_time.Get());
    efi_status_telemetry["exhaust_gas_temperature"]->Set_value(message->payload->exhaust_gas_temperature.Get());
    efi_status_telemetry["throttle_out"]->Set_value(message->payload->throttle_out.Get());
    efi_status_telemetry["pt_compensation"]->Set_value(message->payload->pt_compensation.Get());

    Commit_to_ucs();
}

void
Ardupilot_vehicle::On_parameter(
    mavlink::Message<mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr m)
{
    const auto &name = m->payload->param_id.Get_string();
    auto i = adsb_parameter_map.find(name);
    if (i != adsb_parameter_map.end()) {
        i->second->Set_value(m->payload->param_value.Get());
    }
    if (name == "GND_ALT_OFFSET") {
        current_alt_offset = m->payload->param_value.Get();
    } else if (name == "FENCE_ENABLE") {
        t_fence_enabled->Set_value(m->payload->param_value.Get() != 0);
    } else if (name == "CAM_SERVO_ON") {
        camera_servo_pwm = m->payload->param_value.Get();
    } else if (name == "CAM_DURATION") {
        // CAM_DURATION is in deciseconds.
        camera_servo_time  = m->payload->param_value.Get() / 10;
    } else if (name == "Q_ENABLE") {
        if (m->payload->param_value.Get() != 0) {
           Set_vehicle_type(proto::VEHICLE_TYPE_VTOL);
        }
        if (!Is_registered()) {
            Register();
        }
    } else if (camera_servo_idx != -1
                && (name == std::string("SERVO") + std::to_string(camera_servo_idx) + "_FUNCTION")) {
        int func = m->payload->param_value.Get();
        if (func != 0) {
            VEHICLE_LOG_ERR(
                *this,
                "camera_servo_idx specified but the corresponding function is set to %d. "
                "Camera triggering will not work.",
                func);
        }
    } else if (route_hash_parameter.size() && name == route_hash_parameter) {
        float f = m->payload->param_value.Get();
        uint32_t id;
        memcpy(&id, &f, sizeof(id));
        VEHICLE_LOG_INF((*this), "Parameter %s with mission hash is found.", route_hash_parameter.c_str());
        VEHICLE_LOG_INF((*this), "Got mission hash=%08X", id);
        t_current_mission_id->Set_value(id);
        route_hash_parameter_detected = true;
    } else if (name == "MNT_ANGMAX_PAN") {
        mount_constraints.mount_max_pan =  m->payload->param_value.Get();
    } else if (name == "MNT_ANGMAX_ROL") {
        mount_constraints.mount_max_roll =  m->payload->param_value.Get();
    } else if (name == "MNT_ANGMAX_TIL") {
        mount_constraints.mount_max_tilt =  m->payload->param_value.Get();
    } else if (name == "MNT_ANGMIN_PAN") {
        mount_constraints.mount_min_pan =  m->payload->param_value.Get();
    } else if (name == "MNT_ANGMIN_ROL") {
        mount_constraints.mount_min_roll =  m->payload->param_value.Get();
    } else if (name == "MNT_ANGMIN_TIL") {
        mount_constraints.mount_min_tilt =  m->payload->param_value.Get();
    } else if (name == "MNT_DEFLT_MODE") {
        is_mount_control_enabled =
                m->payload->param_value.Get() == mavlink::MAV_MOUNT_MODE::MAV_MOUNT_MODE_MAVLINK_TARGETING;
    }
    Commit_to_ucs();
}

void
Ardupilot_vehicle::On_string_parameter(
    mavlink::Message<
        mavlink::sph::MESSAGE_ID::PARAM_STR_VALUE,
        mavlink::sph::Extension>::Ptr message)
{
    auto i = adsb_string_parameter_map.find(message->payload->param_id.Get_string());
    if (i != adsb_string_parameter_map.end()) {
        i->second->Set_value(message->payload->param_value.Get_string());
        Commit_to_ucs();
    }
}

void
Ardupilot_vehicle::On_adsb_vehicle(
        ugcs::vsm::mavlink::Message<mavlink::MESSAGE_ID::ADSB_VEHICLE>::Ptr message)
{
    auto icao = message->payload->ICAO_address.Get();
    auto ret = adsb_vehicles.emplace(icao, nullptr);
    auto &vehicle = ret.first->second;
    if (ret.second) {
        // Create vehicle if there is not one with this ICAO already.
        vehicle = Adsb_aircraft::Create(icao);
        vehicle->Enable();

        // Tell server that ADSB receiver has discovered new aircraft.
        vehicle->Register();
        LOG("New ADSB aircraft %06X registered", icao);
    }
    // Send telemetry data for this vehicle.
    vehicle->On_message(message);
}


void
Ardupilot_vehicle::On_mission_item(ugcs::vsm::mavlink::Pld_mission_item_int mi)
{
    if (mi->seq == 0) {
        Report_home_location((static_cast<double>(mi->x) / 1e7) * M_PI / 180,
                             (static_cast<double>(mi->y) / 1e7) * M_PI / 180, mi->z.Get());
    } else {
        // Do not add home location to mission hash because:
        // 1) autopilot can report different HL than uploaded.
        // 2) Get_home_location must not interfere with current downloaded mission
        //    as it will invalidate the mission hash.
        current_command_map.Accumulate_route_id(Get_mission_item_hash(mi));
        VEHICLE_LOG_INF(*this,
            "Mission item %d command %d,\n"
            "  p1=%f p2=%f p3=%f p4=%f p5=%d p6=%d p7=%f",
            mi->seq.Get(),
            mi->command.Get(),
            mi->param1.Get(),
            mi->param2.Get(),
            mi->param3.Get(),
            mi->param4.Get(),
            mi->x.Get(), mi->y.Get(), mi->z.Get());
    }
    if (task_upload.ucs_request) {
        task_upload.On_download_progress(mi->seq);
    }
    current_route.Add_item(mi);
}

void
Ardupilot_vehicle::On_mission_request(int seq)
{
    if (task_upload.ucs_request) {
        task_upload.On_upload_progress(seq);
    }
}

void
Ardupilot_vehicle::On_home_position(
    mavlink::Message<mavlink::MESSAGE_ID::HOME_POSITION>::Ptr hp)
{
    double lat = hp->payload->latitude.Get();
    double lon = hp->payload->longitude.Get();
    double alt = hp->payload->altitude.Get();
    Report_home_location((lat / 10000000) * M_PI / 180, (lon / 10000000) * M_PI / 180, alt / 1000);
}

void
Ardupilot_vehicle::Report_home_location(double lat, double lon, double alt)
{
    // Ignore HL position change for less than 1 meter.
    // (ArduPlane sends HL frequently)
    if (fabs(home_location.latitude - lat) < 2e-7
    &&  fabs(home_location.longitude - lon) < 2e-7
    &&  fabs(home_location.altitude - alt) < 1) {
        return;
    }

    home_location.latitude = lat;
    home_location.longitude = lon;
    home_location.altitude = alt;

    if (Is_home_position_valid()) {
        VEHICLE_LOG_INF(*this,
            "Got home position: x=%f, y=%f, z=%f. Setting altitude origin.",
            lat * 180 / M_PI, lon * 180 / M_PI, alt);
        t_home_latitude->Set_value(lat);
        t_home_longitude->Set_value(lon);
        t_home_altitude_amsl->Set_value(alt);
        Set_altitude_origin(alt);   // this calls Commit_to_ucs.
    }
}

void
Ardupilot_vehicle::On_mission_downloaded(bool ok, const std::string status)
{
    if (ok) {
        VEHICLE_LOG_DBG(*this, "Mission downloaded. mission_id=%08X", current_command_map.Get_route_id());
        t_current_mission_id->Set_value(current_command_map.Get_route_id());
    } else {
        VEHICLE_LOG_DBG(*this, "Mission download failed: %s", status.c_str());
        Add_status_message(status);
    }
    read_waypoints.Disable();
}

bool
Ardupilot_vehicle::Is_home_position_valid()
{
    return (home_location.latitude != 0) || (home_location.longitude != 0);
}

bool
Ardupilot_vehicle::On_home_location_timer()
{
    // Keep requesting HL because it can change during mission flight.
    Get_home_location();
    if (Get_completion_ctx()->Is_enabled()) {
        return true;
    }
    return false;
}

bool
Ardupilot_vehicle::On_adsb_vehicles_timer()
{
    for (auto it = adsb_vehicles.begin(); it != adsb_vehicles.end();) {
        if (it->second->Time_since_last_update() > ADSB_VEHICLE_TIMEOUT) {
            LOG("ADSB aircraft %06X timeouted", it->first);
            it->second->Unregister();
            it = adsb_vehicles.erase(it);
        } else {
            it++;
        }
    }
    if (Get_completion_ctx()->Is_enabled()) {
        return true;
    }
    return false;
}

bool
Ardupilot_vehicle::Vehicle_command_act::Try()
{
    if (!remaining_attempts--) {
        VEHICLE_LOG_WRN(vehicle, "Vehicle_command all attempts failed.");
        Disable("Vehicle_command all attempts failed.");
        return false;
    }

    if (cmd_messages.size()) {
        Send_message(*(cmd_messages.front()));

        auto cmd = cmd_messages.front();
        VEHICLE_LOG_DBG(vehicle, "Sending to vehicle: %s", (*(cmd_messages.front())).Dump().c_str());
        if (cmd->Get_id() == mavlink::apm::MESSAGE_ID::FENCE_POINT) {
            Schedule_verify_timer();
        } else if (cmd->Get_id() == mavlink::MESSAGE_ID::SET_POSITION_TARGET_LOCAL_NED || cmd->Get_id() == mavlink::apm::MESSAGE_ID::MOUNT_CONTROL) {
            // there will be no ack from this cmd so jump to next cmd
            Send_next_command();
        } else {
            Schedule_timer();
        }

    } else {
        // Command list is empty, nothing to do.
        Disable("Command list empty");
    }

    return false;
}

bool
Ardupilot_vehicle::Vehicle_command_act::Try_verify_polyfence()
{
    if (cmd_messages.size()) {
        // for polygon geofence set message send message for verification
        auto cmd = cmd_messages.front();
        if (cmd->Get_id() == mavlink::apm::MESSAGE_ID::FENCE_POINT) {
            auto& payload = (*std::static_pointer_cast<mavlink::apm::Pld_fence_point>(cmd));
            int idx = payload->idx.Get();
            VEHICLE_LOG_DBG(vehicle, "Verify geofence for point %d", idx);
            mavlink::apm::Pld_fence_fetch_point fetch_point;
            Fill_target_ids(fetch_point);
            fetch_point->idx = idx;
            Send_message(fetch_point);
        } else {
            VEHICLE_LOG_WRN(vehicle, "Cancel geofence verifying: last command was not FENCE_POINT");
        }

        // Schedule timer for next common command retry
        Schedule_timer();

    } else {
        // Command list is empty, nothing to do.
        Disable("Command list empty");
    }

    return false;
}

void
Ardupilot_vehicle::Vehicle_command_act::On_point_value(
        mavlink::Message<mavlink::apm::MESSAGE_ID::FENCE_POINT, mavlink::apm::Extension>::Ptr message)
{
    auto cmd = cmd_messages.front();
    if (cmd->Get_id() != mavlink::apm::MESSAGE_ID::FENCE_POINT) {
        // current command is not fence point. Skip checking.
        return;
    }

    auto& payload = (*std::static_pointer_cast<mavlink::apm::Pld_fence_point>(cmd));
    int idx = payload->idx.Get();
    double lat = payload->lat.Get();
    double lng = payload->lng.Get();

    double message_lat = message->payload->lat.Get();
    double message_lng = message->payload->lng.Get();


    if (message->payload->idx.Get() != idx) {
        /* Not the point we requested.
         * Don't reschedule immediately, wait for retry timeout.
         */
        return;
    }

    double relativeError_lat = fabs((message_lat - lat) / lat);
    double relativeError_lng = fabs((message_lng - lng) / lng);
    if ((relativeError_lat > 0.0000002) || (relativeError_lng > 0.0000002)) {
        VEHICLE_LOG_WRN(vehicle, "Geofence point writing validation failed for point [%d]. Lat,Lng [%2.20f, %2.20f] "
                                "expected, but [%2.20f, %2.20f] received. Relative errors are [%2.10f, %2.10f] ",
                        idx,
                        lat,
                        lng,
                        message_lat,
                        message_lng,
                        relativeError_lat,
                        relativeError_lng);
        Try();
        return;
    }
    VEHICLE_LOG_DBG(vehicle,
        "Geofence point [%d:%2.16f, %2.16f] write verified successfully. Relative errors are [%2.10f, %2.10f].",
        message->payload->idx.Get(),
        message->payload->lat.Get(),
        message->payload->lng.Get(),
        relativeError_lat,
        relativeError_lng);
    /* Written OK. */

    Send_next_command();
}

void
Ardupilot_vehicle::On_gimbal_report(
        ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::apm::MESSAGE_ID::GIMBAL_REPORT ,
                ugcs::vsm::mavlink::apm::Extension>::Ptr ) {

}

void
Ardupilot_vehicle::On_mount_status(
        ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::apm::MOUNT_STATUS ,
                ugcs::vsm::mavlink::apm::Extension>::Ptr message) {

    float pitch = message->payload->pointing_a.Get() * 0.01;
    float roll = message->payload->pointing_b.Get() * 0.01;
    float yaw = message->payload->pointing_c.Get()* 0.01;

    t_gimbal_pitch->Set_value(pitch * M_PI / 180.0);
    t_gimbal_roll->Set_value(roll * M_PI / 180.0);
    t_gimbal_heading->Set_value(yaw * M_PI / 180.0);
    Commit_to_ucs();
}

void
Ardupilot_vehicle::Start_rc_override()
{
    if (rc_override == nullptr) {
        // Create rc_override message. timer will delete it when vehicle switched to other mode.
        rc_override = mavlink::Pld_rc_channels_override::Create();
        (*rc_override)->target_system = real_system_id;
        (*rc_override)->target_component = real_component_id;
        rc_override_timer = Timer_processor::Get_instance()->Create_timer(
            RC_OVERRIDE_PERIOD,
            Make_callback(&Ardupilot_vehicle::Send_rc_override_timer, Shared_from_this()),
            Get_completion_ctx());
    }
    rc_override_end_counter = RC_OVERRIDE_END_COUNT;
    // Set larger timeout when turning on joystick mode
    // to let client more time to understand that joystick commands must be sent, now.
    direct_vehicle_control_last_received =
        std::chrono::steady_clock::now() + RC_OVERRIDE_TIMEOUT;
    Set_rc_override(1500, 1500, 1500, 1500);
}



void
Ardupilot_vehicle::Set_rc_override(int p, int r, int t, int y)
{
    if (Is_rc_override_active()) {
        (*rc_override)->chan1_raw = p;
        (*rc_override)->chan2_raw = r;
        (*rc_override)->chan3_raw = t;
        (*rc_override)->chan4_raw = y;
    }
}

void
Ardupilot_vehicle::Stop_rc_override()
{
    if (Is_rc_override_active()) {
        Set_rc_override(0, 0, 0, 0);
        // initiate countdown
        rc_override_end_counter = RC_OVERRIDE_END_COUNT - 1;
    }
}

void
Ardupilot_vehicle::Send_rc_override()
{
    // Do not send
    if (rc_override) {
//        LOG("Direct vehicle %d %d %d %d",
//            (*rc_override)->chan1_raw.Get(),
//            (*rc_override)->chan2_raw.Get(),
//            (*rc_override)->chan3_raw.Get(),
//            (*rc_override)->chan4_raw.Get()
//            );
        mav_stream->Send_message(
                *rc_override,
                255,
                190,
                Mavlink_vehicle::WRITE_TIMEOUT,
                Make_timeout_callback(
                        &Mavlink_vehicle::Write_to_vehicle_timed_out,
                        Shared_from_this(),
                        mav_stream),
                Get_completion_ctx());

        rc_override_last_sent = std::chrono::steady_clock::now();
        if (rc_override_end_counter < RC_OVERRIDE_END_COUNT && rc_override_end_counter > 0) {
            rc_override_end_counter--;
        }
    }
}

bool
Ardupilot_vehicle::Is_rc_override_active()
{
    return (rc_override && rc_override_end_counter == RC_OVERRIDE_END_COUNT);
}

bool
Ardupilot_vehicle::Send_rc_override_timer()
{
    if (rc_override == nullptr) {
        return false;
    }

    auto now = std::chrono::steady_clock::now();

    if (now - direct_vehicle_control_last_received > RC_OVERRIDE_TIMEOUT) {
        // Automatically exit joystick mode if there are no control messages from ucs.
        Stop_rc_override();
    }

    if (now - rc_override_last_sent < RC_OVERRIDE_PERIOD) {
        // Do not spam radio link too much.
        return true;
    }

    Send_rc_override();

    if (rc_override_end_counter == 0) {
        // exiting joystick mode.
        rc_override = nullptr;
        return false;
    }
    return true;
}

void
Ardupilot_vehicle::Vehicle_command_act::Stop_camera_series()
{
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    if (ardu_vehicle.camera_servo_idx == -1) {
        // It is not possible to stop camera_by_distance on ardupilot up until 3.6
        if (ardu_vehicle.do_set_cam_trig_dist_as_command) {
            (*cmd_long)->command = mavlink::MAV_CMD_DO_SET_CAM_TRIGG_DIST;
            (*cmd_long)->param1 = 0;
        } else {
            return;
        }
    } else {
        (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_DO_REPEAT_SERVO;
        (*cmd_long)->param1 = ardu_vehicle.camera_servo_idx;
        (*cmd_long)->param2 = ardu_vehicle.camera_servo_pwm;
        (*cmd_long)->param3 = 1;
        (*cmd_long)->param4 = 0;
    }
    cmd_messages.emplace_back(cmd_long);
}


void Ardupilot_vehicle::Vehicle_command_act::Process_direct_mount_control(const ugcs::vsm::Property_list &params) {
    if (!ardu_vehicle.is_mount_control_enabled ) {
        Disable("Direct Mount control failed. MNT_DEFLT_MODE must be set to 2");
        return;
    }

    auto mc = mavlink::apm::Pld_mount_control::Create();
    Fill_target_system_id(*mc);

    float tmp;
    params.Get_value("pitch", tmp);
    // Convert range (-1, 1) to (MIN, MAX)
    (*mc)->input_a = static_cast<int>(ardu_vehicle.mount_constraints.mount_min_tilt * (1 - tmp) / 2
                                    + ardu_vehicle.mount_constraints.mount_max_tilt * (tmp + 1) / 2);

    params.Get_value("roll", tmp);
    (*mc)->input_b = static_cast<int> (ardu_vehicle.mount_constraints.mount_min_roll * (1 - tmp) / 2
                                    + ardu_vehicle.mount_constraints.mount_max_roll * (tmp + 1) / 2);

    params.Get_value("yaw", tmp);
    (*mc)->input_c = static_cast<int>(ardu_vehicle.mount_constraints.mount_min_pan * (1 - tmp) / 2
                                    + ardu_vehicle.mount_constraints.mount_max_pan * (tmp + 1) / 2);

    cmd_messages.emplace_back(mc);
}

void
Ardupilot_vehicle::Vehicle_command_act::Send_next_command()
{
    cmd_messages.pop_front();
    if (cmd_messages.size()) {
        // send next command in chain.
        remaining_attempts = try_count;

        vehicle.Report_progress(ucs_request, (command_count - cmd_messages.size()) / command_count);

        auto cmd = cmd_messages.front();
        if (cmd->Get_id() == mavlink::apm::MESSAGE_ID::FENCE_POINT) {
            // check if fence already set equal to this point
            auto& payload = (*std::static_pointer_cast<mavlink::apm::Pld_fence_point>(cmd));
            int idx = payload->idx.Get();
            VEHICLE_LOG_DBG(vehicle, "Check current geofence value for point %d", idx);
            mavlink::apm::Pld_fence_fetch_point fetch_point;
            Fill_target_ids(fetch_point);
            fetch_point->idx = idx;
            Send_message(fetch_point);

            Schedule_timer();
        } else {
            Send_message(*(cmd_messages.front()));
            VEHICLE_LOG_DBG(vehicle, "Sending to vehicle: %s", (*(cmd_messages.front())).Dump().c_str());
            if (cmd->Get_id() == mavlink::MESSAGE_ID::SET_POSITION_TARGET_LOCAL_NED) {
                // there will be no ack from this cmd so jump to next cmd
                Send_next_command();
            } else {
                Schedule_timer();
            }
        }

    } else {
        // command chain succeeded.
        Disable_success();
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::On_command_ack(
        mavlink::Message<mavlink::MESSAGE_ID::COMMAND_ACK>::Ptr message)
{
    VEHICLE_LOG_DBG(vehicle, "COMAMND_ACK for command %d, res=%d",
            message->payload->command.Get(), message->payload->result.Get());

    if (cmd_messages.size()) {
        // we are waiting for response.
        auto cmd = cmd_messages.front();
        int command_id = cmd->Get_id();
        if (command_id == mavlink::MESSAGE_ID::COMMAND_LONG) {
            command_id = (*std::static_pointer_cast<mavlink::Pld_command_long>(cmd))->command.Get();
        }
        if (message->payload->command.Get() == command_id) {
            // This is a response to our command.
            if (message->payload->result == mavlink::MAV_RESULT::MAV_RESULT_ACCEPTED) {
                Send_next_command();
            } else {
                auto p = message->payload->result.Get();
                auto s = std::string("Result: ")
                    + std::to_string(p)
                    + " ("
                    + Mav_result_to_string(p)
                    + ")";
                auto e = vehicle.Get_failed_sensor_report();
                if (!e.empty()) {
                    s += std::string(", ") + e;
                }
                Disable(s);
            }
        }
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::On_mission_current(
        mavlink::Message<mavlink::MESSAGE_ID::MISSION_CURRENT>::Ptr message)
{
    if (cmd_messages.size()) {
        // we are waiting for response.
        auto cmd = cmd_messages.front();
        int command_id = cmd->Get_id();
        if (command_id == mavlink::MESSAGE_ID::MISSION_SET_CURRENT) {
            int seq = (*std::static_pointer_cast<mavlink::Pld_mission_set_current>(cmd))->seq.Get();
            // Autopilot did change the current command.
            // > here because sometimes Ardupilot responds with current sequence == seq+1;
            if (message->payload->seq >= seq) {
                Send_next_command();
            }
        }
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::On_mission_ack(
        mavlink::Message<mavlink::MESSAGE_ID::MISSION_ACK>::Ptr message)
{
    VEHICLE_LOG_INF(vehicle, "MISSION_ACK, result %d",
            message->payload->type.Get());

    if (cmd_messages.size()) {
        if (message->payload->type == mavlink::MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED) {
            if (cmd_messages.front()->Get_id() == mavlink::MISSION_CLEAR_ALL) {
                // Invalidate mission id after mission_clear.
                vehicle.t_current_mission_id->Set_value_na();
                vehicle.Commit_to_ucs();
            }
            Send_next_command();
        } else {
            auto p = message->payload->type.Get();
            Disable("MISSION_ACK result: " + std::to_string(p) + " (" + Mav_mission_result_to_string(p).c_str() + ")");
        }
    } else {
        // We are not waiting for messages.
        Disable("Unexpected MISSION_ACK");
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::On_param_str_value(
    mavlink::Message<mavlink::sph::MESSAGE_ID::PARAM_STR_VALUE, mavlink::sph::Extension>::Ptr message)
{
    VEHICLE_LOG_INF(vehicle, "PARAM_STR_VALUE, %s", message->payload.Dump().c_str());

    if (cmd_messages.size()) {
        std::string param_name;
        std::string param_value;
        // we are waiting for response.
        auto cmd = cmd_messages.front();
        auto& payload = (*std::static_pointer_cast<mavlink::sph::Pld_param_str_set>(cmd));
        switch (cmd->Get_id()) {
        case mavlink::sph::MESSAGE_ID::PARAM_STR_SET:
            param_name = payload->param_id.Get_string();
            if (message->payload->param_id.Get_string() == param_name) {
                param_value = payload->param_value.Get_string();
                if (message->payload->param_value.Get_string() == param_value) {
                    Send_next_command();
                } else {
                    Disable("PARAM_SET failed");
                }
            }
            break;
        }
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::On_param_value(
        mavlink::Message<mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr message)
{
    VEHICLE_LOG_INF(vehicle, "PARAM_VALUE, %s", message->payload.Dump().c_str());

    if (cmd_messages.size()) {
        // we are waiting for response.
        auto cmd = cmd_messages.front();
        if (cmd->Get_id() == mavlink::MESSAGE_ID::PARAM_SET) {
            auto param_name = (*std::static_pointer_cast<mavlink::Pld_param_set>(cmd))->param_id.Get_string();
            auto param_value = (*std::static_pointer_cast<mavlink::Pld_param_set>(cmd))->param_value.Get();

            if (message->payload->param_id.Get_string() == param_name) {
                double pv_sent = param_value;
                double pv_received = message->payload->param_value.Get();
                double relativeError = 0;

                if (pv_received == 0 && pv_sent == 0) {
                    // no error
                    relativeError = 0;
                } else if (pv_received == 0 && pv_sent != 0) {
                    // definetly the error
                    relativeError = 1;
                } else {
                    // calculate relative error
                    relativeError = fabs((pv_sent - pv_received) / pv_received);
                }

                if (relativeError <= 0.0000002) {
                    if (pv_sent != pv_received) {
                        VEHICLE_LOG_INF(
                            vehicle,
                            "PARAM_SET, Sent and received values are different"
                            " but relative error is small [sent = %2.10f, received = %2.10f, error = %2.20f]",
                            pv_sent,
                            pv_received,
                            relativeError);
                    }
                    Send_next_command();
                } else {
                    VEHICLE_LOG_ERR(
                        vehicle,
                        "PARAM_SET, Sent and received values are different"
                        " and relative error is large [sent = %2.10f, received = %2.10f, error = %2.20f]",
                        pv_sent,
                        pv_received,
                        relativeError);
                    Disable("PARAM_SET failed.");
                }
            }
        }
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::On_status_text(
        mavlink::Message<mavlink::MESSAGE_ID::STATUSTEXT>::Ptr)
{
    /* Assumed command execution started, so wait longer. */
    if (current_timeout < extended_retry_timeout) {
        current_timeout = extended_retry_timeout;
        VEHICLE_LOG_DBG(vehicle, "Command execution detected, "
                "now waiting longer for a command to finish...");
        /* Start a new longer timer. */
        Schedule_timer();
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Enable()
{
    Register_mavlink_handler<mavlink::MESSAGE_ID::COMMAND_ACK>(
        &Vehicle_command_act::On_command_ack,
        this);

    Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_ACK>(
        &Vehicle_command_act::On_mission_ack,
        this);

    Register_mavlink_handler<mavlink::MESSAGE_ID::PARAM_VALUE>(
        &Vehicle_command_act::On_param_value,
        this);

    Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_CURRENT>(
        &Vehicle_command_act::On_mission_current,
        this);

    Register_mavlink_handler<mavlink::sph::MESSAGE_ID::PARAM_STR_VALUE, mavlink::sph::Extension>(
        &Vehicle_command_act::On_param_str_value,
        this);

    Register_mavlink_handler<mavlink::apm::MESSAGE_ID::FENCE_POINT, mavlink::apm::Extension>(
        &Vehicle_command_act::On_point_value,
        this);

    remaining_attempts = try_count;
    current_timeout = retry_timeout;

    cmd_messages.clear();

    for (int c = 0; ucs_request && c < ucs_request->request.device_commands_size(); c++) {
        auto &vsm_cmd = ucs_request->request.device_commands(c);
        auto cmd = vehicle.Get_command(vsm_cmd.command_id());

        if (cmd != vehicle.c_direct_vehicle_control && cmd != vehicle.c_direct_payload_control) {
            // Do not spam log with direct control messages.
            VEHICLE_LOG_INF(vehicle, "COMMAND %s", vehicle.Dump_command(vsm_cmd).c_str());
        }

        auto params = cmd->Build_parameter_list(vsm_cmd);
        if (cmd == vehicle.c_adsb_set_mode) {
            Process_adsb_set_mode(params);
        } else if (cmd == vehicle.c_adsb_set_parameter) {
            Process_adsb_set_parameter(params);
        } else if (cmd == vehicle.c_adsb_set_ident) {
            Process_adsb_set_ident();
        } else if (cmd == vehicle.c_emergency_land) {
            Process_emergency_land();
        } else if (cmd == vehicle.c_set_servo) {
            Process_set_servo(params);
        } else if (cmd == vehicle.c_repeat_servo) {
            Process_repeat_servo(params);
        } else if (cmd == vehicle.c_set_fence) {
            Process_set_fence(params);
        } else if (cmd == vehicle.c_trigger_calibration) {
            Process_calibration();
        } else if (cmd == vehicle.c_trigger_reboot) {
            Process_reboot();
        } else if (cmd == vehicle.c_takeoff_command) {
            Process_takeoff(params);
        } else if (cmd == vehicle.c_set_heading) {
            Process_heading(params);
        } else if (cmd == vehicle.c_set_relative_heading) {
            Process_relative_heading(params);
        } else if (cmd == vehicle.c_resume) {
            Process_resume();
        } else if (cmd == vehicle.c_pause) {
            Process_pause(params);
        } else if (cmd == vehicle.c_auto) {
            Process_auto();
        } else if (cmd == vehicle.c_manual) {
            Process_manual();
        } else if (cmd == vehicle.c_joystick) {
            Process_joystick();
        } else if (cmd == vehicle.c_rth) {
            Process_rth();
        } else if (cmd == vehicle.c_land_command) {
            Process_land();
        } else if (cmd == vehicle.c_waypoint) {
            Process_waypoint(params);
        } else if (cmd == vehicle.c_arm) {
            Process_arm();
        } else if (cmd == vehicle.c_disarm) {
            Process_disarm();
        } else if (cmd == vehicle.c_guided) {
            Process_guided();
        } else if (cmd == vehicle.c_camera_trigger_command) {
            Process_camera_trigger();
        } else if (cmd == vehicle.c_direct_vehicle_control) {
            Process_direct_vehicle_control(params);
        } else if (cmd == vehicle.c_write_parameter) {
            Process_write_parameter(params);
        } else if (cmd == vehicle.c_set_poi) {
            Process_set_poi(params);
        } else if (cmd == ardu_vehicle.c_mission_clear) {
            Process_mission_clear();
        } else if (cmd == vehicle.c_direct_payload_control) {
            Process_direct_mount_control(params);
        }
    }
    command_count = cmd_messages.size();
    Try();
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_relative_heading(const Property_list& params)
{
    Stop_camera_series();
    if (!ardu_vehicle.Is_armed()) {
        Disable("Vehicle is not armed.");
    } else {
        float heading = 0;
        if (params.Get_value("relative_heading", heading)) {
            Generate_wp_from_current_position();
            auto cmd_mission_item = mavlink::Pld_mission_item_int::Create();
            Fill_target_ids(*cmd_mission_item);
            (*cmd_mission_item)->current = 2;
            (*cmd_mission_item)->frame = mavlink::MAV_FRAME_GLOBAL_RELATIVE_ALT;
            (*cmd_mission_item)->autocontinue = 1;
            (*cmd_mission_item)->command = mavlink::MAV_CMD::MAV_CMD_CONDITION_YAW;
            // yaw angle
            (*cmd_mission_item)->param1 = heading * 180.0 / M_PI;
            // Use default angular speed (AUTO_YAW_SLEW_RATE == 60 deg/s)
            (*cmd_mission_item)->param2 = 0;
            // relative angle
            (*cmd_mission_item)->param4 = 1;
            cmd_messages.emplace_back(cmd_mission_item);
        } else {
            Disable("Heading not present");
        }
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Generate_wp_from_current_position()
{
    if (ardu_vehicle.set_heading_needs_wp) {
        VEHICLE_LOG_INF(vehicle, "Generating WP at current position for heading to work.");
        float altitude;
        double latitude;
        double longitude;
        if (vehicle.t_latitude->Get_value(latitude) &&
            vehicle.t_altitude_raw->Get_value(altitude) &&
            vehicle.t_longitude->Get_value(longitude)) {
            auto cmd_mission_item = mavlink::Pld_mission_item_int::Create();
            Fill_target_ids(*cmd_mission_item);
            (*cmd_mission_item)->current = 2;
            (*cmd_mission_item)->frame = mavlink::MAV_FRAME_GLOBAL_RELATIVE_ALT;
            (*cmd_mission_item)->autocontinue = 1;
            (*cmd_mission_item)->command = mavlink::MAV_CMD::MAV_CMD_NAV_WAYPOINT;
            (*cmd_mission_item)->param1 = 0; // hold time
            (*cmd_mission_item)->param2 = 20; // acct radius
            (*cmd_mission_item)->param3 = 0; // passby radius
            (*cmd_mission_item)->param4 = NAN;
            (*cmd_mission_item)->x = static_cast<int32_t>((latitude * 180.0 / M_PI) * 1e7);
            (*cmd_mission_item)->y = static_cast<int32_t>((longitude * 180.0 / M_PI) * 1e7);
            (*cmd_mission_item)->z = altitude;
            cmd_messages.emplace_back(cmd_mission_item);
            ardu_vehicle.set_heading_needs_wp = false;
        } else {
            VEHICLE_LOG_INF(vehicle, "Failed to get current position. Heading ignored.");
        }
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_heading(const Property_list& params)
{
    Stop_camera_series();
    if (!ardu_vehicle.Is_armed()) {
        Disable("Vehicle is not armed.");
    } else {
        float heading = 0;
        if (params.Get_value("heading", heading)) {
            Generate_wp_from_current_position();
            auto cmd_mission_item = mavlink::Pld_mission_item_int::Create();
            Fill_target_ids(*cmd_mission_item);
            (*cmd_mission_item)->current = 2;
            (*cmd_mission_item)->frame = mavlink::MAV_FRAME_GLOBAL_RELATIVE_ALT;
            (*cmd_mission_item)->autocontinue = 1;
            (*cmd_mission_item)->command = mavlink::MAV_CMD::MAV_CMD_CONDITION_YAW;
            // yaw angle
            (*cmd_mission_item)->param1 = heading * 180.0 / M_PI;
            // Use default angular speed (AUTO_YAW_SLEW_RATE == 60 deg/s)
            (*cmd_mission_item)->param2 = 0;
            // absolute angle
            (*cmd_mission_item)->param4 = 0;
            cmd_messages.emplace_back(cmd_mission_item);
        } else {
            Disable("Heading not present");
        }
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_takeoff(const Property_list& params)
{
    if (ardu_vehicle.is_airborne) {
        Disable("Vehicle already airborne.");
    } else if (!ardu_vehicle.Is_armed()) {
        Disable("Vehicle is not armed.");
    } else {
        float altitude;
        if (!params.Get_value("relative_altitude", altitude)) {
            VEHICLE_LOG_INF(
                vehicle,
                "Takeoff altitude not set, using default %f m.", DEFAULT_TAKEOFF_ALTITUDE);
            altitude = DEFAULT_TAKEOFF_ALTITUDE;
        }
        if (!vehicle.Is_control_mode(proto::CONTROL_MODE_CLICK_GO)) {
            ardu_vehicle.Stop_rc_override();
            // Enter GUIDED mode.
            auto cmd_set_mode = mavlink::Pld_set_mode::Create();
            (*cmd_set_mode)->target_system = vehicle.real_system_id;
            (*cmd_set_mode)->base_mode = mavlink::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            (*cmd_set_mode)->custom_mode = Get_custom_guided_mode();
            cmd_messages.emplace_back(cmd_set_mode);
        }
        auto cmd_long = mavlink::Pld_command_long::Create();
        Fill_target_ids(*cmd_long);
        (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_NAV_TAKEOFF;
        (*cmd_long)->param7 = altitude;
        cmd_messages.emplace_back(cmd_long);
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_mission_clear()
{
    auto cmd = mavlink::Pld_mission_clear_all::Create();
    Fill_target_ids(*cmd);
    (*cmd)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_MISSIONPLANNER;
    (*cmd)->mission_type = mavlink::MAV_MISSION_TYPE_MISSION;
    cmd_messages.emplace_back(cmd);
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_reboot()
{
    // sanity check
    if (ardu_vehicle.Is_armed()) {
        Disable("Reboot is denied, UAV armed.");
    } else {
        auto cmd_long = mavlink::Pld_command_long::Create();
        Fill_target_ids(*cmd_long);
        (*cmd_long)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_SYSTEM_CONTROL;
        (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        (*cmd_long)->param1 = 1; // reboot;
        cmd_messages.emplace_back(cmd_long);
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_calibration()
{
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    // gyro (param1)
    (*cmd_long)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_SYSTEM_CONTROL;
    (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_PREFLIGHT_CALIBRATION;
    (*cmd_long)->param1 = 1;
    cmd_messages.emplace_back(cmd_long);

    // param2 not supported by ardupilot

    // baro (param3)
    cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    (*cmd_long)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_SYSTEM_CONTROL;
    (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_PREFLIGHT_CALIBRATION;
    (*cmd_long)->param3 = 1;
    cmd_messages.emplace_back(cmd_long);

    // skip radio calibration (param4) as it requires user interaction.

    // skip accelerometer (param5) calibration as it requires vehicle to be level.

    // skip compass (param6) calibration as it requires vehicle to be rotated manually.
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_set_fence(const Property_list& params)
{
    proto::List_value lats, lngs;

    /* bitmask, fence type
       0: no fence
       1 bit: altitude fence
       2 bit: cicrle fence
       3 bit: polygon fence (after ardupilot 3.4). */
    int8_t fence_mask = 0;


    bool polygon_data_is_correct = true;

    if (params.Get_value("latitudes", lats) && params.Get_value("longitudes", lngs)) {
        polygon_data_is_correct = false;
        int poly_size = lats.values_size();
        VEHICLE_LOG_INF(vehicle, "Adding polygon fence, Polygon size = %d", poly_size);

        // check if polygon data is correct
        if (!ardu_vehicle.Is_home_position_valid()) {
            // Need valid home location
            vehicle.Command_failed(ucs_request,
                                   "Polygon fence: home location is not valid.",
                                   proto::STATUS_INVALID_PARAM);
        } else if (poly_size != lngs.values_size()) {
            // ERROR: non equal points number
            vehicle.Command_failed(ucs_request,
                                   "Polygon fence: non equal number of points in lists",
                                   proto::STATUS_INVALID_PARAM);
        } else if (poly_size < 4) {
            // ERROR: too few points
            vehicle.Command_failed(ucs_request,
                                   "Polygon fence: need at last 4 points for polygon fence",
                                   proto::STATUS_INVALID_PARAM);
        } else if (poly_size > 84) {
            // ERROR: too much points
            vehicle.Command_failed(ucs_request,
                                   "Polygon fence: need no more than 84 points for polygon fence",
                                   proto::STATUS_INVALID_PARAM);
        } else if (lats.values(0).double_value() !=  lats.values(poly_size-1).double_value() ||
                   lngs.values(0).double_value() !=  lngs.values(poly_size-1).double_value()) {
            // ERROR: first point mast be equal to last point in polygon description
            vehicle.Command_failed(ucs_request,
               "Polygon fence: first point mast be equal to last point in polygon description",
               proto::STATUS_INVALID_PARAM);
        } else if (Is_Outside_Polygon(ardu_vehicle.home_location.latitude,
                                      ardu_vehicle.home_location.longitude,
                                      lats, lngs)) {
            // ERROR: home location mas be inside polygon
            VEHICLE_LOG_INF(vehicle, "HOME LOCATION : %2.20f, %2.20f",
                            ardu_vehicle.home_location.latitude,
                            ardu_vehicle.home_location.longitude);

            vehicle.Command_failed(ucs_request,
                                   "Polygon fence: Home Location must be inside polygon",
                                   proto::STATUS_INVALID_PARAM);
        } else {
            polygon_data_is_correct = true;
            // check if all values are double
            for (int i = 0; i < poly_size; i++) {
                if (!(lats.values(i).has_double_value() && lngs.values(i).has_double_value())) {
                    vehicle.Command_failed(ucs_request,
                                           "Polygon fence: incorrect latitude and longitude values",
                                           proto::STATUS_INVALID_PARAM);
                    polygon_data_is_correct = false;
                }
            }

            if (polygon_data_is_correct) {
                fence_mask |= 4;
                // set fence size
                auto param = mavlink::Pld_param_set::Create();
                Fill_target_ids(*param);
                (*param)->param_id = "FENCE_TOTAL";
                (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
                // +1 cause fist point we send is returning point.
                (*param)->param_value = poly_size + 1;
                cmd_messages.emplace_back(param);

                // zero-point: Virtual return point.
                // Virtual - because Copter returns to land point
                // and not to this "return point". But "return point" needs to be inside
                // polygon anyway due to strange ardupilot logic. So we set return point
                // to Home Location and check that it is inside polygon (earlier) with
                // function Is_Outside_Polygon.
                VEHICLE_LOG_INF(vehicle, "Virtual return point(0) : %2.20f, %2.20f",
                                ardu_vehicle.home_location.latitude * 180.0f / M_PI,
                                ardu_vehicle.home_location.longitude * 180.0f / M_PI);
                auto cmd_fence_point = mavlink::apm::Pld_fence_point::Create();
                Fill_target_ids(*cmd_fence_point);
                (*cmd_fence_point)->lat = ardu_vehicle.home_location.latitude * 180.0f / M_PI;
                (*cmd_fence_point)->lng = ardu_vehicle.home_location.longitude * 180.0f / M_PI;
                (*cmd_fence_point)->count = poly_size;
                (*cmd_fence_point)->idx = 0;
                cmd_messages.emplace_back(cmd_fence_point);

                VEHICLE_LOG_INF(vehicle, "POLYGON FENCE POINTS:");
                for (int i = 0; i < poly_size; i++) {
                    double lat = lats.values(i).double_value()* 180.0f / M_PI;
                    double lng = lngs.values(i).double_value()* 180.0f / M_PI;
                    VEHICLE_LOG_INF(vehicle, "%d : %2.20f, %2.20f", i, lat, lng);
                    cmd_fence_point = mavlink::apm::Pld_fence_point::Create();
                    Fill_target_ids(*cmd_fence_point);
                    (*cmd_fence_point)->lat = lat;
                    (*cmd_fence_point)->lng = lng;
                    (*cmd_fence_point)->count = poly_size;
                    (*cmd_fence_point)->idx = i + 1;
                    cmd_messages.emplace_back(cmd_fence_point);
                }
                VEHICLE_LOG_INF(vehicle, "POLYGON FENCE POINTS END. All points was put to queue.");
            }
        }
    }

    // if polygon data is correct or empty
    if (polygon_data_is_correct) {
        float radius;
        float altitude;
        float altitude_origin;
        if (params.Get_value("radius", radius)) {
            fence_mask |= 2;
            auto param = mavlink::Pld_param_set::Create();
            Fill_target_ids(*param);
            (*param)->param_id = "FENCE_RADIUS";
            (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
            (*param)->param_value = radius;
            cmd_messages.emplace_back(param);
        }
        if (    params.Get_value("altitude_amsl", altitude)
            &&  params.Get_value("altitude_origin", altitude_origin))
        {
            fence_mask |= 1;
            auto param = mavlink::Pld_param_set::Create();
            Fill_target_ids(*param);
            (*param)->param_id = "FENCE_ALT_MAX";
            (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
            (*param)->param_value = altitude - altitude_origin;
            cmd_messages.emplace_back(param);
        } else if (params.Get_value("altitude_raw", altitude)) {
            fence_mask |= 1;
            auto param = mavlink::Pld_param_set::Create();
            Fill_target_ids(*param);
            (*param)->param_id = "FENCE_ALT_MAX";
            (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
            (*param)->param_value = altitude;
            cmd_messages.emplace_back(param);
        }

        if (fence_mask) {
           /* TODO: this shouldn't be hardcoded and must be set from client
            auto param = mavlink::Pld_param_set::Create();
            Fill_target_ids(*param);
            (*param)->param_id = "FENCE_MARGIN";
            (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
            (*param)->param_value = 2;
            cmd_messages.emplace_back(param);
            param = mavlink::Pld_param_set::Create();
            Fill_target_ids(*param);
            (*param)->param_id = "FENCE_ACTION";
            (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
            (*param)->param_value = 1; // RTL
            cmd_messages.emplace_back(param);*/
            auto param = mavlink::Pld_param_set::Create();
            Fill_target_ids(*param);
            (*param)->param_id = "FENCE_TYPE";
            (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
            (*param)->param_value = fence_mask;
            cmd_messages.emplace_back(param);
            param = mavlink::Pld_param_set::Create();
            Fill_target_ids(*param);
            (*param)->param_id = "FENCE_ENABLE";
            (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
            (*param)->param_value = 1;
            cmd_messages.emplace_back(param);
        } else {
            auto param = mavlink::Pld_param_set::Create();
            Fill_target_ids(*param);
            (*param)->param_id = "FENCE_ENABLE";
            (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
            (*param)->param_value = 0;
            cmd_messages.emplace_back(param);
            param = mavlink::Pld_param_set::Create();
            Fill_target_ids(*param);
            (*param)->param_id = "FENCE_TYPE";
            (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
            (*param)->param_value = 0;
            cmd_messages.emplace_back(param);
        }
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_repeat_servo(const Property_list& params)
{
    int servo_id;
    int pwm;
    int count;
    float delay;
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    params.Get_value("servo_id", servo_id);
    params.Get_value("pwm", pwm);
    params.Get_value("delay", delay);
    params.Get_value("count", count);
    (*cmd_long)->command = mavlink::MAV_CMD_DO_REPEAT_SERVO;
    (*cmd_long)->param1 = servo_id;
    (*cmd_long)->param2 = pwm;
    (*cmd_long)->param3 = count;
    if (delay < 0.2) {
        (*cmd_long)->param4 = 0;
    } else {
        // Pixhawk does this slower than expected. Add coefficient.
        (*cmd_long)->param4 = delay * 2 - 0.4;
    }
    cmd_messages.emplace_back(cmd_long);
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_set_servo(const Property_list& params)
{
    int servo_id;
    int pwm;
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    params.Get_value("servo_id", servo_id);
    params.Get_value("pwm", pwm);
    (*cmd_long)->command = mavlink::MAV_CMD_DO_SET_SERVO;
    (*cmd_long)->param1 = servo_id;
    (*cmd_long)->param2 = pwm;
    cmd_messages.emplace_back(cmd_long);
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_adsb_set_mode(const Property_list& params)
{
    int mode;
    if (params.Get_value("mode", mode)) {
        auto cmd_long = mavlink::Pld_command_long::Create();
        Fill_target_ids(*cmd_long);
        (*cmd_long)->command = mavlink::sph::MAV_CMD_ADSB_SET_MODE;
        switch (mode) {
        case proto::ADSB_MODE_OFF:
            (*cmd_long)->param1 = mavlink::sph::ADSB_TRANSPONDER_MODE_OFF;
            break;
        case proto::ADSB_MODE_ON:
            (*cmd_long)->param1 = mavlink::sph::ADSB_TRANSPONDER_MODE_ON;
            break;
        case proto::ADSB_MODE_STBY:
            (*cmd_long)->param1 = mavlink::sph::ADSB_TRANSPONDER_MODE_STBY;
            break;
        case proto::ADSB_MODE_ALT:
            (*cmd_long)->param1 = mavlink::sph::ADSB_TRANSPONDER_MODE_ALT;
            break;
        default:
            VSM_EXCEPTION(Invalid_param_exception, "Unsupported mode");
        }
        cmd_messages.emplace_back(cmd_long);
    } else {
        VSM_EXCEPTION(Invalid_param_exception, "Unsupported mode");
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_adsb_set_parameter(const Property_list& params)
{
    int int_value;
    std::string str_value;
    if (params.Get_value("adsb_icao", int_value)) {
        auto param = mavlink::Pld_param_set::Create();
        Fill_target_ids(*param);
        (*param)->param_id = "ADSB_ICAO_ID";
        (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT32;
        (*param)->param_value = int_value;
        cmd_messages.emplace_back(param);
    }
    if (params.Get_value("adsb_squawk", int_value)) {
        auto param = mavlink::Pld_param_set::Create();
        Fill_target_ids(*param);
        (*param)->param_id = "ADSB_SQUAWK";
        (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT32;
        (*param)->param_value = int_value;
        cmd_messages.emplace_back(param);
    }
    if (params.Get_value("adsb_registration", str_value)) {
        auto param = mavlink::sph::Pld_param_str_set::Create();
        Fill_target_ids(*param);
        (*param)->param_id = "ADSB_TAIL";
        (*param)->param_value = str_value;
        cmd_messages.emplace_back(param);
    }
    if (params.Get_value("adsb_flight_id", str_value)) {
        auto param = mavlink::sph::Pld_param_str_set::Create();
        Fill_target_ids(*param);
        (*param)->param_id = "ADSB_FLIGHT";
        (*param)->param_value = str_value;
        cmd_messages.emplace_back(param);
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_adsb_set_ident()
{
    auto param = mavlink::Pld_param_set::Create();
    Fill_target_ids(*param);
    (*param)->param_id = "ADSB_IDENT";
    (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT32;
    (*param)->param_value = 1;
    cmd_messages.emplace_back(param);
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_emergency_land()
{
    Stop_camera_series();
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    (*cmd_long)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_SYSTEM_CONTROL;
    (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM;
    (*cmd_long)->param1 = 0; /* Do disarm. */
    (*cmd_long)->param2 = DISARM_MAGIC_VALUE; // do hard disarm (even when copter is airborne)
    cmd_messages.emplace_back(cmd_long);
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_auto()
{
    ardu_vehicle.Stop_rc_override();
    auto param = mavlink::Pld_param_set::Create();
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    Fill_target_ids(*param);
    (*param)->param_id = "MIS_RESTART";
    (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
    (*param)->param_value = 1;
    cmd_messages.emplace_back(param);
    (*cmd_long)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_SYSTEM_CONTROL;
    (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_MISSION_START;
    cmd_messages.emplace_back(cmd_long);
    // Reset MIS_RESTART back to 0 so that next time user enters AUTO from
    // RC tx, vehicle continues mission instead of returning back to WP1.
    param = mavlink::Pld_param_set::Create();
    Fill_target_ids(*param);
    (*param)->param_id = "MIS_RESTART";
    (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
    (*param)->param_value = 0;
    cmd_messages.emplace_back(param);
    ardu_vehicle.set_heading_needs_wp = false;
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_manual()
{
    Stop_camera_series();
    /* Current Ardupilot firmware does not send responses to mode changes,
     * so just check the current mode as an indication of completed
     * request. */
    if (vehicle.Is_control_mode(proto::CONTROL_MODE_MANUAL)) {
        Disable_success();
    } else {
        auto cmd_set_mode = mavlink::Pld_set_mode::Create();
        Fill_target_system_id(*cmd_set_mode);
        (*cmd_set_mode)->base_mode = mavlink::MAV_MODE_FLAG::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        (*cmd_set_mode)->custom_mode = Get_custom_manual_mode();
        cmd_messages.emplace_back(cmd_set_mode);
    }
    ardu_vehicle.Stop_rc_override();
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_arm()
{
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    (*cmd_long)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_SYSTEM_CONTROL;
    (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM;
    (*cmd_long)->param1 = 1; /* Do arm. */
    cmd_messages.emplace_back(cmd_long);
    Register_status_text();
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_disarm()
{
    Stop_camera_series();
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    (*cmd_long)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_SYSTEM_CONTROL;
    (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM;
    (*cmd_long)->param1 = 0; /* Do disarm. */
    cmd_messages.emplace_back(cmd_long);
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_pause(const ugcs::vsm::Property_list& params)
{
    VEHICLE_LOG_INF(vehicle, "Pausing mission.");
    Stop_camera_series();
    // If vehicle is in guided mode already then
    // briefly move into manual to reset current guided WP and then
    // switch back to guided so the vehicle hovers at current position.
    if (vehicle.Is_control_mode(proto::CONTROL_MODE_CLICK_GO)) {
        auto cmd_set_mode = mavlink::Pld_set_mode::Create();
        Fill_target_system_id(*cmd_set_mode);
        (*cmd_set_mode)->base_mode = mavlink::MAV_MODE_FLAG::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        (*cmd_set_mode)->custom_mode = Get_custom_manual_mode();
        cmd_messages.emplace_back(cmd_set_mode);
    }
    // Enter GUIDED mode.
    auto cmd_set_mode = mavlink::Pld_set_mode::Create();
    Fill_target_system_id(*cmd_set_mode);
    cmd_set_mode = mavlink::Pld_set_mode::Create();
    (*cmd_set_mode)->target_system = vehicle.real_system_id;
    (*cmd_set_mode)->base_mode = mavlink::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    (*cmd_set_mode)->custom_mode = Get_custom_guided_mode();
    cmd_messages.emplace_back(cmd_set_mode);

    float add_altitude = 0;
    if (params.Get_value("additional_altitude", add_altitude)) {
        if (add_altitude > 0) {
            VEHICLE_LOG_INF(vehicle, "Climbing additional %f meters", add_altitude);
            mavlink::Pld_set_position_target_local_ned::Ptr cmd_set_position =
                    mavlink::Pld_set_position_target_local_ned::Create();
            Fill_target_ids(*cmd_set_position);
            // Control UAV with position offsets
            // this is NED frame, so to climb we need to
            // set negftive z value
            (*cmd_set_position)->coordinate_frame = mavlink::MAV_FRAME::MAV_FRAME_BODY_NED;
            // position mask
            (*cmd_set_position)->type_mask = ~((1 << 0) | (1 << 1) | (1 << 2));
            // move only vertical
            (*cmd_set_position)->x = 0;
            (*cmd_set_position)->y = 0;
            (*cmd_set_position)->z = -add_altitude;

            cmd_messages.emplace_back(cmd_set_position);
        } else if (add_altitude < 0) {
            VEHICLE_LOG_INF(vehicle, "Additional altitude parameter was negative (%f). Ignored.", add_altitude);
        }
   }
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_resume()
{
    // Resume mission only if vehicle is in normal flight conditions
    if (ardu_vehicle.Is_armed()) {
        if (vehicle.Is_flight_mode(proto::FLIGHT_MODE_HOLD)) {
            int command_idx;
            if (vehicle.t_current_command->Get_value(command_idx)) {
                auto set_cur = mavlink::Pld_mission_set_current::Create();
                Fill_target_ids(*set_cur);
                (*set_cur)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_SYSTEM_CONTROL;
                (*set_cur)->seq = command_idx + 1;
                cmd_messages.emplace_back(set_cur);
            }
        } else {
            auto param = mavlink::Pld_param_set::Create();
            Fill_target_ids(*param);
            (*param)->param_id = "MIS_RESTART";
            (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
            (*param)->param_value = 0;
            cmd_messages.emplace_back(param);
            auto cmd_long = mavlink::Pld_command_long::Create();
            Fill_target_ids(*cmd_long);
            (*cmd_long)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_SYSTEM_CONTROL;
            (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_MISSION_START;
            cmd_messages.emplace_back(cmd_long);
        }
    } else {
        Disable("Vehicle not armed.");
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_waypoint(const Property_list& params)
{
    // Does 4 things:
    // Enter guided mode if needed
    // Change current speed
    // Change yaw if needed
    // Got to WP
    float altitude_amsl;
    float altitude_origin;
    float speed;
    double latitude;
    double longitude;
    float acceptance_radius;
    float heading = NAN;

    params.at("ground_speed")->Get_value(speed);
    params.at("altitude_amsl")->Get_value(altitude_amsl);
    params.at("altitude_origin")->Get_value(altitude_origin);
    params.at("latitude")->Get_value(latitude);
    params.at("longitude")->Get_value(longitude);
    params.at("acceptance_radius")->Get_value(acceptance_radius);

    if (!vehicle.Is_control_mode(proto::CONTROL_MODE_CLICK_GO)) {
        Stop_camera_series();
        ardu_vehicle.Stop_rc_override();
        // Enter GUIDED mode.
        auto cmd_set_mode = mavlink::Pld_set_mode::Create();
        (*cmd_set_mode)->target_system = vehicle.real_system_id;
        (*cmd_set_mode)->base_mode = mavlink::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        (*cmd_set_mode)->custom_mode = Get_custom_guided_mode();
        cmd_messages.emplace_back(cmd_set_mode);
    }

    auto param = mavlink::Pld_param_set::Create();
    Fill_target_ids(*param);
    if (vehicle.Get_vehicle_type() == proto::VEHICLE_TYPE_FIXED_WING ||
        vehicle.Get_vehicle_type() == proto::VEHICLE_TYPE_VTOL) {
        (*param)->param_id = "TRIM_ARSPD_CM";
        (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT32;
        (*param)->param_value = static_cast<int32_t>(speed * 100);
        cmd_messages.emplace_back(param);
    } else if (ardu_vehicle.Is_copter()) {
        auto cmd_long = mavlink::Pld_command_long::Create();
        Fill_target_ids(*cmd_long);
        (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_DO_CHANGE_SPEED;
        (*cmd_long)->param1 = 0; // airspeed
        (*cmd_long)->param2 = speed;
        cmd_messages.emplace_back(cmd_long);

        (*param)->param_id = "WPNAV_SPEED";
        (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
        (*param)->param_value = speed * 100;
        cmd_messages.emplace_back(param);

        if (params.Get_value("heading", heading)) {
            Generate_wp_from_current_position();
            // we have heading specified. turn first.
            auto cmd_mission_item = mavlink::Pld_mission_item_int::Create();
            Fill_target_ids(*cmd_mission_item);
            (*cmd_mission_item)->current = 2;
            (*cmd_mission_item)->frame = mavlink::MAV_FRAME_GLOBAL_RELATIVE_ALT;
            (*cmd_mission_item)->autocontinue = 1;
            (*cmd_mission_item)->command = mavlink::MAV_CMD::MAV_CMD_CONDITION_YAW;
            // yaw angle
            (*cmd_mission_item)->param1 = std::isnan(heading)?0:heading * 180.0 / M_PI;
            // Use default angular speed (AUTO_YAW_SLEW_RATE == 60 deg/s)
            (*cmd_mission_item)->param2 = 0;
            // absolute angle
            (*cmd_mission_item)->param4 = 0;
            cmd_messages.emplace_back(cmd_mission_item);
        }
    }

    auto cmd_mission_item = mavlink::Pld_mission_item_int::Create();
    Fill_target_ids(*cmd_mission_item);
    (*cmd_mission_item)->current = 2;
    (*cmd_mission_item)->frame = mavlink::MAV_FRAME_GLOBAL_RELATIVE_ALT;
    (*cmd_mission_item)->autocontinue = 1;
    (*cmd_mission_item)->command = mavlink::MAV_CMD::MAV_CMD_NAV_WAYPOINT;
    (*cmd_mission_item)->param1 = 0; // hold time
    (*cmd_mission_item)->param2 = acceptance_radius; // acct radius
    (*cmd_mission_item)->param3 = 0; // passby radius
    (*cmd_mission_item)->param4 = std::isnan(heading)?0:heading * 180.0 / M_PI;
    (*cmd_mission_item)->x = static_cast<int32_t>((latitude * 180.0 / M_PI) * 1e7);
    (*cmd_mission_item)->y = static_cast<int32_t>((longitude * 180.0 / M_PI) * 1e7);
    (*cmd_mission_item)->z = altitude_amsl - altitude_origin;
    cmd_messages.emplace_back(cmd_mission_item);
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_guided()
{
    Stop_camera_series();
    // Enter GUIDED mode.
    ardu_vehicle.Stop_rc_override();
    auto cmd_set_mode = mavlink::Pld_set_mode::Create();
    (*cmd_set_mode)->target_system = vehicle.real_system_id;
    (*cmd_set_mode)->base_mode = mavlink::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    (*cmd_set_mode)->custom_mode = Get_custom_guided_mode();
    cmd_messages.emplace_back(cmd_set_mode);
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_land()
{
    Stop_camera_series();
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    (*cmd_long)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_SYSTEM_CONTROL;
    (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_NAV_LAND;
    cmd_messages.emplace_back(cmd_long);
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_joystick()
{
    if (vehicle.Is_control_mode(proto::CONTROL_MODE_JOYSTICK)) {
        Disable_success();
    } else {
        // Set vehicle in manual mode and start pushing RC_CHANNELS_OVERRIDE messages.
        auto cmd_set_mode = mavlink::Pld_set_mode::Create();
        Fill_target_system_id(*cmd_set_mode);
        (*cmd_set_mode)->base_mode = mavlink::MAV_MODE_FLAG::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        (*cmd_set_mode)->custom_mode = Get_custom_manual_mode();
        cmd_messages.emplace_back(cmd_set_mode);
    }
    ardu_vehicle.Start_rc_override();
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_direct_vehicle_control(const Property_list& params)
{
    if (!vehicle.Is_control_mode(proto::CONTROL_MODE_JOYSTICK)) {
        return;
    }

    Stop_camera_series();

    float pitch, yaw, roll, throttle;
    params.at("pitch")->Get_value(pitch);
    params.at("yaw")->Get_value(yaw);
    params.at("roll")->Get_value(roll);
    params.at("throttle")->Get_value(throttle);

    //        LOG("Direct Vehicle (rpyt) %1.3f %1.3f %1.3f %1.3f",
    //            roll,
    //            pitch,
    //            yaw,
    //            throttle);

    ardu_vehicle.direct_vehicle_control_last_received = std::chrono::steady_clock::now();

    // Normalize axes depending on vehicle type.
    if (ardu_vehicle.Is_copter()) {
        pitch = -pitch;         // pitch is reversed for copter.
        yaw = yaw * 0.4;    // yaw on copter is too sensitive.
    } else {
        yaw = -yaw;     // yaw is reversed for plane.
    }

    ardu_vehicle.Set_rc_override(
        (roll * 500.0) + 1500,
        (pitch * 500.0) + 1500,
        (throttle * 500.0) + 1500,
        (yaw * 500.0) + 1500);
    ardu_vehicle.Send_rc_override();
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_write_parameter(const Property_list& params)
{
    float value;
    std::string name;
    if (params.Get_value("name", name) && params.Get_value("value", value)) {
        auto param = mavlink::Pld_param_set::Create();
        Fill_target_ids(*param);
        (*param)->param_id = name;
        (*param)->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
        (*param)->param_value = value;
        cmd_messages.emplace_back(param);
    } else {
        // Ardupilot supports only float values as parameters.
        // Ints and doubles are cast implicitly.
        VSM_EXCEPTION(Invalid_param_exception, "Parameter not a numeric value");
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_set_poi(const Property_list& params)
{
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    bool active = false;
    params.at("active")->Get_value(active);
    if (active) {
        double latitude;
        double longitude;
        float altitude_amsl;
        params.at("altitude_amsl")->Get_value(altitude_amsl);
        params.at("latitude")->Get_value(latitude);
        params.at("longitude")->Get_value(longitude);
        (*cmd_long)->param5 = latitude * 180.0 / M_PI;
        (*cmd_long)->param6 = longitude * 180.0 / M_PI;
        (*cmd_long)->param7 = altitude_amsl;
    } else {
        (*cmd_long)->param5 = 0;
        (*cmd_long)->param6 = 0;
        (*cmd_long)->param7 = 0;
    }
    (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_DO_SET_ROI;
    (*cmd_long)->param1 = mavlink::MAV_ROI_LOCATION;
    cmd_messages.emplace_back(cmd_long);
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_rth()
{
    Stop_camera_series();
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    (*cmd_long)->target_component = mavlink::MAV_COMPONENT::MAV_COMP_ID_AUTOPILOT1;
    (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_NAV_RETURN_TO_LAUNCH;
    cmd_messages.emplace_back(cmd_long);
}

void
Ardupilot_vehicle::Vehicle_command_act::Process_camera_trigger()
{
    auto cmd_long = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd_long);
    if (ardu_vehicle.camera_servo_idx == -1) {
        (*cmd_long)->command = mavlink::MAV_CMD_DO_DIGICAM_CONTROL;
        (*cmd_long)->param5 = 1;
    } else {
        (*cmd_long)->command = mavlink::MAV_CMD::MAV_CMD_DO_REPEAT_SERVO;
        (*cmd_long)->param1 = ardu_vehicle.camera_servo_idx;
        (*cmd_long)->param2 = ardu_vehicle.camera_servo_pwm;
        (*cmd_long)->param3 = 1;  // do it once
        (*cmd_long)->param4 = ardu_vehicle.camera_servo_time;
    }
    cmd_messages.emplace_back(cmd_long);
}

void
Ardupilot_vehicle::Vehicle_command_act::On_disable()
{
    cmd_messages.clear();

    Unregister_status_text();

    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
}

void
Ardupilot_vehicle::Vehicle_command_act::Schedule_timer()
{
    if (timer) {
        timer->Cancel();
    }
    timer = Timer_processor::Get_instance()->Create_timer(
                current_timeout,
                Make_callback(&Vehicle_command_act::Try, this),
                vehicle.Get_completion_ctx());
}

void
Ardupilot_vehicle::Vehicle_command_act::Schedule_verify_timer()
{
    if (timer) {
        timer->Cancel();
    }
    timer = Timer_processor::Get_instance()->Create_timer(
            current_timeout,
            Make_callback(&Vehicle_command_act::Try_verify_polyfence, this),
            vehicle.Get_completion_ctx());
}

void
Ardupilot_vehicle::Vehicle_command_act::Register_status_text()
{
    vehicle.statistics.statustext_handler =
            Mavlink_vehicle::Statistics::Make_statustext_handler(
                    &Ardupilot_vehicle::Vehicle_command_act::On_status_text,
                    this);
}

void
Ardupilot_vehicle::Vehicle_command_act::Unregister_status_text()
{
    vehicle.statistics.statustext_handler =
            Mavlink_vehicle::Statistics::Statustext_handler();
}

uint32_t
Ardupilot_vehicle::Vehicle_command_act::Get_custom_manual_mode()
{
    switch (ardu_vehicle.Get_vehicle_type()) {
    case proto::VEHICLE_TYPE_MULTICOPTER:
    case proto::VEHICLE_TYPE_HELICOPTER:
        return static_cast<uint32_t>(Copter_flight_mode::LOITER);
    case proto::VEHICLE_TYPE_FIXED_WING:
    case proto::VEHICLE_TYPE_VTOL:
        return static_cast<uint32_t>(Plane_flight_mode::STABILIZE);
    case proto::VEHICLE_TYPE_GROUND:
        return static_cast<uint32_t>(Rover_flight_mode::MANUAL);
    }
    VSM_EXCEPTION(Internal_error_exception, "Undefined vehicle type");
}

uint32_t
Ardupilot_vehicle::Vehicle_command_act::Get_custom_guided_mode()
{
    switch (ardu_vehicle.Get_vehicle_type()) {
    case proto::VEHICLE_TYPE_MULTICOPTER:
    case proto::VEHICLE_TYPE_HELICOPTER:
        return static_cast<uint32_t>(Copter_flight_mode::GUIDED);
    case proto::VEHICLE_TYPE_FIXED_WING:
    case proto::VEHICLE_TYPE_VTOL:
        return static_cast<uint32_t>(Plane_flight_mode::GUIDED);
    case proto::VEHICLE_TYPE_GROUND:
        return static_cast<uint32_t>(Rover_flight_mode::GUIDED);
    }
    VSM_EXCEPTION(Internal_error_exception, "Undefined vehicle type");
}

bool
Ardupilot_vehicle::Vehicle_command_act::Is_Outside_Polygon(
    double check_point_latitude,
    double check_point_longitude,
    proto::List_value lats,
    proto::List_value lngs)
{
    const uint16_t n = lats.values_size();
    unsigned i, j;
    bool outside = true;

    for (i = 0, j = n-1; i < n; j = i++) {
        if ((lngs.values(i).double_value() > check_point_longitude) ==
            (lngs.values(j).double_value() > check_point_longitude)) {
            continue;
        }

        const double dx1 = check_point_latitude - lats.values(i).double_value();
        const double dx2 = lats.values(j).double_value() - lats.values(i).double_value();
        const double dy1 = check_point_longitude - lngs.values(i).double_value();
        const double dy2 = lngs.values(j).double_value() - lngs.values(i).double_value();
        const int8_t dx1s = (dx1 < 0) ? -1 : 1;
        const int8_t dx2s = (dx2 < 0) ? -1 : 1;
        const int8_t dy1s = (dy1 < 0) ? -1 : 1;
        const int8_t dy2s = (dy2 < 0) ? -1 : 1;
        const int8_t m1 = dx1s * dy2s;
        const int8_t m2 = dx2s * dy1s;

        if (dy2 < 0) {
            if (m1 > m2) {
                outside = !outside;
            } else if (m1 < m2) {
                continue;
            } else if ( dx1 * dy2 > dx2 * dy1 ) {
                outside = !outside;
            }
        } else {
            if (m1 < m2) {
                outside = !outside;
            } else if (m1 > m2) {
                continue;
            } else if ( dx1 * dy2 < dx2 * dy1 ) {
                outside = !outside;
            }
        }
    }
    return outside;
}

Geodetic_tuple
Ardupilot_vehicle::Task_upload::Create_geodetic_tuple(const Property_list& params)
{
    double lat, lon, alt;
    params.Get_value("latitude", lat);
    params.Get_value("longitude", lon);
    params.Get_value("altitude_amsl", alt);
    return Geodetic_tuple(lat, lon, alt);
}

void
Ardupilot_vehicle::Task_upload::Preprocess_mission(const proto::Device_command& vsm_cmd)
{
    auto main_cmd = vehicle.Get_command(vsm_cmd.command_id());

    std::string route_name;
    Property_list route_params;

    if (main_cmd == vehicle.c_mission_upload) {
        route_params = vehicle.c_mission_upload->Build_parameter_list(vsm_cmd);
    } else if (main_cmd == vehicle.c_get_native_route) {
        route_params = vehicle.c_get_native_route->Build_parameter_list(vsm_cmd);
        if (!route_params.Get_value("use_crlf", use_crlf_in_native_route)) {
            VSM_EXCEPTION(Invalid_param_exception, "use_crlf not present");
        }
    }

    if (route_params.Get_value("name", route_name)) {
        VEHICLE_LOG_INF(vehicle, "Route name : %s", route_name.c_str());
    }

    if (!route_params.Get_value("altitude_origin", altitude_origin)) {
        VSM_EXCEPTION(Invalid_param_exception, "Altitude origin not present in route");
    }

    bool takeoff_present = false;
    bool set_home_present = false;

    // Prepare_takeoff_mission uses this feature.
    current_route_command_index = 0;


    // previous move cmd checkers.
    // need to detect incorrect WPs sequences
    // when flying with rangefinder (see algo below)
    bool prev_cmd_move_spline_rangefinder = false;
    bool prev_cmd_move_spline_relative_amsl = false;

    // do we need rangefinder during mission. Means that at least one WP
    // has follow_terrain flag
    bool need_rangefinder = false;

    for (int i = 0; i < vsm_cmd.sub_commands_size(); i++) {
        auto vsm_scmd = vsm_cmd.sub_commands(i);
        auto cmd = vehicle.Get_command(vsm_scmd.command_id());
        if (cmd) {
            if (!cmd->Is_mission_item()) {
                VSM_EXCEPTION(
                    Invalid_param_exception,
                    "Command %s not supported in route",
                    cmd->Get_name().c_str());
            }
            VEHICLE_LOG_INF(vehicle, "ROUTE item %s", vehicle.Dump_command(vsm_scmd).c_str());
            vehicle.current_command_map.Set_current_command(current_route_command_index);

            auto params = cmd->Build_parameter_list(vsm_scmd);
            // check terrain following possibility
            if (cmd == vehicle.c_move) {
                bool is_terrain_following = false;
                if (params.Get_value("follow_terrain", is_terrain_following)) {
                    if (is_terrain_following) {
                        need_rangefinder = true;
                    }
                }

                // If previous move waypoint was spline+terrain_follow
                // and current is relative or AMSL - throw exception
                // ARDU doesn't support this WP sequence and will switch to RTL
                // during flight
                if (prev_cmd_move_spline_rangefinder && !is_terrain_following) {
                    VSM_EXCEPTION(Invalid_param_exception, "ARDUPILOT does not support switching to AGL/AMSL altitude"
                                                           " mode right after spline waypoint with Rangefinder altitude"
                                                           " mode!");
                }
                // If previous move waypoint was spline+AMSL\AGL
                // and current is terrain_follow - throw exception
                // ARDU doesn't support this WP sequence and will switch to RTL
                // during flight
                if (prev_cmd_move_spline_relative_amsl && is_terrain_following) {
                    VSM_EXCEPTION(Invalid_param_exception, "ARDUPILOT does not support switching to Rangefinder "
                                                           "altitude mode right after spline waypoint with AGL/AMSL"
                                                           " altitude mode!");
                }

                // update prev_cmd_move_spline_rangefinder and prev_cmd_move_spline_relative_amsl
                // values
                prev_cmd_move_spline_rangefinder = false;
                prev_cmd_move_spline_relative_amsl = false;
                int tt;
                if (params.Get_value("turn_type", tt)) {
                    prev_cmd_move_spline_rangefinder = ((tt == proto::TURN_TYPE_SPLINE) && is_terrain_following);
                    prev_cmd_move_spline_relative_amsl = ((tt == proto::TURN_TYPE_SPLINE) && !is_terrain_following);
                }
            }

            if (cmd == vehicle.c_set_home) {
                if (takeoff_present) {
                    VSM_EXCEPTION(Invalid_param_exception, "SET_HOME must appear before any WP");
                }
                float alt;
                if (params.Get_value("altitude_amsl", alt)) {
                    if (ardu_vehicle.send_home_position_as_mav_cmd) {
                        // HL altitude becomes altitude origin.
                        // Need to set at the very beginning as it is used to specify safe_altitude, too.
                        altitude_origin = alt;
                        VEHICLE_LOG_INF(vehicle, "Using HL altitude %f as altitude origin", altitude_origin);
                    }
                    home_altitude = alt;
                }
                hl_params = params;

                /* Ardupilot waypoint at index zero is always treated as home position,
                 * so ensure it is always present. In other words, real waypoint should not
                 * be stored at zero index.
                 */
                Prepare_set_home(params);

                // Set home as first mission item. This makes sure HL is set on each mission execution.
                Prepare_set_home(params);
                set_home_present = true;
            } else if (cmd == vehicle.c_takeoff_mission) {
                if (!set_home_present) {
                    // If HL is not specified we must populate the 0 WPt with something.
                    // First WP data in this case.
                    // Newer Ardu versions 3.4+ ignores this.
                    // For older ardupilots it will move HL to first WP.
                    auto mi = Create_mission_item_with_coords(params);
                    (*mi)->command = mavlink::MAV_CMD_NAV_WAYPOINT;
                    Add_mission_item(mi);
                    set_home_present = true;
                }
                takeoff_present = true;
            } else if (cmd == vehicle.c_set_parameter) {
                // For now used only by arduplane params ("LANDING_FLARE_TIME" and friends)
                Prepare_set_parameters(params);
            } else if (cmd == vehicle.c_move) {
                if (!set_home_present) {
                    // If HL is not specified we must populate the 0 WPt with something.
                    // First WP data in this case.
                    // Newer Ardu versions 3.4+ ignores this.
                    // For older ardupilots it will move HL to first WP.
                    auto mi = Create_mission_item_with_coords(params);
                    (*mi)->command = mavlink::MAV_CMD_NAV_WAYPOINT;
                    Add_mission_item(mi);
                    set_home_present = true;
                }
                takeoff_present = true;
                last_move_idx = i;
            }
        } else {
            VSM_EXCEPTION(Invalid_param_exception, "Unregistered route item %d", vsm_scmd.command_id());
        }
        current_route_command_index++;
    }

    // If we need rangefinder for c_mission_upload command
    // we must check if rangefinder is healthy and either throw
    // Altimeter_required_exception or, if suppress code was sent,
    // send warning.
    //
    // For c_get_native_route command no such behaviour is needed
    // because we should always generate mission file as response.
    if (need_rangefinder && main_cmd == vehicle.c_mission_upload) {
        // check if we have STATUS_ALTIMETER_REQUIRED suppressed
        auto it = std::find_if(vsm_cmd.suppresserrors().begin(),
                               vsm_cmd.suppresserrors().end(),
                               [](const int& code)
                               {return code == ugcs::vsm::proto::Status_code::STATUS_ALTIMETER_REQUIRED;});
        if (!vehicle.Is_rangefinder_healthy()) {
            // if STATUS_ALTIMETER_REQUIRED code is not suppressed throw error if
            // rangefinder is not healthy
            if (it == vsm_cmd.suppresserrors().end()) {
                VSM_EXCEPTION(Altimeter_required_exception,
                              "FOLLOW TERRAIN is not available on UAV. No RANGEFINDER "
                              "is healthy at this moment.");
            } else {
                VEHICLE_LOG_WRN(vehicle, "FOLLOW TERRAIN may be not available on UAV. No RANGEFINDER "
                                         "is healthy at this moment. STATUS_ALTIMETER_REQUIRED code "
                                         "suppressed by client");
                vehicle.Add_warning_message("No RANGEFINDER is healthy at this moment. Fly with caution!");
            }
        }

    }

    // This must come after home_altitude and altitude_origin is set.
    Prepare_task_attributes(route_params);
}

void
Ardupilot_vehicle::Task_upload::On_download_progress(int seqnum)
{
    // report progress only on each 10th item
    if ((seqnum % 10 == 0) && seqnum > 0) {
        vehicle.Report_progress(ucs_request, (command_count + param_count + item_count + seqnum) / total_count);
    }
}

void
Ardupilot_vehicle::Task_upload::On_upload_progress(int seqnum)
{
    // report progress only on each 10th item
    if ((seqnum % 10 == 0) && seqnum > 0) {
        vehicle.Report_progress(ucs_request, (command_count + param_count + seqnum) / total_count);
    }
}

void
Ardupilot_vehicle::Task_upload::Enable(bool generate_file)
{
    // Clean state.
    prepared_actions.clear();
    task_attributes.clear();
    hl_params.clear();
    last_move_params.Disengage();
    current_mission_poi.Disengage();
    current_mission_heading.Disengage();
    first_mission_poi_set = false;
    restart_mission_poi = false;
    // Initialize to something out of range to force autoheading (if enabled) after first WP.
    current_heading = 1000;
    heading_to_this_wp = 1000; // must be equal to current_heading for Prepare_move autoheading logic.
    current_speed = -1;
    camera_series_by_dist_active = false;
    camera_series_by_dist_active_in_wp = false;
    camera_series_by_time_active = false;
    camera_series_by_time_active_in_wp = false;
    altitude_origin = 0;
    home_altitude = 0;
    current_route_command_index = -1;
    command_count = 0;
    item_count = 0;
    param_count = 0;
    total_count = 0;

    auto &vsm_cmd = ucs_request->request.device_commands(0);
    if (generate_file) {
        Preprocess_mission(vsm_cmd);
        Prepare_mission(vsm_cmd);
        std::string mission = Generate_wpl(prepared_actions, use_crlf_in_native_route);
        ucs_request->response->mutable_device_response()->set_status(mission);
        Disable_success();
        return;
    }

    // Use GND_ALT_OFFSET to calibrate amsl altitude on mission upload.
    // Do this when uploading on ardupilot versions => 3.4.0
    bool armed = true;
    if (vehicle.t_is_armed->Get_value(armed) && !armed && ardu_vehicle.gnd_alt_offset_allow) {
        vehicle.read_parameters.Disable();
        vehicle.read_parameters.Set_next_action(
                Read_parameters::Make_next_action(
                        &Task_upload::Upload_parameters,
                        this));
        vehicle.read_parameters.Enable({"GND_ALT_OFFSET"});
        command_count = 1;
    } else {
        command_count = 0;
        Upload_parameters(true, "");
    }
}

void
Ardupilot_vehicle::Task_upload::Upload_parameters(bool success, std::string error_msg)
{
    if (success) {
        auto &vsm_cmd = ucs_request->request.device_commands(0);

        vehicle.current_command_map.Reset();
        vehicle.current_route.Reset();

        try {
            Preprocess_mission(vsm_cmd);
            Prepare_mission(vsm_cmd);
        } catch (Invalid_param_exception &e) {
            Disable(e.what());
            return;
        } catch (Altimeter_required_exception &e) {
            Disable(e.what(), proto::Status_code::STATUS_ALTIMETER_REQUIRED);
            return;
        }

        param_count = task_attributes.size();
        item_count = prepared_actions.size();

        if (ardu_vehicle.enable_route_download) {
            total_count = command_count + param_count + (item_count * 2);
        } else {
            total_count = command_count + param_count + item_count;
        }

        if (command_count) {
            vehicle.Report_progress(ucs_request, command_count / total_count);
        }

        vehicle.write_parameters.Disable();
        vehicle.write_parameters.Set_next_action(
                Write_parameters::Make_next_action(
                        &Task_upload::Task_atributes_uploaded,
                        this));
        vehicle.write_parameters.Enable(task_attributes);
    } else {
        if (error_msg.size()) {
            Disable(error_msg);
        } else {
            Disable("Could not read GND_ALT_OFFSET");
        }
    }
}

void
Ardupilot_vehicle::Task_upload::Task_atributes_uploaded(bool success, std::string error_msg)
{
    if (success) {
        if (ardu_vehicle.send_home_position_as_mav_cmd && hl_params.size()) {
            vehicle.do_commands.Disable();
            vehicle.do_commands.Set_next_action(
                    Activity::Make_next_action(
                            &Task_upload::Task_commands_sent,
                            this));

            auto cmd = Create_command_long_with_coords(hl_params);
            (*cmd)->param7 = (*cmd)->param7 + altitude_origin;
            (*cmd)->command = mavlink::MAV_CMD::MAV_CMD_DO_SET_HOME;
            vehicle.do_commands.Enable({*cmd});
        } else {
            Task_commands_sent(true);
        }
    } else {
        if (error_msg.size()) {
            Disable(error_msg);
        } else {
            Disable("Task attributes upload failed");
        }
    }
}

void
Ardupilot_vehicle::Task_upload::Task_commands_sent(bool success, std::string)
{
    if (success) {
        // New home location should be set by now.
        // invalidate current home location and timer handler will retrieve it again.
        ardu_vehicle.home_location.longitude = 0;
        ardu_vehicle.home_location.latitude = 0;

        vehicle.Report_progress(ucs_request, (command_count + param_count) / total_count);

        vehicle.mission_upload.Disable();
        vehicle.mission_upload.mission_items = std::move(prepared_actions);
        vehicle.mission_upload.Set_next_action(
                Activity::Make_next_action(&Task_upload::Mission_uploaded, this));
        vehicle.mission_upload.Enable();
    } else {
        Disable("Failed to set home location");
    }
}

void
Ardupilot_vehicle::Task_upload::Mission_uploaded(bool success, std::string error_msg)
{
    if (success) {
        VEHICLE_LOG_INF(vehicle, "Mission uploaded");
        if (ardu_vehicle.enable_route_download) {
            // Download the route to generate mission_id (hash) so that it will
            // be exactly the same when we download the mission next time.
            vehicle.read_waypoints.Disable();
            vehicle.read_waypoints.Set_next_action(
                    Activity::Make_next_action(
                            &Task_upload::Mission_downloaded,
                            this));
            vehicle.read_waypoints.Enable();
        } else {
            if (ardu_vehicle.Save_hash_on_autopilot()) {
                // When saving hash on the vehicle we do not need to download the whole route.
                vehicle.current_command_map.Fill_command_mapping_response(ucs_request->response);
            }
            Disable_success();
        }
    } else {
        if (error_msg.size()) {
            Disable(error_msg);
        } else {
            Disable("Route upload failed");
        }
    }
}

void
Ardupilot_vehicle::Task_upload::Mission_downloaded(bool success, std::string error_msg)
{
    if (success) {
        VEHICLE_LOG_INF(vehicle, "Verified mission_id=%08X", vehicle.current_command_map.Get_route_id());
        vehicle.t_current_mission_id->Set_value(vehicle.current_command_map.Get_route_id());
        // Populate the command_map in response.
        vehicle.current_command_map.Fill_command_mapping_response(ucs_request->response);
        vehicle.Command_succeeded(ucs_request);
        ucs_request = nullptr;
    }
    if (error_msg.size()) {
        vehicle.Add_status_message(error_msg);
        Disable(error_msg);
    } else {
        Disable("Route upload failed");
    }
}

void
Ardupilot_vehicle::Task_upload::Fill_mavlink_mission_item_coords(
        mavlink::Pld_mission_item_int& msg,
        const Geodetic_tuple& tuple, double heading)
{

    msg->x = static_cast<int32_t>((tuple.latitude * 180.0 / M_PI) * 1e7);
    msg->y = static_cast<int32_t>((tuple.longitude * 180.0 / M_PI) * 1e7);
    /* Fixup absolute altitude - make them relative to
     * take-off altitude.
     */
    msg->z = tuple.altitude - altitude_origin;
    msg->param4 = (heading * 180.0) / M_PI;
}

mavlink::Pld_mission_item_int::Ptr
Ardupilot_vehicle::Task_upload::Create_mission_item()
{
    auto mi = mavlink::Pld_mission_item_int::Create();
    return mi;
}

mavlink::Pld_mission_item_int::Ptr
Ardupilot_vehicle::Task_upload::Create_mission_item_with_coords(const Property_list& params)
{
    auto mi = Create_mission_item();
    double tmp;
    if (params.Get_value("latitude", tmp)) {
        (*mi)->x = static_cast<int32_t>((tmp * 180 / M_PI) * 1e7);
    }
    if (params.Get_value("longitude", tmp)) {
        (*mi)->y = static_cast<int32_t>((tmp * 180 / M_PI) * 1e7);
    }
    if (params.Get_value("altitude_amsl", tmp)) {
        (*mi)->z = tmp - altitude_origin;
    }
    if (params.Get_value("heading", tmp)) {
        (*mi)->param4 = tmp * 180 / M_PI;
    }
    return mi;
}

mavlink::Pld_command_long::Ptr
Ardupilot_vehicle::Task_upload::Create_command_long_with_coords(
    const Property_list& params)
{
    auto cmd = mavlink::Pld_command_long::Create();
    Fill_target_ids(*cmd);
    double tmp;
    if (params.Get_value("heading", tmp)) {
        (*cmd)->param4 = tmp * 180 / M_PI;
    }
    if (params.Get_value("latitude", tmp)) {
        (*cmd)->param5 = tmp * 180 / M_PI;
    }
    if (params.Get_value("longitude", tmp)) {
        (*cmd)->param6 = tmp * 180 / M_PI;
    }
    if (params.Get_value("altitude_amsl", tmp)) {
        (*cmd)->param7 = tmp - altitude_origin;
    }
    return cmd;
}

void
Ardupilot_vehicle::Task_upload::Fill_mavlink_mission_item_common(
        mavlink::Pld_mission_item_int& msg)
{
    ASSERT(vehicle.real_system_id != Mavlink_demuxer::SYSTEM_ID_ANY);

    Fill_target_ids(msg);
    msg->seq = prepared_actions.size();

    vehicle.current_route.Add_item(msg);
    vehicle.current_command_map.Add_command_mapping(msg->seq);
    if (ardu_vehicle.Save_hash_on_autopilot()) {
        vehicle.current_command_map.Accumulate_route_id(Get_mission_item_hash(msg));
    }


    /* Override all frames as relative except Terrain  */
    if (msg->frame != mavlink::MAV_FRAME::MAV_FRAME_GLOBAL_TERRAIN_ALT) {
        msg->frame = mavlink::MAV_FRAME::MAV_FRAME_GLOBAL_RELATIVE_ALT;
    }
    msg->current = 0;
    msg->autocontinue = 1;
}

void
Ardupilot_vehicle::Task_upload::On_disable()
{
    vehicle.write_parameters.Disable();
    vehicle.mission_upload.Disable();
    prepared_actions.clear();
    task_attributes.clear();
    current_mission_poi.Disengage();
    current_mission_heading.Disengage();
    last_move_params.Disengage();
    first_mission_poi_set = false;
    restart_mission_poi = false;
    current_heading = 0;
    heading_to_this_wp = 0;
}

void
Ardupilot_vehicle::Task_upload::Prepare_mission(const proto::Device_command& vsm_cmd)
{
    last_move_params.Disengage();
    current_route_command_index = 0;
    auto takeoff_present = false;
    Property_list prev_camera_params;
    for (int i = 0; i < vsm_cmd.sub_commands_size(); i++) {
        auto vsm_scmd = vsm_cmd.sub_commands(i);
        auto cmd = vehicle.Get_command(vsm_scmd.command_id());
        vehicle.current_command_map.Set_current_command(current_route_command_index);
        auto params = cmd->Build_parameter_list(vsm_scmd);
        if (cmd == vehicle.c_takeoff_mission) {
            Prepare_takeoff_mission(params);
            /* Ardupilot does not fly to the takeoff position after takeoff
             * is done. Add explicit waypoint after the takeoff command with
             * target coordinates.
             */
            Prepare_move(params);
            takeoff_present = true;
        } else if (cmd == vehicle.c_land_mission) {
            Prepare_landing(params);
        } else if (cmd == vehicle.c_move) {
            if (!takeoff_present) {
                // First WP found without prior takeoff action.
                // Insert takeoff action on first WP.
                // Required for ArduCopter 3.4+ otherwise it fails AUTO command.
                VEHICLE_LOG_WRN(vehicle, "Auto-adding TAKEOFF action before 1st WP");
                Prepare_takeoff_mission(params);
                takeoff_present = true;
            }
            Prepare_move(params, i == last_move_idx);
        } else if (cmd == vehicle.c_set_speed) {
            Prepare_change_speed(params);
        } else if (cmd == vehicle.c_wait) {
            Prepare_wait(params);
        } else if (cmd == vehicle.c_set_heading) {
            Prepare_heading(params);
        } else if (cmd == vehicle.c_panorama) {
            Prepare_panorama(params);
        } else if (cmd == vehicle.c_set_poi) {
            Prepare_poi(params);
        } else if (cmd == vehicle.c_camera_trigger_mission) {
            Prepare_camera_trigger(params);
        } else if (cmd == vehicle.c_repeat_servo) {
            Prepare_repeat_servo(params);
        } else if (cmd == vehicle.c_set_servo) {
            Prepare_set_servo(params);
        } else if (cmd == vehicle.c_camera_by_distance) {
            if (prev_camera_params.Is_equal(params) && camera_series_by_dist_active) {
                // Skip generating camera action if it is the same as previous.
                camera_series_by_dist_active_in_wp = true;
            } else {
                Prepare_camera_series_by_distance(params);
                prev_camera_params = params;
            }
        } else if (cmd == vehicle.c_camera_by_time) {
            if (prev_camera_params.Is_equal(params) && camera_series_by_time_active) {
                // Skip generating camera action if it is the same as previous.
                camera_series_by_time_active_in_wp = true;
            } else {
                Prepare_camera_series_by_time(params);
                prev_camera_params = params;
            }
        } else if (cmd == vehicle.c_transition_fixed) {
            Prepare_vtol_transition(false);
        } else if (cmd == vehicle.c_transition_vtol) {
            Prepare_vtol_transition(true);
        } else if (cmd == vehicle.c_wait_until) {
            Prepare_wait_until(params);
        } else if (cmd == vehicle.c_payload_control) {
            Prepare_payload_control(params);
        }
        current_route_command_index++;
    }
    if (last_move_params) {
        // Force stopping of camera series even if it's started in the last WP.
        camera_series_by_dist_active_in_wp = false;
        camera_series_by_time_active_in_wp = false;
        Stop_camera_series();
    }
    if (ardu_vehicle.Save_hash_on_autopilot()) {
        // Save route hash onto the vehicle.
        mavlink::Pld_param_set param;
        Fill_target_ids(param);
        param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
        param->param_id = ardu_vehicle.route_hash_parameter;
        uint32_t id = vehicle.current_command_map.Get_route_id();
        float f;
        memcpy(&f, &id, sizeof(f));
        param->param_value = f;
        VEHICLE_LOG_INF(vehicle, "Writing mission hash=%08X", vehicle.current_command_map.Get_route_id());
        task_attributes.push_back(param);
    }
}

void
Ardupilot_vehicle::Task_upload::Prepare_task_attributes(const Property_list& params)
{
    mavlink::Pld_param_set param;
    Fill_target_ids(param);

    if ( vehicle.Is_copter() ||
         vehicle.Get_vehicle_type() == proto::VEHICLE_TYPE_FIXED_WING ||
         vehicle.Get_vehicle_type() == proto::VEHICLE_TYPE_VTOL) {
        // Use GND_ALT_OFFSET to calibrate amsl altitude on mission upload.
        // Do this when uploading
        float current_alt;
        bool armed = true;
        if (    vehicle.t_altitude_amsl->Get_value(current_alt)
            &&  vehicle.t_is_armed->Get_value(armed)
            &&  !armed
            &&  ardu_vehicle.gnd_alt_offset_allow
            &&  hl_params.size())
        {
            // Vehicle on the ground and mission has HL specified and it is allowed to change GND_ALT_OFFSET
            param->param_id = "GND_ALT_OFFSET";
            param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
            if (ardu_vehicle.set_ground_alt_offset && ardu_vehicle.current_alt_offset) {
                // calibrate based on current altitude.
                param->param_value = (home_altitude - current_alt) + *ardu_vehicle.current_alt_offset;
            } else {
                // reset calibration
                param->param_value = 0;
            }
            task_attributes.push_back(param);
        }

        float safe_altitude = 0;
        int32_t safe_altitude_int = 0;
        if (params.Get_value("safe_altitude", safe_altitude)) {
            safe_altitude_int = (safe_altitude - altitude_origin) * 100;
            if (safe_altitude_int <= 0) {
                VSM_EXCEPTION(Invalid_param_exception, "Safe Altitude is below Altitude Origin");
            }
        }

        if (vehicle.Is_copter()) {
            int fs_value;
            // Set copter specific stuff
            if (params.Get_value("low_battery_action", fs_value)) {
                /* Battery failsafe. */
                if (ardu_vehicle.use_batt_fs_as_batt_failsafe) {
                    param->param_id = "BATT_FS_LOW_ACT";
                } else {
                    param->param_id = "FS_BATT_ENABLE";
                }
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
                switch (fs_value) {
                case proto::FAILSAFE_ACTION_RTH:
                    param->param_value = 2;
                    break;
                case proto::FAILSAFE_ACTION_LAND:
                    param->param_value = 1;
                    break;
                case proto::FAILSAFE_ACTION_WAIT:
                    /* There is no support for such behavior. Override with land. */
                    param->param_value = 1;
                    break;
                case proto::FAILSAFE_ACTION_CONTINUE:
                    param->param_value = 0;
                    break;
                }
                task_attributes.push_back(param);
            }

            if (params.Get_value("gps_loss_action", fs_value)) {
                /* GNSS loss failsafe. */
                if (ardu_vehicle.use_ekf_action_as_gps_failsafe) {
                    param->param_id = "FS_EKF_ACTION";
                    param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
                    switch (fs_value) {
                    case proto::FAILSAFE_ACTION_LAND:
                        param->param_value = 1;
                        break;
                    case proto::FAILSAFE_ACTION_WAIT:
                        param->param_value = 2;
                        break;
                    default:
                        LOG_WARN("Invalid FS action for GPS");
                        break;
                    }
                } else {
                    param->param_id = "FS_GPS_ENABLE";
                    param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
                    switch (fs_value) {
                    case proto::FAILSAFE_ACTION_LAND:
                        param->param_value = 3;
                        break;
                    case proto::FAILSAFE_ACTION_WAIT:
                        param->param_value = 2;
                        break;
                    default:
                        LOG_WARN("Invalid FS action for GPS");
                        break;
                    }
                }
                task_attributes.push_back(param);
            }

            /* Radio Control loss failsafe. */
            if (params.Get_value("rc_loss_action", fs_value)) {
                param->param_id = "FS_THR_ENABLE";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
                switch (fs_value) {
                case proto::FAILSAFE_ACTION_RTH:
                    param->param_value = 1;
                    break;
                case proto::FAILSAFE_ACTION_LAND:
                    param->param_value = 3;
                    break;
                case proto::FAILSAFE_ACTION_WAIT:
                    /* Wait not supported, do land. */
                    param->param_value = 3;
                    break;
                case proto::FAILSAFE_ACTION_CONTINUE:
                    /* Continue in auto, land in other modes. */
                    param->param_value = 2;
                    break;
                }
                task_attributes.push_back(param);
            }

            if (params.Get_value("rth_action", fs_value)) {
                param->param_id = "RTL_ALT_FINAL";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT16;
                float final_alt;
                switch (fs_value) {
                case proto::RTH_ACTION_LAND:
                    param->param_value = 0;
                    task_attributes.push_back(param);
                    break;
                case proto::RTH_ACTION_WAIT:
                    if (params.Get_value("rth_wait_altitude", final_alt)) {
                        param->param_value = (final_alt - altitude_origin) * 100;
                        task_attributes.push_back(param);
                    } else {
                        if (safe_altitude_int) {
                            param->param_value = safe_altitude_int;
                            task_attributes.push_back(param);
                        }
                    }
                    break;
                }
            }

            if (safe_altitude_int) {
                /* RTL altitude. */
                param->param_id = "RTL_ALT";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT16;
                param->param_value = safe_altitude_int;
                task_attributes.push_back(param);
            }

            if (ardu_vehicle.autoheading) {
                /* Don't change yaw during auto mission, because there is an auto-POI
                * feature. Besides that, duplicated waypoints are used to implement
                * actions like panorama, so it is not desirable to change the yaw while
                * switching between waypoints sharing the same location. */
                param->param_id = "WP_YAW_BEHAVIOR";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
                param->param_value = 0; /* Never change yaw. */
                task_attributes.push_back(param);
            }
        } else {
            // Plane specific stuff.
            if (safe_altitude_int) {
                /* RTL altitude. */
                param->param_id = "ALT_HOLD_RTL";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT16;
                param->param_value = safe_altitude_int;
                task_attributes.push_back(param);
            }
        }
    }
}

void
Ardupilot_vehicle::Task_upload::Prepare_set_parameters(const Property_list& params)
{
    /* Add plane specific task attributes */
    mavlink::Pld_param_set param;
    Fill_target_ids(param);
    float v;    // All my parameters are float.
    for (auto & p : params) {
        if (p.second->Get_value(v)) {
            if        (p.first == "landing_flare_altitude") {
                param->param_id = "LAND_FLARE_ALT";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
                param->param_value = v;
            } else if (p.first == "landing_flare_time") {
                param->param_id = "LAND_FLARE_SEC";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
                param->param_value = v;
            } else if (p.first == "min_landing_pitch") {
                param->param_id = "LAND_PITCH_CD";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT16;
                param->param_value = static_cast<int16_t>(lround(v * 18000 / M_PI));
            } else if (p.first == "landing_flare_damp") {
                param->param_id = "TECS_LAND_DAMP";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
                param->param_value = v;
            } else if (p.first == "landing_approach_airspeed") {
                param->param_id = "TECS_LAND_ARSPD";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
                param->param_value = v;
            } else if (p.first == "landing_speed_weighting") {
                param->param_id = "TECS_LAND_SPDWGT";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
                param->param_value = v;
            } else if (p.first == "max_auto_flight_pitch") {
                param->param_id = "TECS_PITCH_MAX";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
                param->param_value = static_cast<int8_t>(lround(v * 180 / M_PI));
            } else if (p.first == "max_pitch") {
                param->param_id = "LIM_PITCH_MAX";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT16;
                param->param_value = static_cast<int16_t>(lround(v * 18000 / M_PI));
            } else if (p.first == "min_throttle") {
                param->param_id = "THR_MIN";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
                param->param_value = static_cast<int8_t>(lround(v));
            } else if (p.first == "landing_sink_rate") {
                param->param_id = "TECS_LAND_SINK";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32;
                param->param_value = v;
            } else if (p.first == "landing_rangefinder_enabled") {
                param->param_id = "RNGFND_LANDING";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8;
                param->param_value = static_cast<int8_t>(lround(v));
            } else if (p.first == "min_rangefinder_distance") {
                param->param_id = "RNGFND_MIN_CM";
                param->param_type = mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT16;
                param->param_value = static_cast<int16_t>(lround(v * 100));
            } else {
                VEHICLE_LOG_WRN(vehicle, "Unsupported parameter %s received.", p.first.c_str());
            }
            task_attributes.push_back(param);
        } else {
            if (!p.second->Is_value_na()) {
                VEHICLE_LOG_WRN(vehicle, "Invalid value for parameter %s", p.first.c_str());
            }
        }
    }
}

void
Ardupilot_vehicle::Task_upload::Add_mission_item(mavlink::Pld_mission_item_int::Ptr mi)
{
    Fill_mavlink_mission_item_common(*mi);
    prepared_actions.push_back(mi);
}

void
Ardupilot_vehicle::Task_upload::Stop_camera_series()
{
    /* Turn off camera series if active. */
    if (!camera_series_by_dist_active_in_wp) {
        if (camera_series_by_dist_active) {
            camera_series_by_dist_active = false;
            auto mi = Create_mission_item();
            (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_SET_CAM_TRIGG_DIST;
            (*mi)->param1 = 0;  // distance
            Add_mission_item(mi);
        }
    }
    if (!camera_series_by_time_active_in_wp) {
        if (camera_series_by_time_active) {
            camera_series_by_time_active = false;
            auto mi = Create_mission_item();
            (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_REPEAT_SERVO;
            (*mi)->param1 = ardu_vehicle.camera_servo_idx;
            (*mi)->param2 = ardu_vehicle.camera_servo_pwm;
            (*mi)->param3 = 1;  // cycle count
            (*mi)->param4 = 0;  // seconds
            Add_mission_item(mi);
        }
    }
    camera_series_by_dist_active_in_wp = false;
    camera_series_by_time_active_in_wp = false;
}

void
Ardupilot_vehicle::Task_upload::Prepare_move(const Property_list& params, bool is_last)
{
    Stop_camera_series();

    auto mi = Build_wp_mission_item(params);
    if (is_last) {
        // Force last WP to straight.
        (*mi)->command = mavlink::MAV_CMD::MAV_CMD_NAV_WAYPOINT;
    }
    if (last_move_params) {
        auto from = Wgs84_position(Create_geodetic_tuple(*last_move_params));
        auto to = Wgs84_position(Create_geodetic_tuple(params));
        if (    fabs(from.Get_geodetic().latitude - to.Get_geodetic().latitude) > 0.00000001
            ||  fabs(from.Get_geodetic().longitude - to.Get_geodetic().longitude) > 0.00000001) {
            // Handle several waypoints at the same coords.
            float calculated_heading = from.Bearing(to);
            calculated_heading = Normalize_angle_0_2pi(calculated_heading);
            heading_to_this_wp = calculated_heading;
        } else {
            // Use previously calculated heading_to_this_wp.
        }
    }

    if (current_mission_poi) {
        if (!first_mission_poi_set && (ardu_vehicle.auto_generate_mission_poi || restart_mission_poi)) {
            // Add automatic POI on each consecutive WP.
            LOG("Set AutoPOI");
            Add_mission_item(Build_roi_mission_item(*current_mission_poi));
        }
    } else {
        float new_heading = current_heading;
        if (current_mission_heading) {
            // Set current heading as yaw angle.
            new_heading = *current_mission_heading;
        } else {
            if (ardu_vehicle.autoheading) {
                // Set heading to next waypoint as yaw angle.
                new_heading = heading_to_this_wp;
            }
        }
        // Do not autoadd set_heading if it changes less than given treshold.
        if (fabs(new_heading - current_heading) > CHANGE_HEADING_TRESHOLD) {
            current_heading = new_heading;
            if (last_move_params && ardu_vehicle.Is_copter()) {
                // Autoheading is copter specific.
                VEHICLE_LOG_INF(vehicle, "Set Autoheading to %f", current_heading);
                Add_mission_item(Build_heading_mission_item(current_heading));
            }
        }
    }

    Handle_terrain_following(*mi, params);

    Add_mission_item(mi);
    last_move_params = params;

    restart_mission_poi = false;
    first_mission_poi_set = false;
    current_mission_heading.Disengage();
}

void
Ardupilot_vehicle::Task_upload::Prepare_wait(const Property_list& params)
{
    if (last_move_params) {
        first_mission_poi_set = false;
        restart_mission_poi = true;
        float wait_time;
        params.Get_value("time", wait_time);
        auto wp = Build_wp_mission_item(*last_move_params);
        (*wp)->param1 = wait_time;
        switch (ardu_vehicle.Get_vehicle_type()) {
        case proto::VEHICLE_TYPE_MULTICOPTER:
        case proto::VEHICLE_TYPE_HELICOPTER:
            if (!current_mission_poi && ardu_vehicle.autoheading) {
                Add_mission_item(Build_heading_mission_item(Normalize_angle_0_2pi(current_heading)));
            }
            if (wait_time < 0) {
                (*wp)->command = mavlink::MAV_CMD::MAV_CMD_NAV_LOITER_UNLIM;
            } else {
                (*wp)->command = mavlink::MAV_CMD::MAV_CMD_NAV_LOITER_TIME;
            }

            // add terrain following frame based on previous move
            Handle_terrain_following(*wp, *last_move_params);

            break;
        case proto::VEHICLE_TYPE_FIXED_WING:
        case proto::VEHICLE_TYPE_VTOL:
            if (wait_time < 0) {
                (*wp)->command = mavlink::MAV_CMD::MAV_CMD_NAV_LOITER_UNLIM;
            } else {
                (*wp)->command = mavlink::MAV_CMD::MAV_CMD_NAV_LOITER_TIME;
            }
            break;
        default:
            VEHICLE_LOG_WRN(vehicle, "Wait supported only by plane and copter");
            return;
        }
        Add_mission_item(wp);
    } else {
        VEHICLE_LOG_WRN(vehicle, "No move action before wait action, ignored.");
    }
}

void
Ardupilot_vehicle::Task_upload::Prepare_takeoff_mission(const Property_list& params)
{
    auto mi = Create_mission_item_with_coords(params);
    if (vehicle.Is_vehicle_type(proto::VEHICLE_TYPE_VTOL)) {
        (*mi)->command = mavlink::MAV_CMD_NAV_VTOL_TAKEOFF;
    } else {
        (*mi)->command = mavlink::MAV_CMD_NAV_TAKEOFF;
    }
    (*mi)->param1 = 0; /* No data for pitch. */
    Add_mission_item(mi);
}

void
Ardupilot_vehicle::Task_upload::Prepare_landing(const Property_list& params)
{
    /* Ardupilot does not take the altitude of the landing, so
     * add explicit waypoint guiding vehicle to the landing start
     * position.
     */
    Stop_camera_series();
    Prepare_move(params);

    auto mi = Create_mission_item_with_coords(params);
    if (vehicle.Is_vehicle_type(proto::VEHICLE_TYPE_VTOL)) {
        (*mi)->command = mavlink::MAV_CMD::MAV_CMD_NAV_VTOL_LAND;
    } else {
        (*mi)->command = mavlink::MAV_CMD::MAV_CMD_NAV_LAND;
    }
    Add_mission_item(mi);

    /* Don't duplicate waypoint if last action is land. */
    last_move_params.Disengage();
}

void
Ardupilot_vehicle::Task_upload::Prepare_set_servo(const Property_list& params)
{
    int tmp;
    auto mi = Create_mission_item();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_SET_SERVO;
    params.Get_value("servo_id", tmp);
    (*mi)->param1 = tmp;
    params.Get_value("pwm", tmp);
    (*mi)->param2 = tmp;
    Add_mission_item(mi);
}

void
Ardupilot_vehicle::Task_upload::Prepare_repeat_servo(const Property_list& params)
{
    int tmp;
    float tmpf;
    auto mi = Create_mission_item();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_REPEAT_SERVO;
    params.Get_value("servo_id", tmp);
    (*mi)->param1 = tmp;
    params.Get_value("pwm", tmp);
    (*mi)->param2 = tmp;
    params.Get_value("count", tmp);
    (*mi)->param3 = tmp;
    params.Get_value("delay", tmpf);
    if (tmpf < 0.2) {
        (*mi)->param4 = 0;
    } else {
        // Pixhawk does this slower than expected. Add coefficient.
        (*mi)->param4 = tmpf * 2 - 0.4;
    }
    Add_mission_item(mi);
}

void
Ardupilot_vehicle::Task_upload::Prepare_payload_control(const Property_list& params)
{
    float tmp;
    auto mi = Create_mission_item();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_MOUNT_CONTROL;
    params.Get_value("tilt", tmp);
    (*mi)->param1 = tmp * 180.0 / M_PI;
    params.Get_value("roll", tmp);
    (*mi)->param2 = tmp * 180.0 / M_PI;
    params.Get_value("yaw", tmp);
    (*mi)->param3 = tmp * 180.0 / M_PI;
    (*mi)->z = mavlink::MAV_MOUNT_MODE::MAV_MOUNT_MODE_MAVLINK_TARGETING;
    Add_mission_item(mi);
}

void
Ardupilot_vehicle::Task_upload::Prepare_change_speed(const Property_list& params)
{
    if (ardu_vehicle.ignore_speed_in_route) {
        return;
    }

    auto mi = Create_mission_item();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_CHANGE_SPEED;

    float xspeed, zspeed;
    params.Get_value("ground_speed", xspeed);
    params.Get_value("vertical_speed", zspeed);
    auto speed = hypot(xspeed, zspeed);

    if (fabs(current_speed - speed) < CHANGE_SPEED_TRESHOLD) {
        // Do not generate change_speed if the change is too small.
        return;
    }
    current_speed = speed;

    switch (ardu_vehicle.Get_vehicle_type()) {
    case proto::VEHICLE_TYPE_MULTICOPTER:
    case proto::VEHICLE_TYPE_HELICOPTER:
        /* ArduCopter version up to 3.2 use p1 for speed and ignores p2.
         * Starting from 3.7 we do not support 3.2
         * ArduCopter version up to 3.2+ use p2 for speed and ignores p1.
         * ArduCopter version up to 4.0+ it is mandatory to setup first param correct.
         */
        (*mi)->param1 = 0; /* Airspeed. */
        (*mi)->param2 = current_speed;
        break;
    default:
        /* Ground rover takes only airspeed into account, others seems to
         * take both, but we have only airspeed from UCS, so use only air. */
        (*mi)->param1 = 0; /* Airspeed. */
        (*mi)->param2 = current_speed;
        break;
    }

    (*mi)->param3 = -1; /* Throttle no change. */
    Add_mission_item(mi);
}

void
Ardupilot_vehicle::Task_upload::Prepare_set_home(const Property_list& params)
{
    auto mi = Create_mission_item_with_coords(params);
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_SET_HOME;
    (*mi)->param1 = 0;
    (*mi)->z = (*mi)->z.Get() + altitude_origin;
    (*mi)->frame = mavlink::MAV_FRAME_GLOBAL;
    Add_mission_item(mi);
}

void
Ardupilot_vehicle::Task_upload::Prepare_poi(const Property_list& params)
{
    if (vehicle.Is_copter()) {
        bool active;
        params.Get_value("active", active);
        if (active) {
            // Set up POI for succeeding waypoints.
            current_mission_poi = Create_geodetic_tuple(params);
            Add_mission_item(Build_roi_mission_item(*current_mission_poi));
            first_mission_poi_set = true;
        } else {
            // Reset POI.
            if (ardu_vehicle.autoheading) {
                // Set to something out of normal range to make sure heading action is added
                // to terminate POI.
                current_heading = 1000;
            } else {
                Add_mission_item(Build_roi_mission_item(Geodetic_tuple(0, 0, 0)));
            }
            current_mission_poi.Disengage();
        }
    } else {
        VEHICLE_LOG_ERR(vehicle, "set_poi ignored");
    }
}

void
Ardupilot_vehicle::Task_upload::Prepare_heading(const Property_list& params)
{
    if (vehicle.Is_copter()) {
        float heading;
        params.Get_value("heading", heading);
        Add_mission_item(Build_heading_mission_item(heading));
        current_heading = heading;
        current_mission_heading = heading;
        // Heading action terminates current POI.
        restart_mission_poi = true;
    } else {
        VEHICLE_LOG_ERR(vehicle, "set_heading ignored");
    }
}

void
Ardupilot_vehicle::Task_upload::Prepare_camera_series_by_distance(const Property_list& params)
{
    if (ardu_vehicle.camera_servo_idx != -1) {
        VEHICLE_LOG_WRN(vehicle, "trigger_by_distance ignored.");
        return;
    }
    float tmp;
    auto mi = Create_mission_item();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_SET_CAM_TRIGG_DIST;
    params.Get_value("distance", tmp);
    (*mi)->param1 = tmp;
    Add_mission_item(mi);
    camera_series_by_dist_active = true;
    camera_series_by_dist_active_in_wp = true;
}

void
Ardupilot_vehicle::Task_upload::Prepare_camera_series_by_time(const Property_list& params)
{
    if (ardu_vehicle.camera_servo_idx == -1) {
        VEHICLE_LOG_WRN(vehicle, "trigger_by_time ignored.");
        return;
    }
    int tmp;
    float tmpf;
    auto mi = Create_mission_item();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_REPEAT_SERVO;
    (*mi)->param1 = ardu_vehicle.camera_servo_idx;
    (*mi)->param2 = ardu_vehicle.camera_servo_pwm;
    if (params.Get_value("count", tmp) && tmp > 0) {
        (*mi)->param3 = tmp;
    } else {
        (*mi)->param3 = 32767;    // infinity packed in signed short because Ardupilot uses short internally.
    }
    params.Get_value("period", tmpf);
    (*mi)->param4 = tmpf;
    Add_mission_item(mi);
    camera_series_by_time_active = true;
    camera_series_by_time_active_in_wp = true;
}

void
Ardupilot_vehicle::Task_upload::Prepare_camera_trigger(const Property_list& params)
{
    int state;
    params.Get_value("state", state);

    if (state != proto::CAMERA_MISSION_TRIGGER_STATE_SINGLE_PHOTO &&
        state != proto::CAMERA_MISSION_TRIGGER_STATE_SERIAL_PHOTO) {
        VEHICLE_LOG_WRN(vehicle, "Unsupported camera trigger state %d ignored.", state);
        return;
    }
    if (state == proto::CAMERA_MISSION_TRIGGER_STATE_SINGLE_PHOTO) {
        Add_camera_trigger_item();
    } else {
        if (ardu_vehicle.camera_servo_idx == -1) {
            VEHICLE_LOG_WRN(vehicle, "Command repeat_trigger ignored.");
            return;
        }
        auto mi = Create_mission_item();
        (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_REPEAT_SERVO;
        (*mi)->param1 = ardu_vehicle.camera_servo_idx;
        (*mi)->param2 = ardu_vehicle.camera_servo_pwm;
        (*mi)->param3 = 32767;    // infinity packed in signed short.
        (*mi)->param4 = ardu_vehicle.camera_servo_time;
        camera_series_by_time_active = true;
        camera_series_by_time_active_in_wp = true;
        Add_mission_item(mi);
    }
}

void
Ardupilot_vehicle::Task_upload::Prepare_vtol_transition(bool vtol)
{
    if (vehicle.Get_vehicle_type() == proto::VEHICLE_TYPE_VTOL) {
        auto mi = Create_mission_item();
        (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_VTOL_TRANSITION;
        if (vtol) {
            (*mi)->param1 = mavlink::MAV_VTOL_STATE_MC;
        } else {
            (*mi)->param1 = mavlink::MAV_VTOL_STATE_FW;
        }
        Add_mission_item(mi);
    } else {
        VEHICLE_LOG_WRN(vehicle, "VTOL transition not supported by vehicle. Ignored.");
    }
}

void
Ardupilot_vehicle::Task_upload::Prepare_wait_until(const Property_list& params)
{
    auto mi = Create_mission_item();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_NAV_DELAY;
    int64_t t;
    if (params.Get_value("time", t)) {
        std::time_t tt = t / 1000;  // remove the millisecond part.
        tm local_tm = *localtime_r(&tt, &local_tm);
        tm utc_tm = *gmtime_r(&tt, &utc_tm);
        VEHICLE_LOG_INF(vehicle, "Will wait until %04d-%02d-%02d %02d:%02d:%02d UTC (%02d:%02d:%02d local)",
            utc_tm.tm_year + 1900,
            utc_tm.tm_mon + 1,
            utc_tm.tm_mday,
            utc_tm.tm_hour,
            utc_tm.tm_min,
            utc_tm.tm_sec,
            local_tm.tm_hour,
            local_tm.tm_min,
            local_tm.tm_sec);
        (*mi)->param1 = -1;
        (*mi)->param2 = utc_tm.tm_hour;
        (*mi)->param3 = utc_tm.tm_min;
        (*mi)->param4 = utc_tm.tm_sec;
    } else {
        (*mi)->param1 = 409968000;   // let infinity be 13 years.
    }
    Add_mission_item(mi);
}

void
Ardupilot_vehicle::Task_upload::Prepare_panorama(const Property_list& params)
{
    if (!vehicle.Is_copter()) {
        VEHICLE_LOG_ERR(vehicle, "Only copters support panorama");
    }
    float angle, speed, panorama_step, panorama_delay;
    int mode;

    params.Get_value("mode", mode);
    params.Get_value("angle", angle);
    params.Get_value("speed", speed);
    params.Get_value("step", panorama_step);
    params.Get_value("delay", panorama_delay);

    if (!last_move_params) {
        VEHICLE_LOG_WRN(vehicle, "No previous move action found to generate panorama action, ignored.");
        return;
    }

    /* Panorama is always done in steps less then 180 degree to make sure that
     * turns over 180 degrees are supported.
     */
    double MAX_STEP = 3;
    speed = std::abs(speed);
    if (speed == 0 || speed > 1) {
        speed = 1; /* 1 rad/second assumed max speed. */
    }

    float cur_angle = 0;
    float target_angle = std::abs(angle);
    float step = MAX_STEP;
    int delay = 0;
    // Set delay to slightly more than the calculated time.
    double panorama_duration = std::abs(angle) / speed + 3;

    switch (mode) {
    case proto::PANORAMA_MODE_VIDEO:
        break;
    case proto::PANORAMA_MODE_PHOTO:
        /* Per-sector delay. */
        delay = panorama_delay;
        // non zero delay. Use step from action.
        step = std::abs(panorama_step);
        // Add some time the delay to make the turn.
        delay += step / speed + 2;
        // do not add the long pause waypoint at the end.
        panorama_duration = 0;
        break;
    }

    if (step == 0) {
        VEHICLE_LOG_WRN(vehicle, "Zero step angle, ignoring panorama.");
        return;
    }

    Add_mission_item(Build_heading_mission_item(current_heading));
    /* Create additional waypoint at the current position with wait to
     * stabilize before doing panorama. */
    auto waiter = Build_wp_mission_item(*last_move_params);
    (*waiter)->param1 = 2; /* seconds. */
    Add_mission_item(waiter);

    // Set off the trigger
    Add_camera_trigger_item();

    while (cur_angle < target_angle) {
        if (cur_angle + step > target_angle) {
            step = target_angle - cur_angle;
        }

        float cur_step_angle = 0;
        while (cur_step_angle < step) {
            if (step > MAX_STEP) {
                cur_step_angle += MAX_STEP;
            } else {
                cur_step_angle += step;
            }
            if (cur_step_angle > step) {
                cur_step_angle = step;
            }
            float temp_heading = cur_angle + cur_step_angle;
            if (angle < 0) {
                temp_heading = -temp_heading;
            }
            Add_mission_item(Build_heading_mission_item(
                    Normalize_angle_0_2pi(current_heading + temp_heading),
                    speed));
        }
        if (delay) {
            waiter = Build_wp_mission_item(*last_move_params);
            (*waiter)->param1 = delay;
            Add_mission_item(waiter);

            // Set off trigger
            Add_camera_trigger_item();

            if (ardu_vehicle.panorama_post_trigger_delay > 0) {
                // Camera trigger is not instant, hence we wait a bit before engaging turn action
                waiter = Build_wp_mission_item(*last_move_params);
                (*waiter)->command = mavlink::MAV_CMD::MAV_CMD_NAV_LOITER_TIME;
                (*waiter)->param1 = ardu_vehicle.panorama_post_trigger_delay; /* seconds. */
                Add_mission_item(waiter);
            }
        }
        cur_angle += cur_step_angle;
    }
    if (angle > 0) {
        current_heading = Normalize_angle_0_2pi(current_heading + target_angle);
    } else {
        current_heading = Normalize_angle_0_2pi(current_heading - target_angle);
    }


    /* Create a waypoint with hold time slightly more than estimated
     * panorama duration. Used only with Trigger_state::ON.
     */
    if (panorama_duration) {
        VEHICLE_LOG_WRN(vehicle, "Estimated panorama duration is %f seconds.", panorama_duration);
        auto long_wait = Build_wp_mission_item(*last_move_params);
        if (panorama_duration > 255) {
            VEHICLE_LOG_WRN(vehicle, "Estimated panorama duration is truncated to 255 seconds.");
            (*long_wait)->param1 = 255; /* Max possible wait for Ardupilot. */
        } else {
            (*long_wait)->param1 = panorama_duration;
        }
        Add_mission_item(long_wait);
        LOG("long_wait WP %f", panorama_duration);
    }
    // panorama action terminates current POI.
    restart_mission_poi = true;
}

mavlink::Pld_mission_item_int::Ptr
Ardupilot_vehicle::Task_upload::Build_heading_mission_item(
        float heading,
        float speed,
        bool absolute_angle,
        bool clockwise
        )
{
    auto mi = Create_mission_item();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_CONDITION_YAW;
    (*mi)->param1 = (Normalize_angle_0_2pi(heading) * 180.0) / M_PI;
    (*mi)->param2 = (speed * 180) / M_PI;
    (*mi)->param3 = clockwise? 1: -1;       // Not implemented in AP.
    (*mi)->param4 = absolute_angle? 0: 1;   /* absolute angle. */
    return mi;
}

void
Ardupilot_vehicle::Task_upload::Add_camera_trigger_item()
{
    auto mi = Create_mission_item();
    if (ardu_vehicle.camera_servo_idx == -1) {
        (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_DIGICAM_CONTROL;
        (*mi)->x = 1;
    } else {
        (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_REPEAT_SERVO;
        (*mi)->param1 = ardu_vehicle.camera_servo_idx;
        (*mi)->param2 = ardu_vehicle.camera_servo_pwm;
        (*mi)->param3 = 1;  // do it once
        (*mi)->param4 = ardu_vehicle.camera_servo_time;
    }
    Add_mission_item(mi);
}

mavlink::Pld_mission_item_int::Ptr
Ardupilot_vehicle::Task_upload::Build_roi_mission_item(const Geodetic_tuple& coords)
{
    auto mi = Create_mission_item();
    (*mi)->command = mavlink::MAV_CMD::MAV_CMD_DO_SET_ROI;
    if (coords.latitude == 0 && coords.longitude == 0 && coords.altitude == 0)
    {
        (*mi)->param1 = mavlink::MAV_ROI::MAV_ROI_NONE;
    } else {
        (*mi)->param1 = mavlink::MAV_ROI::MAV_ROI_LOCATION;
    }
    Fill_mavlink_mission_item_coords(*mi, coords, 0);
    return mi;
}

mavlink::Pld_mission_item_int::Ptr
Ardupilot_vehicle::Task_upload::Build_wp_mission_item(const Property_list& params)
{
    auto mi = Create_mission_item_with_coords(params);

    int tt;
    if (ardu_vehicle.Is_copter() && params.Get_value("turn_type", tt) && last_move_params) {
        // Check turn_type only for copter and non-first waypoints.
        // (If first WP is spline, autopilot effectively ignores it)
        switch (tt) {
        case proto::TURN_TYPE_SPLINE:
            (*mi)->command = mavlink::MAV_CMD::MAV_CMD_NAV_SPLINE_WAYPOINT;
            break;
        case proto::TURN_TYPE_STRAIGHT:
            (*mi)->command = mavlink::MAV_CMD::MAV_CMD_NAV_WAYPOINT;
            break;
        default:
            VEHICLE_LOG_WRN(vehicle, "Invalid turn type: %d, defaulting to 'straight'.", tt);
            (*mi)->command = mavlink::MAV_CMD::MAV_CMD_NAV_WAYPOINT;
            break;
        }
    } else {
        // plane, rover, turn_type not specified or this is the first waypoint
        (*mi)->command = mavlink::MAV_CMD::MAV_CMD_NAV_WAYPOINT;
    }
    float tmp;
    if (params.Get_value("wait_time", tmp)) {
        (*mi)->param1 = tmp;
    }
    if (params.Get_value("acceptance_radius", tmp)) {
        /* Set acceptance radius to something reasonable. */
        if (tmp < ACCEPTANCE_RADIUS_MIN) {
            (*mi)->param2 = ACCEPTANCE_RADIUS_MIN;
            VEHICLE_LOG_INF(vehicle, "Acceptance radius normalized from %f to %f",
                tmp, (*mi)->param2.Get());
        } else {
            (*mi)->param2 = tmp;
        }
    }
    if (params.Get_value("loiter_orbit", tmp)) {
        (*mi)->param3 = tmp;
    }
    return mi;
}


void
Ardupilot_vehicle::Task_upload::Handle_terrain_following(ugcs::vsm::mavlink::Pld_mission_item_int& mi,
                                                         const ugcs::vsm::Property_list& params)
{
    bool use_terrain_following = false;
    params.Get_value("follow_terrain", use_terrain_following);
    if (use_terrain_following) {
        // for terrain_following alt above ground is calculated as
        // altitude_amsl - ground_elevation
        double param_ground_elevation;
        double param_altitude_amsl;
        if (params.Get_value("ground_elevation", param_ground_elevation) &&
            params.Get_value("altitude_amsl", param_altitude_amsl)) {
            mi->z = param_altitude_amsl - param_ground_elevation;
            mi->frame = mavlink::MAV_FRAME_GLOBAL_TERRAIN_ALT;
            VEHICLE_LOG_INF(vehicle, "Using TF for WP. Altitude = %f", static_cast<float>(mi->z));
            if (mi->z <= 0) {
                VSM_EXCEPTION(Invalid_param_exception, "Desired TF altitude is negative (%f m)!", static_cast<float>(mi->z));
            }
        } else {
            // parameters error
            VSM_EXCEPTION(Invalid_param_exception, "Cannot calculate TF altitude. One of parameters (ground_elevation, altitude_amsl) was not set.");
        }
    }
}


void
Ardupilot_vehicle::Process_heartbeat(
            mavlink::Message<mavlink::MESSAGE_ID::HEARTBEAT>::Ptr message)
{
    // Process heartbeats only from vehicle
    if (!Is_vehicle_heartbeat_valid(message)) {
        return;
    }

    auto new_mode = message->payload->custom_mode.Get();

    Update_control_mode(new_mode);
    Update_flight_mode(new_mode);

    if (current_native_flight_mode != static_cast<int>(new_mode)) {
        const char* name = Get_native_flight_mode_name(new_mode);
        const char* prev_name = Get_native_flight_mode_name(current_native_flight_mode);
        VEHICLE_LOG_INF((*this),
            "Flight mode changed from %s (%d) to %s (%d)",
            prev_name,
            static_cast<int>(current_native_flight_mode),
            name,
            static_cast<int>(new_mode));
        current_native_flight_mode = new_mode;
        std::string s;
        if (name == nullptr) {
            s = std::to_string(new_mode);
        } else {
            s = std::string(name);
        }
        t_native_flight_mode->Set_value(s);
        Add_status_message("Flight mode changed to " + s);
    }

    bool armed_new = message->payload->base_mode.Get() & mavlink::MAV_MODE_FLAG::MAV_MODE_FLAG_SAFETY_ARMED;
    bool armed_old = false;
    t_is_armed->Get_value(armed_old);
    t_is_armed->Set_value(armed_new);

    if (armed_new && !armed_old && !set_heading_needs_wp) {
        // Reset "WP required before set_heading" on arm.
        set_heading_needs_wp = true;
    }

    is_airborne =
        message->payload->system_status.Get() == mavlink::MAV_STATE_ACTIVE ||
        message->payload->system_status.Get() == mavlink::MAV_STATE_CRITICAL;

    Update_capabilities();
    Update_capability_states();
}

/** Map custom flight mode from the heartbeat to our flight mode. */
void
Ardupilot_vehicle::Update_flight_mode(int mode)
{
    switch (Get_vehicle_type()) {
    case proto::VEHICLE_TYPE_MULTICOPTER:
    case proto::VEHICLE_TYPE_HELICOPTER:
        switch (static_cast<Copter_flight_mode>(mode)) {
        case Copter_flight_mode::AUTO:
            if (Is_current_command(mavlink::MAV_CMD_NAV_LOITER_UNLIM)) {
                current_flight_mode = proto::FLIGHT_MODE_HOLD;
            } else {
                current_flight_mode = proto::FLIGHT_MODE_WAYPOINTS;
            }
            break;
        case Copter_flight_mode::RTL:
        case Copter_flight_mode::SMART_RTL:
            current_flight_mode = proto::FLIGHT_MODE_RTH;
            break;
        case Copter_flight_mode::LAND:
            current_flight_mode = proto::FLIGHT_MODE_LAND;
            break;
        default:
            current_flight_mode.Disengage();
        }
        break;
    case proto::VEHICLE_TYPE_FIXED_WING:
    case proto::VEHICLE_TYPE_VTOL:
        switch (static_cast<Plane_flight_mode>(mode)) {
        case Plane_flight_mode::AUTO:
            current_flight_mode = proto::FLIGHT_MODE_WAYPOINTS;
            break;
        case Plane_flight_mode::RTL:
            current_flight_mode = proto::FLIGHT_MODE_RTH;
            break;
        default:
            current_flight_mode.Disengage();
        }
        break;
    case proto::VEHICLE_TYPE_GROUND:
        switch (static_cast<Rover_flight_mode>(mode)) {
        case Rover_flight_mode::AUTO:
            current_flight_mode = proto::FLIGHT_MODE_WAYPOINTS;
            break;
        case Rover_flight_mode::RTL:
            current_flight_mode = proto::FLIGHT_MODE_RTH;
            break;
        default:
            current_flight_mode.Disengage();
        }
        break;
    default:
        current_flight_mode.Disengage();
        break;
    }

    if (current_flight_mode) {
        t_flight_mode->Set_value(*current_flight_mode);
    } else {
        t_flight_mode->Set_value_na();
    }
}

void
Ardupilot_vehicle::Get_home_location()
{
    auto cmd_long = mavlink::Pld_command_long::Create();
    (*cmd_long)->target_system = real_system_id;
    (*cmd_long)->target_component = real_component_id;
    (*cmd_long)->command = mavlink::MAV_CMD_GET_HOME_POSITION;
    Send_message(*cmd_long);
}

void
Ardupilot_vehicle::Update_capabilities()
{
    switch (Get_vehicle_type()) {
    case proto::VEHICLE_TYPE_MULTICOPTER:
    case proto::VEHICLE_TYPE_HELICOPTER:
        c_arm->Set_available(true);
        c_disarm->Set_available(true);
        c_waypoint->Set_available(true);
        c_auto->Set_available(true);
        c_guided->Set_available(true);
        c_manual->Set_available(true);
        c_pause->Set_available(true);
        c_resume->Set_available(true);
        c_rth->Set_available(true);
        c_set_servo->Set_available(true);
        c_repeat_servo->Set_available(true);
        c_camera_trigger_command->Set_available(true);
        // Copter has LAND and sometimes -- joystick.
        c_land_command->Set_available(true);
        c_joystick->Set_available(true);
        c_direct_vehicle_control->Set_available(Is_control_mode(proto::CONTROL_MODE_JOYSTICK));
        c_set_fence->Set_available(true);
        break;
    case proto::VEHICLE_TYPE_FIXED_WING:
    case proto::VEHICLE_TYPE_VTOL:
        c_arm->Set_available(true);
        c_disarm->Set_available(true);
        c_waypoint->Set_available(true);
        c_auto->Set_available(true);
        c_guided->Set_available(true);
        c_manual->Set_available(true);
        c_pause->Set_available(true);
        c_resume->Set_available(true);
        c_rth->Set_available(true);
        c_set_servo->Set_available(true);
        c_repeat_servo->Set_available(true);
        c_camera_trigger_command->Set_available(true);
        if (enable_joystick_control_for_fixed_wing) {
            c_joystick->Set_available(true);
            c_direct_vehicle_control->Set_available(Is_control_mode(proto::CONTROL_MODE_JOYSTICK));
        } else {
            c_joystick->Set_available(false);
            c_direct_vehicle_control->Set_available(false);
        }
        break;
    case proto::VEHICLE_TYPE_GROUND:
        c_auto->Set_available(true);
        c_manual->Set_available(true);
        c_rth->Set_available(true);
        break;
    }
}

void
Ardupilot_vehicle::Configure_common()
{
    c_transition_fixed = flight_controller->Add_command("transition_fixed", true);

    c_transition_vtol = flight_controller->Add_command("transition_vtol", true);

    c_wait_until = flight_controller->Add_command("wait_until", true);
    c_wait_until->Add_parameter("time", proto::FIELD_SEMANTIC_TIMESTAMP);

    if (Is_vehicle_type(proto::VEHICLE_TYPE_FIXED_WING)) {
        c_set_parameter = flight_controller->Add_command("set_parameter", true);
        c_set_parameter->Add_parameter("landing_flare_altitude", Property::VALUE_TYPE_FLOAT);
        c_set_parameter->Add_parameter("landing_flare_time", Property::VALUE_TYPE_FLOAT);
        c_set_parameter->Add_parameter("min_landing_pitch", Property::VALUE_TYPE_FLOAT);
        c_set_parameter->Add_parameter("landing_flare_damp", Property::VALUE_TYPE_FLOAT);
        c_set_parameter->Add_parameter("landing_approach_airspeed", Property::VALUE_TYPE_FLOAT);
        c_set_parameter->Add_parameter("landing_speed_weighting", Property::VALUE_TYPE_FLOAT);
        c_set_parameter->Add_parameter("max_auto_flight_pitch", Property::VALUE_TYPE_FLOAT);
        c_set_parameter->Add_parameter("max_pitch", Property::VALUE_TYPE_FLOAT);
        c_set_parameter->Add_parameter("min_throttle", Property::VALUE_TYPE_FLOAT);
        c_set_parameter->Add_parameter("landing_sink_rate", Property::VALUE_TYPE_FLOAT);
        c_set_parameter->Add_parameter("landing_rangefinder_enabled", Property::VALUE_TYPE_FLOAT);
        c_set_parameter->Add_parameter("min_rangefinder_distance", Property::VALUE_TYPE_FLOAT);
    }

    // There set the p_rc_loss_action, p_gps_loss_action, p_low_battery_action, p_rth_action.
    // the specified properties are tied to mission_upload for real vehicle,
    // and to get_native_route for command processor via constructor.
    if (Is_vehicle_type(proto::VEHICLE_TYPE_FIXED_WING)) {
        Set_rc_loss_actions({
            proto::FAILSAFE_ACTION_RTH,
            proto::FAILSAFE_ACTION_CONTINUE
            });

    } else {
        Set_rc_loss_actions({
            proto::FAILSAFE_ACTION_RTH,
            proto::FAILSAFE_ACTION_CONTINUE,
            proto::FAILSAFE_ACTION_LAND
            });

        Set_gps_loss_actions({
            proto::FAILSAFE_ACTION_WAIT,
            proto::FAILSAFE_ACTION_LAND
            });
    }

    Set_low_battery_actions({
        proto::FAILSAFE_ACTION_RTH,
        proto::FAILSAFE_ACTION_CONTINUE
        });

    Set_rth_actions({
        proto::RTH_ACTION_WAIT,
        proto::RTH_ACTION_LAND
    });

    auto props = Properties::Get_instance().get();
    if (props->Exists("vehicle.ardupilot.camera_servo_idx")) {
        camera_servo_idx = props->Get_int("vehicle.ardupilot.camera_servo_idx");
        if (props->Exists("vehicle.ardupilot.camera_servo_pwm")) {
            camera_servo_pwm = props->Get_int("vehicle.ardupilot.camera_servo_pwm");
        }
        if (props->Exists("vehicle.ardupilot.camera_servo_time")) {
            camera_servo_time = props->Get_float("vehicle.ardupilot.camera_servo_time");
        }
        VEHICLE_LOG_WRN(
            (*this),
            "camera_servo_idx=%d. Camera trigger by distance in mission is disabled.",
            camera_servo_idx);
    } else {
        VEHICLE_LOG_WRN((*this), "No camera_servo_idx specified. Camera trigger by time in mission is disabled.");
    }

    if (props->Exists("vehicle.ardupilot.autoheading")) {
        auto yes = props->Get("vehicle.ardupilot.autoheading");

        if (yes == "no") {
            autoheading = false;
        } else if (yes == "yes") {
            autoheading = true;
        } else {
            VEHICLE_LOG_ERR((*this), "Invalid value '%s' for autoheading", yes.c_str());
        }

        if (autoheading) {
            VEHICLE_LOG_INF((*this), "Autoheading is on.");
        } else {
            VEHICLE_LOG_INF((*this), "Autoheading is off.");
        }
    }

    if (props->Exists("vehicle.ardupilot.ignore_speed_in_route")) {
        auto yes = props->Get("vehicle.ardupilot.ignore_speed_in_route");

        if (yes == "no") {
            ignore_speed_in_route = false;
        } else if (yes == "yes") {
            ignore_speed_in_route = true;
        } else {
            VEHICLE_LOG_ERR((*this), "Invalid value '%s' for ignore_speed_in_route", yes.c_str());
        }

        if (ignore_speed_in_route) {
            VEHICLE_LOG_INF((*this), "ignore_speed_in_route is on.");
        } else {
            VEHICLE_LOG_INF((*this), "ignore_speed_in_route is off.");
        }
    }
}

void
Ardupilot_vehicle::Configure_real_vehicle()
{
    auto props = Properties::Get_instance().get();
    if (props->Exists("vehicle.ardupilot.enable_joystick_control_for_fixed_wing")) {
        auto yes = props->Get("vehicle.ardupilot.enable_joystick_control_for_fixed_wing");
        if (yes == "yes") {
            VEHICLE_LOG_INF((*this), "Enabled joystick mode for fixed wing.");
            enable_joystick_control_for_fixed_wing = true;
        }
    }
    if (props->Exists("vehicle.ardupilot.report_relative_altitude")) {
        auto yes = props->Get("vehicle.ardupilot.report_relative_altitude");
        if (yes == "no") {
            report_relative_altitude = false;
            VEHICLE_LOG_INF(*this, "VSM will not report relative altitude.");
        } else if (yes == "yes") {
            report_relative_altitude = true;
            VEHICLE_LOG_INF((*this), "VSM will report relative altitude.");
        } else {
            VEHICLE_LOG_ERR((*this), "Invalid value '%s' for report_relative_altitude", yes.c_str());
        }
    }
    if (props->Exists("vehicle.ardupilot.set_ground_alt_offset")) {
        auto yes = props->Get("vehicle.ardupilot.set_ground_alt_offset");
        if (yes == "no") {
            set_ground_alt_offset = false;
            VEHICLE_LOG_INF((*this), "VSM will set GND_ALT_OFFSET to zero during mission upload.");
        } else if (yes == "yes") {
            set_ground_alt_offset = true;
            VEHICLE_LOG_INF((*this), "VSM will calibrate GND_ALT_OFFSET during mission upload");
        } else {
            VEHICLE_LOG_ERR((*this), "Invalid value '%s' for set_ground_alt_offset. "
                "VSM will calibrate GND_ALT_OFFSET during mission upload.",
                yes.c_str());
        }
    }

    if (props->Exists("vehicle.ardupilot.telemetry_rate")) {
        int rate = props->Get_int("vehicle.ardupilot.telemetry_rate");
        if (rate < 1) {
            rate = 1;
        } else if (rate > 50) {
            rate = 50;
        }
        VEHICLE_LOG_INF((*this), "Setting telemetry rate to %d", rate);
        telemetry_rate_hz = rate;
    }

    if (props->Exists("vehicle.ardupilot.route_hash_parameter")) {
        route_hash_parameter = props->Get("vehicle.ardupilot.route_hash_parameter");
        enable_route_download = false;
    } else {
        // Download route only if there is no route_hash_parameter defined.
        // This is the case when user does not have any parameter available but still wants to
        // have the "Current WP" feature.
        if (props->Exists("vehicle.ardupilot.enable_route_download")) {
            auto yes = props->Get("vehicle.ardupilot.enable_route_download");

            if (yes == "no") {
                enable_route_download = false;
            } else if (yes == "yes") {
                enable_route_download = true;
            } else {
                VEHICLE_LOG_ERR((*this), "Invalid value '%s' for enable_route_download", yes.c_str());
            }

            if (enable_route_download) {
                VEHICLE_LOG_INF((*this), "Route download is enabled.");
            } else {
                VEHICLE_LOG_INF((*this), "Route download is disabled.");
            }
        }
    }

    if (props->Exists("vehicle.ardupilot.panorama_post_trigger_delay")) {
        panorama_post_trigger_delay = props->Get_int("vehicle.ardupilot.panorama_post_trigger_delay");
        VEHICLE_LOG_INF((*this), "Panorama post trigger set to %f.", panorama_post_trigger_delay);
    } else {
        panorama_post_trigger_delay = 0;
        VEHICLE_LOG_INF((*this), "No Panorama post trigger delay specified.");
    }
}

void
Ardupilot_vehicle::Update_capability_states()
{
    c_manual->Set_enabled(!Is_control_mode(proto::CONTROL_MODE_MANUAL));
    c_emergency_land->Set_enabled(Is_armed());

    // servo commands are always enabled.
    c_set_servo->Set_enabled(true);
    c_repeat_servo->Set_enabled(true);
    c_camera_trigger_command->Set_enabled(true);

    c_set_fence->Set_enabled(true);

    // Calibration and reboot possible only while disarmed on the ground.
    c_trigger_calibration->Set_enabled(!Is_armed() && !is_airborne);
    c_trigger_reboot->Set_enabled(!Is_armed() && !is_airborne);

    // Takeoff and disarm is valid only while armed and on the ground.
    c_takeoff_command->Set_enabled(Is_armed() && !is_airborne);
    c_disarm->Set_enabled(Is_armed() && !is_airborne);

    // Set heading, RTH, land, waypoint valid only when airborne.
    c_set_heading->Set_enabled(Is_armed() && is_airborne);
    c_set_relative_heading->Set_enabled(Is_armed() && is_airborne);
    c_rth->Set_enabled(Is_armed() && is_airborne);
    c_land_command->Set_enabled(Is_armed() && is_airborne);
    c_waypoint->Set_enabled(Is_armed() && is_airborne);
    c_mission_clear->Set_enabled(!Is_control_mode(proto::CONTROL_MODE_AUTO));

    if (Is_armed() && is_airborne) {
        c_auto->Set_enabled(!Is_flight_mode(proto::FLIGHT_MODE_WAYPOINTS) && !Is_flight_mode(proto::FLIGHT_MODE_HOLD));
        c_guided->Set_enabled(!Is_control_mode(proto::CONTROL_MODE_CLICK_GO));
        c_pause->Set_enabled(!Is_control_mode(proto::CONTROL_MODE_MANUAL) && !Is_flight_mode(proto::FLIGHT_MODE_HOLD));
        // Disable resume when flight mode is auto except when in HOLD mode.
        c_resume->Set_enabled(!Is_control_mode(proto::CONTROL_MODE_AUTO) || Is_flight_mode(proto::FLIGHT_MODE_HOLD));
        c_joystick->Set_enabled(!Is_control_mode(proto::CONTROL_MODE_JOYSTICK));
        c_arm->Set_enabled(false);
        c_direct_vehicle_control->Set_enabled(Is_control_mode(proto::CONTROL_MODE_JOYSTICK));
    } else {
        c_auto->Set_enabled(Is_armed());
        c_guided->Set_enabled(false);
        c_joystick->Set_enabled(false);
        c_pause->Set_enabled(false);
        c_resume->Set_enabled(false);
        c_arm->Set_enabled(!Is_armed() && !Is_control_mode(proto::CONTROL_MODE_AUTO));
        c_direct_vehicle_control->Set_enabled(false);
    }
    Commit_to_ucs();
}

const char*
Ardupilot_vehicle::Get_native_flight_mode_name(int mode)
{
    switch (Get_vehicle_type()) {
    case proto::VEHICLE_TYPE_MULTICOPTER:
    case proto::VEHICLE_TYPE_HELICOPTER:
        switch (static_cast<Copter_flight_mode>(mode)) {
        case Copter_flight_mode::STABILIZE:
            return "STABILIZE";
        case Copter_flight_mode::ACRO:
            return "ACRO";
        case Copter_flight_mode::ALT_HOLD:
            return "ALT_HOLD";
        case Copter_flight_mode::AUTO:
            return "AUTO";
        case Copter_flight_mode::GUIDED:
            return "GUIDED";
        case Copter_flight_mode::LOITER:
            return "LOITER";
        case Copter_flight_mode::RTL:
            return "RTL";
        case Copter_flight_mode::CIRCLE:
            return "CIRCLE";
        case Copter_flight_mode::LAND:
            return "LAND";
        case Copter_flight_mode::OF_LOITER:
            return "OF_LOITER";
        case Copter_flight_mode::DRIFT:
            return "DRIFT";
        case Copter_flight_mode::SPORT:
            return "SPORT";
        case Copter_flight_mode::FLIP:
            return "FLIP";
        case Copter_flight_mode::AUTOTUNE:
            return "AUTOTUNE";
        case Copter_flight_mode::POSHOLD:
            return "POSHOLD";
        case Copter_flight_mode::BRAKE:
            return "BRAKE";
        case Copter_flight_mode::THROW:
            return "THROW";
        case Copter_flight_mode::AVOID_ADSB:
            return "AVOID_ADSB";
        case Copter_flight_mode::GUIDED_NOGPS:
            return "GUIDED_NOGPS";
        case Copter_flight_mode::SMART_RTL:
            return "SMART_RTL";
        }
        break;
    case proto::VEHICLE_TYPE_FIXED_WING:
    case proto::VEHICLE_TYPE_VTOL:
        switch (static_cast<Plane_flight_mode>(mode)) {
        case Plane_flight_mode::MANUAL:
            return "MANUAL";
        case Plane_flight_mode::CIRCLE:
            return "CIRCLE";
        case Plane_flight_mode::STABILIZE:
            return "STABILIZE";
        case Plane_flight_mode::TRAINING:
            return "TRAINING";
        case Plane_flight_mode::ACRO:
            return "ACRO";
        case Plane_flight_mode::FLY_BY_WIRE_A:
            return "FLY_BY_WIRE_A";
        case Plane_flight_mode::FLY_BY_WIRE_B:
            return "FLY_BY_WIRE_B";
        case Plane_flight_mode::CRUISE:
            return "CRUISE";
        case Plane_flight_mode::AUTO:
            return "AUTO";
        case Plane_flight_mode::AUTOTUNE:
            return "AUTOTUNE";
        case Plane_flight_mode::RTL:
            return "RTL";
        case Plane_flight_mode::LOITER:
            return "LOITER";
        case Plane_flight_mode::GUIDED:
            return "GUIDED";
        case Plane_flight_mode::INITIALISING:
            return "INITIALISING";
        case Plane_flight_mode::AVOID_ADSB:
            return "AVOID_ADSB";
        case Plane_flight_mode::QSTABILIZE:
            return "QSTABILIZE";
        case Plane_flight_mode::QHOVER:
            return "QHOVER";
        case Plane_flight_mode::QLOITER:
            return "QLOITER";
        case Plane_flight_mode::QLAND:
            return "QLAND";
        case Plane_flight_mode::QRTL:
            return "QRTL";
        }
        break;
    case proto::VEHICLE_TYPE_GROUND:
        switch (static_cast<Rover_flight_mode>(mode)) {
        case Rover_flight_mode::MANUAL:
            return "MANUAL";
        case Rover_flight_mode::ACRO:
            return "ACRO";
        case Rover_flight_mode::STEERING:
            return "STEERING";
        case Rover_flight_mode::HOLD:
            return "HOLD";
        case Rover_flight_mode::AUTO:
            return "AUTO";
        case Rover_flight_mode::RTL:
            return "RTL";
        case Rover_flight_mode::SMART_RTL:
            return "SMART_RTL";
        case Rover_flight_mode::GUIDED:
            return "GUIDED";
        case Rover_flight_mode::INITIALISING:
            return "INITIALISING";
        }
        break;
    }
    return nullptr;
}

void
Ardupilot_vehicle::Update_control_mode(int mode)
{
    switch (Get_vehicle_type()) {
    case proto::VEHICLE_TYPE_MULTICOPTER:
    case proto::VEHICLE_TYPE_HELICOPTER:
        switch (static_cast<Copter_flight_mode>(mode)) {
        case Copter_flight_mode::GUIDED:
        case Copter_flight_mode::GUIDED_NOGPS:
            t_control_mode->Set_value(proto::CONTROL_MODE_CLICK_GO);
            break;
        case Copter_flight_mode::AUTO:
        case Copter_flight_mode::RTL:
        case Copter_flight_mode::SMART_RTL:
        case Copter_flight_mode::CIRCLE:
        case Copter_flight_mode::LAND:
        case Copter_flight_mode::AUTOTUNE:
        case Copter_flight_mode::BRAKE:
        case Copter_flight_mode::THROW:
            t_control_mode->Set_value(proto::CONTROL_MODE_AUTO);
            break;
        case Copter_flight_mode::STABILIZE:
        case Copter_flight_mode::ACRO:
        case Copter_flight_mode::ALT_HOLD:
        case Copter_flight_mode::OF_LOITER:
        case Copter_flight_mode::LOITER:
        case Copter_flight_mode::DRIFT:
        case Copter_flight_mode::SPORT:
        case Copter_flight_mode::FLIP:
        case Copter_flight_mode::POSHOLD:
            if (Is_rc_override_active()) {
                t_control_mode->Set_value(proto::CONTROL_MODE_JOYSTICK);
            } else {
                t_control_mode->Set_value(proto::CONTROL_MODE_MANUAL);
            }
            break;
        case Copter_flight_mode::AVOID_ADSB:
            t_control_mode->Set_value_na();
            break;
        }
        break;
    case proto::VEHICLE_TYPE_GROUND:
        switch (static_cast<Rover_flight_mode>(mode)) {
        case Rover_flight_mode::AUTO:
        case Rover_flight_mode::RTL:
        case Rover_flight_mode::SMART_RTL:
            t_control_mode->Set_value(proto::CONTROL_MODE_AUTO);
            break;
        case Rover_flight_mode::GUIDED:
            t_control_mode->Set_value(proto::CONTROL_MODE_CLICK_GO);
            break;
        case Rover_flight_mode::MANUAL:
        case Rover_flight_mode::STEERING:
        case Rover_flight_mode::HOLD:
        case Rover_flight_mode::ACRO:
            if (Is_rc_override_active()) {
                t_control_mode->Set_value(proto::CONTROL_MODE_JOYSTICK);
            } else {
                t_control_mode->Set_value(proto::CONTROL_MODE_MANUAL);
            }
            break;
        case Rover_flight_mode::INITIALISING:
            t_control_mode->Set_value_na();
        }
        break;
    case proto::VEHICLE_TYPE_FIXED_WING:
    case proto::VEHICLE_TYPE_VTOL:
        switch (static_cast<Plane_flight_mode>(mode)) {
        case Plane_flight_mode::AUTO:
        case Plane_flight_mode::LOITER:
        case Plane_flight_mode::CIRCLE:
        case Plane_flight_mode::RTL:
        case Plane_flight_mode::QLOITER:
        case Plane_flight_mode::QRTL:
        case Plane_flight_mode::QLAND:
            t_control_mode->Set_value(proto::CONTROL_MODE_AUTO);
            break;
        case Plane_flight_mode::GUIDED:
            t_control_mode->Set_value(proto::CONTROL_MODE_CLICK_GO);
            break;
        case Plane_flight_mode::MANUAL:
        case Plane_flight_mode::STABILIZE:
        case Plane_flight_mode::TRAINING:
        case Plane_flight_mode::ACRO:
        case Plane_flight_mode::FLY_BY_WIRE_A:
        case Plane_flight_mode::FLY_BY_WIRE_B:
        case Plane_flight_mode::CRUISE:
        case Plane_flight_mode::QSTABILIZE:
        case Plane_flight_mode::QHOVER:
            if (Is_rc_override_active()) {
                t_control_mode->Set_value(proto::CONTROL_MODE_JOYSTICK);
            } else {
                t_control_mode->Set_value(proto::CONTROL_MODE_MANUAL);
            }
            break;
        case Plane_flight_mode::INITIALISING:
        case Plane_flight_mode::AVOID_ADSB:
        case Plane_flight_mode::AUTOTUNE:
            t_control_mode->Set_value_na();
        }
        break;
    default:
        t_control_mode->Set_value_na();
    }
}

void Ardupilot_vehicle::Initialize_telemetry() {
    auto p = Properties::Get_instance();
    std::unordered_map<int, std::string> stream_list;

    stream_list[mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_EXTRA1] = "EXTRA1"; // attitude
    stream_list[mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_EXTRA2] = "EXTRA2"; // vfr_hud
    stream_list[mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_EXTRA3] = "EXTRA3"; // vibrations
    stream_list[mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_POSITION] = "POSITION"; // gps
    stream_list[mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_EXTENDED_STATUS] = "EXTENDED_STATUS"; // gps fix & sysstatus
    stream_list[mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_RC_CHANNELS] = "RC_CHANNELS"; // output pwm

    for (auto it = stream_list.begin(); it != stream_list.end(); it++) {
        mavlink::Pld_request_data_stream msg;
        msg->target_system = real_system_id;
        msg->target_component = real_component_id;
        msg->req_stream_id = it->first;

        std::string key = "vehicle.ardupilot.telemetry_rate." + it->second;
        if (p->Exists(key)) {
            msg->req_message_rate = p->Get_int(key);
        } else {
            msg->req_message_rate = telemetry_rate_hz;
        }
        VEHICLE_LOG_INF(
            (*this),
            "Setting telemetry rate for %s to %d",
            it->second.c_str(),
            static_cast<int>(msg->req_message_rate));
        msg->start_stop = 1; /* start */
        Send_message(msg);
        Send_message(msg);
    }
    // We are counting 6 messages as telemetry:
    // SYS_STATUS, GLOBAL_POSITION_INT, ATTITUDE, VFR_HUD, GPS_RAW_INT, ALTITUDE
    expected_telemetry_rate = telemetry_rate_hz * 6;
}
