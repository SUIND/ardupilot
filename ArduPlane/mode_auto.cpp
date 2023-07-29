#include "mode.h"
#include "Plane.h"

bool ModeAuto::_enter()
{
#if HAL_QUADPLANE_ENABLED
    // check if we should refuse auto mode due to a missing takeoff in
    // guided_wait_takeoff state
    if (plane.previous_mode == &plane.mode_guided &&
        quadplane.guided_wait_takeoff_on_mode_enter) {
        if (!plane.mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_ERROR,"Takeoff waypoint required");
            quadplane.guided_wait_takeoff = true;
            return false;
        }
    }
    
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
#else
    plane.auto_state.vtol_mode = false;
#endif
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

#if HAL_SOARING_ENABLED
    plane.g2.soaring_controller.init_cruising();
#endif

    return true;
}

void ModeAuto::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();

        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id)) {
            restart = false;
        }
#endif
        if (restart) {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModeAuto::update()
{
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        // plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        // gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        // run parachute deployment sequence
        // return;
        if(!plane.para_seq_initiated)
        {
          plane.para_seq_initiated = true;
          plane.target_loc_para = plane.next_WP_loc;
          // target heading
          plane.target_heading_para = plane.prev_WP_loc.get_bearing_to(plane.next_WP_loc)/100;
//          hal.console->printf("Lat: %d, Lon: %d \n", plane.target_loc_para.lat, plane.target_loc_para.lng);
          int32_t alt_target, engkill_alt_min, engkill_alt_max;
          engkill_alt_min = plane.g2.engkill_alt_min * 100;
          engkill_alt_max = plane.g2.engkill_alt_max * 100;
          alt_target = (engkill_alt_min + engkill_alt_max) / 2;
          plane.target_loc_para.set_alt_cm(alt_target, Location::AltFrame::ABOVE_HOME);
          plane.eng_kill_target_airspeed = (plane.g2.engkill_airspd_min * 100 + plane.g2.engkill_airspd_max * 100) / 2;
          plane.new_airspeed_cm = plane.eng_kill_target_airspeed;
          plane.set_next_WP(plane.target_loc_para);
        }
        uint16_t radius = abs(plane.g.rtl_radius);
        if (radius > 0) {
            plane.loiter.direction = (plane.g.rtl_radius < 0) ? -1 : 1;
        }

        plane.update_loiter(radius);

        // Check Engine Kill Condition
        int32_t alt_curr_int;
        int32_t curr_heading = (plane.ahrs.yaw_sensor / 100) % 360;
        if (plane.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_curr_int))
        {
          float alt_curr = alt_curr_int / 100.f;
          if (plane.smoothed_airspeed < plane.g2.engkill_airspd_max &&
              plane.smoothed_airspeed > plane.g2.engkill_airspd_min &&
              alt_curr < plane.g2.engkill_alt_max && alt_curr > plane.g2.engkill_alt_min &&
              curr_heading > plane.target_heading_para - plane.g2.engkill_heading_tolerance &&
              curr_heading < plane.target_heading_para + plane.g2.engkill_heading_tolerance)
          {
            plane.set_mode(plane.mode_fbwa, ModeReason::MISSION_END);
//            hal.console->printf("Airspeed %f, Altitude %f Engine Kill Initiated \n", plane.smoothed_airspeed, alt_curr);
            plane.gcs().send_text(MAV_SEVERITY_WARNING,
                                  "Airspeed %f, Altitude %f, Heading %d, Engine Idle Initiated \n",
                                  plane.smoothed_airspeed, alt_curr, static_cast<int16_t>(curr_heading));
            // Engine kill commands will be sent in FBWA since wings have to be leveled
            plane.t_engkill_init = AP_HAL::millis();
            plane.gcs().send_text(MAV_SEVERITY_WARNING, "Levelling Wings First \n");
            plane.engine_idle_initiated = true;
            return;
          }
        }
    }

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        plane.quadplane.control_auto();
        return;
    }
#endif

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        if (plane.landing.is_throttle_suppressed()) {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else {
            plane.calc_throttle();
        }
#if AP_SCRIPTING_ENABLED
    } else if (nav_cmd_id == MAV_CMD_NAV_SCRIPT_TIME) {
        // NAV_SCRIPTING has a desired roll and pitch rate and desired throttle
        plane.nav_roll_cd = ahrs.roll_sensor;
        plane.nav_pitch_cd = ahrs.pitch_sensor;
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.nav_scripting.throttle_pct);
#endif
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

void ModeAuto::navigate()
{
    if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
}


bool ModeAuto::does_auto_navigation() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

bool ModeAuto::does_auto_throttle() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

// returns true if the vehicle can be armed in this mode
bool ModeAuto::_pre_arm_checks(size_t buflen, char *buffer) const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.enabled()) {
        if (plane.quadplane.option_is_set(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO) &&
                !plane.quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
            hal.util->snprintf(buffer, buflen, "not in VTOL takeoff");
            return false;
        }
        if (!plane.mission.starts_with_takeoff_cmd()) {
            hal.util->snprintf(buffer, buflen, "missing takeoff waypoint");
            return false;
        }
    }
#endif
    // Note that this bypasses the base class checks
    return true;
}

bool ModeAuto::is_landing() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::LAND);
}

void ModeAuto::run()
{
    if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_ALTITUDE_WAIT) {
        // Wiggle servos
        plane.set_servos_idle();

        // Relax attitude control
        reset_controllers();

    } else {
        // Normal flight, run base class
        Mode::run();

    }
}
