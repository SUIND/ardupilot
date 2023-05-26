#include "mode.h"
#include "Plane.h"

void ModeFBWA::update()
{
//    if(!plane.para_seq_initiated)
//    {
//      uint8_t para_channel = plane.g2.para_channel;
//      uint32_t tnow = AP_HAL::millis();
//      RC_Channels::set_override(2, 1216, tnow);
//      RC_Channels::set_override(para_channel, 1200, tnow);
//    }
    // check if parachute has to be triggered
    if(plane.para_seq_initiated)
    {
      int32_t alt_curr_int;
      uint32_t tnow = AP_HAL::millis();
      uint32_t t_elapsed = tnow - plane.t_engkill_init;
      uint16_t override_data = 980;
      if (t_elapsed < 3000)
      {
        RC_Channels::set_override(2, 1100, tnow); // channel 3 index is 2
      }
      else if (t_elapsed > 3000 && t_elapsed < 8000)
      {
        RC_Channels::set_override(2, override_data, tnow); // channel 3 index is 2
        plane.t_para_init = tnow;
      }
      else if(t_elapsed > 8000)
      {
        if (plane.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_curr_int))
        {
          // check airspeed and altitude conditions
          float alt_curr = alt_curr_int / 100.f;
          if (plane.smoothed_airspeed < plane.g2.para_airspd_max &&
              plane.smoothed_airspeed > plane.g2.para_airspd_min && alt_curr < plane.g2.para_alt_max &&
              alt_curr > plane.g2.para_alt_min)
          {
            uint32_t t_elapsed_para = tnow - plane.t_para_init;
            // set para channel low
            if (t_elapsed_para < 2000)
            {
              uint8_t para_channel = plane.g2.para_channel - 1;
              RC_Channels::set_override(para_channel, override_data, tnow);
            }
            else
            {
              // hal.console->printf("Airspeed %f, Altitude %f Parachute Deployed \n", plane.smoothed_airspeed,
              // alt_curr);
              plane.gcs().send_text(MAV_SEVERITY_WARNING, "Airspeed %f, Altitude %f Parachute Deployed \n",
                                    plane.smoothed_airspeed, alt_curr);
              plane.para_seq_initiated = false;
//               hal.console->printf("GCS override enabled %d", RC_Channels::gcs_overrides_enabled());
            }
          }
          else
          {
            plane.t_para_init = tnow;
          }
        }
      }
    }
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max_cd;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min_cd);
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}
