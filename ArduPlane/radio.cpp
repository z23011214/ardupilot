#include "Plane.h"

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

/*
  allow for runtime change of control channel ordering
 */
void Plane::set_control_channels(void)
{
    if (g.rudder_only) {
        // in rudder only mode the roll and rudder channels are the
        // same.
        channel_roll = RC_Channels::rc_channel(rcmap.yaw()-1);
    } else {
        channel_roll = RC_Channels::rc_channel(rcmap.roll()-1);
    }
    channel_pitch    = RC_Channels::rc_channel(rcmap.pitch()-1);
    channel_throttle = RC_Channels::rc_channel(rcmap.throttle()-1);
    channel_rudder   = RC_Channels::rc_channel(rcmap.yaw()-1);

    // set rc channel ranges
    channel_roll->set_angle(SERVO_MAX);
    channel_pitch->set_angle(SERVO_MAX);
    channel_rudder->set_angle(SERVO_MAX);
    if (!have_reverse_thrust()) {
        // normal operation
        channel_throttle->set_range(100);
    } else {
        // reverse thrust
        if (have_reverse_throttle_rc_option) {
            // when we have a reverse throttle RC option setup we use throttle
            // as a range, and rely on the RC switch to get reverse thrust
            channel_throttle->set_range(100);
        } else {
            channel_throttle->set_angle(100);
        }
        SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);
        SRV_Channels::set_angle(SRV_Channel::k_throttleLeft, 100);
        SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 100);
    }

    if (!arming.is_armed() && arming.arming_required() == AP_Arming::Required::YES_MIN_PWM) {
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, have_reverse_thrust()?SRV_Channel::Limit::TRIM:SRV_Channel::Limit::MIN);
    }

    if (!quadplane.enable) {
        // setup correct scaling for ESCs like the UAVCAN ESCs which
        // take a proportion of speed. For quadplanes we use AP_Motors
        // scaling
        g2.servo_channels.set_esc_scaling_for(SRV_Channel::k_throttle);
    }
}

/*
  initialise RC input channels
 */
void Plane::init_rc_in()
{
    // set rc dead zones
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
    channel_rudder->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
}

/*
  initialise RC output for main channels. This is done early to allow
  for BRD_SAFETYENABLE=0 and early servo control
 */
void Plane::init_rc_out_main()
{
    /*
      change throttle trim to minimum throttle. This prevents a
      configuration error where the user sets CH3_TRIM incorrectly and
      the motor may start on power up
     */
    if (!have_reverse_thrust()) {
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttle);
    }

    SRV_Channels::set_failsafe_limit(SRV_Channel::k_aileron, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_elevator, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_rudder, SRV_Channel::Limit::TRIM);
    
    // setup flight controller to output the min throttle when safety off if arming
    // is setup for min on disarm
    if (arming.arming_required() == AP_Arming::Required::YES_MIN_PWM) {
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, have_reverse_thrust()?SRV_Channel::Limit::TRIM:SRV_Channel::Limit::MIN);
    }
}

/*
  initialise RC output channels for aux channels
 */
void Plane::init_rc_out_aux()
{
    SRV_Channels::enable_aux_servos();

    SRV_Channels::cork();
    
    servos_output();
    
    // setup PWM values to send if the FMU firmware dies
    // allows any VTOL motors to shut off
    SRV_Channels::setup_failsafe_trim_all_non_motors();
}

/*
  check for pilot input on rudder stick for arming/disarming
*/
void Plane::rudder_arm_disarm_check()
{
    AP_Arming::RudderArming arming_rudder = arming.get_rudder_arming_type();

    //未启用舵解锁，则直接返回
    if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
        //parameter disallows rudder arming/disabling
        return;
    }

    // if throttle is not down, then pilot cannot rudder arm/disarm
    //有油门时，无法解锁
    if (get_throttle_input() != 0){
        rudder_arm_timer = 0;
        return;
    }

    // if not in a manual throttle mode and not in CRUISE or FBWB
    // modes then disallow rudder arming/disarming
    //若不在手动油门模式，且不在cruise模式或FBWB模式，则禁用舵解锁
    if (auto_throttle_mode &&
        (control_mode != &mode_cruise && control_mode != &mode_fbwb)) {
        rudder_arm_timer = 0;
        return;      
    }

	if (!arming.is_armed()) {//未解锁时
		// when not armed, full right rudder starts arming counter
        //未解锁时，右满舵能解锁
		if (channel_rudder->get_control_in() > 4000) {//舵值大于4000时，开始计时
			uint32_t now = millis();

            
			if (rudder_arm_timer == 0 ||
				now - rudder_arm_timer < 3000) {
                //初始化计时时刻
				if (rudder_arm_timer == 0) {
                    rudder_arm_timer = now;
                }
			} else {//时间大于3000ms（3s），解锁
				//time to arm!
				arming.arm(AP_Arming::Method::RUDDER);
				rudder_arm_timer = 0;
			}
		} else {//不是右满舵，则取消计时
			// not at full right rudder
			rudder_arm_timer = 0;
		}
	} else if ((arming_rudder == AP_Arming::RudderArming::ARMDISARM) && !is_flying()) {//舵能够上锁，且不在飞行时
		// when armed and not flying, full left rudder starts disarming counter
        //左满舵能够上锁
		if (channel_rudder->get_control_in() < -4000) {
			uint32_t now = millis();

			if (rudder_arm_timer == 0 ||//初始化计时判断
				now - rudder_arm_timer < 3000) {
				if (rudder_arm_timer == 0) {
                    rudder_arm_timer = now;
                }
			} else {//时间大于3000ms（3s），上锁
				//time to disarm!
				arming.disarm();
				rudder_arm_timer = 0;
			}
		} else {//不是左满舵，则取消计时
			// not at full left rudder
			rudder_arm_timer = 0;
		}
	}
}

void Plane::read_radio()
{
    //没有输入，将各通道输入量设为平衡值，将控制输入量设为0
    if (!rc().read_input()) {
        control_failsafe();
        return;
    }

    //未进入故障保护模式时，标记最后的时间
    if (!failsafe.rc_failsafe)
    {
        failsafe.AFS_last_valid_rc_ms = millis();
    }

    //标记油门正常的最后时间，通过该时间判断是否进入故障安全模式
    if (rc_throttle_value_ok()) {
        failsafe.last_valid_rc_ms = millis();
    }

    if (control_mode == &mode_training) {//在训练模式下，各通道无死区
        // in training mode we don't want to use a deadzone, as we
        // want manual pass through when not exceeding attitude limits
        channel_roll->recompute_pwm_no_deadzone();
        channel_pitch->recompute_pwm_no_deadzone();
        channel_throttle->recompute_pwm_no_deadzone();
        channel_rudder->recompute_pwm_no_deadzone();
    }

    control_failsafe();//在来一次故障安全检测
    //油门微动 且 油门控制输入大于50 且 是否允许杆量混合操纵
    if (g.throttle_nudge && channel_throttle->get_control_in() > 50 && geofence_stickmixing()) {
        float nudge = (channel_throttle->get_control_in() - 50) * 0.02f;
        if (ahrs.airspeed_sensor_enabled()) {//空速传感器已使用
            airspeed_nudge_cm = (aparm.airspeed_max * 100 - aparm.airspeed_cruise_cm) * nudge;//则，空速应按比例增加至油门杆的位置
        } else {
            throttle_nudge = (aparm.throttle_max - aparm.throttle_cruise) * nudge;//油门按比例算出响应值
        }
    } else {//否则，两个都是0
        airspeed_nudge_cm = 0;
        throttle_nudge = 0;
    }

    //舵解锁及上锁判断
    rudder_arm_disarm_check();

    // potentially swap inputs for tailsitters
    //尾座式布局时，倾转过去后，进入固定翼模式，交换偏航和滚转的操作通道
    quadplane.tailsitter_check_input();

    // check for transmitter tuning changes
    //各飞行模式下的参数调整
    //注意，其中有几个函数被重写
    //有些问题，RC_Channel调PID参数？
    tuning.check_input(control_mode->mode_number());
}

int16_t Plane::rudder_input(void)
{
    if (g.rudder_only != 0) {
        // in rudder only mode we discard rudder input and get target
        // attitude from the roll channel.
        return 0;
    }

    if ((g2.flight_options & FlightOptions::DIRECT_RUDDER_ONLY) &&
        !(control_mode == &mode_manual || control_mode == &mode_stabilize || control_mode == &mode_acro)) {
        // the user does not want any input except in these modes
        return 0;
    }

    if (stick_mixing_enabled()) {
        return channel_rudder->get_control_in();
    }

    return 0;
    
}

void Plane::control_failsafe()
{
    if (millis() - failsafe.last_valid_rc_ms > 1000 || rc_failsafe_active()) {//前一个判断没有必要吧，包含于后一个
        // we do not have valid RC input. Set all primary channel
        // control inputs to the trim value and throttle to min
        channel_roll->set_radio_in(channel_roll->get_radio_trim());
        channel_pitch->set_radio_in(channel_pitch->get_radio_trim());
        channel_rudder->set_radio_in(channel_rudder->get_radio_trim());

        // note that we don't set channel_throttle->radio_in to radio_trim,
        // as that would cause throttle failsafe to not activate
        channel_roll->set_control_in(0);
        channel_pitch->set_control_in(0);
        channel_rudder->set_control_in(0);

        switch (control_mode->mode_number()) {
            case Mode::Number::QSTABILIZE:
            case Mode::Number::QHOVER:
            case Mode::Number::QLOITER:
            case Mode::Number::QLAND: // throttle is ignored, but reset anyways
            case Mode::Number::QRTL:  // throttle is ignored, but reset anyways
            case Mode::Number::QACRO:
            case Mode::Number::QAUTOTUNE:
                if (quadplane.available() && quadplane.motors->get_desired_spool_state() > AP_Motors::DesiredSpoolState::GROUND_IDLE) {
                    // set half throttle to avoid descending at maximum rate, still has a slight descent due to throttle deadzone
                    channel_throttle->set_control_in(channel_throttle->get_range() / 2);
                    break;
                }
                FALLTHROUGH;
            default:
                channel_throttle->set_control_in(0);
                break;
        }
    }

    //油门故障保护未开则直接返回
    if(g.throttle_fs_enabled == 0) {
        return;
    }

    
    if (rc_failsafe_active()) { 

        // we detect a failsafe from radio
        // throttle has dropped below the mark
        failsafe.throttle_counter++;//故障保护.油门计数++
        if (failsafe.throttle_counter == 10) {//油门计数达到10
            gcs().send_text(MAV_SEVERITY_WARNING, "Throttle failsafe on");//向地面站发送信息，油门故障保护打开
            failsafe.rc_failsafe = true;//rc安全保护打开
            AP_Notify::flags.failsafe_radio = true;//指示信息将rc故障保护设为true，可能是指示灯的
        }
        if (failsafe.throttle_counter > 10) {//限值
            failsafe.throttle_counter = 10;
        }
    } else if(failsafe.throttle_counter > 0) {//若此前有值则继续，若一直正常（=0）则返回

        // we are no longer in failsafe condition
        // but we need to recover quickly
        //迅速恢复，不直接恢复，有3次缓冲时间
        failsafe.throttle_counter--;
        if (failsafe.throttle_counter > 3) {
            failsafe.throttle_counter = 3;
        }
        if (failsafe.throttle_counter == 1) {//向地面站发送信息，油门故障保护关闭
            gcs().send_text(MAV_SEVERITY_WARNING, "Throttle failsafe off");
        } else if(failsafe.throttle_counter == 0) {//=0时，真正关闭油门故障保护，相应flag设为false
            failsafe.rc_failsafe = false;
            AP_Notify::flags.failsafe_radio = false;
        }
    }
}

bool Plane::trim_radio()
{
    if (failsafe.rc_failsafe) {
        // can't trim if we don't have valid input
        return false;
    }

    int16_t trim_roll_range = (channel_roll->get_radio_max() - channel_roll->get_radio_min())/5;
    int16_t trim_pitch_range = (channel_pitch->get_radio_max() - channel_pitch->get_radio_min())/5;
    if (channel_roll->get_radio_in() < channel_roll->get_radio_min()+trim_roll_range ||
        channel_roll->get_radio_in() > channel_roll->get_radio_max()-trim_roll_range ||
        channel_pitch->get_radio_in() < channel_pitch->get_radio_min()+trim_pitch_range ||
        channel_pitch->get_radio_in() > channel_pitch->get_radio_max()-trim_pitch_range) {
        // don't trim for extreme values - if we attempt to trim so
        // there is less than 20 percent range left then assume the
        // sticks are not properly centered. This also prevents
        // problems with starting APM with the TX off
        return false;
    }

    // trim main surfaces
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_aileron);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_elevator);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_rudder);

    // trim elevons
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_elevon_left);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_elevon_right);

    // trim vtail
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_vtail_left);
    SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_vtail_right);
    
    if (SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) == 0) {
        // trim differential spoilers if no rudder input
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerLeft1);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerLeft2);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerRight1);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_dspoilerRight2);
    }

    if (SRV_Channels::get_output_scaled(SRV_Channel::k_flap_auto) == 0 &&
        SRV_Channels::get_output_scaled(SRV_Channel::k_flap) == 0) {
        // trim flaperons if no flap input
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_flaperon_left);
        SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_flaperon_right);
    }

    // now save input trims, as these have been moved to the outputs
    channel_roll->set_and_save_trim();
    channel_pitch->set_and_save_trim();
    channel_rudder->set_and_save_trim();

    return true;
}

/*
  check if throttle value is within allowed range
 */
bool Plane::rc_throttle_value_ok(void) const
{
    if (!g.throttle_fs_enabled) {
        return true;
    }
    if (channel_throttle->get_reverse()) {
        return channel_throttle->get_radio_in() < g.throttle_fs_value;
    }
    return channel_throttle->get_radio_in() > g.throttle_fs_value;
}

/*
  return true if throttle level is below throttle failsafe threshold
  or RC input is invalid
  油门值没在正常范围内或1s内未收到遥控信号，则返回true
 */
bool Plane::rc_failsafe_active(void) const
{
    if (!rc_throttle_value_ok()) {
        return true;
    }
    if (millis() - failsafe.last_valid_rc_ms > 1000) {
        // we haven't had a valid RC frame for 1 seconds
        return true;
    }
    return false;
}
