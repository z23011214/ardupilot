#include "Plane.h"

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
 */


/*
  calculate maximum tilt change as a proportion from 0 to 1 of tilt
  up，是否设置了上升的最大角速度
 */
float QuadPlane::tilt_max_change(bool up)
{
    float rate;
    if (up || tilt.max_rate_down_dps <= 0) {//dps是个单位吗？°/s?
        rate = tilt.max_rate_up_dps;
    } else {
        rate = tilt.max_rate_down_dps;
    }
    if (tilt.tilt_type != TILT_TYPE_BINARY && !up) {
        bool fast_tilt = false;
        if (plane.control_mode == &plane.mode_manual) {
            fast_tilt = true;
        }
        if (hal.util->get_soft_armed() && !in_vtol_mode() && !assisted_flight) {
            fast_tilt = true;
        }
        if (fast_tilt) {
            // allow a minimum of 90 DPS in manual or if we are not
            // stabilising, to give fast control
            rate = MAX(rate, 90);
        }
    }
    return rate * plane.G_Dt / 90.0f;
}

/*
  output a slew limited tiltrotor angle. tilt is from 0 to 1
  该函数设置倾转舵机的倾转角
 */
void QuadPlane::tiltrotor_slew(float newtilt)
{
    float max_change = tilt_max_change(newtilt < tilt.current_tilt);
    tilt.current_tilt = constrain_float(newtilt, tilt.current_tilt-max_change, tilt.current_tilt+max_change);

    // translate to 0..1000 range and output
    //0-1000的范围并输出
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * tilt.current_tilt);//为千倍关系
}

/*
  update motor tilt for continuous tilt servos
 */
void QuadPlane::tiltrotor_continuous_update(void)
{
    //default to inactive
    //默认false
    tilt.motors_active = false;

    //the maximum rate of throttle change
    //油门最大变化率
    float max_change;
    
    if (!in_vtol_mode() && (!hal.util->get_soft_armed() || !assisted_flight)) {
        //当我们处于纯固定翼模式时
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor
        //设置tiltrotor角度，参数为设定的目标倾转角度/90，舵机角度会以步长max_change趋近该设定值
        tiltrotor_slew(1);
 
        max_change = tilt_max_change(false);
        
        //获取当前舵机通道值，并限幅
        float new_throttle = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01, 0, 1);
<<<<<<< HEAD
        if (tilt.current_tilt < 1) {//未倾转至90°
=======
        if (tilt.current_tilt < 1) {//连续倾转，限制倾转速率
>>>>>>> e6db18b48a7675d3e8d0af5af53ff1b235be2d58
            tilt.current_throttle = constrain_float(new_throttle,
                                                    tilt.current_throttle-max_change,
                                                    tilt.current_throttle+max_change);
        } else {
<<<<<<< HEAD
            tilt.current_throttle = new_throttle;//否则，不变
=======
            tilt.current_throttle = new_throttle;//否则，不限制
>>>>>>> e6db18b48a7675d3e8d0af5af53ff1b235be2d58
        }
        if (!hal.util->get_soft_armed()) {//未解锁，设为0
            tilt.current_throttle = 0;
        } else {
            // the motors are all the way forward, start using them for fwd thrust
            uint8_t mask = is_zero(tilt.current_throttle)?0:(uint8_t)tilt.tilt_mask.get();//是否存在motor有倾转标志
            motors->output_motor_mask(tilt.current_throttle, mask, plane.rudder_dt);
            // prevent motor shutdown
            tilt.motors_active = true;
        }
        return;
    }

    // remember the throttle level we're using for VTOL flight
    float motors_throttle = motors->get_throttle();
    max_change = tilt_max_change(motors_throttle<tilt.current_throttle);
    tilt.current_throttle = constrain_float(motors_throttle,
                                            tilt.current_throttle-max_change,
                                            tilt.current_throttle+max_change);
    
    /*
      we are in a VTOL mode. We need to work out how much tilt is
      needed. There are 3 strategies we will use:

      1) in QSTABILIZE or QHOVER the angle will be set to zero. This
         enables these modes to be used as a safe recovery mode.

      2) in fixed wing assisted flight or velocity controlled modes we
         will set the angle based on the demanded forward throttle,
         with a maximum tilt given by Q_TILT_MAX. This relies on
         Q_VFWD_GAIN being set

      3) if we are in TRANSITION_TIMER mode then we are transitioning
         to forward flight and should put the rotors all the way forward
    */
    if (plane.control_mode == &plane.mode_qstabilize ||
        plane.control_mode == &plane.mode_qhover ||
        plane.control_mode == &plane.mode_qautotune) {//纯多旋翼模式，倾转角为0(0°)
        tiltrotor_slew(0);
        return;
    }

    if (assisted_flight &&
        transition_state >= TRANSITION_TIMER) {//固定翼模式 ，倾转角固定为1(90°)
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
        tiltrotor_slew(1);
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
<<<<<<< HEAD
        //Q_TILT_MAX(参数表里就叫TILT_MAX) 该参数设置wait speed时的最大倾转角 默认45° 
=======
>>>>>>> e6db18b48a7675d3e8d0af5af53ff1b235be2d58
        float settilt = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 50.0f, 0, 1);//油门前50%线性变化，之后饱和
        tiltrotor_slew(settilt * tilt.max_angle_deg / 90.0f);//于是该值0~45°
    }
}


/*
  output a slew limited tiltrotor angle. tilt is 0 or 1
 */
void QuadPlane::tiltrotor_binary_slew(bool forward)
{
    // The servo output is binary, not slew rate limited
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

    // rate limiting current_tilt has the effect of delaying throttle in tiltrotor_binary_update
    float max_change = tilt_max_change(!forward);
    if (forward) {
        tilt.current_tilt = constrain_float(tilt.current_tilt+max_change, 0, 1);
    } else {
        tilt.current_tilt = constrain_float(tilt.current_tilt-max_change, 0, 1);
    }
}

/*
  update motor tilt for binary tilt servos
 */
void QuadPlane::tiltrotor_binary_update(void)
{
    // motors always active
    tilt.motors_active = true;

    if (!in_vtol_mode()) {
        // we are in pure fixed wing mode. Move the tiltable motors
        // all the way forward and run them as a forward motor
        tiltrotor_binary_slew(true);

        float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
        if (tilt.current_tilt >= 1) {
            uint8_t mask = is_zero(new_throttle)?0:(uint8_t)tilt.tilt_mask.get();
            // the motors are all the way forward, start using them for fwd thrust
            motors->output_motor_mask(new_throttle, mask, plane.rudder_dt);
        }
    } else {
        tiltrotor_binary_slew(false);
    }
}


/*
  update motor tilt
 */
void QuadPlane::tiltrotor_update(void)
{
    if (tilt.tilt_mask <= 0) {
        // no motors to tilt
        //无倾转的旋翼
        return;
    }

    if (tilt.tilt_type == TILT_TYPE_BINARY) {
        tiltrotor_binary_update();//二进制倾转
    } else {
        tiltrotor_continuous_update();//连续倾转
    }

    if (tilt.tilt_type == TILT_TYPE_VECTORED_YAW) {
        tiltrotor_vectored_yaw();
    }
}


/*
  compensate for tilt in a set of motor outputs

  Compensation is of two forms. The first is to apply _tilt_factor,
  which is a compensation for the reduces vertical thrust when
  tilted. This is supplied by set_motor_tilt_factor().

  The second compensation is to use equal thrust on all tilted motors
  when _tilt_equal_thrust is true. This is used when the motors are
  tilted by a large angle to prevent the roll and yaw controllers from
  causing instability. Typically this would be used when the motors
  are tilted beyond 45 degrees. At this angle it is assumed that roll
  control can be achieved using fixed wing control surfaces and yaw
  control with the remaining multicopter motors (eg. tricopter tail).

  By applying _tilt_equal_thrust the tilted motors effectively become
  a single pitch control motor.

  Note that we use a different strategy for when we are transitioning
  into VTOL as compared to from VTOL flight. The reason for that is
  we want to lean towards higher tilted motor throttle when
  transitioning to fixed wing flight, in order to gain airspeed,
  whereas when transitioning to VTOL flight we want to lean to towards
  lower fwd throttle. So we raise the throttle on the tilted motors
  when transitioning to fixed wing, and lower throttle on tilted
  motors when transitioning to VTOL
 */
void QuadPlane::tilt_compensate_down(float *thrust, uint8_t num_motors)
{
    float inv_tilt_factor;
    if (tilt.current_tilt > 0.98f) {
        inv_tilt_factor = 1.0 / cosf(radians(0.98f*90));
    } else {
        inv_tilt_factor = 1.0 / cosf(radians(tilt.current_tilt*90));
    }

    // when we got past Q_TILT_MAX we gang the tilted motors together
    // to generate equal thrust. This makes them act as a single pitch
    // control motor while preventing them trying to do roll and yaw
    // control while angled over. This greatly improves the stability
    // of the last phase of transitions
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool equal_thrust = (tilt.current_tilt > tilt_threshold);

    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply inv_tilt_factor first
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            thrust[i] *= inv_tilt_factor;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;

    // now constrain and apply _tilt_equal_thrust if enabled
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            if (equal_thrust) {
                thrust[i] = tilt_total / tilt_count;
            }
            largest_tilted = MAX(largest_tilted, thrust[i]);
        }
    }

    // if we are saturating one of the tilted motors then reduce all
    // motors to keep them in proportion to the original thrust. This
    // helps maintain stability when tilted at a large angle
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}


/*
  tilt compensation when transitioning to VTOL flight
 */
void QuadPlane::tilt_compensate_up(float *thrust, uint8_t num_motors)
{
    float tilt_factor = cosf(radians(tilt.current_tilt*90));

    // when we got past Q_TILT_MAX we gang the tilted motors together
    // to generate equal thrust. This makes them act as a single pitch
    // control motor while preventing them trying to do roll and yaw
    // control while angled over. This greatly improves the stability
    // of the last phase of transitions
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool equal_thrust = (tilt.current_tilt > tilt_threshold);//是否倾转至固定翼的角度范围

    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    //要倾转的旋翼推力求和然后平均
    //不倾转的减小了一点。。。。
    // apply tilt_factor first
    for (uint8_t i=0; i<num_motors; i++) {
        if (!is_motor_tilting(i)) {
            thrust[i] *= tilt_factor;
        } else {
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    // now constrain and apply _tilt_equal_thrust if enabled
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            if (equal_thrust) {
                thrust[i] = tilt_total / tilt_count;
            }
        }
    }
}

/*
  choose up or down tilt compensation based on flight mode When going
  to a fixed wing mode we use tilt_compensate_down, when going to a
  VTOL mode we use tilt_compensate_up
 */
void QuadPlane::tilt_compensate(float *thrust, uint8_t num_motors)
{
    if (tilt.current_tilt <= 0) {
        // the motors are not tilted, no compensation needed
        return;
    }
    if (in_vtol_mode()) {
        // we are transitioning to VTOL flight
        tilt_compensate_up(thrust, num_motors);
    } else {
        tilt_compensate_down(thrust, num_motors);
    }
}

/*
  return true if the rotors are fully tilted forward
 */
bool QuadPlane::tiltrotor_fully_fwd(void)
{
    if (tilt.tilt_mask <= 0) {
        return false;
    }
    return (tilt.current_tilt >= 1);
}

/*
  control vectored yaw with tilt multicopters
 */
void QuadPlane::tiltrotor_vectored_yaw(void)
{
    // total angle the tilt can go through
    float total_angle = 90 + tilt.tilt_yaw_angle;
    // output value (0 to 1) to get motors pointed straight up
    float zero_out = tilt.tilt_yaw_angle / total_angle;

    // calculate the basic tilt amount from current_tilt
    float base_output = zero_out + (tilt.current_tilt * (1 - zero_out));
    
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool no_yaw = (tilt.current_tilt > tilt_threshold);
    if (no_yaw) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * base_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * base_output);
    } else {
        float yaw_out = motors->get_yaw();
        float yaw_range = zero_out;

        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * (base_output + yaw_out * yaw_range));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * (base_output - yaw_out * yaw_range));
    }
}

/*
  control bicopter tiltrotors
 */
void QuadPlane::tiltrotor_bicopter(void)
{
    if (tilt.tilt_type != TILT_TYPE_BICOPTER) {
        return;
    }

    if (!in_vtol_mode() && tiltrotor_fully_fwd()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  -SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, -SERVO_MAX);
        return;
    }

    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (assisted_flight) {
        hold_stabilize(throttle * 0.01f);
        motors_output(true);
    } else {
        motors_output(false);
    }

    // bicopter assumes that trim is up so we scale down so match
    float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

    if (is_negative(tilt_left)) {
        tilt_left *= tilt.tilt_yaw_angle / 90.0f;
    }
    if (is_negative(tilt_right)) {
        tilt_right *= tilt.tilt_yaw_angle / 90.0f;
    }

    // reduce authority of bicopter as motors are tilted forwards
    const float scaling = cosf(tilt.current_tilt * M_PI_2);
    tilt_left  *= scaling;
    tilt_right *= scaling;

    // add current tilt and constrain
    tilt_left  = constrain_float(-(tilt.current_tilt * SERVO_MAX) + tilt_left,  -SERVO_MAX, SERVO_MAX);
    tilt_right = constrain_float(-(tilt.current_tilt * SERVO_MAX) + tilt_right, -SERVO_MAX, SERVO_MAX);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
}
