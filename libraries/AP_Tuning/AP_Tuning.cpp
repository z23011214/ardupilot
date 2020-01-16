#include "AP_Tuning.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Tuning::var_info[] = {
    // @Param: CHAN
    // @DisplayName: Transmitter tuning channel
    // @Description: This sets the channel for transmitter tuning. This should be connected to a knob or slider on your transmitter. It needs to be setup to use the PWM range given by TUNE_CHAN_MIN to TUNE_CHAN_MAX
    // @Values: 0:Disable,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("CHAN", 1, AP_Tuning, channel, 0),
    
    // @Param: CHAN_MIN
    // @DisplayName: Transmitter tuning channel minimum pwm
    // @Description: This sets the PWM lower limit for the tuning channel
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MIN", 2, AP_Tuning, channel_min, 1000),

    // @Param: CHAN_MAX
    // @DisplayName: Transmitter tuning channel maximum pwm
    // @Description: This sets the PWM upper limit for the tuning channel
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MAX", 3, AP_Tuning, channel_max, 2000),

    // @Param: SELECTOR
    // @DisplayName: Transmitter tuning selector channel
    // @Description: This sets the channel for the transmitter tuning selector switch. This should be a 2 position switch, preferably spring loaded. A PWM above 1700 means high, below 1300 means low. If no selector is set then you won't be able to switch between parameters during flight or re-center the tuning knob
    // @Values: 0:Disable,1:Chan1,2:Chan3,3:Chan3,4:Chan4,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("SELECTOR", 4, AP_Tuning, selector, 0),
    
    // @Param: RANGE
    // @DisplayName: Transmitter tuning range
    // @Description: This sets the range over which tuning will change a parameter. A value of 2 means the tuning parameter will go from 0.5 times the start value to 2x the start value over the range of the tuning channel
    // @User: Standard
    AP_GROUPINFO("RANGE", 5, AP_Tuning, range, 2.0f),

    // @Param: MODE_REVERT
    // @DisplayName: Revert on mode change
    // @Description: This controls whether tuning values will revert on a flight mode change.
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    AP_GROUPINFO("MODE_REVERT", 6, AP_Tuning, mode_revert, 1),

    // @Param: ERR_THRESH
    // @DisplayName: Controller error threshold
    // @Description: This sets the controller error threshold above which an alarm will sound and a message will be sent to the GCS to warn of controller instability
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("ERR_THRESH", 7, AP_Tuning, error_threshold, 0.15f),
    
    AP_GROUPEND
};

/*
  handle selector switch input
  2-5s高pwm随后低pwm能re_center
  5s高pwm能保存参数并重新获取re_center,随后低能执行next_parameter
*/
void AP_Tuning::check_selector_switch(void)
{
    if (selector == 0) {//未启用，返回
        // no selector switch enabled
        return;
    }
    //没有正确输入，返回，取消计时
    if (!rc().has_valid_input()) {
        selector_start_ms = 0;
        return;
    }
    RC_Channel *selchan = rc().channel(selector-1);//获取对应通道，selector0为禁用，因此从1开始表示通道，要-1
    if (selchan == nullptr) {//若不存在该通道，为空
        return;
    }
    uint16_t selector_in = selchan->get_radio_in();//获取该通道输入pwm
    if (selector_in >= 1700) {//为高时
        // high selector
        if (selector_start_ms == 0) {//若未开始，则初始化计时
            selector_start_ms = AP_HAL::millis();
        }
        uint32_t hold_time = AP_HAL::millis() - selector_start_ms;//持续时间
        if (hold_time > 5000 && changed) {//大于5s，且changed为true，则保存参数
            // save tune
            save_parameters();
            re_center();
            gcs().send_text(MAV_SEVERITY_INFO, "Tuning: Saved");
            AP_Notify::events.tune_save = 1;
            changed = false;
            need_revert = 0;
        }
    } else if (selector_in <= 1300) {
        // low selector
        if (selector_start_ms != 0) {
            uint32_t hold_time = AP_HAL::millis() - selector_start_ms;
            if (hold_time < 200) {
                // debounce!
            } else if (hold_time < 2000) {
                // re-center the value
                re_center();
                gcs().send_text(MAV_SEVERITY_INFO, "Tuning: recentered %s", get_tuning_name(current_parm));
            } else if (hold_time < 5000) {
                // change parameter
                next_parameter();
            }
            selector_start_ms = 0;
        }
    }
}

/*
  re-center the tuning value
 */
void AP_Tuning::re_center(void)
{
    AP_Float *f = get_param_pointer(current_parm);
    if (f != nullptr) {
        center_value = f->get();
    }
    mid_point_wait = true;
}

/*
  check for changed tuning input
 */
void AP_Tuning::check_input(uint8_t flightmode)
{
    //parmset 该值小于50为quadplane，50-100为固定翼均为为枚举体tuning_func中的值，100以上表示为tuning_sets里的参数组
    //current_parm为枚举体tuning_func中的值，表示当前参数parmset(相等关系)，或当前参数组parmset中的序号(不是相等关系了)
    //current_set为tuning_sets里的参数组，在parmset大于100时，与之相等？感觉没用到
    //channel负责调节值的大小。select为二位开关，负责re_center或next_parameter。本文件的参数表中，设置对应调参通道及参数切换通道。
    //parmset参数集在tuning.cpp的参数表中，选择参数组，通过GCS设置？
    if (channel <= 0 || parmset <= 0) {
        // disabled
        return;
    }

    // check for revert on changed flightmode
    //检查改变飞行模式时是否需要重置某些参数的中间值
    if (flightmode != last_flightmode) {//飞行模式改变了
        if (need_revert != 0 && mode_revert != 0) {//各需要恢复的标志位有不为0，且打开了恢复模式
            gcs().send_text(MAV_SEVERITY_INFO, "Tuning: reverted");//参数重置
            revert_parameters();//恢复参数
            re_center();//重置center位置
        }
        last_flightmode = flightmode;
    }
    
    // only adjust values at 10Hz
    //10Hz的检查频率
    uint32_t now = AP_HAL::millis();
    uint32_t dt_ms = now - last_check_ms;
    if (dt_ms < 100) {
        return;
    }
    last_check_ms = now;

    if (channel > RC_Channels::get_valid_channel_count()) {
        // not valid channel
        //大于有效通道数，无效，返回
        return;
    }

    // check for invalid range
    if (range < 1.1f) {
        range.set(1.1f);
    }

    if (current_parm == 0) {//为0时，直接跳转下一个
        next_parameter();
    }

    // cope with user changing parmset while tuning
    //貌似没啥用？除了定义只有这里赋了个值
    if (current_set != parmset) {
        re_center();
    }
    current_set = parmset;
    
    //selector设置，5s高电平保存参数，然后跳转下一个参数
    check_selector_switch();

    if (selector_start_ms) {//若check_selector在计时中，直接跳过，等待计时
        // no tuning while selector high
        return;
    }

    if (current_parm == 0) {//此时还等于0？说明一个tuning_sets中的参数组刚设置完一圈？
        return;
    }
    
    RC_Channel *chan = rc().channel(channel-1);//获取当前channel的指针，channel应该跟paraset一样，由参数表得来
    if (chan == nullptr) {//指针为空，则返回
        return;
    }
    //线性插值，根据chan->get_radio_in()在channel_min和channel_max间的位置，确定输出-1到1中的值
    //即将该通道输入值单位化至[-1,1]区间内
    float chan_value = linear_interpolate(-1, 1, chan->get_radio_in(), channel_min, channel_max);
    if (dt_ms > 500) {//隔500ms更新一次，之前不是100ms会刷新一次吗，怎么会到500ms？
    //除非有过RC信号中断，未进入该函数，使得last_check_ms与now之间间隔较大
    //所以这是信号重新连上的重置？但这不是只能重置当前通道的吗？别的通道不用？
        last_channel_value = chan_value;
    }

    //检查控制器误差
    // check for controller error
    check_controller_error();
    
    //若两次变化在0.01以内，则忽略变化
    if (fabsf(chan_value - last_channel_value) < 0.01) {
        // ignore changes of less than 1%
        return;
    }
    //这句是显示两次通道的值，但被注释了
    //hal.console->printf("chan_value %.2f last_channel_value %.2f\n", chan_value, last_channel_value);

    if (mid_point_wait) {//若之前有过recenter,则该值被设为true
        // see if we have crossed the mid-point. We use a small deadzone to make it easier
        // to move to the "indent" portion of a slider to start tuning
        //死区内，仍等待
        const float dead_zone = 0.02;
        if ((chan_value > dead_zone && last_channel_value > 0) ||
            (chan_value < -dead_zone && last_channel_value < 0)) {
            // still waiting
            return;
        }
        // starting tuning
        mid_point_wait = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Tuning: mid-point %s", get_tuning_name(current_parm));
        AP_Notify::events.tune_started = 1;//指示灯？开始调参？
    }
    last_channel_value = chan_value;//记录上一次的值

    //这个操作有什么用？在center附近通过channel进行调节？
    //range表示每次调节的范围？
    //[center_value/range,center_value]  [center_value,center_value*range]这个范围再插值
    float new_value;
    if (chan_value > 0) {
        new_value = linear_interpolate(center_value, range*center_value, chan_value, 0, 1);
    } else {
        new_value = linear_interpolate(center_value/range, center_value, chan_value, -1, 0);
    }
    changed = true;//selector能执行next_parameter
    need_revert |= (1U << current_parm_index);
    set_value(current_parm, new_value);
    Log_Write_Parameter_Tuning(new_value);
}


/*
  log a tuning change
 */
void AP_Tuning::Log_Write_Parameter_Tuning(float value)
{
    AP::logger().Write("PTUN", "TimeUS,Set,Parm,Value,CenterValue", "QBBff",
                                           AP_HAL::micros64(),
                                           parmset,
                                           current_parm,
                                           (double)value,
                                           (double)center_value);
}

/*
  save parameters in the set
  保存参数
 */
void AP_Tuning::save_parameters(void)
{
    uint8_t set = (uint8_t)parmset.get();
    if (set < set_base) {
        // single parameter tuning
        save_value(set);
        return;
    }
    // multiple parameter tuning
    for (uint8_t i=0; tuning_sets[i].num_parms != 0; i++) {
        if (tuning_sets[i].set+set_base == set) {
            for (uint8_t p=0; p<tuning_sets[i].num_parms; p++) {
                save_value(tuning_sets[i].parms[p]);
            }
            break;
        }
    }
}


/*
  save parameters in the set
 */
void AP_Tuning::revert_parameters(void)
{
    uint8_t set = (uint8_t)parmset.get();
    if (set < set_base) {
        // single parameter tuning
        reload_value(set);
        return;
    }
    for (uint8_t i=0; tuning_sets[i].num_parms != 0; i++) {//tuning_sets定义在tuning.cpp内为各参数组的集合
        if (tuning_sets[i].set+set_base == set) {//.set==set-set_base
            for (uint8_t p=0; p<tuning_sets[i].num_parms; p++) {
                if (p >= 32 || (need_revert & (1U<<p))) {
                    reload_value(tuning_sets[i].parms[p]);
                }
            }
            need_revert = 0;
            break;
        }
    }
}

/*
  switch to the next parameter in the set
 */
void AP_Tuning::next_parameter(void)
{
    uint8_t set = (uint8_t)parmset.get();
    if (set < set_base) {
        // nothing to do but re-center
        current_parm = set;
        re_center();        
        return;
    }
    for (uint8_t i=0; tuning_sets[i].num_parms != 0; i++) {
        if (tuning_sets[i].set+set_base == set) {//可以理解为.set==set-setbase
            if (current_parm == 0) {
                current_parm_index = 0;
            } else {
                current_parm_index = (current_parm_index + 1) % tuning_sets[i].num_parms;
            }
            current_parm = tuning_sets[i].parms[current_parm_index];//当前参数序号赋给current_parm
            re_center();
            gcs().send_text(MAV_SEVERITY_INFO, "Tuning: started %s", get_tuning_name(current_parm));
            AP_Notify::events.tune_next = current_parm_index+1;
            break;
        }
    }
}

/*
  return a string representing a tuning parameter
 */
const char *AP_Tuning::get_tuning_name(uint8_t parm)
{
    for (uint8_t i=0; tuning_names[i].name != nullptr; i++) {
        if (parm == tuning_names[i].parm) {
            return tuning_names[i].name;
        }
    }
    return "UNKNOWN";
}

/*
  check for controller error
  若该值大于设计的阈值且处于解锁状态，则每隔两秒向地面站报一次错
 */
void AP_Tuning::check_controller_error(void)
{
    float err = controller_error(current_parm);//control monitor，看过之后再回来看△
    if (err > error_threshold) {
        uint32_t now = AP_HAL::millis();
        if (now - last_controller_error_ms > 2000 && hal.util->get_soft_armed()) {
            AP_Notify::events.tune_error = 1;
            gcs().send_text(MAV_SEVERITY_INFO, "Tuning: error %.2f", (double)err);
            last_controller_error_ms = now;
        }
    }
}
