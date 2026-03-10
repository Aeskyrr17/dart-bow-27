/**
 * @file TaskLauncher.cpp
 * @author Aeskyrr17
 * @brief 发射状态机和控制逻辑
 * @todo void Dart_Load(msg_sensor_t* sensor);
 * @todo launcher_status_t的使用（暂未确定）
 */
#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"
#include "config_launcher.hpp"
#include "config_motor.hpp"
#include "math.hpp"

TX_THREAD LauncherThread;
uint8_t LauncherThreadStack[2048] = {0};

Launcher_Cxt_t launcher{};

void Dart_Load_Test(msg_sensor_t* sensor);
bool DelayReached(delay_t* delay, bool delay_enable, ULONG delay_ticks);

[[nonreturn]] void LauncherThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 
    om_topic_t *motorctrl_topic = om_config_topic(nullptr, "ca", "motorctrl", sizeof(msg_motor_ctrl_t));
    msg_motor_ctrl_t motorctrl{};
    om_topic_t *lch2sys_topic = om_config_topic(nullptr, "ca", "lch2sys", sizeof(msg_launcher2sysctrl_t));
    msg_launcher2sysctrl_t lch2sys{};

    om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    msg_cmd_t cmd{};
    om_suber_t *sensor_suber = om_subscribe(om_find_topic("sensor", UINT32_MAX));
    msg_sensor_t sensor{};

    delay_t trig_lock_delay{};
    delay_t coil_delay{};

    float Coil_pull_spd = 17.0f; //卷簧速度
    float Coil_retern_spd = -30.0f; //卷簧复位速度，注意方向


    motorctrl.Coil_L_spd = 0.0f;
    motorctrl.Coil_R_spd = 0.0f;

    bool trigger_delay_ok = false;
    bool coil_delay_ok = false;
    bool hand_trigger_lock = true;
    for (;;) 
    {
        om_suber_export(cmd_suber, &cmd, false);
        om_suber_export(sensor_suber, &sensor, false);

        //! 测试用
        sensor.is_string_tight = true;
        sensor.is_door_open = true;

        motorctrl.trigger_lock = true;
        lch2sys.is_fire_finished = false;

        // //yaw轴控制,独立于发射逻辑
        // if (launcher.fsm_state != IDLE )
        // {
        //     motorctrl.yaw_spd = cmd.yaw;
        // }
        // else 
        // {
        //     motorctrl.yaw_spd = 0.0f;
        //     motorctrl.Coil_L_spd = 0.0f;
        //     motorctrl.Coil_R_spd = 0.0f;
        // }
        motorctrl.yaw_spd = cmd.yaw;//yaw轴控制,独立于发射逻辑,在relax状态下也可动//todo:考虑是否需要在IDLE状态下强制关闭yaw轴

        if (cmd.action == DART_RELAX) 
        {
            if (launcher.fsm_state != HAND_CONTROL || hand_trigger_lock)
                launcher.fsm_state = IDLE; 
        }
        else if (cmd.action == DART_COIL_ADJUST || cmd.action == DART_STRING_ADJUST ||
                 cmd.action == DART_TRIGGER_OPEN || cmd.action == DART_TRIGGER_CLOSE ||
                 cmd.action == DART_YAW_ADJUST)
        {
            // 如果遥控器发出了手动调试指令，强行切入手动状态
            launcher.fsm_state = HAND_CONTROL;
        }


        //FSM具体实现逻辑   
        switch (launcher.fsm_state)
        {
            case HAND_CONTROL:
                motorctrl.trigger_lock = hand_trigger_lock;
                motorctrl.Coil_L_spd = 0.0f;
                motorctrl.Coil_R_spd = 0.0f;
                motorctrl.String_L_spd = 0.0f;
                motorctrl.String_R_spd = 0.0f;
                if (cmd.action == DART_COIL_ADJUST)
                {
                    motorctrl.Coil_L_spd = cmd.Coil_L_spd;
                    motorctrl.Coil_R_spd = cmd.Coil_R_spd;
                }
                else if (cmd.action == DART_STRING_ADJUST)
                {
                    motorctrl.String_L_spd = cmd.String_L_spd;
                    motorctrl.String_R_spd = cmd.String_R_spd;
                }
                else if (cmd.action == DART_FIRE)
                {
                    launcher.fsm_state = FIRING;
                    break;
                }
                else if (cmd.action == DART_PREPARE)
                    launcher.fsm_state = RESETTING;
                else if (cmd.action == DART_TRIGGER_CLOSE)
                {
                    hand_trigger_lock = true;
                    motorctrl.trigger_lock = true;
                }
                else if (cmd.action == DART_TRIGGER_OPEN)
                {
                    hand_trigger_lock = false;
                    motorctrl.trigger_lock = false;
                }

                break;
                
            case IDLE:
                // motorctrl.yaw_spd = 0.0f;
                motorctrl.Coil_L_spd = 0.0f;
                motorctrl.Coil_R_spd = 0.0f;
                motorctrl.trigger_lock = true;

                if (cmd.action == DART_PREPARE) 
                    launcher.fsm_state = RESETTING;
                break;

            case RESETTING:
                motorctrl.trigger_lock = false;//扳机打开

                motorctrl.Coil_L_spd = Coil_pull_spd; 
                motorctrl.Coil_R_spd = Coil_pull_spd;

                //!测试：：：
                sensor.is_launchplat_return = true;

                //根据sensor.is_launchplat_return判断发射台是否已经回位，进入延时保证卷簧完全停止后再锁定扳机
                coil_delay_ok = DelayReached(&coil_delay, sensor.is_launchplat_return, 0);
                trigger_delay_ok = DelayReached(&trig_lock_delay, sensor.is_launchplat_return, 1000);
                if (coil_delay_ok)
                {
                    motorctrl.trigger_lock = true;

                    motorctrl.Coil_L_spd = 0.0f; 
                    motorctrl.Coil_R_spd = 0.0f;
                    coil_delay_ok = false;
                };

                if (trigger_delay_ok)
                {
                    launcher.fsm_state = RETRACT_AND_LOAD;

                    trigger_delay_ok = false;
                }
                
               
                break;

            case RETRACT_AND_LOAD:
                motorctrl.trigger_lock = true; 
                
                if (!sensor.is_coil_reset) 
                {
                    motorctrl.Coil_L_spd = Coil_retern_spd; 
                    motorctrl.Coil_R_spd = Coil_retern_spd;
                }
                else 
                {
                    motorctrl.Coil_L_spd = 0.0f;
                    motorctrl.Coil_R_spd = 0.0f;
                }

                Dart_Load_Test(&sensor); 
                
                //todo:飞镖装填

                //!测试！！！
                sensor.is_coil_reset = true;

                // 两个任务都完成了，才能进入READY
                if (sensor.is_coil_reset && sensor.is_dart_loaded)
                {
                    launcher.fsm_state = READY;
                };
                break;  

            case READY:
                motorctrl.trigger_lock = true;
                motorctrl.String_target_tension = cmd.tension;//保持力矩

                if (!sensor.is_string_tight)
                {
                    launcher.fsm_state = TENSIONING;
                }
                if (cmd.action == DART_FIRE)
                {
                    if (sensor.is_door_open)
                        launcher.fsm_state = FIRING;
                }
                break;

            case TENSIONING:
                motorctrl.String_target_tension = cmd.tension;
                if (sensor.is_string_tight)
                    launcher.fsm_state = READY;
                break;


            case FIRING:
                motorctrl.trigger_lock = false; //解锁扳机
                if (sensor.is_fire_done)//todo：需修改
                {
                    launcher.fsm_state = RESETTING;
                    //! 测试逻辑
                    lch2sys.is_fire_finished = true;
                    // launcher.fsm_state = IDLE;
                }
                break;
        }

        om_publish(lch2sys_topic, &lch2sys, sizeof(msg_launcher2sysctrl_t), true, false);
        om_publish(motorctrl_topic, &motorctrl, sizeof(msg_motor_ctrl_t), true, false);
        tx_thread_sleep(1);
    }

}


/**
 * @brief 龙门架逻辑，手动装填测试版本
 * 
 */
void Dart_Load_Test(msg_sensor_t* sensor)
{
    // 定义静态变量来记录时间，static变量在函数退出后值依然保留
    static uint32_t start_time = 0;
    static bool is_loading = false;

    // 如果不在装填状态，重置标志位，直接返回
    if (launcher.fsm_state != RETRACT_AND_LOAD)
    {
        is_loading = false;
        sensor->is_dart_loaded = false; // 确保非装填状态下标志位为假
        return;
    }

    // 刚进入装填状态时，记录起始时间
    if (is_loading == false)
    {
        start_time = tx_time_get(); // 获取当前系统时间 tick
        is_loading = true;          // 标记开始计时
    }

    // 计算流逝的时间 (假设 TX_TIMER_TICKS_PER_SECOND 为 1000，即1ms一个tick)
    uint32_t current_time = tx_time_get();
    
    // === 这里设置你想要模拟的手动装填时间 (例如 3000ms) ===
    if (current_time - start_time > 10000) 
    {
        // 3秒时间到，假装装填完毕
        sensor->is_dart_loaded = true;
    }
    else
    {
        // 时间没到，还在装填中
        sensor->is_dart_loaded = false;
    }
};

void Dart_Load(msg_motor_ctrl_t* motorctrl)
{
    // 正式装填逻辑
    motorctrl->gantry_open = true; // 打开龙门架
    //todo:延时逻辑
    motorctrl->gantry_open = false;
    // if ()//todo:不知道是否要保留，可能使用延时处理
    // {
    //     sensor->is_dart_loaded = true;
    // }
    // else 
    // {
    //     sensor->is_dart_loaded = false;
    // }
};



/**
 * @brief delay helper
 * 
 * @param ctx 
 * @param delay_enable 开始延时的条件，外部控制
 * @param delay_ticks 延时时间，单位tick
 * 
 */
bool DelayReached(delay_t* delay, bool delay_enable, ULONG delay_ticks)
{
    if (!delay_enable)
    {
        delay->started = false;
        return false;
    }
    if (!delay->started)
    {
        delay->start_tick = tx_time_get();
        delay->started = true;
        return false;
    }

    return (tx_time_get() - delay->start_tick) >= delay_ticks;
}
