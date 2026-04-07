/**
 * @file TaskLauncher.cpp
 * @author Aeskyrr17
 * @brief 发射状态机和控制逻辑
 * @todo void Dart_Load(msg_sensor_t* sensor);
 * @todo launcher_status_t的使用（暂未确定）
 */
#include "DJIMotorHandler.hpp"
#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"
#include "config_launcher.hpp"
#include "config_motor.hpp"
#include "math.hpp"

TX_THREAD LauncherThread;
uint8_t LauncherThreadStack[2048] = {0};

extern DJIMotorHandler*DJIMotorhandler;extern TaskMotors* motors; //todo：之后可以重新整理到taskmotor中
extern float Find_gantry_pos(DART_SLOT slot);

Launcher_Cxt_t launcher{};

void Dart_Load_Test(msg_sensor_t* sensor);
bool DelayReached(delay_t* delay, bool delay_enable, ULONG delay_ticks);

msg_motorfdb_t debug_motorfdb{};

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
    om_suber_t *motorfdb_suber = om_subscribe(om_find_topic("motorfdb", UINT32_MAX));
    msg_motorfdb_t motorfdb{};

    delay_t trig_lock_delay{};
    delay_t coil_delay{};

    CoilResetCxt_t coil_L_reset{};
    CoilResetCxt_t coil_R_reset{};
    delay_t coil_L_zero_delay{};
    delay_t coil_R_zero_delay{};

    float Coil_pull_spd = 15.0f; //卷簧速度
    float Coil_return_spd = -30.0f; //卷簧复位速度，注意方向
    float Coil_return_spd_slow = -10.0f;

    motorctrl.Coil_L_spd = 0.0f;
    motorctrl.Coil_R_spd = 0.0f;

    bool trigger_delay_ok = false;
    bool coil_delay_ok = false;

    bool hand_trigger_lock = true;

    bool is_first_dart = false; //!用于准备阶段区分第一发，第一发不需要龙门架移动
    for (;;) 
    {
        om_suber_export(cmd_suber, &cmd, false);
        om_suber_export(sensor_suber, &sensor, false);
        om_suber_export(motorfdb_suber, &motorfdb, false);

        sensor.is_coil_reset = sensor.is_coil_L_reset && sensor.is_coil_R_reset;

        //! 测试用
        sensor.is_string_tight = true;
        sensor.is_door_open = true;
        lch2sys.is_fire_finished = false;
        lch2sys.next_dart_slot = cmd.next_dart_slot;

        motorctrl.trigger_lock = true;
        motorctrl.Coil_mode = SPD;
        motorctrl.Coil_L_mode = SPD;
        motorctrl.Coil_R_mode = SPD;
        motorctrl.Coil_L_spd = 0.0f;
        motorctrl.Coil_R_spd = 0.0f;
        motorctrl.String_L_spd = 0.0f;
        motorctrl.String_R_spd = 0.0f;
        motorctrl.String_target_tension = 0.0f;
        motorctrl.gantry_target_slot = DART_SLOT_NONE;
        motorctrl.String_L_tq = cmd.tension;
        motorctrl.String_R_tq = cmd.tension;//!要确定一下一开始需要张紧到多少是由谁决定的?或者不这么写？？？
        motorctrl.String_able = false;

        //处理coilposfdb零点问题
        if (!coil_L_reset.homed){
            if (DelayReached(&coil_L_zero_delay, sensor.is_coil_L_reset, 10)){
                motorctrl.Coil_L_mode = SPD;
                motorctrl.Coil_L_spd = 0.0f;
                DJIMotorhandler->ResetMotorPosFeedback(&motors->CoilSpringMotorL);
                coil_L_reset.homed = true;
            }
        }
        else {
            motorctrl.Coil_L_mode = SPD; //? 没归零就不让动？
            motorctrl.Coil_L_spd = 0.0f;
        }
        if (!coil_R_reset.homed){
            if (DelayReached(&coil_R_zero_delay, sensor.is_coil_R_reset, 10)){
                motorctrl.Coil_R_mode = SPD;
                motorctrl.Coil_R_spd = 0.0f;
                DJIMotorhandler->ResetMotorPosFeedback(&motors->CoilSpringMotorR);
                coil_R_reset.homed = true;
            }
        }
        else {
            motorctrl.Coil_R_mode = SPD; //? 没归零就不让动？
            motorctrl.Coil_R_spd = 0.0f;
        }


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
                motorctrl.Coil_L_mode = SPD;
                motorctrl.Coil_R_mode = SPD;
                motorctrl.Coil_L_spd = 0.0f;
                motorctrl.Coil_R_spd = 0.0f;
                motorctrl.String_L_spd = 0.0f;
                motorctrl.String_R_spd = 0.0f;
                motorctrl.String_able = false;
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
                else if (cmd.action == DART_YAW_ADJUST)
                {
                    motorctrl.yaw_spd = cmd.yaw;
                }
                else if (cmd.action == DART_FIRE)
                {
                    launcher.fsm_state = FIRING;
                    break;
                }
                else if (cmd.action == DART_PREPARE)
                {
                    launcher.fsm_state = PREPARING;
                    launcher.prep_state = COIL_1;
                }
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
                motorctrl.Coil_L_mode = SPD;
                motorctrl.Coil_R_mode = SPD;
                
                motorctrl.trigger_lock = true;
                motorctrl.gantry_target_slot = DART_SLOT_NONE;//龙门架在默认位置
                motorctrl.String_able = false;

                if (cmd.action == DART_PREPARE) 
                {
                    launcher.fsm_state = PREPARING;
                    launcher.prep_state = COIL_1;
                }
                break;

            case PREPARING:

                switch (launcher.prep_state)
                {
                    case COIL_1:
                        motorctrl.Coil_mode = SPD;
                        motorctrl.Coil_L_spd = Coil_pull_spd;
                        motorctrl.Coil_R_spd = Coil_pull_spd;
                        motorctrl.Coil_L_pos = 30.0f; //todo:后期要考虑收集这些magicnumber。。。
                        motorctrl.Coil_R_pos = 30.0f;
                        if(is_first_dart)
                        {
                            launcher.prep_state = COIL_TRIGGER_READY;
                            is_first_dart = false;
                        }
                        if (Numeric::abs(motors->CoilSpringMotorL.motorFeedback.positionFdb - motorctrl.Coil_L_pos) <= 2.0f &&
                            Numeric::abs(motors->CoilSpringMotorR.motorFeedback.positionFdb - motorctrl.Coil_R_pos) <= 2.0f)
                        {
                            motorctrl.Coil_L_spd = 0.0f;
                            motorctrl.Coil_R_spd = 0.0f;
                            launcher.prep_state = GANTRY_1;
                        }
                        else 
                        {
                            if (sensor.is_launchplat_return)
                            {
                                motorctrl.Coil_L_spd = 0.0f;
                                motorctrl.Coil_R_spd = 0.0f;
                            }
                        }
                        break;
                    case GANTRY_1:
                        //! ！！！！！！！！！！！还未写完，暂时先固定龙门架位置在slot1
                        motorctrl.gantry_target_slot = DART_SLOT_1;
                        if (Numeric::abs(Find_gantry_pos(DART_SLOT_1) - motors->GantryMotor.motorFeedback.positionFdb) <= 0.05f)
                        {
                            launcher.prep_state = COIL_2;
                        };
                        break;
                    case COIL_2:
                        motorctrl.gantry_target_slot = DART_SLOT_1;
                        motorctrl.Coil_mode = SPD;
                        motorctrl.Coil_L_spd = Coil_return_spd_slow;
                        motorctrl.Coil_R_spd = Coil_return_spd_slow;
                        motorctrl.Coil_L_pos = 15.0f; 
                        motorctrl.Coil_R_pos = 15.0f;
                        if (Numeric::abs(motors->CoilSpringMotorL.motorFeedback.positionFdb - motorctrl.Coil_L_pos) <= 2.0f &&
                            Numeric::abs(motors->CoilSpringMotorR.motorFeedback.positionFdb - motorctrl.Coil_R_pos) <= 2.0f)
                        {
                            motorctrl.Coil_L_spd = 0.0f;
                            motorctrl.Coil_R_spd = 0.0f;
                            launcher.prep_state = GANTRY_2;
                        };
                        break;
                    case GANTRY_2:
                        motorctrl.gantry_target_slot = DART_SLOT_NONE;
                        if (Numeric::abs(Find_gantry_pos(DART_SLOT_NONE) - motors->GantryMotor.motorFeedback.positionFdb) <= 0.05f)
                        {
                            launcher.prep_state = COIL_TRIGGER_READY;
                        };
                        break;
                    case COIL_TRIGGER_READY:
                        motorctrl.Coil_mode = SPD;
                        motorctrl.trigger_lock = false;//扳机打开

                        motorctrl.Coil_L_spd = Coil_pull_spd; 
                        motorctrl.Coil_R_spd = Coil_pull_spd;

                        //根据sensor.is_launchplat_return判断发射台是否已经回位，进入延时保证卷簧完全停止后再锁定扳机
                        coil_delay_ok = DelayReached(&coil_delay, sensor.is_launchplat_return, 0);
                        trigger_delay_ok = DelayReached(&trig_lock_delay, sensor.is_launchplat_return, 1000);

                        if (coil_delay_ok)
                        {
                            motorctrl.Coil_L_spd = 0.0f; 
                            motorctrl.Coil_R_spd = 0.0f;
                            motorctrl.trigger_lock = true;


                            coil_delay_ok = false;
                        };

                        if (trigger_delay_ok)
                        {
                            launcher.prep_state = TENSION_AND_RETRACT;
                            trigger_delay_ok = false;
                        }
                        break;
                    case TENSION_AND_RETRACT:
                    {
                        motorctrl.Coil_mode = SPD;
                        motorctrl.trigger_lock = true;
                        motorctrl.String_target_tension = cmd.tension;
                        motorctrl.String_able = true;

                        bool string_L_ok = Numeric::abs(sensor.string_L_force - cmd.tension) <= 50.0f;
                        bool string_R_ok = Numeric::abs(sensor.string_R_force - cmd.tension) <= 50.0f;


                        if (string_L_ok && string_R_ok && sensor.is_coil_reset)
                        {
                            launcher.fsm_state = READY;
                        }
                        else
                        {
                            if (!sensor.is_coil_reset) 
                            {
                                motorctrl.Coil_L_spd = Coil_return_spd; 
                                motorctrl.Coil_R_spd = Coil_return_spd;
                            }

                            if (!string_L_ok || !string_R_ok)
                            {
                                motorctrl.String_target_tension = cmd.tension;
                            }
                        }                           
                        break;
                    }
                    default:
                        launcher.fsm_state = IDLE;
                        break;                               
                }
                break;

            // case RESETTING:
            //     motorctrl.trigger_lock = false;//扳机打开

            //     motorctrl.Coil_L_spd = Coil_pull_spd; 
            //     motorctrl.Coil_R_spd = Coil_pull_spd;

            //     //!测试：：：
            //     sensor.is_launchplat_return = true;

            //     //根据sensor.is_launchplat_return判断发射台是否已经回位，进入延时保证卷簧完全停止后再锁定扳机
            //     coil_delay_ok = DelayReached(&coil_delay, sensor.is_launchplat_return, 0);
            //     trigger_delay_ok = DelayReached(&trig_lock_delay, sensor.is_launchplat_return, 1000);
            //     if (coil_delay_ok)
            //     {
            //         motorctrl.trigger_lock = true;

            //         motorctrl.Coil_L_spd = 0.0f; 
            //         motorctrl.Coil_R_spd = 0.0f;
            //         coil_delay_ok = false;
            //     };

            //     if (trigger_delay_ok)
            //     {
            //         launcher.fsm_state = RETRACT_AND_LOAD;
                    
            //         trigger_delay_ok = false;
            //     }
            //     break;

            // case RETRACT_AND_LOAD:
            //     motorctrl.trigger_lock = true; 
                
            //     if (!sensor.is_coil_reset) 
            //     {
            //         motorctrl.Coil_L_spd = Coil_return_spd; 
            //         motorctrl.Coil_R_spd = Coil_return_spd;
            //     }
            //     else 
            //     {
            //         motorctrl.Coil_L_spd = 0.0f;
            //         motorctrl.Coil_R_spd = 0.0f;
            //     }

            //     Dart_Load_Test(&sensor); 
                
            //     //todo:飞镖装填

            //     //!测试！！！
            //     sensor.is_coil_reset = true;

            //     // 两个任务都完成了，才能进入READY
            //     if (sensor.is_coil_reset && sensor.is_dart_loaded)
            //     {
            //         launcher.fsm_state = READY;
            //     };
            //     break;  

            case READY:
                motorctrl.trigger_lock = true;
                motorctrl.String_able = true;
                motorctrl.String_target_tension = cmd.tension;//保持力矩

                // if (!sensor.is_string_tight)
                // {
                //     launcher.fsm_state = TENSIONING;
                // }
                if (cmd.action == DART_FIRE &&
                     sensor.is_string_tight && sensor.is_door_open)
                {
                    if (sensor.is_door_open)
                        launcher.fsm_state = FIRING;
                }
                break;

            // case TENSIONING:
            //     motorctrl.String_target_tension = cmd.tension;
            //     if (sensor.is_string_tight)
            //         launcher.fsm_state = READY;
            //     break;


            case FIRING:
                motorctrl.String_able = false;
                motorctrl.trigger_lock = false; //解锁扳机
                if (sensor.is_fire_done)//todo：需修改
                {
                    launcher.fsm_state = PREPARING;
                    launcher.prep_state = COIL_1;
                    //! 测试逻辑
                    lch2sys.is_fire_finished = true;
                    // launcher.fsm_state = IDLE;
                }
                break;
        }

        lch2sys.current_state = launcher.fsm_state;
        om_publish(lch2sys_topic, &lch2sys, sizeof(msg_launcher2sysctrl_t), true, false);
        om_publish(motorctrl_topic, &motorctrl, sizeof(msg_motor_ctrl_t), true, false);
        memcpy(&debug_motorfdb, &motorfdb, sizeof(motorfdb));
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
