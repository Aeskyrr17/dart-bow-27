#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"
#include "config_launcher.hpp"
#include "config_motor.hpp"
#include "math.hpp"

TX_THREAD LauncherThread;
uint8_t LauncherThreadStack[2048] = {0};

Launcher_Context_t launcher{};
msg_cmd_t cmd{};
msg_motor_ctrl_t msg_motorctrl{};
msg_sensor_t sensor{};

// msg_launcher_status_t msg_launcher_status{};
// tof_data_t tof{};

void Dart_Load();

[[nonreturn]] void LauncherThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *motorctrl_topic = om_config_topic(nullptr, "ca", "motorctrl", sizeof(msg_motor_ctrl_t));
    // om_topic_t *launcherstatus_topic = om_config_topic(nullptr, "ca", "launcherstatus", sizeof(msg_launcher_status_t));

    om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    // om_suber_t *tof_suber = om_subscribe(om_find_topic("tof", UINT32_MAX));

    float Coil_target_speed = 0.0f;
    //初始化
    Launcher_Init();

    for (;;) 
    {
        om_suber_export(cmd_suber, &cmd, false);
        // om_suber_export(tof_suber, &tof, false);

        msg_motorctrl.trigger_lock = true; //默认情况锁死扳机，只在特定状态中解锁
        msg_motorctrl.Coil_speed = 0.0f;

        //yaw轴控制,独立于发射逻辑
        if (launcher.current_state != IDLE )
        {
            msg_motorctrl.target_yaw = cmd.final_target_yaw;
        }
        else 
        {
            msg_motorctrl.target_yaw = 0.0f;
        }

        //FSM   
        switch (launcher.current_state)
        {
            case IDLE:
                msg_motorctrl.yaw_speed = 0.0f;
                msg_motorctrl.Coil_speed = 0.0f;

                if (cmd.launcher_action == DART_PREPARE) 
                    launcher.current_state = RESETTING;
                break;

            case RESETTING:
                msg_motorctrl.trigger_lock = false;//扳机打开
                msg_motorctrl.Coil_speed = Coil_target_speed;//todo:注意正负号

                if (sensor.launchplat_return)
                {
                    msg_motorctrl.trigger_lock = true;
                    msg_motorctrl.Coil_speed = 0.0f;
                    launcher.current_state = RETRACT_AND_LOAD;
                }

                break;

            case RETRACT_AND_LOAD:
                msg_motorctrl.trigger_lock = true; //全程锁死扳机
                
                //卷簧回拉
                if (!sensor.coil_reset) 
                {
                    //继续后退
                    msg_motorctrl.Coil_speed = - Coil_target_speed; 
                }
                else 
                {
                    //卷簧退到位了
                    msg_motorctrl.Coil_speed = 0.0f;
                }

                Dart_Load(); 
                
                //todo:飞镖装填

                // 两个任务都完成了，才能进入READY
                if (sensor.coil_reset && sensor.dart_loaded)
                {
                    launcher.current_state = READY;
                };
                break;  

            case READY:
                msg_motorctrl.trigger_lock = true;
                msg_motorctrl.String_target_tension = cmd.final_target_tension;//保持力矩

                if (!sensor.string_tight)
                {
                    launcher.current_state = TENSIONING;
                }
                if (cmd.launcher_action == DART_FIRE)
                {
                    if (sensor.door_open)
                        launcher.current_state = FIRING;
                }
                break;

            case TENSIONING:
                msg_motorctrl.String_target_tension = cmd.final_target_tension;
                if (sensor.string_tight)
                    launcher.current_state = READY;
                break;


            case FIRING:
                msg_motorctrl.trigger_lock = false; //解锁扳机
                if (sensor.fire_done)
                {
                    launcher.current_state = RESETTING;
                }
                break;
        }


        om_publish(motorctrl_topic, &msg_motorctrl, sizeof(msg_motor_ctrl_t), true, false);
        tx_thread_sleep(1);
    }

}
//todo:完善init
void Launcher_Init()
{
    msg_motorctrl.yaw_mode = SPD;
    msg_motorctrl.Coil_mode = SPD;
    msg_motorctrl.Coil_speed = 0.0f;
    msg_motorctrl.yaw_speed = 0.0f;
    msg_motorctrl.trigger_lock = true;
}

/**
 * @brief 龙门架逻辑，手动装填测试版本
 * 
 */
void Dart_Load()
{
    // 定义静态变量来记录时间，static变量在函数退出后值依然保留
    static uint32_t start_time = 0;
    static bool is_loading = false;

    // 如果不在装填状态，重置标志位，直接返回
    if (launcher.current_state != RETRACT_AND_LOAD)
    {
        is_loading = false;
        sensor.dart_loaded = false; // 确保非装填状态下标志位为假
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
    if (current_time - start_time > 5000) 
    {
        // 3秒时间到，假装装填完毕
        sensor.dart_loaded = true;
    }
    else
    {
        // 时间没到，还在装填中
        sensor.dart_loaded = false;
    }
}