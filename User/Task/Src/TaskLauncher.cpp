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
    om_topic_t *launcherstatus_topic = om_config_topic(nullptr, "ca", "launcherstatus", sizeof(msg_launcher_status_t));

    om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    // om_suber_t *tof_suber = om_subscribe(om_find_topic("tof", UINT32_MAX));

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

                // 收到prepare
                if (cmd.launcher_action == DART_PREPARE) 
                    launcher.current_state = RESETTING;
                break;

            case RESETTING:
                msg_motorctrl.trigger_lock = false;//扳机打开
                msg_motorctrl.Coil_speed = 10.0f;//todo:注意正负号
                if (sensor.is_coil_reset)
                    msg_motorctrl.trigger_lock = true;
                    launcher.current_state = WAIT_LOADING;
                break;

            case WAIT_LOADING:
                msg_motorctrl.trigger_lock = true;
                msg_motorctrl.Coil_speed = 0.0f;

                launcher.current_state = LOADING;
                break;

            case LOADING:
                msg_motorctrl.Coil_speed = 0.0f;
                Dart_Load();
                //todo:确保飞镖装填完毕，待定

                launcher.current_state = READY;
                break;

            case TENSIONING:
                msg_motorctrl.String_target_force = cmd.final_target_tension;
                if (sensor.is_string_tight)
                    launcher.current_state = READY;
                break;

            case READY:
                msg_motorctrl.trigger_lock = true;
                msg_motorctrl.String_target_force = cmd.final_target_tension;//保持力矩

                if (!sensor.is_string_tight)
                {
                    launcher.current_state = TENSIONING;
                }
                if (cmd.launcher_action == DART_FIRE)
                {
                    if (sensor.is_door_open)
                        launcher.current_state = FIRING;
                }
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
 * @brief 龙门架逻辑
 * 
 */
void Dart_Load()
{

}