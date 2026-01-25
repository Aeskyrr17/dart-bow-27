#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"
#include "config_launcher.hpp"
#include "config_motor.hpp"
#include "math.hpp"

TX_THREAD LauncherThread;
uint8_t LauncherThreadStack[2048] = {0};

Launcher_Context_t dart;
Loading_State loading_state;
msg_cmd_t cmd{};
msg_motor_ctrl_t msg_motorctrl{};
msg_sensors_t sensors{};
msg_launcher_status_t msg_launcher_status{};
// tof_data_t tof{};

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

        //yaw轴控制
        if (dart.current_state != IDLE )
        {
            msg_motorctrl.target_yaw = cmd.final_target_yaw;
        }
        else 
        {
            msg_motorctrl.target_yaw = 0;
        }
        
        //FSM   
        switch (dart.current_state)
        {
            case IDLE:
                msg_motorctrl.yaw_speed = 0.0f;
                msg_motorctrl.Coil_speed = 0.0f;
                msg_motorctrl.trigger_lock = true;

                // 收到prepare
                if (cmd.launcher_action == DART_PREPARE) 
                    dart.current_state = RETRACTING;
                break;

            case RETRACTING:
                msg_motorctrl.Coil_speed = 10.0f;//todo:注意正负号

                if (sensors.is_coil_reset)
                    msg_motorctrl.trigger_lock = true;
                    dart.current_state = LOCKED;
                break;

            case LOCKED:
                msg_motorctrl.Coil_speed = 0.0f;
                if (cmd.launcher_action == DART_PREPARE)
                {
                    dart.current_state = LOADING;
                }
                break;

            case LOADING:
                loading_state.ready_for_loading = true;
                if (loading_state.load_is_done)
                {
                    loading_state.ready_for_loading = false;
                    dart.current_state = TENSIONING;
                }
                break;


            case TENSIONING:
                msg_motorctrl.String_target_force = cmd.final_target_tension;
                if (sensors.is_string_tight)
                    dart.current_state = TENSIONED;
                break;

            case TENSIONED:
                msg_motorctrl.String_target_force = cmd.final_target_tension;//保持力矩

                if (cmd.launcher_action == DART_FIRE)
                {
                    if (sensors.is_door_open)
                        dart.current_state = FIRING;
                }
                if (!sensors.is_string_tight)
                {
                    dart.current_state = TENSIONING;
                }
                break;

            case FIRING:
                msg_motorctrl.trigger_lock = false;
                //todo:添加发射完成逻辑
                if (sensors.fire_done)
                {
                    msg_launcher_status.is_fire_finished = true;
                }
                break;
        }



        om_publish(motorctrl_topic, &msg_motorctrl, sizeof(msg_motor_ctrl_t), true, false);
        om_publish(launcherstatus_topic, &msg_launcher_status,sizeof(msg_launcher_status_t), true, false);
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
    msg_launcher_status.is_fire_finished = false;


}