#include "main.h"
#include "om_msg.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"
#include "TaskSysCtrl.hpp"

TX_THREAD SysctrlThread;
uint8_t SysctrlThreadStack[2048] = {0};

// __attribute__((section(".RAM_D1"))) uint8_t tof_rx[TOF_DATA_SIZE];

void Run_Hand_Control(msg_remoter_t* remoter, msg_cmd_t* cmd);
void Run_Auto_Control();

msg_cmd_t msg_cmd{};
msg_remoter_t remoter{};
msg_launcher_status_t launcher_status{};

DartLibrary dart_lib;

[[nonreturn]] void SysctrlThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *cmd_topic = om_config_topic(nullptr, "ca", "cmd", sizeof(msg_cmd_t));

    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    om_suber_t *sensor_suber = om_subscribe(om_find_topic("sensor",UINT32_MAX));
    
    for (;;)
    {   
        memset(&msg_cmd, 0, sizeof(msg_cmd_t));//每次循环清空cmd

        om_suber_export(remoter_suber, &remoter, false);

        dart_lib.Update_State(launcher_status.msg_fire_finished);

        if (remoter.left_sw == Down)
        {
            msg_cmd.launcher_action = DART_RELAX;
        }
        else if (remoter.left_sw == Mid)
        {
            Run_Hand_Control(&remoter, &msg_cmd);
        }
        else if (remoter.left_sw == Up && remoter.right_sw == Up)
        {
            Run_Auto_Control();
            msg_cmd.launcher_action = DART_RELAX; //todo:默认不发射，还未确定auto逻辑
        }
        else 
        {
            msg_cmd.launcher_action = DART_RELAX;
        }
        om_publish(cmd_topic, &msg_cmd, sizeof(msg_cmd_t), true, false);
        tx_thread_sleep(1);
    }
    
}



/**
 * @brief 用于调试的手控模式Hand Control
 * @param remoter 
 * @param cmd 
 */
void Run_Hand_Control(msg_remoter_t* remoter, msg_cmd_t* cmd)
{
    dart_lib.Find_Dart_id();
    int id = dart_lib.current_dart_id;
    float my_offset = dart_lib.dart[id].yaw_offset;
    float my_tension = dart_lib.dart[id].tension_torque;

    if (remoter->right_sw == Down)
    {
        cmd->launcher_action = DART_RELAX;
    }
    else if (remoter->right_sw == Mid) 
    {
        cmd->launcher_action = DART_PREPARE;
        cmd->final_target_yaw = remoter->right_x * 100 + my_offset;  //todo:确定摇杆灵敏度,不确定手控模式需不需要加上offset。
        cmd->final_target_tension = my_tension;

    }
    else if (remoter->right_sw == Up)
    {
        cmd->launcher_action = DART_FIRE;
        cmd->final_target_yaw = (remoter->right_x * 100.0f) + my_offset; //fire模式下yaw角度和torque都需要保持
        cmd->final_target_tension = my_tension;
    }
    else 
    {
        cmd->launcher_action = DART_RELAX;
    }
}

/**
 * @brief Auto Mode
 * 
 */
void Run_Auto_Control()
{

}

