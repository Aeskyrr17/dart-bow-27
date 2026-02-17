/**
 * @file TaskSysCtrl.cpp
 * @author Aeskyrr17
 * @brief  处理手控遥控器逻辑与自动模式逻辑，发布cmd_topic
 * 
 */
#include "main.h"
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
msg_visiontx_t msg_vision_tx{};
msg_remoter_t remoter{};
msg_launcher_status_t launcher_status{};

DartLibrary dart_lib;

[[nonreturn]] void SysctrlThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *cmd_topic = om_config_topic(nullptr, "ca", "cmd", sizeof(msg_cmd_t));
   
    om_topic_t *visiontx_topic = om_config_topic(nullptr, "ca", "visiontx", sizeof(msg_visiontx_t));

    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    om_suber_t *sensor_suber = om_subscribe(om_find_topic("sensor",UINT32_MAX));
    msg_sensor_t sensor{};


    for (;;)
    {   
        memset(&msg_cmd, 0, sizeof(msg_cmd_t));//每次循环清空cmd

        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(sensor_suber, &sensor, false);

        dart_lib.Update_State(sensor.coil_reset);//todo:不知道知否可以用这个来判断


        dart_lib.Find_Dart_id();
        int id = dart_lib.current_dart_id;
        float my_offset = dart_lib.dart[id].yaw_offset;
        float my_tension = dart_lib.dart[id].tension_torque;
        float target_yaw = remoter.right_x + my_offset;

        if (remoter.left_sw == Down)
        {
            if (remoter.right_sw == Down)
            {
                msg_cmd.launcher_action = DART_RELAX;
            }
            else if (remoter.right_sw == Mid)
            {
                msg_cmd.launcher_action = DART_COIL_ADJUST;
                msg_cmd.Coil_L_spd =  - remoter.left_y * 10;   //todo:注意负号
                msg_cmd.Coil_R_spd =  - remoter.right_y * 10;
            }
            else if (remoter.right_sw == Up)
            {
                msg_cmd.launcher_action = DART_STRING_ADJUST;
                msg_cmd.String_L_spd = remoter.left_y;
                msg_cmd.String_R_spd = remoter.right_y;
            }
        }
        else if (remoter.left_sw == Mid)
        {
            if (remoter.right_sw == Down)
            {
                msg_cmd.launcher_action = DART_RELAX;
            }
            else if (remoter.right_sw == Mid) 
            {
                msg_cmd.launcher_action = DART_PREPARE;
                msg_cmd.final_target_yaw = target_yaw;
                msg_cmd.final_target_tension = my_tension;

            }
            else if (remoter.right_sw == Up)
            {
                msg_cmd.launcher_action = DART_FIRE;
                msg_cmd.final_target_yaw = target_yaw; //fire模式下yaw角度和torque都需要保持
                msg_cmd.final_target_tension = my_tension;
            }
            else 
            {
                msg_cmd.launcher_action = DART_RELAX;
            }
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
        om_publish(visiontx_topic, &msg_vision_tx, sizeof(msg_visiontx_t),true, false);
        tx_thread_sleep(1);
    };
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
    float target_yaw = remoter->right_x + my_offset;

    if (remoter->right_sw == Down)
    {
        cmd->launcher_action = DART_RELAX;
    }
    else if (remoter->right_sw == Mid) 
    {
        cmd->launcher_action = DART_PREPARE;
        cmd->final_target_yaw = target_yaw;
        cmd->final_target_tension = my_tension;


    }
    else if (remoter->right_sw == Up)
    {
        cmd->launcher_action = DART_FIRE;
        cmd->final_target_yaw = target_yaw; //fire模式下yaw角度和torque都需要保持
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

