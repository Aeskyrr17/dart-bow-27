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
void Update_Dart_Index();

msg_cmd_t msg_cmd{};
msg_remoter_t remoter{};
msg_launcher_status_t launcher_status{};


//配置各飞镖offset及torque
Dart_Config_t dart_config[10] = {
{0.0f,0.0f}, //不使用0号
{0.0f,0.0f},
{0.0f,0.0f},
{0.0f,0.0f},
{0.0f,0.0f},
{0.0f,0.0f},
{0.0f,0.0f},
};  

SysCtrl_Context_t dart_context = {
    0,0,
    {1,2,3,4}
};

[[nonreturn]] void SysctrlThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *cmd_topic = om_config_topic(nullptr, "ca", "cmd", sizeof(msg_cmd_t));

    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    om_suber_t *launcherstatus_suber = om_subscribe(om_find_topic("launcherstatus",UINT32_MAX));

    // om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    // msg_ins_t ins{};
    // om_suber_t *launcher_suber = om_subscribe(om_find_topic("launcher", UINT32_MAX))




    for (;;)
    {   
        memset(&msg_cmd, 0, sizeof(msg_cmd_t));//每次循环清空cmd

        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(launcherstatus_suber,&launcher_status,false);
        // om_suber_export(ins_suber, &ins, false);


        // /* Receive and Check TOF Msg *///todo:未确定是否封装TOF相关逻辑
        // bool tof_valid = false;
        // tof_data_t *tof_raw =  reinterpret_cast<tof_data_t *>(tof_rx);
        // if (tof_raw->header[0]==0x59 && tof_raw->header[1]==0x59)
        // {
        //     uint8_t checksum = 0;
        //     for (int i = 0; i < 8; i++)
        //     {
        //         checksum += ((uint8_t*)tof_raw)[i];
        //     }
        //     if (checksum == tof_raw->check_sum)
        //     {
        //         tof_valid = true;
        //     }
        // }
        // float tof_distance = static_cast<float>(tof_raw->distance) * 1.0f; // cm
        // float tof_temp = static_cast<float>(tof_raw->temp_raw) / 8.0f - 256.0f; // °C

        Update_Dart_Index();


        //todo:切换手控模式与自动模式。还没有写龙门架相关.
        //todo:确定遥控器逻辑，考虑一下归位/上弦/发射的逻辑
        /*remoter：左下or右下-relax；左中-handcontrol；左上&右上-auto
        *左中：右下to中-调整弓弦松紧&发射架归位，上弦；右中to上-发射（松开扳机）
        */

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
 * 
 * @param remoter 
 * @param cmd 
 */
void Run_Hand_Control(msg_remoter_t* remoter, msg_cmd_t* cmd)
{
    int shot_index = dart_context.current_dart_index;
    int dart_id = dart_context.load_sequence[shot_index];

    float my_offset = dart_config[dart_id].yaw_offset;
    float my_tension = dart_config[dart_id].tension_torque;


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

/**
 * @brief 更新dart_id,利用上升沿判断上一阶段是否开火
 * 
 */
void Update_Dart_Index()
{
    if (launcher_status.is_fire_finished && !dart_context.last_fire_status)
    {
        dart_context.current_dart_index++;
    }
    dart_context.last_fire_status = launcher_status.is_fire_finished;//更新状态
}

