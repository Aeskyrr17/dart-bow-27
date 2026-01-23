#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"
#include "config_launcher.hpp"

TX_THREAD SysctrlThread;
uint8_t SysctrlThreadStack[2048] = {0};

__attribute__((section(".RAM_D1"))) uint8_t tof_rx[TOF_DATA_SIZE];

void Run_Hand_Control(msg_remoter_t* remoter, msg_cmd_t* cmd);
void Run_Auto_Control();

[[nonreturn]] void SysctrlThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *cmd_topic = om_config_topic(nullptr, "ca", "cmd", sizeof(msg_cmd_t));
    msg_cmd_t cmd{};

    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};
    // om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    // msg_ins_t ins{};


    for (;;)
    {   
        memset(&cmd, 0, sizeof(msg_cmd_t));//每次循环清空cmd

        om_suber_export(remoter_suber, &remoter, false);
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

        //todo:切换手控模式与自动模式。还没有写龙门架相关.
        //todo:确定遥控器逻辑，考虑一下归位/上弦/发射的逻辑
        /*remoter：左下or右下-relax；左中-handcontrol；左上&右上-auto
        *左中：右下to中-调整弓弦松紧&发射架归位，上弦；右中to上-发射（松开扳机）
        */

        if (remoter.left_sw == Down)
        {
            cmd.launcher_action = DART_RELAX;
        }
        else if (remoter.left_sw == Mid)
        {
            Run_Hand_Control(&remoter, &cmd);
        }
        else if (remoter.left_sw == Up && remoter.right_sw == Up)
        {
            Run_Auto_Control();
            cmd.launcher_action = DART_RELAX; //todo:默认不发射，还未确定auto逻辑
        }
        else 
        {
            cmd.launcher_action = DART_RELAX;
        }
        om_publish(cmd_topic, &cmd, sizeof(msg_cmd_t), true, false);
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
    if (remoter->right_sw == Down)
    {
        cmd->launcher_action = DART_TENSION;
        cmd->tension_value = 0.1f; //todo:手动设定一个默认值，后续可通过遥控器调节?还是使用tension_set？还是用啥东西来计算？
    }
    else if (remoter->right_sw == Mid)
    {
        cmd->launcher_action = DART_RETRACT;
    }
    else if (remoter->right_sw == Up)
    {
        cmd->launcher_action = DART_FIRE;
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