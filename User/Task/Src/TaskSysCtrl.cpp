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

void Run_Auto_Control();

DartLibrary dart_lib;

[[nonreturn]] void SysctrlThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *cmd_topic = om_config_topic(nullptr, "ca", "cmd", sizeof(msg_cmd_t));
    msg_cmd_t cmd{};
    om_topic_t *visiontx_topic = om_config_topic(nullptr, "ca", "visiontx", sizeof(msg_visiontx_t));
    msg_visiontx_t vision_tx{};

    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};
    om_suber_t *sensor_suber = om_subscribe(om_find_topic("sensor",UINT32_MAX));
    msg_sensor_t sensor{};
    om_suber_t *lch2sys_suber = om_subscribe(om_find_topic("lch2sys",UINT32_MAX));
    msg_launcher2sysctrl_t lch2sys{};

    dart_lib.dart[1] = {1, 0.0f,280000.0f};
    dart_lib.dart[2] = {2, 0.0f,150000.0f};
    dart_lib.dart[3] = {3, 0.0f,150000.0f};
    dart_lib.dart[4] = {4, 0.0f,150000.0f};
    dart_lib.dart[5] = {5, 0.0f,5000.0f};
    dart_lib.dart[6] = {6, 0.0f,5000.0f};
    dart_lib.dart[7] = {7, 0.0f,5000.0f};
    dart_lib.dart[8] = {8, 0.0f,5000.0f};
    dart_lib.dart[9] = {9, 0.0f,5000.0f};

    dart_lib.sequence[0] = 1;
    dart_lib.sequence[1] = 2;
    dart_lib.sequence[2] = 3;
    dart_lib.sequence[3] = 4;

    float pre_tension = 10000.0f; //! 上电后relax模式默认的张力，后续考虑调整

    for (;;)
    {   
        memset(&cmd, 0, sizeof(msg_cmd_t)); //每次循环清空cmd

        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(sensor_suber, &sensor, false);
        om_suber_export(lch2sys_suber, &lch2sys, false);

        dart_lib.Find_Dart_id();
        int id = dart_lib.current_dart_id;
        float my_offset = dart_lib.dart[id].yaw_offset;
        float my_tension = dart_lib.dart[id].tension_tq;
        float target_yaw = remoter.right_x + my_offset;

        //先判断edge判断的fire
        if (remoter.left_sw == Mid && remoter.right_sw == M2U)
        {
            cmd.action = DART_FIRE;
            cmd.yaw = target_yaw;
            cmd.tension = my_tension;
        }
        //右上，手动trigger
        else if (remoter.right_sw == Up && remoter.left_sw == M2U)
        {
            cmd.action = DART_TRIGGER_OPEN;
        }
        else if (remoter.right_sw == Up && remoter.left_sw == U2M)
        {
            cmd.action = DART_TRIGGER_CLOSE;
        }
        else if (remoter.left_sw == Down)
        {
            if (remoter.right_sw == Down)
            {
                cmd.action = DART_RELAX;
            }
            else if (remoter.right_sw == Mid)
            {
                cmd.action = DART_COIL_ADJUST;
                cmd.Coil_L_spd =  - remoter.left_y * 10;
                cmd.Coil_R_spd =  - remoter.right_y * 10;
            }
            else if (remoter.right_sw == Up)
            {
                cmd.action = DART_STRING_ADJUST;
                cmd.String_L_spd = remoter.left_y;
                cmd.String_R_spd = remoter.right_y;
            }
        }
        else if (remoter.left_sw == Mid)
        {
            if (remoter.right_sw == Down)
            {
                cmd.action = DART_YAW_ADJUST;
                cmd.yaw = remoter.right_x;
            }
            else if (remoter.right_sw == Mid) 
            {
                cmd.action = DART_PREPARE;
                cmd.yaw = target_yaw;
                cmd.tension = my_tension;

            }
            else 
            {
                cmd.action = DART_RELAX;
                cmd.tension = pre_tension; //! todo: 这个pre_tension的逻辑可能需要调整
            }
        }
        else if (remoter.left_sw == Up && remoter.right_sw == Up)
        {
            Run_Auto_Control();
            cmd.action = DART_RELAX; //todo:默认不发射，auto逻辑待定
            cmd.tension = pre_tension;//! todo: 这个pre_tension的逻辑可能需要调整
        }
        else 
        {
            cmd.action = DART_RELAX;

        }

        dart_lib.Update_State(&lch2sys);

        om_publish(cmd_topic, &cmd, sizeof(msg_cmd_t), true, false);
        om_publish(visiontx_topic, &vision_tx, sizeof(msg_visiontx_t),true, false);
        tx_thread_sleep(1);
    };
}




/**
 * @brief Auto Mode
 * 
 */
void Run_Auto_Control()
{

}

