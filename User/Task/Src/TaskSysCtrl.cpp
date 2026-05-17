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
TX_SEMAPHORE VisionErrorSem;

void Run_Auto_Control(const msg_visionrx_t* rx, const msg_sensor_t* sensor, DartLibrary* dart, msg_cmd_t* cmd, float tension, float yaw);
void Update_referee_data(msg_referee_t* rawdata, DartLibrary* dart);

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
    om_suber_t *referee_suber = om_subscribe(om_find_topic("referee", UINT32_MAX));
    msg_referee_t referee_pack{};
    om_suber_t *visionrx_suber = om_subscribe(om_find_topic("visionrx",UINT32_MAX));
    msg_visionrx_t vision_rx{};

    dart_lib.dart[1] = {1, -1.2f,800000.0f, 500000.0f};
    // dart_lib.dart[1] = {1, -1.9f,480000.0f};

    dart_lib.dart[2] = {2, -1.20f,800000.0f, 500000.0f};
    dart_lib.dart[3] = {3, -1.2f,800000.0f, 500000.0f};
    dart_lib.dart[4] = {4, -1.2f,792000.0f, 500000.0f};
    dart_lib.dart[5] = {5, -1.2f,810000.0f, 500000.0f};
    dart_lib.dart[6] = {6, -1.2f,805000.0f, 500000.0f};
    dart_lib.dart[7] = {7,  0.00f,820000.0f, 500000.0f};
    dart_lib.dart[8] = {8,  0.00f,690000.0f, 500000.0f};
    dart_lib.dart[9] = {9,  0.00f,690000.0f, 500000.0f};

    dart_lib.sequence[0] = 3;
    dart_lib.sequence[1] = 4;
    dart_lib.sequence[2] = 5;
    dart_lib.sequence[3] = 6;


    for (;;)
    {   
        memset(&cmd, 0, sizeof(msg_cmd_t)); //每次循环清空cmd
        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(sensor_suber, &sensor, false);
        om_suber_export(lch2sys_suber, &lch2sys, false);
        om_suber_export(visionrx_suber,&vision_rx,false);
        om_suber_export(referee_suber,&referee_pack,false);
        Update_referee_data(&referee_pack,&dart_lib);

        dart_lib.Update_Current_State(&lch2sys);
        dart_lib.is_door_open = (dart_lib.referee.launch_station_status == 0) && 
                                (vision_rx.distance > 10.0f) && (vision_rx.distance < 50.0f);
        dart_lib.Update_Current_Dart_Id();

        // //! !!!!!！！！！！！！！！！！！！！！！！!测试代码
        // vision_rx.distance = 25.0f;
        // dart_lib.referee.game_status = 4;
        // dart_lib.is_door_open = true;
        // dart_lib.referee.chosen_target = 1;
        // dart_lib.referee.chosen_target = 0; //前哨
        
        dart_lib.autoAim.light_lost = (vision_rx.distance == 666);

        //更新tension和yaw数据
        int id = dart_lib.current_dart_id;
        float my_offset = dart_lib.dart[id].yaw_offset;
        float my_tension;
        if (dart_lib.referee.chosen_target == 0) //前哨站
        {
            my_tension = dart_lib.dart[id].tension_tq_outpost;
        }
        else 
        {
            my_tension = dart_lib.dart[id].tension_tq_base;
        }
        float target_yaw = remoter.right_x;  //target_yaw是速度，这里只为手控模式提供。

        cmd.tension = my_tension;
        cmd.next_dart_slot = dart_lib.Get_Prepare_Slot();

        //处理vision_tx数据
        vision_tx.header = 0x5A;
        vision_tx.offset = my_offset;
        vision_tx.DartNumber = id;
        vision_tx.target_id = dart_lib.referee.chosen_target;
        vision_tx.start_state = dart_lib.referee.game_status;
        // vision_tx.target_id = 1;
        // vision_tx.target_id = 0; //! 前哨站

        //遥控器offline保护和visionrx数据异常的灯控提示 //?! remoteroffline 可能需要删除
        if (remoter.offline)
        {
            cmd.action = DART_RELAX;
            dart_lib.Update_Fired_State(&lch2sys);
            om_publish(cmd_topic, &cmd, sizeof(msg_cmd_t), true, false);
            om_publish(visiontx_topic, &vision_tx, sizeof(msg_visiontx_t), true, false);
            tx_thread_sleep(1);
            continue;
        }
        if (vision_rx.header != 0xA5 || vision_rx.distance == 0.0f || vision_rx.yaw == 0.0f || vision_rx.checksum == 0)
        {
            tx_semaphore_put(&VisionErrorSem);
        }


        //先判断edge判断的fire
        if (remoter.left_sw == Mid && remoter.right_sw == M2U)
        {
            cmd.action = DART_FIRE;
            cmd.yaw = target_yaw;
            cmd.tension = my_tension;
        }
        else if (remoter.left_sw == Down)
        {
            if (remoter.right_sw == Down)
            {
                cmd.action = DART_RELAX;
            }
            else if (remoter.right_sw == Mid)
            {
                cmd.action = DART_SYN_ADJUST;
                cmd.rc_syn =  remoter.right_y * 0.01f;
            }
            else if (remoter.right_sw == Up)
            {
                cmd.action = DART_STRING_ADJUST;
                cmd.rc_string_L = remoter.left_y;
                cmd.rc_string_R = remoter.right_y;
            }
        }
        else if (remoter.left_sw == Mid)
        {
            if (remoter.right_sw == Down)
            {
                cmd.action = DART_YAW_ADJUST;
                cmd.yaw = remoter.right_x;
                if (remoter.left_x > 0.7f || remoter.left_x < -0.7f)
                {
                    cmd.action = DART_TRIGGER_OPEN;
                }
                else {
                    cmd.action = DART_TRIGGER_CLOSE;
                }
            }
            else if (remoter.right_sw == Mid) 
            {
                cmd.action = DART_PREPARE;
                cmd.yaw = target_yaw;
                cmd.tension = my_tension;

            }
            else if (remoter.right_sw == Up)
            {
                cmd.action = DART_FIRE;
                cmd.yaw = target_yaw;
                cmd.tension = my_tension;
                // cmd.tension = pre_tension; //! todo: 这个pre_tension的逻辑可能需要调整
            }
        }
        else if (remoter.left_sw == Up && remoter.right_sw == Up)
        {
            if (dart_lib.referee.game_status == 4)
            {
                Run_Auto_Control(&vision_rx, &sensor, &dart_lib, &cmd, my_tension, my_offset);
            }
            else
            {
                cmd.action = DART_RELAX;
            }

        }
        else
        {
            cmd.action = DART_RELAX;

        }

        dart_lib.Update_Fired_State(&lch2sys);


        om_publish(cmd_topic, &cmd, sizeof(msg_cmd_t), true, false);
        om_publish(visiontx_topic, &vision_tx, sizeof(msg_visiontx_t),true, false);
        tx_thread_sleep(1);
    };
}




/**
 * @brief Auto Mode
 * 
 */
void Run_Auto_Control(const msg_visionrx_t* rx,const msg_sensor_t* sensor, DartLibrary* dart,  msg_cmd_t* cmd, float tension, float yaw)
{
    //比赛开始之前都不执行自动模式
    // if (dart->referee.game_status != 4)
    // {
    //     cmd->action = DART_RELAX;
    //     return;
    // }

    if (!(dart->current_shot_number >= 1 && dart->current_shot_number <= 4))
    {
        cmd->action = DART_RELAX;
        cmd->tension = tension;
        // cmd->yaw = yaw;
        cmd->yaw = 0;
        return;
    }

    cmd->action = DART_PREPARE;
    cmd->tension = tension;
    // cmd->yaw = yaw;

    dart->autoAim.yaw_ok = false;

    if      (rx->yaw > 0.08f)
    {
        cmd->yaw = 1.0f;
    }
    else if (rx->yaw < -0.08f)
    {
        cmd->yaw = -1.0f;
    }
    else if (rx->yaw > 0.05f)
    {
        cmd->yaw = 0.5f;
    }
    else if (rx->yaw < -0.05f)
    {
        cmd->yaw = -0.5f;
    }
    else if (rx->yaw > 0.015f)
    {
        cmd->yaw = 0.15f;
    }
    else if (rx->yaw < -0.015f)
    {
        cmd->yaw = -0.15f;
    }
    else if (rx->yaw <= 0.015f && rx->yaw >= -0.015f)
    {
        dart->autoAim.yaw_ok = true;
    };

    if ((dart->is_door_open &&
        dart->fired_count_this_open < 2 &&
        dart->autoAim.yaw_ok &&
        rx->stable_state == 1))
    {
        cmd->action = DART_FIRE;
    }
};


void Update_referee_data(msg_referee_t* referee_rx, DartLibrary* dart)
{
    dart->referee.last_launch_station_status = dart->referee.launch_station_status;
    //! todo 强制开始比赛
    dart->referee.game_status =  referee_rx->GameStatus.Game_progress;
    // dart->referee.game_status = 4;
    dart->referee.shooting_remaining_time =  referee_rx->DartInfo.dart_remaining_time;
    dart->referee.chosen_target = ( referee_rx->DartInfo.dart_info >> 6) & 0x07;
    dart->referee.launch_station_status =  referee_rx->DartClientCmd.dart_launch_opening_status;

    if (dart->referee.launch_station_status == 0 &&
        dart->referee.last_launch_station_status != 0)
    {
        dart->fired_count_this_open = 0;
    }
}

