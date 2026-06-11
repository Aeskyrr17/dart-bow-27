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

msg_remoter_t remoter{};
msg_cmd_t cmd{};

struct AimTarget
{
    float yaw_offset;
    float tension;
};

AimTarget current_aim_target{};

#define FORCE_TABLING
const bool auto_aim_on_power_up = false;
//! true: 直接run autoAim直到遥控器干预。
//! false: 先用遥控器remoter Up/Up来开启autoAim并latch
//! 测试的时候用dart_lib里面的固定值    

DartLibrary::DartLibrary()
{
    config.dart[0] = {0, 0.0f, 0.0f, 0.0f};
    config.dart[1] = {1, 0.0f, 0.0f, 0.0f};
    config.dart[2] = {2, 0.0f, 0.0f, 0.0f};
    config.dart[3] = {3, 0.0f, 0.0f, 0.0f};
    config.dart[4] = {4, 0.0f, 0.0f, 0.0f};
    config.dart[5] = {5, 0.0f, 0.0f, 0.0f};
    config.dart[6] = {6, 0.0f, 0.0f, 0.0f};
    config.dart[7] = {7, 0.0f, 0.0f, 0.0f};
    config.dart[8] = {8, 0.0f, 0.0f, 0.0f};
    config.dart[9] = {9, 0.0f, 0.0f, 0.0f};
    config.dart[10] = {10, 0.0f, 0.0f, 0.0f};
    config.dart[11] = {11, 0.0f, 0.0f, 0.0f};
    config.dart[12] = {12, 0.0f, 0.0f, 0.0f};
    config.dart[13] = {13, 0.0f, 0.0f, 0.0f};
    config.dart[14] = {14, 0.0f, 0.0f, 0.0f};
    config.dart[15] = {15, 0.0f, 0.0f, 0.0f}; 
    config.dart[16] = {16, 0.0f, 0.0f, 0.0f};

    config.base_distance_table_len = 0;
    config.sequence[0] = 1;
    config.sequence[1] = 2;
    config.sequence[2] = 3;
    config.sequence[3] = 4;
    config.pre_tension = 0.0f;

    runtime.current_shot_number = 1;
    runtime.current_dart_id = config.sequence[0];
    runtime.door_status = DOOR_CLOSED;
    runtime.last_door_status = DOOR_CLOSED;
    runtime.last_fire_finished = false;
    runtime.fired_count_this_open = 0;

    runtime.game_status_ladar = false;
    runtime.game_status_char = '\0';

    runtime.referee.game_status = 0;
    runtime.referee.shooting_remaining_time = 0;
    runtime.referee.chosen_target = 0;
    runtime.referee.launch_station_status = 1;
    runtime.referee.last_launch_station_status = 1;

    runtime.autoAim.enable = false;
    runtime.autoAim.yaw_ok = false;
    runtime.autoAim.light_lost = false;
    runtime.autoAim.running = false;
    runtime.autoAim.autoaim_allow = false;
    runtime.autoAim.last_autoaim_allow = false;
    runtime.autoAim.referee_launch_closed_stable = false;
    runtime.autoAim.referee_launch_open_stable = false;
    runtime.autoAim.vision_door_closed_stable = false;
    runtime.autoAim.vision_door_open_stable = false;
}

// void UPDATE_DOOR_STATUS(msg_visionrx_t* rx)
// {
//     if (referee.launch_station_status == 1)
//     {
//         door_status = DOOR_CLOSED;
//     }
//     else if (rx->distance > 10.0f && rx->distance < 50.0f && referee.launch_station_status == 0)
//     {
//         door_status = DOOR_OPEN;
//     }
//     else if (referee.launch_station_status == 2 )
//     {
//         if (last_door_status == DOOR_CLOSED || last_door_status == DOOR_OPENING)
//         {
//             door_status = DOOR_OPENING;
//         }
//         else if (last_door_status == DOOR_OPEN || last_door_status == DOOR_CLOSING)
//         {
//             door_status = DOOR_CLOSING;
//         }
//         else
//         {
//             door_status = DOOR_CLOSED;
//         }

//     }

// };
void DartLibrary::Update_game_status()
{
}

void DartLibrary::UPDATE_DOOR_STATUS(msg_visionrx_t* rx)
{
    if (rx->light_detected == 1 || rx->light_detected == 2)
    {
        runtime.door_status = DOOR_OPEN;
    }
    else if (rx->light_detected == 0 || rx->light_detected == 3 ) 
    {
        runtime.door_status = DOOR_CLOSED;
    }

    // if (autoAim.vision_door_open_delay.ReachStable(door_status == DOOR_OPEN, 10))
    // {
    //     autoAim.vision_door_open_stable = true;
    // }
    // else{
    //     autoAim.vision_door_open_stable = false;
    // }
    // Direct open-state judgement disabled; use stable latch below.
    // if (door_status == DOOR_OPEN)
    // {
    //     autoAim.vision_door_open_stable = true;
    // }
    // else
    // {
    //     autoAim.vision_door_open_stable = false;
    // }

    if (runtime.door_status != DOOR_OPEN)
    {
        runtime.autoAim.vision_door_open_stable = false;
        runtime.autoAim.vision_door_open_delay.Reset();
    }
    else if (!runtime.autoAim.vision_door_open_stable &&
             runtime.autoAim.vision_door_open_delay.ReachStable(true, 10))
    {
        runtime.autoAim.vision_door_open_stable = true;
    }


    if (runtime.door_status != DOOR_CLOSED)
    {
        runtime.autoAim.vision_door_closed_stable = false;
        runtime.autoAim.vision_door_closed_delay.Reset();
    }
    else if (!runtime.autoAim.vision_door_closed_stable &&
             runtime.autoAim.vision_door_closed_delay.ReachStable(true, 700))
    {
        runtime.autoAim.vision_door_closed_stable = true;
    }

    if (runtime.referee.launch_station_status != 0)
    {
        runtime.autoAim.referee_launch_open_stable = false;
        runtime.autoAim.referee_launch_open_delay.Reset();
    }
    else if (!runtime.autoAim.referee_launch_open_stable &&
             runtime.autoAim.referee_launch_open_delay.ReachStable(true, 10))
    {
        runtime.autoAim.referee_launch_open_stable = true;
    }

    if (runtime.referee.launch_station_status != 1)
    {
        runtime.autoAim.referee_launch_closed_stable = false;
        runtime.autoAim.referee_launch_closed_delay.Reset();
    }
    else if (!runtime.autoAim.referee_launch_closed_stable &&
             runtime.autoAim.referee_launch_closed_delay.ReachStable(true, 10))
    {
        runtime.autoAim.referee_launch_closed_stable = true;
    }
}

void DartLibrary::Update_AutoAim_Prepare_Allowed()
{

    if (runtime.autoAim.door_open_delay.ReachStable(runtime.autoAim.vision_door_open_stable || runtime.autoAim.referee_launch_open_stable, 25))
    {
        if (!runtime.autoAim.autoaim_allow)
        {
            runtime.fired_count_this_open = 0;
        }
        runtime.autoAim.autoaim_allow = true;
    }
    if (runtime.autoAim.door_closed_delay.ReachStable(runtime.autoAim.referee_launch_closed_stable && runtime.autoAim.vision_door_closed_stable, 25))
    {
        runtime.autoAim.autoaim_allow = false;
    }

    // bool vision_door_closed = door_status == DOOR_CLOSED;
    // if (!vision_door_closed)
    // {
    //     autoAim.vision_door_closed_stable = false;
    //     autoAim.vision_door_closed_delay.Reset();
    // }
    // else if (!autoAim.vision_door_closed_stable &&
    //          autoAim.vision_door_closed_delay.ReachStable(true, 10))
    // {
    //     autoAim.vision_door_closed_stable = true;
    // }

    // if (autoAim.vision_door_closed_stable)
    // {
    //     autoAim.autoaim_allow = false;
    // }

    
}

void DartLibrary::Set_Base_Distance_Table(const Dart_Base_Table_Point_t* table, uint16_t table_len)
{
    config.base_distance_table_len = table_len;
    if (config.base_distance_table_len > BASE_DISTANCE_TABLE_MAX)
    {
        config.base_distance_table_len = BASE_DISTANCE_TABLE_MAX;
    }

    for (uint16_t i = 0; i < config.base_distance_table_len; i++)
    {
        config.base_distance_table[i] = table[i];
    }
}

Dart_Base_Aim_t DartLibrary::Get_Base_Aim_By_Distance(int id, float distance) const
{
    int dart_id = (id >= 1 && id <= 16) ? id : 0;
    Dart_Base_Aim_t aim = {config.dart[dart_id].yaw_offset, config.dart[dart_id].tension_tq_base};
    if (dart_id == 0 || distance <= 0.0f || config.base_distance_table_len == 0)
    {
        return aim;
    }

    const Dart_Base_Table_Point_t* lower = nullptr;
    const Dart_Base_Table_Point_t* upper = nullptr;

    for (uint16_t i = 0; i < config.base_distance_table_len; i++)
    {
        const Dart_Base_Table_Point_t* point = &config.base_distance_table[i];
        if (point->id != dart_id)
        {
            continue;
        }

        if (point->distance <= distance &&
            (lower == nullptr || point->distance > lower->distance))
        {
            lower = point;
        }

        if (point->distance >= distance &&
            (upper == nullptr || point->distance < upper->distance))
        {
            upper = point;
        }
    }

    if (lower == nullptr && upper == nullptr)
    {
        return aim;
    }
    if (lower == nullptr)
    {
        return {upper->yaw_offset, upper->tension_tq};
    }
    if (upper == nullptr)
    {
        return {lower->yaw_offset, lower->tension_tq};
    }
    if (upper->distance == lower->distance)
    {
        return {lower->yaw_offset, lower->tension_tq};
    }

    float k = (distance - lower->distance) / (upper->distance - lower->distance);
    aim.yaw_offset = lower->yaw_offset + (upper->yaw_offset - lower->yaw_offset) * k;
    aim.tension_tq = lower->tension_tq + (upper->tension_tq - lower->tension_tq) * k;
    return aim;
}


DART_SLOT DartLibrary::Get_Prepare_Slot() const
{
    switch (runtime.current_shot_number)
    {
        case 1:
            return DART_SLOT_NONE;
        case 2:
            return DART_SLOT_1;
        case 3:
            return DART_SLOT_2;
        case 4:
            return DART_SLOT_3;
        default:
            return DART_SLOT_NONE;
    }
}

void DartLibrary::Update_Current_Dart_Id()
{
    if (runtime.current_shot_number >= 1 && runtime.current_shot_number <= 4)
    {
        runtime.current_dart_id = config.sequence[runtime.current_shot_number - 1];
    }
    else
    {
        runtime.current_dart_id = 0;
    }
}

void DartLibrary::Update_Current_State(msg_launcher2sysctrl_t* msg)
{
    if (msg->is_fire_finished && !runtime.last_fire_finished)
    {
        runtime.current_shot_number++;
        Update_Current_Dart_Id();
    }

}

void DartLibrary::Update_Fired_State(msg_launcher2sysctrl_t* msg)
{
    if (msg->is_fire_finished && !runtime.last_fire_finished)
    {
        runtime.fired_count_this_open++;
    }

    // if (last_door_status == DOOR_CLOSING && door_status == DOOR_CLOSED)
    // {
    //     fired_count_this_open = 0;
    // }
    //用判断stable后的视觉逻辑来重置发射计数
    // Reset moved to the next door-open allow edge.
    // if (!autoAim.autoaim_allow && autoAim.last_autoaim_allow)
    // {
    //     fired_count_this_open = 0;
    // }
    //直接用视觉数据
    // if (last_door_status != DOOR_CLOSED && door_status == DOOR_CLOSED)
    // {
    //     fired_count_this_open = 0;
    // }

}

void DartLibrary::Update_History(msg_launcher2sysctrl_t* msg)
{
    runtime.last_door_status = runtime.door_status;
    runtime.last_fire_finished = msg->is_fire_finished;
    runtime.autoAim.last_autoaim_allow = runtime.autoAim.autoaim_allow;
}

void Init_Dart_Config(DartConfig* config)
{
    //24.08
    //
    config->dart[1] = {1, -0.4f,645000.0f, 720000.0f};//烂了
    //1p，前后散布比较大，左右还好
    config->dart[2] = {2, -0.4f,645000.0f, 720000.0f}; //烂了

    config->dart[3] = {3, -0.08f,700000.0f, 720000.0f};//2 //有跳变，偏右上//!
    config->dart[4] = {4, -0.08f,687500.0f, 720000.0f};//7中//!
    // dart_lib.dart[1] = {1, -0.4f,450000.0f, 720000.0f};//烂了
    // //1p，前后散布比较大，左右还好
    // dart_lib.dart[2] = {2, -0.4f,400000.0f, 720000.0f}; //烂了

    // dart_lib.dart[3] = {3, -0.19f,400000.0f, 720000.0f};//2 //有跳变，偏右上
    // dart_lib.dart[4] = {4, -0.18f,400000.0f, 720000.0f};//7中


    config->dart[5] = {5, -0.10f,685000.0f, 500000.0f};//4//!
    config->dart[6] = {6, -0.2f,655000.0f, 500000.0f};//先不用 碳杆长了

    config->dart[7] = {7,  -0.18f,690000.0f, 500000.0f};//3
    config->dart[8] = {8,  -0.08f,684000.0f, 500000.0f};//!

    config->dart[9] = {9,  0.00f,690000.0f, 500000.0f};
    config->dart[10] = {10, 0.00f,690000.0f, 500000.0f};
    config->dart[11] = {11, 0.00f,690000.0f, 500000.0f};
    config->dart[12] = {12, 0.00f,690000.0f, 500000.0f};
    config->dart[13] = {13, 0.00f,690000.0f, 500000.0f};
    config->dart[14] = {14, 0.00f,690000.0f, 500000.0f};
    config->dart[15] = {15, 0.00f,690000.0f, 500000.0f};
    config->dart[16] = {16, 0.00f,690000.0f, 500000.0f};

    const Dart_Base_Table_Point_t base_distance_table[] = {
        {1,  25.0f, -0.8f,  660000.0f},
        {2,  25.0f, -1.2f,  670000.0f},
        {3,  25.0f, -0.68f, 660000.0f},
        {4,  25.0f, -1.2f,  670000.0f},
        {5,  25.0f, -0.9f,  645000.0f},
        {6,  25.0f, -0.75f, 655000.0f},
        {7,  25.0f,  0.00f, 820000.0f},
        {8,  25.0f, -0.7f,  660000.0f},
        {9,  25.0f,  0.00f, 690000.0f},
        {10, 25.0f,  0.00f, 690000.0f},
        {11, 25.0f,  0.00f, 690000.0f},
        {12, 25.0f,  0.00f, 690000.0f},
        {13, 25.0f,  0.00f, 690000.0f},
        {14, 25.0f,  0.00f, 690000.0f},
        {15, 25.0f,  0.00f, 690000.0f},
        {16, 25.0f,  0.00f, 690000.0f},
    };
    config->base_distance_table_len = sizeof(base_distance_table) / sizeof(base_distance_table[0]);
    for (uint16_t i = 0; i < config->base_distance_table_len; i++)
    {
        config->base_distance_table[i] = base_distance_table[i];
    }

    config->sequence[0] = 3;
    config->sequence[1] = 4;
    config->sequence[2] = 5;
    config->sequence[3] = 8;

    config->pre_tension = 320000.0f; //调整预张紧的值
}

AimTarget Resolve_Current_Aim_Target(const DartLibrary& dart, float vision_distance)
{
    int id = dart.runtime.current_dart_id;
    AimTarget target;
    if (dart.runtime.referee.chosen_target == 0) //前哨站
    {
        target.yaw_offset = dart.config.dart[id].yaw_offset;
        target.tension = dart.config.dart[id].tension_tq_outpost;
    }
    else 
    {
#ifdef FORCE_TABLING
        target.yaw_offset = dart.config.dart[id].yaw_offset;
        target.tension = dart.config.dart[id].tension_tq_base;
#else
        Dart_Base_Aim_t base_aim = dart.Get_Base_Aim_By_Distance(id, vision_distance);
        target.yaw_offset = base_aim.yaw_offset;
        target.tension = base_aim.tension_tq;
#endif
    }

    return target;
}

void Build_Remoter_Command(
    const msg_remoter_t& remoter,
    float target_yaw,
    float tension,
    msg_cmd_t* cmd)
{
    if (remoter.left_sw == Mid && remoter.right_sw == M2U)
    {
        cmd->action = DART_FIRE;
        cmd->yaw = target_yaw;
        cmd->tension = tension;
    }
    else if (remoter.left_sw == Down)
    {
        if (remoter.right_sw == Down)
        {
            cmd->action = DART_RELAX;
        }
        else if (remoter.right_sw == Mid)
        {
            cmd->action = DART_SYN_ADJUST;
            cmd->rc_syn =  remoter.right_y * 0.01f;
        }
        else if (remoter.right_sw == Up)
        {
            cmd->action = DART_STRING_ADJUST;
            cmd->rc_string_L = remoter.left_y;
            cmd->rc_string_R = remoter.right_y;
        }
    }
    else if (remoter.left_sw == Mid)
    {
        if (remoter.right_sw == Down)
        {
            cmd->action = DART_YAW_ADJUST;
            cmd->yaw = remoter.right_x;
            if (remoter.left_x > 0.7f || remoter.left_x < -0.7f)
            {
                cmd->action = DART_TRIGGER_OPEN;
            }
            else {
                cmd->action = DART_TRIGGER_CLOSE;
            }
        }
        else if (remoter.right_sw == Mid) 
        {
            cmd->action = DART_PREPARE;
            cmd->yaw = target_yaw;
            cmd->tension = tension;

        }
        else if (remoter.right_sw == Up)
        {
            cmd->action = DART_FIRE;
            cmd->yaw = target_yaw;
            cmd->tension = tension;
            // cmd.tension = pre_tension; //! todo: 这个pre_tension的逻辑可能需要调整
        }
    }
    else
    {
        cmd->action = DART_RELAX;

    }
}

[[nonreturn]] void SysctrlThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *cmd_topic = om_config_topic(nullptr, "ca", "cmd", sizeof(msg_cmd_t));
    // msg_cmd_t cmd{};

    om_topic_t *visiontx_topic = om_config_topic(nullptr, "ca", "visiontx", sizeof(msg_visiontx_t));
    msg_visiontx_t vision_tx{};

    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    // msg_remoter_t remoter{};
    om_suber_t *sensor_suber = om_subscribe(om_find_topic("sensor",UINT32_MAX));
    msg_sensor_t sensor{};
    om_suber_t *lch2sys_suber = om_subscribe(om_find_topic("lch2sys",UINT32_MAX));
    msg_launcher2sysctrl_t lch2sys{};
    om_suber_t *referee_suber = om_subscribe(om_find_topic("referee", UINT32_MAX));
    msg_referee_t referee_pack{};
    om_suber_t *visionrx_suber = om_subscribe(om_find_topic("visionrx",UINT32_MAX));
    msg_visionrx_t vision_rx{};

    Init_Dart_Config(&dart_lib.config);
    cmd.pre_tension = dart_lib.config.pre_tension;
    

    for (;;)
    {
        memset(&cmd, 0, sizeof(msg_cmd_t)); //每次循环清空cmd
        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(sensor_suber, &sensor, false);
        om_suber_export(lch2sys_suber, &lch2sys, false);
        om_suber_export(visionrx_suber,&vision_rx,false);
        om_suber_export(referee_suber,&referee_pack,false);
        Update_referee_data(&referee_pack,&dart_lib);

        // if (vision_rx.distance > 15.0f && vision_rx.distance < 50.0f && vision_rx.light_detected)
        // {
        //     dart_lib.runtime.door_status = DOOR_OPEN;
        // }
        // else 
        // {
        //     dart_lib.runtime.door_status = DOOR_CLOSED;
        
        // }
        dart_lib.Update_Current_State(&lch2sys);
        dart_lib.UPDATE_DOOR_STATUS(&vision_rx);
        dart_lib.Update_AutoAim_Prepare_Allowed();
        dart_lib.runtime.autoAim.running = false;
        dart_lib.Update_Fired_State(&lch2sys);
        dart_lib.Update_Current_Dart_Id();

        const bool autoAim_request = (!remoter.offline && remoter.left_sw == Up && remoter.right_sw == Up);
        bool autoAim_control = false;
        if (auto_aim_on_power_up)
        {
            const bool remoter_intervention = (!remoter.offline && !autoAim_request);
            dart_lib.runtime.autoAim.enable = !remoter_intervention;
            autoAim_control = dart_lib.runtime.autoAim.enable;
        }
        else
        {
            if (!remoter.offline)
            {
                dart_lib.runtime.autoAim.enable = autoAim_request;
            }
            autoAim_control = dart_lib.runtime.autoAim.enable && (remoter.offline || autoAim_request);
        }

        // ! !!!!!！！！！！！！！！！！！！！！！！!测试代码
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // vision_rx.distance = 25.0f;
        // dart_lib.runtime.referee.game_status = 4;
        // dart_lib.runtime.door_status = DOOR_OPEN;
        dart_lib.runtime.referee.chosen_target = 1;
        // dart_lib.runtime.referee.chosen_target = 0; //前哨
        

        //更新tension和yaw数据
        int id = dart_lib.runtime.current_dart_id;
        current_aim_target = Resolve_Current_Aim_Target(dart_lib, vision_rx.distance);
        float target_yaw = remoter.right_x;  //target_yaw是速度，这里只为手控模式提供。

        cmd.tension = current_aim_target.tension;
        cmd.next_dart_slot = dart_lib.Get_Prepare_Slot();
        cmd.current_shot_number = dart_lib.runtime.current_shot_number;

        //处理vision_tx数据
        vision_tx.header = 0x5A;
        vision_tx.offset = current_aim_target.yaw_offset;
        vision_tx.DartNumber = id;
        vision_tx.target_id = dart_lib.runtime.referee.chosen_target;

        if (dart_lib.runtime.referee.game_status == 4 || dart_lib.runtime.game_status_ladar == 1 || 
            (dart_lib.runtime.referee.shooting_remaining_time <= 30 && dart_lib.runtime.referee.shooting_remaining_time > 1))       
        {
            vision_tx.start_state = 4;
            dart_lib.runtime.game_status_stable = 4;
            vision_tx.start_state_char = 'R';
        }
        else {
            dart_lib.runtime.game_status_stable = 0;
            // vision_tx.start_state = dart_lib.runtime.referee.game_status;
            vision_tx.start_state = 0;
            vision_tx.start_state_char = '\0';
            // vision_tx.start_state_char = dart_lib.runtime.game_status_char;
        }
                //! 开比赛
        // vision_tx.start_state = 4;
    
        // vision_tx.start_state = dart_lib.runtime.referee.game_status;
        // vision_tx.target_id = 1;
        // vision_tx.target_id = 0; //! 前哨站

        //遥控器offline保护和visionrx数据异常的灯控提示 //?! remoteroffline 可能需要删除
        if (remoter.offline && !autoAim_control)
        {
            cmd.action = DART_RELAX;
            dart_lib.Update_Fired_State(&lch2sys);
            dart_lib.Update_History(&lch2sys);
            om_publish(cmd_topic, &cmd, sizeof(msg_cmd_t), true, false);
            om_publish(visiontx_topic, &vision_tx, sizeof(msg_visiontx_t), true, false);
            tx_thread_sleep(1);
            continue;
        }
        if (vision_rx.header != 0xA5 || vision_rx.distance == 0.0f ||  vision_rx.checksum == 0)
        {
            tx_semaphore_put(&VisionErrorSem);
        }

        //先判断edge判断的fire
        if (autoAim_control)
        {
            if (dart_lib.runtime.game_status_stable == 4 &&
                dart_lib.runtime.autoAim.autoaim_allow &&
                dart_lib.runtime.fired_count_this_open < 2)
            {
                Run_Auto_Control(&vision_rx, &sensor, &dart_lib, &cmd, current_aim_target.tension, current_aim_target.yaw_offset);
            }
            else
            {
                cmd.action = DART_PRE_TENSION;
                cmd.tension = dart_lib.config.pre_tension;
            }
        }
        else
        {
            Build_Remoter_Command(remoter, target_yaw, current_aim_target.tension, &cmd);
        }

        // dart_lib.Update_Fired_State(&lch2sys);
        dart_lib.Update_History(&lch2sys);


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
    dart->runtime.autoAim.running = true;
    //比赛开始之前都不执行自动模式
    // if (dart->runtime.referee.game_status != 4)
    // {
    //     cmd->action = DART_RELAX;
    //     return;
    // }

    if (!(dart->runtime.current_shot_number >= 1 && dart->runtime.current_shot_number <= 4))
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

    dart->runtime.autoAim.yaw_ok = false;
    if (rx->yaw == 666)
    {
        cmd->yaw = 0;
        return;
    }

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
        dart->runtime.autoAim.yaw_ok = true;
    };

    if ((dart->runtime.door_status == DOOR_OPEN &&
        dart->runtime.fired_count_this_open < 2 &&
        dart->runtime.autoAim.yaw_ok &&
        rx->stable_state == 1))
    {
        cmd->action = DART_FIRE;
    }
};


void Update_referee_data(msg_referee_t* referee_rx, DartLibrary* dart)
{
    dart->runtime.referee.last_launch_station_status = dart->runtime.referee.launch_station_status;
    dart->runtime.referee.game_status =  referee_rx->GameStatus.Game_progress;
    // dart->runtime.referee.game_status = 4;
    dart->runtime.referee.shooting_remaining_time =  referee_rx->DartInfo.dart_remaining_time;
    dart->runtime.referee.chosen_target = ( referee_rx->DartInfo.dart_info >> 6) & 0x07;
    dart->runtime.referee.launch_station_status =  referee_rx->DartClientCmd.dart_launch_opening_status;

    // if (dart->runtime.referee.launch_station_status == 0 &&
    //     dart->runtime.referee.last_launch_station_status != 0)
    // {
    //     dart->runtime.fired_count_this_open = 0;
    // }
    const RoboInteractData_t* payload = &referee_rx->RoboInteractData;
    if (payload->data_cmd_id != 0x0201)
    {
        return;
    }

    const bool is_blue =
        payload->sender_id == BlueRadar &&
        (payload->receiver_id == BlueSentry || payload->receiver_id == BlueDart);
    const bool is_red =
        payload->sender_id == RedRadar &&
        (payload->receiver_id == RedSentry || payload->receiver_id == RedDart);
    if (!is_blue && !is_red)
    {
        return;
    }

    dart->runtime.game_status_ladar = payload->event == 1;
}

