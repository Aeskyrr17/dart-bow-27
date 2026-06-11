#pragma once

#include "DelayHelper.hpp"
#include "magicmsgs.hpp"

struct Dart_Config_t
{
    int id;
    float yaw_offset;
    float tension_tq_base;
    float tension_tq_outpost;
};

struct Dart_Base_Table_Point_t
{
    int id;
    float distance;
    float yaw_offset;
    float tension_tq;
};

struct Dart_Base_Aim_t
{
    float yaw_offset;
    float tension_tq;
};

struct RefereeInfo_t
{
    uint32_t game_status;
    uint8_t shooting_remaining_time;
    uint8_t chosen_target;
    uint8_t launch_station_status;
    uint8_t last_launch_station_status;
};

struct AutoAim_t
{
    bool enable;
    bool yaw_ok;
    bool light_lost; //视觉看不到绿灯
    bool running;
    bool autoaim_allow; //是否允许进入准备状态
    bool last_autoaim_allow;
    bool referee_launch_closed_stable;
    bool referee_launch_open_stable;
    bool vision_door_open_stable;
    bool vision_door_closed_stable;
    delay_t door_open_delay{};
    delay_t door_closed_delay{};
    delay_t referee_launch_closed_delay{};
    delay_t referee_launch_open_delay{};
    delay_t vision_door_open_delay{};
    delay_t vision_door_closed_delay{};

};
enum DOOR_STATUS
{
    // DOOR_OPENING,
    DOOR_OPEN,
    // DOOR_CLOSING,
    DOOR_CLOSED,
    // DOOR_UNKNOWN
};

struct DartConfig
{
    Dart_Config_t dart[17];

    Dart_Base_Table_Point_t base_distance_table[128];
    uint16_t base_distance_table_len;

    int sequence[4];

    float pre_tension;
};

struct DartRuntime
{
    int current_shot_number;
    int current_dart_id;

    DOOR_STATUS door_status;
    DOOR_STATUS last_door_status;

    bool last_fire_finished;
    int fired_count_this_open;

    uint8_t game_status_stable;
    uint8_t game_status_ladar; //为了处理裁判系统不稳定的问题
    char game_status_char;

    AutoAim_t autoAim;
    RefereeInfo_t referee;

    msg_visionrx_t vision_rx;//!? 暂时没有使用
};

class DartLibrary
{
public:
    static constexpr uint16_t BASE_DISTANCE_TABLE_MAX = 128;

    DartConfig config;
    DartRuntime runtime;

    DartLibrary()
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
    void Update_game_status()
        {

    }

    void UPDATE_DOOR_STATUS(msg_visionrx_t* rx)
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

    void Update_AutoAim_Prepare_Allowed()
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

    void Set_Base_Distance_Table(const Dart_Base_Table_Point_t* table, uint16_t table_len)
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

    Dart_Base_Aim_t Get_Base_Aim_By_Distance(int id, float distance) const
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


    DART_SLOT Get_Prepare_Slot() const
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

    void Update_Current_Dart_Id()
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

    void Update_Current_State(msg_launcher2sysctrl_t* msg)
    {
        if (msg->is_fire_finished && !runtime.last_fire_finished)
        {
            runtime.current_shot_number++;
            Update_Current_Dart_Id();
        }

    }

    void Update_Fired_State(msg_launcher2sysctrl_t* msg)
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

    void Update_History(msg_launcher2sysctrl_t* msg)
    {
        runtime.last_door_status = runtime.door_status;
        runtime.last_fire_finished = msg->is_fire_finished;
        runtime.autoAim.last_autoaim_allow = runtime.autoAim.autoaim_allow;
    }
};
