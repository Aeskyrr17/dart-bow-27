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

    DartLibrary();

    void Update_game_status();
    void UPDATE_DOOR_STATUS(msg_visionrx_t* rx);

    void Update_AutoAim_Prepare_Allowed();

    void Set_Base_Distance_Table(const Dart_Base_Table_Point_t* table, uint16_t table_len);

    Dart_Base_Aim_t Get_Base_Aim_By_Distance(int id, float distance) const;


    DART_SLOT Get_Prepare_Slot() const;

    void Update_Current_Dart_Id();
    void Update_Current_State(msg_launcher2sysctrl_t* msg);
    void Update_Fired_State(msg_launcher2sysctrl_t* msg);
    void Update_History(msg_launcher2sysctrl_t* msg);
};
