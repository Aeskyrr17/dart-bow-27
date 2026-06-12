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

/**
 * @brief 用于打表的结构体，包含了每个dart在不同距离下的yaw_offset和tension_tq
 * 通过插值的方式可以得到更准确的aim参数
 */
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

/**
 * @brief 飞镖系统中需要用到的referee信息
 */
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
    bool enable;            //<是否允许进入自动模式（主要用于处理赛场遥控器可能离线或手动干预的情况）
    bool yaw_ok;            //<yaw是否已经调整到位
    bool light_lost;        //<视觉绿灯丢失
    bool running;           //<是否正在自动模式
    bool autoaim_allow;     //<是否允许进入准备状态
    bool last_autoaim_allow;//<上一次是否允许进入准备状态的
    bool referee_launch_closed_stable;  //<裁判系统发射站状态稳定在关闭的标志
    bool referee_launch_open_stable;    //<裁判系统发射站状态稳定在打开的标志
    bool vision_door_open_stable;       //<视觉系统门状态稳定在打开的标志
    bool vision_door_closed_stable;     //<视觉系统门状态稳定在关闭的标志
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
    Dart_Config_t dart[17]; //飞镖id从1-16，0号位不使用

    Dart_Base_Table_Point_t base_distance_table[128]; //打表数据，最多128条
    uint16_t base_distance_table_len;

    int sequence[4]; //发射顺序，长度为4，值为1-16的dart id

    float pre_tension;
};

/**
 * @brief 用于储存飞镖系统运行时状态的结构体
 * 
 */
struct DartRuntime
{
    struct AimTarget
    {
        float yaw_offset;
        float tension;
    };

    int current_shot_number;        //<当前是第几发，范围1-4
    int current_dart_id;            //<当前正在发射的飞镖id
    AimTarget current_aim_target;   //<当前飞镖解析出的瞄准参数

    DOOR_STATUS vision_door_status;        //<当前视觉判断门的状态
    DOOR_STATUS last_vision_door_status;   //<上一次视觉判断的门状态

    bool last_fire_finished;        //<上一次发射是否完成
    int fired_count_this_open;      //<当前门打开时已发射的飞镖数量

    uint8_t game_status_stable;     //<经过处理后的比赛状态，主要是为了处理裁判系统状态不稳定的情况
    uint8_t game_status_ladar;      //<26赛季为了处理裁判系统不稳定,解析雷达的数据来判断比赛状态，目前没有使用
    char game_status_char;          //<像视觉发送的字符串，用于开始比赛的标志，防止uint8_t类型gamestatus不稳定

    AutoAim_t autoAim;
    RefereeInfo_t referee;
};

class DartLibrary
{
public:
    static constexpr uint16_t BASE_DISTANCE_TABLE_MAX = 128;
    DartConfig config;  //<飞镖系统的配置参数
    DartRuntime runtime;//<飞镖系统的运行时状态

    DartLibrary();

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
