#include "magicmsgs.hpp"

struct Dart_Config_t
{
    int id;
    float yaw_offset;
    float tension_tq_base;
    float tension_tq_outpost;
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
};

class DartLibrary
{
public:
    Dart_Config_t dart[10];
    int sequence[4];
    int current_shot_number;
    int current_dart_id;
    bool is_door_open;

    bool last_fire_finished;
    int fired_count_this_open;

    RefereeInfo_t referee;
    AutoAim_t autoAim;

    DartLibrary()
    {
        dart[0] = {0, 0.0f, 0.0f, 0.0f};
        dart[1] = {1, 0.0f, 0.0f, 0.0f};
        dart[2] = {2, 0.0f, 0.0f, 0.0f};
        dart[3] = {3, 0.0f, 0.0f, 0.0f};
        dart[4] = {4, 0.0f, 0.0f, 0.0f};
        dart[5] = {5, 0.0f, 0.0f, 0.0f};
        dart[6] = {6, 0.0f, 0.0f, 0.0f};
        dart[7] = {7, 0.0f, 0.0f, 0.0f};
        dart[8] = {8, 0.0f, 0.0f, 0.0f};
        dart[9] = {9, 0.0f, 0.0f, 0.0f};

        sequence[0] = 1;
        sequence[1] = 2;
        sequence[2] = 3;
        sequence[3] = 4;

        current_shot_number = 1;
        current_dart_id = sequence[0];
        is_door_open = false;
        last_fire_finished = false;
        fired_count_this_open = 0;

        referee.game_status = 0;
        referee.shooting_remaining_time = 0;
        referee.chosen_target = 0;
        referee.launch_station_status = 1;
        referee.last_launch_station_status = 1;

        autoAim.enable = false;
        autoAim.yaw_ok = false;
    }


    DART_SLOT Get_Prepare_Slot() const
    {
        switch (current_shot_number)
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
        if (current_shot_number >= 1 && current_shot_number <= 4)
        {
            current_dart_id = sequence[current_shot_number - 1];
        }
        else
        {
            current_dart_id = 0;
        }
    }

    void Update_Current_State(msg_launcher2sysctrl_t* msg)
    {
        if (msg->is_fire_finished && !last_fire_finished)
        {
            current_shot_number++;
            Update_Current_Dart_Id();
        }

    }

    void Update_Fired_State(msg_launcher2sysctrl_t* msg)
    {
        if (msg->is_fire_finished && !last_fire_finished)
        {
            fired_count_this_open++;
        }

        last_fire_finished = msg->is_fire_finished;
    }
};
