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
enum DOOR_STATUS
{
    DOOR_OPENING,
    DOOR_OPEN,
    DOOR_CLOSING,
    DOOR_CLOSED,
    // DOOR_UNKNOWN
};
class DartLibrary
{
public:
    Dart_Config_t dart[17];
    int sequence[4];
    int current_shot_number;
    int current_dart_id;
    DOOR_STATUS door_status;
    DOOR_STATUS last_door_status;

    bool last_fire_finished;
    int fired_count_this_open;

    AutoAim_t autoAim;
    RefereeInfo_t referee;
    msg_visionrx_t vision_rx;//!? 暂时没有使用

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
        dart[10] = {10, 0.0f, 0.0f, 0.0f};
        dart[11] = {11, 0.0f, 0.0f, 0.0f};
        dart[12] = {12, 0.0f, 0.0f, 0.0f};
        dart[13] = {13, 0.0f, 0.0f, 0.0f};
        dart[14] = {14, 0.0f, 0.0f, 0.0f};
        dart[15] = {15, 0.0f, 0.0f, 0.0f}; 
        dart[16] = {16, 0.0f, 0.0f, 0.0f};

        sequence[0] = 1;
        sequence[1] = 2;
        sequence[2] = 3;
        sequence[3] = 4;

        current_shot_number = 1;
        current_dart_id = sequence[0];
        door_status = DOOR_CLOSED;
        last_door_status = DOOR_CLOSED;
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

    void UPDATE_DOOR_STATUS(msg_visionrx_t* rx)
    {
        if (referee.launch_station_status == 1)
        {
            door_status = DOOR_CLOSED;
        }
        else if (rx->distance > 10.0f && rx->distance < 50.0f && referee.launch_station_status == 0)
        {
            door_status = DOOR_OPEN;
        }
        else if (referee.launch_station_status == 2 )
        {
            if (last_door_status == DOOR_CLOSED || last_door_status == DOOR_OPENING)
            {
                door_status = DOOR_OPENING;
            }
            else if (last_door_status == DOOR_OPEN || last_door_status == DOOR_CLOSING)
            {
                door_status = DOOR_CLOSING;
            }
            else
            {
                door_status = DOOR_CLOSED;
            }

        }
  
    };


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

        if (last_door_status == DOOR_CLOSING && door_status == DOOR_CLOSED)
        {
            fired_count_this_open = 0;
        }

    }

    void Update_History(msg_launcher2sysctrl_t* msg)
    {
        last_door_status = door_status;
        last_fire_finished = msg->is_fire_finished;
    }
};
