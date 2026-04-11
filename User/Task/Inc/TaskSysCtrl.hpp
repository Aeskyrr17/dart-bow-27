#include "magicmsgs.hpp"

struct Dart_Config_t
{
    int id;                     //飞镖编号
    float yaw_offset;           //yaw轴偏移量
    float tension_tq;       //需要的力矩
};

class DartLibrary
{
public:

    Dart_Config_t dart[10];                 //储存各飞镖参数 
    int sequence[4];                        //装填与发射顺序

    int current_shot_number;                //当前打的第几发镖
    int current_dart_id;                    //当前飞镖id

    bool last_fire_finished;                //true表示上一次为fire状态，用于边缘检测
    int fired_count_this_open;              // 当前开门周期已发射的飞镖数

    DartLibrary()
    {
        dart[1] = {1, 0.0f, 0.0f};
        dart[2] = {2, 0.0f, 0.0f};
        dart[3] = {3, 0.0f, 0.0f};
        dart[4] = {4, 0.0f, 0.0f};
        dart[5] = {5, 0.0f, 0.0f};
        dart[6] = {6, 0.0f, 0.0f};
        dart[7] = {7, 0.0f, 0.0f};
        dart[8] = {8, 0.0f, 0.0f};
        dart[9] = {9, 0.0f, 0.0f};

        sequence[0] = 1;//装填顺序
        sequence[1] = 2;
        sequence[2] = 3;
        sequence[3] = 4;

        current_shot_number = 1;
        current_dart_id = sequence[0];
        last_fire_finished = false;
        fired_count_this_open = 0;
    }

    void Update_Current_Dart_Id()
    {
        current_dart_id = sequence[current_shot_number - 1];
    }

    void Update_State(msg_launcher2sysctrl_t* msg)
    {
        if (msg->is_fire_finished && !last_fire_finished)
        {
            current_shot_number++;
            if (current_shot_number > 4)
            {
                current_shot_number = 1;
            }

            Update_Current_Dart_Id();
        }

        last_fire_finished = msg->is_fire_finished;
    }
};
