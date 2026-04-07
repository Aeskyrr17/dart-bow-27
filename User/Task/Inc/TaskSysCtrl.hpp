
/**
 * @brief 飞镖参数设置
 * 
 */
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

    int current_index;                      //装填seq中的index
    int current_dart_id;                    //当前飞镖id

    bool last_fire_finished;                //true表示上一次为fire状态，用于边缘检测
    int fired_count_this_open;              // 当前开门周期已发射的飞镖数

    DartLibrary()
    {
        dart[1] = {1, 0.0f,0.0f};
        dart[2] = {2, 0.0f,0.0f};
        dart[3] = {3, 0.0f,0.0f};
        dart[4] = {4, 0.0f,0.0f};
        dart[5] = {5, 0.0f,0.0f};
        dart[6] = {6, 0.0f,0.0f};
        dart[7] = {7, 0.0f,0.0f};
        dart[8] = {8, 0.0f,0.0f};
        dart[9] = {9, 0.0f,0.0f};

        sequence[0] = 1;//装填顺序
        sequence[1] = 2;
        sequence[2] = 3;
        sequence[3] = 4;

        current_index = 0;

        last_fire_finished = false; //上电时应该为未发出状态
        fired_count_this_open = 0; //todo:未限制
    }

    /**
     * @brief 根据current_index在sequence中检索出当前飞镖id
     */
    void Find_Dart_id()
    {
        this->current_dart_id = this->sequence[this->current_index];
    }


    /**
     * @brief 在循环中一直调用，检测是否发射完成,同时处理current_index的增加
     * 
     * @param msg_fire_finished TaskLauncher发送的msg
     * @todo 未测试
     */
    void Update_State(msg_launcher2sysctrl_t* msg)
    {

        if (msg->is_fire_finished && !this->last_fire_finished)
        {
            this->last_fire_finished = true;
            if (msg->is_fire_finished && !this->last_fire_finished)
            {
                this->current_index++;
                if (this->current_index >= 4) 
                {
                this->current_index = 0;
                }

            }       
            this->last_fire_finished = msg->is_fire_finished;    // 更新历史记录
        }   
    }



};