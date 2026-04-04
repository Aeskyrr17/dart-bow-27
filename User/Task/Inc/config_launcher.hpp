
#include "magicmsgs.hpp"
#include <cstdint>

#define TOF_DATA_SIZE 9

/**
 * @brief 飞镖发射机构FSM
 * 
 */
typedef enum
{
    IDLE = 0,
    RESETTING,          //正在复位
    RETRACT_AND_LOAD,
    TENSIONING,         //正在调整弓弦松紧
    READY,              //调整完毕->弓弦保持力矩，扳机锁定&弓弦调整完毕
    FIRING,             //正在发射

    HAND_CONTROL        //!手动调试模式，可控遥控器调整coil和弓弦
}LAUNCHER_FSM_STATE;


struct Launcher_Cxt_t 
{
    LAUNCHER_FSM_STATE fsm_state;

    bool is_fire_done;  //todo:不知道需不需要
    bool is_load_done;  //todo:
};


struct delay_t
{ 
    bool started; 
    ULONG start_tick; 
};
