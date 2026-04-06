
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
    PREPARING,
    READY,              //调整完毕->弓弦保持力矩，扳机锁定&弓弦调整完毕
    FIRING,             //正在发射
    RESETTING,
    TENSIONING,
    RETRACT_AND_LOAD,
    HAND_CONTROL        //!手动调试模式，可控遥控器调整coil和弓弦
}LAUNCHER_FSM_STATE;

typedef enum{
    COIL_1 = 0,         //卷簧向后移动
    GANTRY_1,           //龙门架运动到相应的slot位置
    COIL_2,             //卷簧向前运动
    GANTRY_2,           //龙门架回到原点
    COIL_TRIGGER_READY, //卷簧向后运动并锁上扳机
    TENSION_AND_RETRACT,//调整弓弦张力，coil回正
}PREPARE_STSTE;


struct Launcher_Cxt_t 
{
    LAUNCHER_FSM_STATE fsm_state;
    PREPARE_STSTE prep_state;
    bool is_fire_done;  //todo:不知道需不需要
    bool is_load_done;  //todo:
};

//用于储存卷簧零点
struct CoilResetCxt_t
{
    bool homed;
    float zero_pos;
};


struct delay_t
{ 
    bool started; 
    ULONG start_tick; 
};
