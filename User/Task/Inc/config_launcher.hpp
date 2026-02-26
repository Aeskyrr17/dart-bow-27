
#include "magicmsgs.hpp"
#include <cstdint>
#define TOF_DATA_SIZE 9

#define MAX_DART_NUMBER 4 //可调试的所有飞镖数量

/**
 * @brief 飞镖发射机构FSM
 * 
 */
typedef enum
{
    IDLE = 0,
    RESETTING,          //正在复位
    // WAIT_LOADING,             //扳机已锁定，等待装弹
    // LOADING,            //正在装弹
    RETRACT_AND_LOAD,
    TENSIONING,         //正在调整弓弦松紧
    READY,              //调整完毕->弓弦保持力矩，扳机锁定&弓弦调整完毕

    FIRING,//正在发射

    HAND_CONTROL //!手动调试模式，不管霍尔的数据
}LAUNCHER_FSM_STATE;


struct Launcher_Cxt_t //todo:不知道需不需要
{
    LAUNCHER_FSM_STATE fsm_state;

    bool is_fire_done;
    bool is_load_done; //todo:
};

typedef struct 
{ 
    bool started; 
    ULONG start_tick; 

} delay_t;
