

#include "DMMotorHandler.hpp"
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
    // TARGETING,       //正在调整yaw轴角度
    // TARGETED, 
    RESETTING,          //正在复位
    WAIT_LOADING,             //扳机已锁定，等待装弹
    LOADING,            //正在装弹
    // LOCKED,          //扳机已锁定
    TENSIONING,         //正在调整弓弦松紧
    READY,              //调整完毕->弓弦保持力矩，扳机锁定&弓弦调整完毕

    FIRING//正在发射
}LAUNCHER_FSM_STATE;


struct Launcher_Context_t
{
    LAUNCHER_FSM_STATE current_state;
    uint32_t state_start_tick; //暂未使用，用于处理FSM的延时/超时

    bool fire_is_done;
    bool load_is_done; //todo:
};




void Launcher_Init();