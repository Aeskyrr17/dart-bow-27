

#define TOF_DATA_SIZE 9

/**
 * @brief 飞镖发射机构FSM
 * 
 */
typedef enum
{
    IDLE = 1,
    TENSIONING, //正在调整弓弦松紧
    TENSIONED, //弓弦调整完毕
    RETRACTING, //正在复位
    LOCKED, //扳机已锁定
    FIRING//正在发射
}LAUNCHER_FSM_STATE;

