#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    Down = 2, //Relax
    Mid = 3,  //Normal
    Up = 1,   //Spin
    D2M = 4,  //R2N
    M2D = 5,  //N2R
    M2U = 6,  //N2S
    U2M = 7   //S2N
}CTRL_STATE;

// typedef enum {
//     Closed = 2,
//     Warm = 3,
//     Fire = 1
// }SHOOT_STATE;

typedef enum {
    SPD,
    POS,
    TORQUE
}CTRL_MODE;

/**
 * @brief 遥控器消息结构
 */
struct msg_remoter_t 
{
    CTRL_STATE left_sw;
    CTRL_STATE right_sw;
    CTRL_STATE last_left_sw;
    CTRL_STATE last_right_sw;
    float left_x;
    float left_y;
    float right_x;
    float right_y;
    float mouse_x;
    float mouse_y;
    float mouse_z;
    bool mouse_left;
    bool mouse_right;
    
    struct __attribute__((packed)) {
        uint16_t W : 1;
        uint16_t S : 1;
        uint16_t A : 1;
        uint16_t D : 1;
        uint16_t SHIFT : 1;
        uint16_t CTRL : 1;
        uint16_t Q : 1;
        uint16_t E : 1;
        uint16_t R : 1;
        uint16_t F : 1;
        uint16_t G : 1;
        uint16_t Z : 1;
        uint16_t X : 1;
        uint16_t C : 1;
        uint16_t V : 1;
        uint16_t B : 1;
    } key;

    struct __attribute__((packed)) {
       uint16_t W : 1;
       uint16_t S : 1;
       uint16_t A : 1;
       uint16_t D : 1;
       uint16_t SHIFT : 1;
       uint16_t CTRL : 1;
       uint16_t Q : 1;
       uint16_t E : 1;
       uint16_t R : 1;
       uint16_t F : 1;
       uint16_t G : 1;
       uint16_t Z : 1;
       uint16_t X : 1;
       uint16_t C : 1;
       uint16_t V : 1;
       uint16_t B : 1;
    } last_key;

    bool offline;
};

/**
 * @brief AHRS消息结构
 */
struct msg_ins_t {
    float quaternion[4];    ///< 四元数
    float roll;             ///< 横滚角, deg
    float pitch;            ///< 俯仰角, deg
    float yaw;              ///< 偏航角, deg
    float total_yaw;        ///< 偏航总角度, deg
    float gyro_r;           ///< roll角速度, rad/s
    float gyro_p;           ///< pitch角速度, rad/s
    float gyro_y;           ///< yaw角速度, rad/s
    float accel[3];
};

// struct msg_solver_t
// {
//     float llen;
//     float llen_dot;
//     float rlen;
//     float rlen_dot;

//     float lphi;
//     float lphi_dot;
//     float rphi;
//     float rphi_dot;

//     float lalpha;
//     float lalpha_dot;
//     float ralpha;
//     float ralpha_dot;
    
//     float N;
// };

// struct msg_ctrl_t
// {
//     float Tl[2];
//     float Tr[2];
//     float Twl;
//     float Twr;
// };

// struct msg_odometry_t
// {
//     float x;
//     float v;
//     float a_z;
// };

// struct msg_cmd_t
// {
//     float x;
//     float v;
//     float w;
//     float dyaw;
//     float dlen;
//     float roll;
//     bool move;
//     bool ifjump;
//     bool ifflip;
// };

struct msg_visionrx_t
{
    uint8_t header;       // 发送数据包的头
    uint8_t tracking : 1; // 跟踪的颜色
    uint8_t fire : 1;     // 是否开火
    uint8_t id : 4;       // 识别的id
    uint8_t reserved : 2; // 保留位

    float pitch;
    float pitch_vel;
    float pitch_acc;
    float yaw;
    float yaw_vel;
    float yaw_acc;

    float project_x;
    float project_y;

    uint16_t checksum; // 校验和
}__attribute__((packed));

struct msg_visiontx_t
{
    uint8_t header;           // 发送数据包的头
    uint8_t detect_color : 1; // 检测到的颜色
    bool reset_tracker : 1;   // 是否重置追踪
    uint8_t set_target : 4;   // 设置目标
    uint8_t reserved : 2;     // 保留位
    float q1;                 // 四元数
    float q2;
    float q3;
    float q4;
    float gyro_yaw;
    float gyro_pitch;
    uint16_t checksum; // 校验和
} __attribute__((packed));

/**
 * @brief 飞镖发射指令
 */
typedef enum
{
    DART_RELAX = 0,     // 放松或急停
    DART_PREPARE= 1,    //调整yaw角度，发射台归位，调整副弦松紧
    DART_FIRE = 2      // 发射
}LAUNCHER_ACTION;

/**
 * @brief 龙门架动作指令
 */
//todo:补充龙门架指令
typedef enum
{
    GANRTY_IDLE = 0
}GANTRY_ACTION;

/**
 * @brief 飞镖cmd，由TaskSysctrl发送给TaskLauncher和TaskGantry
 */
struct msg_cmd_t
{
    LAUNCHER_ACTION launcher_action; // 飞镖发射指令
    GANTRY_ACTION gantry_action;     // 龙门架动作指令
    float final_target_yaw;   //期望角度（已包含offset）
    float final_target_tension; //拉力值
};


/**
 * @brief 由TaskLauncher发送给TaskSysctrl
 * 
 */
 struct msg_launcher_status_t
 {
    uint8_t current_state;
    bool msg_fire_finished; //发射是否完成,需要用传感器判断
 };



/**
 * @brief 电机控制消息结构，由Tasklauncher和TaskGantry发送给Taskmotors
 * yaw轴步进电机
 */
struct msg_motor_ctrl_t {
    float yaw_speed;
    float yaw_torque;
    float target_yaw;
    CTRL_MODE yaw_mode;

    bool trigger_lock;

    float Coil_speed;
    float Coil_torque;
    CTRL_MODE Coil_mode;

    float String_target_force;

    //todo:添加龙门架电机
};


// struct tof_data_t
// {
//     uint8_t header[2];
//     uint16_t distance;
//     uint16_t strength;
//     uint16_t temp_raw;
//     uint8_t check_sum;
// };


/**
 * @brief 储存各传感器发送的标志位
 * 
 */
struct msg_sensor_t
{
    bool is_coil_reset;//卷簧是否归位
    bool is_door_open;//舱门是否打开
    bool is_string_tight;//弦是否拉紧
    float string_L_force;//左副弦力矩
    float string_R_force;//右副弦力矩
    bool fire_done; //是否发射完成
};


#ifdef __cplusplus
}
#endif