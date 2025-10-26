//
// Created by cosmosmount on 2025/9/14.
//

#ifndef RM26_H7_MAGICMSG_HPP
#define RM26_H7_MAGICMSG_HPP

typedef enum {
    Relax = 2,
    Spin = 1,
    Normal = 3,
    R2N = 4,
    N2R = 5,
    N2S = 6,
    S2N = 7
}CTRL_STATE;

typedef enum {
    Closed = 2,
    Warm = 3,
    Fire = 1
}SHOOT_STATE;

typedef enum {
    SPD,
    POS,
    TORQUE
}CTRL_MODE;

/**
 * @brief 遥控器消息结构
 */
struct msg_remoter_t {
    CTRL_STATE ctrl_sw;             ///< Dr16左侧开关状态，VT03中间档位状态
    SHOOT_STATE shoot_sw;           ///< Dr16右侧开关状态，VT03拨轮状态
    CTRL_STATE last_ctrl_sw;             ///< Dr16左侧开关状态，VT03中间档位状态
    SHOOT_STATE last_shoot_sw;           ///< Dr16右侧开关状态，VT03拨轮状态
    float left_x;                  ///< 左侧摇杆X轴值
    float left_y;                  ///< 左侧摇杆Y轴值
    float right_x;                 ///< 右侧摇杆X轴值
    float right_y;                 ///< 右侧摇杆Y轴值
    float mouse_x;                 ///< 鼠标X轴值
    float mouse_y;                 ///< 鼠标Y轴值
    float mouse_z;                 ///< 鼠标滚轮值
    bool mouse_left;               ///< 鼠标左键状态
    bool mouse_right;              ///< 鼠标右键状态
    __PACKED_STRUCT
    {
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
    }key;
    __PACKED_STRUCT
    {
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
    }last_key;
    bool offline;
};

/**
 * @brief AHRS消息结构
 */
struct msg_ins_t {
    float quaternion[4]; ///< 四元数
    float roll;   ///< 横滚角
    float pitch;  ///< 俯仰角
    float yaw;    ///< 偏航角
    float total_yaw; ///< 偏航总角度
    float gyro_r; ///< roll角速度
    float gyro_p; ///< pitch角速度
    float gyro_y; ///< yaw角速度
    float accel[3];
};

/**
 * @brief 电机控制消息结构
 */
struct msg_chassis_ctrl_t {
    float Rhip1_torque;
    float Rhip2_torque;
    float Rwheel_torque;
    float Lhip1_torque;
    float Lhip2_torque;
    float Lwheel_torque;
};

/**
 * @brief 云台反馈消息结构
 */
struct pid_tuning_t {
    float kp;
    float ki;
    float kd;
};

struct motor_debug_t
{
    float spd_set;
    float spd_fdb;
    float pos_set;
    float pos_fdb;
    float cur_set;
    float cur_fdb;
};

struct msg_comm_t
{
    float vw;
};

struct msg_referee_t
{

};

struct msg_rod_t
{
    float leg_len;
    float leg_len_dot;
    float leg_len_dot_last;

    float leg_theta;
    float leg_theta_dot;
    float leg_theta_dot_last;
};

struct msg_torque_t
{
    float Tp;
    float F;
    float Tlwheel;
    float Trwheel;
};

#endif //RM26_H7_MAGICMSG_HPP