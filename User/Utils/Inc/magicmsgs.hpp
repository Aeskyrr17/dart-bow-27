//
// Created by cosmosmount on 2025/9/14.
//

#ifndef RM26_H7_MAGICMSG_HPP
#define RM26_H7_MAGICMSG_HPP

#include "Dr16.hpp"
#include "VT03.hpp"

/**
 * @brief Dr16遥控器消息结构
 */
struct msg_dr16_t {
    Dr16::RC_SWITCH_STATE left_sw;   ///< 左侧开关状态
    Dr16::RC_SWITCH_STATE right_sw;  ///< 右侧开关状态
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
    bool offline;
};

/**
 * @brief VT03图传遥控器消息结构
 */
struct msg_vt03_t {
    VT03::MODE_SW_STATE mode_sw;
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
};

/**
 * @brief AHRS消息结构
 */
struct msg_ahrs_t {
    float roll;   ///< 横滚角
    float pitch;  ///< 俯仰角
    float yaw;    ///< 偏航角
};

/**
 * @brief 电机控制消息结构
 */
struct msg_motor_ctrl_t {

};

/**
 * @brief 云台反馈消息结构
 */
struct msg_gimbal_fdb_t {

};

#endif //RM26_H7_MAGICMSG_HPP