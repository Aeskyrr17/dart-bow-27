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


struct msg_visionrx_t
{
    uint8_t header; // 发送数据包的头
    float distance;
    float yaw; 
    uint8_t stable_state;//0不稳定，1稳定
    uint16_t checksum; // 校验和
    
}__attribute__((packed));


struct msg_visiontx_t
{
    uint8_t header; //0x5A
    uint8_t start_state;
    uint8_t target_id; //0-outpost 1-base
    uint8_t DartNumber;//1,2,3,4
    uint8_t selected_target_id;
    uint16_t checksum;
} __attribute__((packed));


/**
 * @brief 飞镖发射指令
 */
typedef enum
{
    DART_RELAX = 0,     // 放松或急停
    DART_PREPARE= 1,    //调整yaw角度，
    DART_COIL_ADJUST = 2,    //调整coil
    DART_STRING_ADJUST = 3, //调整副弦
    DART_FIRE = 4,  // 发射
    DART_TRIGGER_OPEN,
    DART_TRIGGER_CLOSE
}LAUNCHER_ACTION;


/**
 * @brief 飞镖cmd，由TaskSysctrl发送给TaskLauncher
 */
struct msg_cmd_t
{
    LAUNCHER_ACTION action;

    float yaw;
    float tension;
    float Coil_L_spd;
    float Coil_R_spd;
    float String_L_spd;
    float String_R_spd;
};


/**
 * @brief 电机控制消息结构，由Tasklauncher发送给Taskmotors
 * yaw轴步进电机
 */
struct msg_motor_ctrl_t {
    float yaw_spd;
    float yaw_tq;
    CTRL_MODE yaw_mode;

    bool trigger_lock;

    CTRL_MODE Coil_mode;

    float Coil_L_spd;
    float Coil_R_spd;

    float Coil_L_tq;
    float Coil_R_tq;

    float String_L_spd;
    float String_R_spd;

    float String_target_tension;

    //龙门架电机
    bool gantry_reset;
    bool gantry_open;
    bool gantry_lock;

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
 */
struct msg_sensor_t
{
    bool is_coil_reset;         //卷簧是否归位(上方)
    bool is_door_open;          //舱门是否打开
    bool is_string_tight;       //暂保留，后续可能删除
    bool is_launchplat_return;  //发射台是否归位
    bool is_fire_done;          //是否发射完成，可能不需要
    bool is_dart_loaded;        //飞镖装填完毕
    float string_L_force;       //左副弦力矩
    float string_R_force;       //右副弦力矩
};

struct debug_motor_t
{
    float speed;
    float position;
    float current;
    float torque;
};

struct msg_motor2launcher_t
{
    
};

struct msg_launcher2sysctrl_t
{
    uint8_t current_state; //保留
    bool is_fire_finished; //todo:发射是否完成,由launcher逻辑判断
};

#ifdef __cplusplus
}
#endif