#pragma once

#include <stdint.h>

// MotorThread publishes one snapshot per control tick. update_seq is odd while
// fields are being written and even when a reader can use the snapshot.
struct YawIdentDebug
{
    uint32_t update_seq;
    uint32_t timestamp_us;
    uint32_t sample_seq;
    uint32_t motor_rx_seq;
    uint32_t imu_sample_seq;
    uint8_t state; // 0=idle, 1=chirp, 2=braking, 3=finished
    uint8_t imu_status;
    float elapsed_s;
    float frequency_hz;
    float position_rad;
    float gyro_roll_rad_s;
    float gyro_pitch_rad_s;
    float gyro_yaw_rad_s;
    float motor_speed_rad_s;
    float torque_command_nm;
    float torque_feedback_nm;
};

extern volatile YawIdentDebug yaw_ident_debug;
