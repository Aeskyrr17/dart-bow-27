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
    uint8_t state; // 0=idle, 1=settle, 2=ramp, 3=coast, 4=done, 5=aborted
    uint8_t imu_status;
    uint8_t trial; // 1..6, alternating positive and negative
    int8_t direction; // +1 or -1
    uint8_t abort_reason; // 0=none, 1=angle, 2=no onset, 3=feedback, 4=start range
    uint32_t onset_seq;
    uint32_t onset_timestamp_us;
    float elapsed_s;
    float frequency_hz;
    float position_rad;
    float gyro_roll_rad_s;
    float gyro_pitch_rad_s;
    float gyro_yaw_rad_s;
    float motor_speed_rad_s;
    float torque_command_nm;
    float torque_feedback_nm;
    float trial_start_position_rad;
    float baseline_feedback_nm;
    float pre_onset_command_nm;
    float pre_onset_feedback_nm;
    float onset_command_nm;
    float onset_feedback_nm;
    float onset_position_rad;
    float onset_speed_rad_s;
};

extern volatile YawIdentDebug yaw_ident_debug;
