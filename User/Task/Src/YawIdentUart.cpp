#include "YawIdentDebug.hpp"

#include <cstdio>
#include <cstdint>

#include "tx_api.h"
#include "usart.h"

TX_THREAD YawIdentUartThread;
uint8_t YawIdentUartThreadStack[3072] = {};
volatile uint32_t yaw_ident_uart_tx_fail_count = 0;

namespace
{
constexpr ULONG kPeriodTicks = 20; // 50 Hz leaves room for the longer CSV rows.
constexpr char kHeader[] =
    "timestamp_us,sample_seq,state,trial,direction,position_mrad,"
    "motor_speed_mrad_s,gyro_yaw_mrad_s,torque_command_mNm,torque_feedback_mNm,"
    "trial_start_mrad,baseline_feedback_mNm,pre_onset_command_mNm,"
    "pre_onset_feedback_mNm,onset_command_mNm,onset_feedback_mNm,"
    "onset_position_mrad,onset_speed_mrad_s,onset_timestamp_us,onset_seq,"
    "abort_reason,motor_rx_seq,imu_status,imu_sample_seq\r\n";

bool ReadSnapshot(YawIdentDebug& dst)
{
    const volatile YawIdentDebug& src = yaw_ident_debug;
    for (unsigned attempt = 0; attempt < 3; ++attempt)
    {
        const uint32_t before = src.update_seq;
        if (before & 1U)
        {
            continue;
        }
        dst.timestamp_us = src.timestamp_us;
        dst.sample_seq = src.sample_seq;
        dst.motor_rx_seq = src.motor_rx_seq;
        dst.imu_sample_seq = src.imu_sample_seq;
        dst.state = src.state;
        dst.trial = src.trial;
        dst.direction = src.direction;
        dst.abort_reason = src.abort_reason;
        dst.onset_seq = src.onset_seq;
        dst.onset_timestamp_us = src.onset_timestamp_us;
        dst.imu_status = src.imu_status;
        dst.position_rad = src.position_rad;
        dst.gyro_yaw_rad_s = src.gyro_yaw_rad_s;
        dst.motor_speed_rad_s = src.motor_speed_rad_s;
        dst.torque_command_nm = src.torque_command_nm;
        dst.torque_feedback_nm = src.torque_feedback_nm;
        dst.trial_start_position_rad = src.trial_start_position_rad;
        dst.baseline_feedback_nm = src.baseline_feedback_nm;
        dst.pre_onset_command_nm = src.pre_onset_command_nm;
        dst.pre_onset_feedback_nm = src.pre_onset_feedback_nm;
        dst.onset_command_nm = src.onset_command_nm;
        dst.onset_feedback_nm = src.onset_feedback_nm;
        dst.onset_position_rad = src.onset_position_rad;
        dst.onset_speed_rad_s = src.onset_speed_rad_s;
        if (before == src.update_seq)
        {
            return true;
        }
    }
    return false;
}

int32_t Milli(float value)
{
    return static_cast<int32_t>(value * 1000.0f);
}
}

[[noreturn]] void YawIdentUartThreadFun(ULONG)
{
    if (HAL_UART_Transmit(&huart10, reinterpret_cast<const uint8_t*>(kHeader),
                          sizeof(kHeader) - 1U, 100) != HAL_OK)
    {
        ++yaw_ident_uart_tx_fail_count;
    }

    ULONG next = tx_time_get();
    char line[256];
    YawIdentDebug sample{};
    for (;;)
    {
        next += kPeriodTicks;
        if (ReadSnapshot(sample) && sample.sample_seq != 0U)
        {
            const int count = std::snprintf(
                line, sizeof(line),
                "%lu,%lu,%u,%u,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
                "%ld,%ld,%ld,%ld,%lu,%lu,%u,%lu,%u,%lu\r\n",
                static_cast<unsigned long>(sample.timestamp_us),
                static_cast<unsigned long>(sample.sample_seq),
                static_cast<unsigned>(sample.state),
                static_cast<unsigned>(sample.trial),
                static_cast<int>(sample.direction),
                static_cast<long>(Milli(sample.position_rad)),
                static_cast<long>(Milli(sample.motor_speed_rad_s)),
                static_cast<long>(Milli(sample.gyro_yaw_rad_s)),
                static_cast<long>(Milli(sample.torque_command_nm)),
                static_cast<long>(Milli(sample.torque_feedback_nm)),
                static_cast<long>(Milli(sample.trial_start_position_rad)),
                static_cast<long>(Milli(sample.baseline_feedback_nm)),
                static_cast<long>(Milli(sample.pre_onset_command_nm)),
                static_cast<long>(Milli(sample.pre_onset_feedback_nm)),
                static_cast<long>(Milli(sample.onset_command_nm)),
                static_cast<long>(Milli(sample.onset_feedback_nm)),
                static_cast<long>(Milli(sample.onset_position_rad)),
                static_cast<long>(Milli(sample.onset_speed_rad_s)),
                static_cast<unsigned long>(sample.onset_timestamp_us),
                static_cast<unsigned long>(sample.onset_seq),
                static_cast<unsigned>(sample.abort_reason),
                static_cast<unsigned long>(sample.motor_rx_seq),
                static_cast<unsigned>(sample.imu_status),
                static_cast<unsigned long>(sample.imu_sample_seq));
            if (count <= 0 || count >= static_cast<int>(sizeof(line)) ||
                HAL_UART_Transmit(&huart10, reinterpret_cast<uint8_t*>(line),
                                  static_cast<uint16_t>(count), 20) != HAL_OK)
            {
                ++yaw_ident_uart_tx_fail_count;
            }
        }

        const int32_t remaining = static_cast<int32_t>(next - tx_time_get());
        if (remaining > 0)
        {
            tx_thread_sleep(static_cast<ULONG>(remaining));
        }
        else
        {
            next = tx_time_get();
            tx_thread_sleep(1);
        }
    }
}
