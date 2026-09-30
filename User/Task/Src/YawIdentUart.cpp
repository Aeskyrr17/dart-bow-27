#include "YawIdentDebug.hpp"

#include <cstdio>
#include <cstdint>

#include "tx_api.h"
#include "usart.h"

TX_THREAD YawIdentUartThread;
uint8_t YawIdentUartThreadStack[2048] = {};
volatile uint32_t yaw_ident_uart_tx_fail_count = 0;

namespace
{
constexpr ULONG kPeriodTicks = 10; // 100 Hz with the 1 kHz ThreadX tick.
constexpr char kHeader[] =
    "timestamp_us,sample_seq,state,position_mrad,gyro_yaw_mrad_s,"
    "motor_speed_mrad_s,torque_command_mNm,torque_feedback_mNm,"
    "frequency_mHz,imu_status,motor_rx_seq,imu_sample_seq\r\n";

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
        dst.imu_status = src.imu_status;
        dst.frequency_hz = src.frequency_hz;
        dst.position_rad = src.position_rad;
        dst.gyro_yaw_rad_s = src.gyro_yaw_rad_s;
        dst.motor_speed_rad_s = src.motor_speed_rad_s;
        dst.torque_command_nm = src.torque_command_nm;
        dst.torque_feedback_nm = src.torque_feedback_nm;
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
    char line[192];
    YawIdentDebug sample{};
    for (;;)
    {
        next += kPeriodTicks;
        if (ReadSnapshot(sample) && sample.sample_seq != 0U)
        {
            const int count = std::snprintf(
                line, sizeof(line), "%lu,%lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u,%lu,%lu\r\n",
                static_cast<unsigned long>(sample.timestamp_us),
                static_cast<unsigned long>(sample.sample_seq),
                static_cast<unsigned>(sample.state),
                static_cast<long>(Milli(sample.position_rad)),
                static_cast<long>(Milli(sample.gyro_yaw_rad_s)),
                static_cast<long>(Milli(sample.motor_speed_rad_s)),
                static_cast<long>(Milli(sample.torque_command_nm)),
                static_cast<long>(Milli(sample.torque_feedback_nm)),
                static_cast<long>(Milli(sample.frequency_hz)),
                static_cast<unsigned>(sample.imu_status),
                static_cast<unsigned long>(sample.motor_rx_seq),
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
