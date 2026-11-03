#pragma once

#include "bsp_pwm.hpp"
#include "bsp_can.hpp"
#include <cstdint>

namespace dart::cfg
{
namespace task
{
inline constexpr uint32_t period_ticks = 1;
inline constexpr uint32_t thread_priority = 5;
inline constexpr uint32_t stack_size = 4096;
} // namespace task

namespace launcher
{
inline constexpr float syn_pos_3_rad = -40.0f;
inline constexpr float syn_pos_5_rad = 2.08f;
inline constexpr float syn_pos_deadzone_rad = 0.7f;
inline constexpr uint32_t retract_ms = 1500;
inline constexpr uint32_t latch_ms = 250;
inline constexpr uint32_t ready_ms = 750;
} // namespace launcher

namespace sysctrl
{
inline constexpr float syn_adjust_step_rad = 0.01f; // Per control cycle at full stick.
} // namespace sysctrl

namespace motor
{
inline constexpr auto syn_bus = bsp::can::handle::fdcan1;
inline constexpr auto string_bus = bsp::can::handle::fdcan3;
inline constexpr uint16_t syn_id = 0x02;
inline constexpr uint16_t syn_feedback_id = 0x06;
inline constexpr float pi = 3.14159265358979323846f;

inline constexpr uint32_t syn_feedback_timeout_ms = 100;
inline constexpr float syn_pos_kp = 10.0f;
inline constexpr float syn_pos_ki = 0.0f;
inline constexpr float syn_pos_kd = 10.0f;
inline constexpr float syn_max_speed_rad_s = 15.0f;

inline constexpr uint16_t string_accel = 3000;
inline constexpr uint16_t string_max_current_ma = 5000;
inline constexpr float string_max_rpm = 2000.0f;
inline constexpr uint32_t string_period_ms = 10;
} // namespace motor

namespace servo
{
inline constexpr auto timer = bsp::peripherals::tim1;
inline constexpr auto channel = bsp::pwm::channel::ch3;
inline constexpr float open_duty = 0.105f;
inline constexpr float idle_duty = 0.075f;
inline constexpr uint32_t period_us = 20000;
inline constexpr uint32_t hold_ms = 1000;
} // namespace servo
} // namespace dart::cfg
