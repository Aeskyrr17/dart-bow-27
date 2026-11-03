#include "motors.hpp"
#include "config.hpp"
#include "bsp_can.hpp"
#include "dm4310.hpp"
#include "dm_manager.hpp"
#include "pid.hpp"
#include "zdt_stepper.hpp"
#include "tx_api.h"
#include <algorithm>
#include <cmath>

namespace dart::motors
{

namespace
{

using namespace dart::cfg::motor;

motor::dm::dm4310 syn{motor::dm::firmware_mode::vel}; //初始化同步带电机

motor::dm::manager dm_motors; //初始化dm电机的manager
control::pid syn_pos_pid{cfg::motor::syn_pos_kp, cfg::motor::syn_pos_ki, cfg::motor::syn_pos_kd,
    cfg::motor::syn_max_speed_rad_s, 0.0f};
zdt_stepper left{string_bus, 2, 0};
zdt_stepper right{string_bus, 1, 1};
uint32_t last_feedback = 0;
float last_position = 0.0f;
float position = 0.0f;
bool valid = false;
uint32_t last_string = 0;
uint8_t string_phase = 0;

} // namespace

bool start()
{
    using namespace bsp::can;
    syn.motor_mode = motor::mode::vel;

    if (!dm_motors.register_motor(syn, syn_bus, syn_id, syn_feedback_id) ||
        init(bsp::can::config{syn_bus, id_type::standard}) != pnxtypes::status::ok ||
        init(bsp::can::config{string_bus, id_type::extended}) != pnxtypes::status::ok)
    {
        return false;
    }

    return dm_motors.special(syn, motor::dm::special_command::enable); //发送使能
}

launcher_feedback read(uint32_t now)
{
    if (dm_motors.update_feedback(syn)) //dm电机的多圈位置反馈处理
    {
        const float raw = syn.fdb.position;
        if (!valid)
        {
            position = raw;
        }
        else
        {
            float delta = raw - last_position;
            if (delta > pi)
            {
                delta -= 2.0f * pi;
            }
            else if (delta < -pi)
            {
                delta += 2.0f * pi;
            }
            position += delta;
        }
        last_position = raw;
        valid = true;
        last_feedback = now;
    }
    if (valid && now - last_feedback >= cfg::motor::syn_feedback_timeout_ms)
    {
        valid = false;
    }
    return {position, valid, false};
}

void write(const launcher_output& output, uint32_t now)
{
    syn.cmd.velocity = 0.0f;
    if (valid && output.syn_hold)
    {
        syn_pos_pid.ref = output.syn_pos;
        syn_pos_pid.fdb = position;
        syn_pos_pid.update();
        syn.cmd.velocity = syn_pos_pid.result;
    }
    else
    {
        syn_pos_pid.clear();
    }

    dm_motors.update();
    dm_motors.transmit(now * 1000U); // ThreadX ticks are 1 ms; transmit uses microseconds.

    if (string_phase == 0 && now - last_string >= cfg::motor::string_period_ms)
    {
        const float rpm =
            std::clamp(output.string_l_spd, -1.0f, 1.0f) * cfg::motor::string_max_rpm;
        (void)left.send_speed(rpm, cfg::motor::string_accel, cfg::motor::string_max_current_ma);
        string_phase = 1;
    }
    else if (string_phase == 1 && left.flush() && !left.pending())
    {
        const float rpm =
            std::clamp(output.string_r_spd, -1.0f, 1.0f) * cfg::motor::string_max_rpm;
        (void)right.send_speed(rpm, cfg::motor::string_accel, cfg::motor::string_max_current_ma);
        string_phase = 2;
    }
    else if (string_phase == 2 && right.flush() && !right.pending())
    {
        last_string = now;
        string_phase = 0;
    }
}
} // namespace dart::motors
