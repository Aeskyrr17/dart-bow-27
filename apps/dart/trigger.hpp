#pragma once

#include "config.hpp"
#include "bsp_pwm.hpp"
#include <cstdint>

namespace dart::trigger
{
enum class phase : uint8_t
{
    idle,
    opening,
    resetting
};

inline phase current = phase::idle;
inline uint32_t started = 0;
inline bool completed = false;
inline bool last_release = false;
inline bool running = false;

inline bool pulse(float duty)
{
    const auto& timer = cfg::servo::timer;
    const auto channel = cfg::servo::channel;
    if (bsp::pwm::set_duty(timer.instance, channel, duty) != pnxtypes::status::ok)
    {
        return false;
    }
    if (!running)
    {
        running = bsp::pwm::start(timer.instance, channel) == pnxtypes::status::ok;
    }
    return running;
}

inline bool start()
{
    return bsp::pwm::set_period_us(cfg::servo::timer, cfg::servo::period_us) ==
               pnxtypes::status::ok &&
           pulse(cfg::servo::idle_duty);
}

inline void update(bool release, uint32_t now)
{
    const bool release_edge = release && !last_release;
    last_release = release;
    if (current == phase::idle && release_edge)
    {
        completed = false;
        if (pulse(cfg::servo::open_duty))
        {
            started = now;
            current = phase::opening;
        }
    }
    else if (current == phase::opening && now - started >= cfg::servo::hold_ms)
    {
        if (pulse(cfg::servo::idle_duty))
        {
            started = now;
            current = phase::resetting;
        }
    }
    else if (current == phase::resetting && now - started >= cfg::servo::hold_ms)
    {
        if (bsp::pwm::stop(cfg::servo::timer.instance, cfg::servo::channel) ==
            pnxtypes::status::ok)
        {
            current = phase::idle;
            running = false;
            completed = true;
        }
    }
}

inline bool done()
{
    return completed;
}
} // namespace dart::trigger
