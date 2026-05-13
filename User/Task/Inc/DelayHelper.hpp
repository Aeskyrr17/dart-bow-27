#pragma once

#include "tx_api.h"

class delay_t
{
public:
    void Reset()
    {
        started = false;
        delay_ok = false;
        start_tick = 0;
    }

    /**
     * @brief Check if the delay has been reached
     * 
     * @param delay_ticks 
     * @param delay_init 在调用时是否重置延时，默认为false
     * @return true 
     * @return false 
     */
    bool Reach(ULONG delay_ticks, bool delay_init = false)
    {
        if (delay_init)
        {
            Reset();
        }

        if (!started)
        {
            start_tick = tx_time_get();
            started = true;
        }

        delay_ok = (tx_time_get() - start_tick) >= delay_ticks;
        if (!delay_ok)
        {
            return false;
        }

        Reset();
        return true;
    }

/**
 * @brief Check if the delay has been reached based on a trigger condition
 * 
 * @param delay_trigger 触发条件，只需要一次满足就开始计时
 * @param delay_ticks 
 * @param delay_init 在调用时是否重置延时，默认为false
 * @return true 
 * @return false 
 */
    bool Reach(bool delay_trigger, ULONG delay_ticks, bool delay_init = false)
    {
        if (!started && !delay_trigger)
        {
            return false;
        }

        return Reach(delay_ticks, delay_init);
    }

    /**
     * @brief check if the delay has been reached, with an enable condition to control whether to count the delay
     * 
     * @param delay_enable 开始延时的条件，需要持续满足，如果不满足则重置延时
     * @param delay_ticks 
     * @param delay_init 在调用时是否重置延时，默认为false
     * @return true 
     * @return false 
     */
    bool ReachStable(bool delay_enable, ULONG delay_ticks, bool delay_init = false)
    {
        if (!delay_enable)
        {
            Reset();
            return false;
        }

        return Reach(delay_ticks, delay_init);
    }

    /**
     * @brief check if the delay has been reached, once the delay is reached, it will keep returning true until reset
     * 
     * @param delay_ticks 
     * @param delay_init 
     * @return true 
     * @return false 
     */
    bool ReachLatched(ULONG delay_ticks, bool delay_init = false)
    {
        if (delay_init)
        {
            Reset();
        }

        if (delay_ok)
        {
            return true;
        }

        if (!started)
        {
            start_tick = tx_time_get();
            started = true;
        }

        delay_ok = (tx_time_get() - start_tick) >= delay_ticks;
        return delay_ok;
    }

    bool ReachLatched(bool delay_trigger, ULONG delay_ticks, bool delay_init = false)
    {
        if (delay_init)
        {
            Reset();
        }

        if (delay_ok)
        {
            return true;
        }

        if (!started && !delay_trigger)
        {
            return false;
        }

        return ReachLatched(delay_ticks);
    }

    bool IsStarted() const { return started; }

private:
    bool started = false;
    bool delay_ok = false;
    ULONG start_tick = 0;
};
