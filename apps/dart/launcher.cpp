#include "launcher.hpp"
#include "config.hpp"

#include <cmath>

namespace dart
{
launcher_output launcher::update(const command& cmd, const launcher_feedback& feedback,
    uint32_t now)
{
    launcher_output out{};
    if (cmd.action == action::relax)
    {
        state_ = launcher_state::idle;
        syn_adjust_active_ = false;
        return out;
    }

    if (cmd.action == action::syn_adjust)
    {
        state_ = launcher_state::idle;
        if (feedback.syn_feedback_valid)
        {
            if (!syn_adjust_active_)
            {
                syn_adjust_pos_ = feedback.syn_pos_fdb;
                syn_adjust_active_ = true;
            }
            syn_adjust_pos_ += cmd.rc_syn;
            out.syn_hold = true;
            out.syn_pos = syn_adjust_pos_;
        }
        else
        {
            syn_adjust_active_ = false;
        }
        return out;
    }
    syn_adjust_active_ = false;
    if (cmd.action == action::trigger_open || cmd.action == action::trigger_close ||
        cmd.action == action::string_adjust)
    {
        state_ = launcher_state::idle;
        if (cmd.action == action::string_adjust)
        {
            out.string_l_spd = cmd.rc_string_l;
            out.string_r_spd = cmd.rc_string_r;
        }
        out.trigger_release = cmd.action == action::trigger_open;
        return out;
    }
    if (!feedback.syn_feedback_valid)
    {
        state_ = launcher_state::idle;
        return out;
    }

    switch (state_)
    {
        case launcher_state::idle:
            idle(cmd, now);
            break;
        case launcher_state::preparing:
            preparing(feedback, out, now);
            break;
        case launcher_state::ready:
            ready(cmd, out, now);
            break;
        case launcher_state::firing:
            firing(feedback, out);
            break;
    }
    return out;
}

void launcher::idle(const command& cmd, uint32_t now)
{
    if (cmd.action == action::prepare)
    {
        state_ = launcher_state::preparing;
        prep_ = preparation::retract;
        entered_ = now;
        syn_ready_timing_ = false;
    }
}

void launcher::preparing(const launcher_feedback& feedback, launcher_output& out, uint32_t now)
{
    out.syn_hold = true;
    if (prep_ == preparation::retract)
    {
        out.syn_pos = feedback.syn_pos_fdb;
        if (now - entered_ >= cfg::launcher::retract_ms)
        {
            prep_ = preparation::syn_trigger_ready;
        }
    }
    else if (prep_ == preparation::syn_trigger_ready)
    {
        out.syn_pos = cfg::launcher::syn_pos_3_rad;
        if (std::fabs(feedback.syn_pos_fdb - out.syn_pos) <= cfg::launcher::syn_pos_deadzone_rad)
        {
            if (!syn_ready_timing_)
            {
                syn_ready_timing_ = true;
                syn_ready_since_ = now;
            }
            if (now - syn_ready_since_ >= cfg::launcher::latch_ms)
            {
                prep_ = preparation::syn_retract;
            }
        }
        else
        {
            syn_ready_timing_ = false;
        }
    }
    else
    {
        out.syn_pos = cfg::launcher::syn_pos_5_rad;
        if (std::fabs(feedback.syn_pos_fdb - out.syn_pos) <= cfg::launcher::syn_pos_deadzone_rad)
        {
            state_ = launcher_state::ready;
            entered_ = now;
        }
    }
}

void launcher::ready(const command& cmd, launcher_output& out, uint32_t now)
{
    out.syn_hold = true;
    out.syn_pos = cfg::launcher::syn_pos_5_rad;
    if (cmd.action == action::fire && now - entered_ >= cfg::launcher::ready_ms)
    {
        state_ = launcher_state::firing;
        out.trigger_release = true;
    }
}

void launcher::firing(const launcher_feedback& feedback, launcher_output& out)
{
    out.syn_hold = true;
    out.syn_pos = cfg::launcher::syn_pos_5_rad;
    if (feedback.trigger_done) //这个完成完全按软件逻辑判断的, trigger没有反馈
    {
        state_ = launcher_state::idle;
    }
}
} // namespace dart
