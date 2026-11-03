#pragma once

#include <cstdint>

namespace dart
{
enum class action : uint8_t
{
    relax,
    syn_adjust,
    string_adjust,
    trigger_open,
    trigger_close,
    prepare,
    fire
};

struct command
{
    dart::action action = action::relax;
    float rc_syn = 0.0f;
    float rc_string_l = 0.0f;
    float rc_string_r = 0.0f;
};

enum class launcher_state : uint8_t
{
    idle,
    preparing,
    ready,
    firing
};

struct launcher_feedback
{
    float syn_pos_fdb = 0.0f;
    bool syn_feedback_valid = false;
    bool trigger_done = false;
};

struct launcher_output
{
    bool syn_hold = false;
    float syn_pos = 0.0f;
    float string_l_spd = 0.0f;
    float string_r_spd = 0.0f;
    bool trigger_release = false;
};

class launcher
{
public:
    launcher_output update(const command& cmd, const launcher_feedback& feedback, uint32_t now);

    launcher_state state() const
    {
        return state_;
    }

private:
    enum class preparation : uint8_t
    {
        retract,
        syn_trigger_ready,
        syn_retract
    };

    void idle(const command& cmd, uint32_t now);
    void preparing(const launcher_feedback& feedback, launcher_output& out, uint32_t now);
    void ready(const command& cmd, launcher_output& out, uint32_t now);
    void firing(const launcher_feedback& feedback, launcher_output& out);

    launcher_state state_ = launcher_state::idle;
    preparation prep_ = preparation::retract;
    uint32_t entered_ = 0;
    uint32_t syn_ready_since_ = 0;
    bool syn_ready_timing_ = false;
    bool syn_adjust_active_ = false;
    float syn_adjust_pos_ = 0.0f;
};
} // namespace dart
