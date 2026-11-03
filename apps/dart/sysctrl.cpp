#include "sysctrl.hpp"
#include "config.hpp"

namespace dart
{
command sysctrl::update(const remotes::reading& input, bool online, launcher_state state)
{
    command cmd{};
    if (!online)
    {
        return cmd;
    }

    using remotes::sw_state;
    if (input.left_sw == sw_state::low && input.right_sw == sw_state::mid)
    {
        cmd.action = action::syn_adjust;
        cmd.rc_syn = input.right_y * cfg::sysctrl::syn_adjust_step_rad;
    }
    else if (input.left_sw == sw_state::low && input.right_sw == sw_state::up)
    {
        cmd.action = action::string_adjust;
        cmd.rc_string_l = input.left_y;
        cmd.rc_string_r = input.right_y;
    }
    else if (input.left_sw == sw_state::mid && input.right_sw == sw_state::low)
    {
        cmd.action = input.left_x > 0.7f || input.left_x < -0.7f ? action::trigger_open
                                                                 : action::trigger_close;
    }
    else if (input.left_sw == sw_state::mid && input.right_sw == sw_state::mid)
    {
        cmd.action = action::prepare;
    }
    else if (input.left_sw == sw_state::mid && input.right_sw == sw_state::up)
    {
        cmd.action = action::fire;
    }
    if (state == launcher_state::firing)
    {
        cmd.action = action::prepare;
    }
    return cmd;
}
} // namespace dart
