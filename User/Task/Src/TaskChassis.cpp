//
// Created by ASUS on 2025/10/15.
//

#include "TaskChassis.hpp"

using namespace Filter;

TX_THREAD ChassisThread;
uint8_t ChassisThreadStack[4096] = {0};

[[noreturn]] void ChassisThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);
    PID lleg_len_pd(4700.0f, 0.0f, 700.0f, 3000.0f, 0.0f, PID_POSITION);
    PID rleg_len_pd(4700.0f, 0.0f, 700.0f, 3000.0f, 0.0f, PID_POSITION);
    IIRFilter leg_len_filter(2,LOWPASS,1);
    SLOPE leg_len_updater(0.001f,0.001f,0.18f);

    PID roll_pd(0.7f, 0.0f, 0.01f, 3.0f, 0.0f, PID_POSITION);
    SLOPE roll_updater(0.0002f,0.0002f,0.0f);

    PID yaw_pd(18.0f, 0.0f, 4.5f, 5.0f, 0.0f, PID_POSITION);
    PID yaw_dot_pd(0.8f, 0.0f, 0.6f, 4.5f, 0.0f, PID_POSITION);
    SLOPE yaw_updater(0.01f, 0.01f, 0.0f);

    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};

    for (;;)
    {

    }
}