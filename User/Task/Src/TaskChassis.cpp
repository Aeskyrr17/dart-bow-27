//
// Created by ASUS on 2025/10/15.
//

#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "pid.hpp"
#include "slope.hpp"
#include "filter.hpp"
#include "magicmsgs.hpp"

using namespace Filter;

TX_THREAD ChassisThread;
uint8_t ChassisThreadStack[4096] = {0};

[[noreturn]] void ChassisThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);
    PID lleg_len_pd(4700.0f, 0.0f, 700.0f, 3000.0f, 0.0f, PID_POSITION);
    PID rleg_len_pd(4700.0f, 0.0f, 700.0f, 3000.0f, 0.0f, PID_POSITION);
    PID roll_pd(0.7f, 0.0f, 0.01f, 3.0f, 0.0f, PID_POSITION);
    PID yaw_pd(18.0f, 0.0f, 4.5f, 5.0f, 0.0f, PID_POSITION);
    PID yaw_dot_pd(0.8f, 0.0f, 0.6f, 4.5f, 0.0f, PID_POSITION);
    IIRFilter leg_len_filter(2,LOWPASS,1);
    SLOPE leg_len_updater;

    for (;;){}
}