//
// Created by ASUS on 2025/10/15.
//

#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "pid.hpp"
#include "filter.hpp"
#include "magicmsgs.hpp"

using namespace Filter;

TX_THREAD ChassisThread;
uint8_t ChassisThreadStack[4096] = {0};

[[noreturn]] void ChassisThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);


    for (;;){}
}