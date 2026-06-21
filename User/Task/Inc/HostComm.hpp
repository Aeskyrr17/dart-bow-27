#pragma once

#include <stdint.h>

#include "tx_api.h"

extern TX_THREAD HostCommThread;
extern uint8_t HostCommThreadStack[1024];

[[noreturn]] void HostCommThreadFun(ULONG initial_input);
