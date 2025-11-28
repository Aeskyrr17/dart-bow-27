#include "om.h"
#include "main.h"
#include "tx_api.h"
#include "ui.hpp"

TX_THREAD UIThread;
uint8_t UIThreadStack[2048] = {0};

[[noreturn]] void UIThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);
    UI ui;
    for (;;)
    {
        ui.Update();
        tx_thread_sleep(30);
    }
}