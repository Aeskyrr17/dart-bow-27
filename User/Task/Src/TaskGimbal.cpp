//
// Created by cosmosmount on 2025/9/10.
//

#include "main.h"
#include "tx_api.h"

TX_THREAD GimbalThread;
uint8_t GimbalThreadStack[2048] = {0};
TX_SEMAPHORE GimbalThreadSem;

[[noreturn]] void GimbalThreadFun(ULONG initial_input) {
    UNUSED(initial_input);



    for (;;) {

    }
}