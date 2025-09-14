//
// Created by cosmosmount on 2025/9/10.
//

#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"

TX_THREAD GimbalThread;
uint8_t GimbalThreadStack[4096] = {0};

msg_dr16_t debug_dr16;

[[noreturn]] void GimbalThreadFun(ULONG initial_input) {
    UNUSED(initial_input);

    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_dr16_t remoter{};


    for (;;) {
        om_suber_export(remoter_suber, &remoter, false);
        memcpy (&debug_dr16, &remoter, sizeof(msg_dr16_t));
        tx_thread_sleep(1);
    }
}