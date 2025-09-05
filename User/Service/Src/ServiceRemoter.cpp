#include "usart.h"
#include "ServiceRemoter.hpp"

__attribute__((section (".AXI_SRAM"))) uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

TX_THREAD RemoterThread;
uint8_t RemoterThreadStack[2048] = {0};
TX_SEMAPHORE RemoterThreadSem;

static uint8_t size_of_data;
extern Dr16 dr16;


[[noreturn]] void RemoterThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    dr16.Init();
    /* Remoter Topic */

    for (;;) {
        while (tx_semaphore_get(&RemoterThreadSem, 100) != TX_SUCCESS) {
            // 超时/掉线逻辑
            // 比如可以清零，或者标记掉线
            dr16.AliveFlag = false;
            // 可以加上状态标记，比如 rc_raw.online = false;
            tx_thread_sleep(3);
            continue;
        }

        dr16.Update();


        tx_thread_sleep(1);
    }
}

