#include "TaskBooster.hpp"

#include "main.h"
#include "tx_api.h"

extern TX_THREAD ChassisThread;
extern uint8_t ChassisThreadStack[4096];
extern void ChassisThreadFun(ULONG initial_input);

#define TX_NAME(s) const_cast<CHAR*>(s)
void TaskBooster(void)
{
    /* Create the main thread.  */
    tx_thread_create(&ChassisThread, TX_NAME("ChassisThread"), ChassisThreadFun, 0x1234,
                     ChassisThreadStack, sizeof(ChassisThreadStack),
                     6, 6, TX_NO_TIME_SLICE, TX_AUTO_START);
    // tx_semaphore_create(&GimbalThreadSem, "Gimbal Semaphore", 0);
}