#include "main.h"
#include "tx_api.h"

extern TX_THREAD UIThread;
extern uint8_t UIThreadStack[2048];
extern void UIThreadFun(ULONG initial_input);

extern TX_THREAD TaskMotorThread;
extern uint8_t TaskMotorThreadStack[2048];
extern void TaskMotorThreadFun(ULONG initial_input);

#define TX_NAME(s) const_cast<CHAR*>(s)
extern "C" void TaskBooster(void)
{
    tx_thread_create(&UIThread, TX_NAME("UIThread"), UIThreadFun, 0x1234,
                     UIThreadStack, sizeof(UIThreadStack),
                     8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&TaskMotorThread, TX_NAME("TaskMotorThread"), TaskMotorThreadFun, 0x1234,
                     TaskMotorThreadStack, sizeof(TaskMotorThreadStack),
                        7, 7, TX_NO_TIME_SLICE, TX_AUTO_START);
}