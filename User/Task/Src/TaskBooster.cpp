#include "main.h"
#include "tx_api.h"

extern TX_THREAD UIThread;
extern uint8_t UIThreadStack[2048];
extern void UIThreadFun(ULONG initial_input);

extern TX_THREAD LauncherThread;
extern uint8_t LauncherThreadStack[2048];   
extern void LauncherThreadFun(ULONG initial_input);

extern TX_THREAD SysctrlThread;
extern uint8_t SysctrlThreadStack[2048];    
extern void SysctrlThreadFun(ULONG initial_input);

extern TX_THREAD MotorCtrlThread;
extern uint8_t MotorCtrlThreadStack[2048];
extern void MotorThreadFun(ULONG initial_input);

extern TX_THREAD GantryThread;
extern uint8_t GantryThreadStack[2048];
extern void GantryThreadFun(ULONG initial_input);

//todo:添加其他任务
#define TX_NAME(s) const_cast<CHAR*>(s)
extern "C" void TaskBooster(void)
{
    tx_thread_create(&UIThread, TX_NAME("UIThread"), UIThreadFun, 0x1234,
                     UIThreadStack, sizeof(UIThreadStack),
                     8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);

    
}