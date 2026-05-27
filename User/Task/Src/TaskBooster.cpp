#include "main.h"
#include "tx_api.h"
#include "bsp_can.hpp"

extern TX_THREAD UIThread;
extern uint8_t UIThreadStack[2048];
extern void UIThreadFun(ULONG initial_input);

extern TX_THREAD LauncherThread;
extern uint8_t LauncherThreadStack[2048];   
extern void LauncherThreadFun(ULONG initial_input);

extern TX_THREAD SysctrlThread;
extern uint8_t SysctrlThreadStack[2048];    
extern void SysctrlThreadFun(ULONG initial_input);

extern TX_THREAD MotorThread;
extern uint8_t MotorThreadStack[2048];
extern void MotorThreadFun(ULONG initial_input);

extern TX_THREAD SensorThread;
extern uint8_t SensorThreadStack[2048];
extern void SensorThreadFun(ULONG initial_input);

extern TX_THREAD AliveThread;
extern uint8_t AliveThreadStack[512];
extern void AliveThreadFun(ULONG thread_input);

extern TX_SEMAPHORE MotorAlive;
extern TX_SEMAPHORE CANErrorSem;
extern TX_SEMAPHORE VisionErrorSem;
extern TX_SEMAPHORE G4ForceGot;
extern TX_SEMAPHORE GantryMotorErrorSem;


//todo:确定优先级
#define TX_NAME(s) const_cast<CHAR*>(s)

extern "C" void TaskBooster(void)
{
    tx_semaphore_create(&MotorAlive, TX_NAME("MotorAlive"), 0);
    tx_semaphore_create(&CANErrorSem, TX_NAME("CANErrorSem"), 0);
    tx_semaphore_create(&VisionErrorSem, TX_NAME("VisionErrorSem"), 0);
    tx_semaphore_create(&G4ForceGot, TX_NAME("G4ForceGot"), 0);
    tx_semaphore_create(&GantryMotorErrorSem, TX_NAME("GantryMotorErrorSem"), 0);


    CAN_Init();

    tx_thread_create(&UIThread, TX_NAME("UIThread"), UIThreadFun, 0x1234,
                     UIThreadStack, sizeof(UIThreadStack),
                     8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&LauncherThread, TX_NAME("LauncherThread"), LauncherThreadFun, 0x1234,
                     LauncherThreadStack, sizeof(LauncherThreadStack),
                     6, 6, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&SysctrlThread, TX_NAME("SysctrlThread"), SysctrlThreadFun, 0x1234,
                     SysctrlThreadStack, sizeof(SysctrlThreadStack),
                     5, 5, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&MotorThread, TX_NAME("MotorCtrlThread"), MotorThreadFun, 0x1234,
                     MotorThreadStack, sizeof(MotorThreadStack),
                     7, 7, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&SensorThread, TX_NAME("SensorThread"), SensorThreadFun, 0x1234,
                     SensorThreadStack, sizeof(SensorThreadStack),
                     3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&AliveThread, TX_NAME("AliveThread"), AliveThreadFun, 0x1234,
                     AliveThreadStack, sizeof(AliveThreadStack),
                     19, 19, TX_NO_TIME_SLICE, TX_AUTO_START);
}
