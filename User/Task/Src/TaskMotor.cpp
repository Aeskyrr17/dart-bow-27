#include "DMMotor.hpp"
#include "fdcan.h"
#include "tx_api.h"
#include "DM4310.hpp"
#include "DMMotorHandler.hpp"

TX_THREAD TaskMotorThread;
uint8_t TaskMotorThreadStack[2048] = {0};

[[noreturn]] void TaskMotorThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input);
    DM4310 yourmotor;
    yourmotor.controlMode = DMMotor::MIT_MODE;
    DMMotorHandler::Instance()->registerMotor(&yourmotor, &hfdcan1, 0x01);

    for (;;) 
    {
        // Motor control code goes here
        yourmotor.torqueSet = 0.0f;
        yourmotor.SetOutput();
        tx_thread_sleep(1); // Adjust sleep time as necessary
    }
}