#include "DMMotor.hpp"
#include "led.hpp"
#include "tx_api.h"
#include "DMMotorHandler.hpp"

TX_THREAD AliveThread;
uint8_t AliveThreadStack[512] = {0};

/* Semaphores for Alivecheck */
extern TX_SEMAPHORE RefereeThreadSem;
extern TX_SEMAPHORE CANErrorSem;
extern TX_SEMAPHORE VisionErrorSem;
extern TX_SEMAPHORE MotorAlive;


[[noreturn]] void AliveThreadFun(ULONG thread_input)
{
    LED_ALL_ON();

    while(1)
    {
        bool can_error = tx_semaphore_get(&CANErrorSem, TX_NO_WAIT) == TX_SUCCESS;
        bool referee_alive = tx_semaphore_get(&RefereeThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        bool vision_error = tx_semaphore_get(&VisionErrorSem, TX_NO_WAIT) == TX_SUCCESS;
        if (can_error)
        {
            LED_blink(LED_COLOR::LED_RED);
        }
        // else if (!imu_alive) 
        // {
        //     LED_blink(LED_COLOR::LED_WHITE);        
        // }
        else if (vision_error)
        {
            LED_blink(LED_COLOR::LED_BLUE);
        }
        else
        {
            if (referee_alive)
            {
                LED_blink(LED_COLOR::LED_GREEN);
                // if (DMMotorHandler::Instance()->AllMotorAliveCheck())
                // {
                //     tx_semaphore_put(&MotorAlive);
                //     LED_blink(LED_COLOR::LED_GREEN);
                // }
                // else
                // {
                //     LED_blink(LED_COLOR::LED_BLUE);
                // }
            }
        }
        tx_thread_sleep(2);
    }
}
