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
extern TX_SEMAPHORE GantryMotorErrorSem;
extern TX_SEMAPHORE G4SensorLostSem;



[[noreturn]] void AliveThreadFun(ULONG thread_input)
{
    LED_ALL_ON();

    while(1)
    {
        bool can_error = tx_semaphore_get(&CANErrorSem, TX_NO_WAIT) == TX_SUCCESS;
        bool referee_alive = tx_semaphore_get(&RefereeThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        bool vision_error = tx_semaphore_get(&VisionErrorSem, TX_NO_WAIT) == TX_SUCCESS;
        bool gantry_motor_error = tx_semaphore_get(&GantryMotorErrorSem, TX_NO_WAIT) == TX_SUCCESS;
        bool g4force_error = tx_semaphore_get(&G4SensorLostSem, TX_NO_WAIT) == TX_SUCCESS;

        if (can_error && vision_error && gantry_motor_error)
        {
            LED_blink(LED_COLOR::LED_RED, LED_COLOR::LED_BLUE, LED_COLOR::LED_WHITE);
        }
        else if (can_error && vision_error)
        {
            LED_blink(LED_COLOR::LED_RED, LED_COLOR::LED_BLUE);
        }
        else if (can_error && gantry_motor_error)
        {
            LED_blink(LED_COLOR::LED_RED, LED_COLOR::LED_WHITE);
        }
        else if (vision_error && gantry_motor_error)
        {
            LED_blink(LED_COLOR::LED_BLUE, LED_COLOR::LED_WHITE);
        }
        else if (can_error)
        {
            LED_blink(LED_COLOR::LED_RED);
        }
        else if (vision_error)
        {
            LED_blink(LED_COLOR::LED_BLUE);
        }
        else if (gantry_motor_error)
        {
            LED_blink(LED_COLOR::LED_WHITE);
        }
        else
        {
            if (referee_alive)
            {
                LED_blink(LED_COLOR::LED_GREEN);
            }
            else
            {
                LED_ALL_OFF();
            }
        }
        tx_thread_sleep(2);
    }
}
