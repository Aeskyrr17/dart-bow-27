//
// Created by cosmosmount on 2025/9/8.
//

#include "main.h"
#include "tx_api.h"
#include "led.hpp"

TX_THREAD my_thread1;
uint8_t my_thread_stack1[1024];
TX_SEMAPHORE my_semaphore1;

TX_THREAD my_thread2;
uint8_t my_thread_stack2[1024];

[[noreturn]] void my_thread_entry(ULONG thread_input)
{
    LED_ALL_ON();
    /* Enter into a forever loop. */
    while(1)
    {

        /* Increment thread counter. */
        tx_semaphore_put(&my_semaphore1);
        /* Sleep for 1 tick. */
        tx_thread_sleep(500);
    }
}

[[noreturn]] void my_thread_entry2(ULONG thread_input)
{
    /* Enter into a forever loop. */
    while(1)
    {
        /* Increment thread counter. */
        if (tx_semaphore_get(&my_semaphore1, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            LED_blink();
            /* Sleep for 1 tick. */
        }
    }
}

#define TX_NAME(s) const_cast<CHAR*>(s)

extern "C" void TaskBooster()
{
    tx_semaphore_create(&my_semaphore1, TX_NAME("my_semaphore1"), 0);

    /* Create my_thread! */
    tx_thread_create(&my_thread1, TX_NAME("my_thread1"),
        my_thread_entry, 0x1234, my_thread_stack1, sizeof(my_thread_stack1),
        10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&my_thread2, TX_NAME("my_thread2"),
        my_thread_entry2, 0x1234, my_thread_stack2, sizeof(my_thread_stack2),
        10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);
}