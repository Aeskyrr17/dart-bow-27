#include "main.h"
#include "tx_api.h"

#include "om.h"


TX_THREAD GantryThread;
uint8_t GantryThreadStack[2048] = {0};

[[noreturn]] void GantryThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    for (;;) 
    {
        
        
        tx_thread_sleep(1);
    }
}