#include "bsp.hpp"
#include "bsp_usart.hpp"
#include "bsp_dwt.hpp"

uint16_t test;

void bsp_Init() {
    USART_Init();
    DWT_Init(192);
}