#include "bsp.hpp"

#include "bsp_can.hpp"
#include "bsp_usart.hpp"
#include "bsp_dwt.hpp"
#include "bsp_pwm.hpp"

uint16_t test;

void bsp_Init() {
    USART_Init();
    DWT_Init(192);
    CAN_Init();
    PWM_Init(); //todo:不知道需不需要
}