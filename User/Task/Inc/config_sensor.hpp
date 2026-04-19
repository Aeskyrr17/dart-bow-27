#include "main.h"
#include "stm32h7xx_hal_uart.h"


#define HALL_R_PORT                GPIOE
#define HALL_R_PIN                 GPIO_PIN_1

#define HALL_L_PORT                GPIOE
#define HALL_L_PIN                 GPIO_PIN_0

#define LIGHT_PORT                 GPIOE
#define LIGHT_PIN                  GPIO_PIN_14

#define FORCE_DATA_RX_SIZE         9

extern uint8_t u2_rx_buffer[FORCE_DATA_RX_SIZE];
extern uint8_t u3_rx_buffer[FORCE_DATA_RX_SIZE];

void ForceSensor_RequestAll(void);
void ForceSensor_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void ForceSensor_ErrorCallback(UART_HandleTypeDef *huart);
