#include "main.h"
#include "stm32h7xx_hal_uart.h"


#define HALL_R_PORT                GPIOE
#define HALL_R_PIN                 GPIO_PIN_1

#define HALL_L_PORT                GPIOE
#define HALL_L_PIN                 GPIO_PIN_0

#define LIGHT_PORT                 GPIOE
#define LIGHT_PIN                  GPIO_PIN_14

#define FORCE_DATA_RX_SIZE         9
#define G4_FORCE_RX_DATA_SIZE      8
#define G4_FORCE_RX_BUFFER_SIZE    32

extern uint8_t u2_rx_buffer[FORCE_DATA_RX_SIZE];
extern uint8_t u3_rx_buffer[FORCE_DATA_RX_SIZE];
extern uint8_t g4_rx_buffer[G4_FORCE_RX_BUFFER_SIZE];

void ForceSensor_RxCpltCallback(UART_HandleTypeDef *huart);
void ForceSensor_ErrorCallback(UART_HandleTypeDef *huart);
