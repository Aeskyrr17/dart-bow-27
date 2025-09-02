//
// Created by cosmosmount on 2025/8/30.
//

#ifndef RM26_BSP_USART_HPP
#define RM26_BSP_USART_HPP

#include "dma.h"
#include "usart.h"

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

void USART_Init(void);

#endif //RM26_BSP_USART_HPP