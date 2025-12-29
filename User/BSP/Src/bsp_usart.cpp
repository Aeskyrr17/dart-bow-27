#include "bsp_usart.hpp"
#include "XRobot.hpp"
#include "referee.hpp"
#include "config_chassis.hpp"

/*------------全局变量------------*/
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

__attribute__((section (".RAM_D1"))) uint8_t UART7RxBuffer[TOF_DATA_SIZE] = {0};
__attribute__((section (".RAM_D1"))) uint8_t USART1RxBuffer[1] = {0};
extern uint8_t tof_rx[TOF_DATA_SIZE];

/**
 * @brief  Configures the USART.
 * @param  None
 * @retval None
 */

void USART_Init()
{
  // uart5 双缓冲区初始化
  // USART_RxDMA_MultiBuffer_Init(&huart5, (uint32_t *)SBUS_MultiRx_Buf[0], (uint32_t *)SBUS_MultiRx_Buf[1], SBUS_RX_BUF_NUM);
  // uart7
  __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_uart7_rx, DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(&hdma_uart7_tx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_uart7_tx, DMA_IT_TC);
  __HAL_UART_SEND_REQ(&huart7, UART_RXDATA_FLUSH_REQUEST);
  HAL_UART_Receive_DMA(&huart7, tof_rx, TOF_DATA_SIZE);

  // usart1
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_usart1_tx, DMA_IT_TC);
  HAL_UART_Receive_DMA(&huart1, USART1RxBuffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart7)
  {
    //清空cache
    SCB_InvalidateDCache_by_Addr(tof_rx, TOF_DATA_SIZE);
    __HAL_UART_SEND_REQ(&huart7, UART_RXDATA_FLUSH_REQUEST);
    HAL_UART_Receive_DMA(&huart7, tof_rx, TOF_DATA_SIZE);
  }
  else if (huart == &huart1)
  {
    //清空cache
    SCB_InvalidateDCache_by_Addr(USART1RxBuffer, 1);
    HAL_UART_Receive_DMA(&huart1, USART1RxBuffer, 1);
  }
}
