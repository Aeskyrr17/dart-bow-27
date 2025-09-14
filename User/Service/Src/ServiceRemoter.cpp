#include "usart.h"
#include "om.h"
#include "magicmsgs.hpp"
#include "ServiceRemoter.hpp"

TX_THREAD RemoterThread;
uint8_t RemoterThreadStack[2048] = {0};
TX_SEMAPHORE RemoterThreadSem;

Dr16 *dr16 = Dr16::Instance();

[[noreturn]] void RemoterThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    dr16->Init();

    /* Remoter Topic */
    om_topic_t *remoter_topic = om_config_topic(nullptr, "ca", "remoter", sizeof(msg_dr16_t));
    msg_dr16_t msg_remoter{};

    for (;;) {
        msg_remoter.offline = false;
        while (tx_semaphore_get(&RemoterThreadSem, 100) != TX_SUCCESS) {
            // 超时/掉线逻辑
            // 比如可以清零，或者标记掉线
            dr16->AliveFlag = false;
            msg_remoter.offline = true;
            // 可以加上状态标记，比如 rc_raw.online = false;
            tx_thread_sleep(3);
        }
        // 开关
        msg_remoter.left_sw  = static_cast<Dr16::RC_SWITCH_STATE>(dr16->Dr16_Data._data.s1);
        msg_remoter.right_sw = static_cast<Dr16::RC_SWITCH_STATE>(dr16->Dr16_Data._data.s2);

        // 摇杆 11 位 -> float [-1,1]
        msg_remoter.right_x  = (static_cast<float>(dr16->Dr16_Data._data.ch_0) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
        msg_remoter.right_y  = (static_cast<float>(dr16->Dr16_Data._data.ch_1) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
        msg_remoter.left_x = (static_cast<float>(dr16->Dr16_Data._data.ch_2) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
        msg_remoter.left_y = (static_cast<float>(dr16->Dr16_Data._data.ch_3) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;

        // 鼠标
        msg_remoter.mouse_x  = static_cast<float>(dr16->Dr16_Data._data.mouse_x);
        msg_remoter.mouse_y  = static_cast<float>(dr16->Dr16_Data._data.mouse_y);
        msg_remoter.mouse_z  = static_cast<float>(dr16->Dr16_Data._data.mouse_z);
        msg_remoter.mouse_left  = dr16->Dr16_Data._data.mouse_left != 0;
        msg_remoter.mouse_right = dr16->Dr16_Data._data.mouse_right != 0;

        // 键盘位域可以直接 memcpy
        memcpy(&msg_remoter.key, &dr16->Dr16_Data._data.key, sizeof(msg_remoter.key));
        // dr16->Update();
        om_publish(remoter_topic, &msg_remoter, sizeof(msg_remoter), true, false);

        tx_thread_sleep(1);
    }
}

/**
 * @brief  USER USART5 Reception Event Callback.(SBUS remote_ctrl)
 * @param  huart UART handle
 * @param  Size  Number of data available in application reception buffer (indicates a position in
 *               reception buffer until which, data are available)
 * @retval None
 */
void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size) {
  /* Current memory buffer used is Memory 0 */
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET){
    /* Disable DMA */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* Switch Memory 0 to Memory 1*/
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

    /* Reset the receive count */
    __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM*2);

    /* Juge whether size is equal to the length of the received data */
    if(Size == SBUS_RX_BUF_NUM)
    {
      /* Memory 0 data update to remote_ctrl*/
      tx_semaphore_put(&RemoterThreadSem);
      memcpy(dr16->Dr16_Data.ReceiveBuffer, SBUS_MultiRx_Buf[0], DR16_DATA_SIZE);
    }
  }
  /* Current memory buffer used is Memory 1 */
  else{
    /* Disable DMA */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* Switch Memory 1 to Memory 0*/
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

    /* Reset the receive count */
    __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM*2);

    if(Size == SBUS_RX_BUF_NUM)
    {
      /* Memory 1 to data update to remote_ctrl*/
      tx_semaphore_put(&RemoterThreadSem);
      memcpy(dr16->Dr16_Data.ReceiveBuffer, SBUS_MultiRx_Buf[1], DR16_DATA_SIZE);

    }
  }
}

volatile uint16_t size_test = 0;
/**
 * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
 * @param  huart UART handle
 * @param  Size  Number of data available in application reception buffer (indicates a position in
 *               reception buffer until which, data are available)
 * @retval None
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart5)
  {
    size_test = Size;
    //清空cache
    DMA_Cache_PrepareForReceive((uint32_t *)SBUS_MultiRx_Buf[0], SBUS_RX_BUF_NUM * 2);
    USER_USART5_RxHandler(huart, Size);
  }
  huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}