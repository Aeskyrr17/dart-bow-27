#include "usart.h"
#include "ServiceRemoter.hpp"

TX_THREAD RemoterThread;
uint8_t RemoterThreadStack[2048] = {0};
TX_SEMAPHORE RemoterThreadSem;

Dr16 *dr16 = Dr16::Instance();

float left_x = 0.0f;

[[noreturn]] void RemoterThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    dr16->Init();
    /* Remoter Topic */

    for (;;) {
        while (tx_semaphore_get(&RemoterThreadSem, 100) != TX_SUCCESS) {
            // 超时/掉线逻辑
            // 比如可以清零，或者标记掉线
            dr16->AliveFlag = false;
            // 可以加上状态标记，比如 rc_raw.online = false;
            tx_thread_sleep(3);
            continue;
        }
        // memcpy(dr16->Dr16_Data.ReceiveBuffer, SBUS_MultiRx_Buf[0], DR16_DATA_SIZE);//TODO:不知道只写0有没有bug
        dr16->Update();
        left_x = dr16->GetLeftX();
        tx_thread_sleep(1);
    }
}

uint8_t size_test = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart5)
    {
		size_test = Size;
        //清空cache
        DMA_Cache_PrepareForReceive((uint32_t *)SBUS_MultiRx_Buf[0], SBUS_RX_BUF_NUM * 2);
        tx_semaphore_put(&RemoterThreadSem);

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

void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{

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
            // tx_semaphore_put(&RemoterThreadSem);
            memcpy(dr16->Dr16_Data.ReceiveBuffer, SBUS_MultiRx_Buf[0], DR16_DATA_SIZE);//TODO:不知道只写0有没有bug


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
            // tx_semaphore_put(&RemoterThreadSem);
            memcpy(dr16->Dr16_Data.ReceiveBuffer, SBUS_MultiRx_Buf[1], DR16_DATA_SIZE);//TODO:不知道只写0有没有bug

        }

    }

}