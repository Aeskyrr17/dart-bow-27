#include "usart.h"
#include "om.h"
#include "magicmsgs.hpp"
#include "ServiceRemoter.hpp"

TX_THREAD RemoterThread;
uint8_t RemoterThreadStack[2048] = {0};
TX_SEMAPHORE RemoterThreadSem;

// __attribute__((section(".RAM_D3"))) uint8_t data_rx[32];
// 数组在 D3 RAM
__attribute__((section(".RAM_D3"))) uint8_t data_rx[DR16_DATA_SIZE];

inline dr16_data_t& Dr16_Data()
{
    return *reinterpret_cast<dr16_data_t*>(data_rx);
}

[[noreturn]] void RemoterThreadFun(ULONG initial_input) {
    UNUSED(initial_input);

    /* Remoter Topic */
    om_topic_t *remoter_topic = om_config_topic(nullptr, "ca", "remoter", sizeof(msg_dr16_t));
    msg_dr16_t msg_remoter{};
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, data_rx, DR16_DATA_SIZE);
    for (;;) {
        msg_remoter.offline = false;
        while (tx_semaphore_get(&RemoterThreadSem, 100) != TX_SUCCESS) {
            // 超时/掉线逻辑
            // 比如可以清零，或者标记掉线
            msg_remoter.offline = true;
            HAL_UART_Abort(&huart5);
            // 可以加上状态标记，比如 rc_raw.online = false;
            tx_thread_sleep(3);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, data_rx, DR16_DATA_SIZE);
        }
        // 开关
        msg_remoter.left_sw  = static_cast<Dr16::RC_SWITCH_STATE>(Dr16_Data().s1);
        msg_remoter.right_sw = static_cast<Dr16::RC_SWITCH_STATE>(Dr16_Data().s2);

        // 摇杆 11 位 -> float [-1,1]
        msg_remoter.right_x  = (static_cast<float>(Dr16_Data().ch_0) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
        msg_remoter.right_y  = (static_cast<float>(Dr16_Data().ch_1) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
        msg_remoter.left_x = (static_cast<float>(Dr16_Data().ch_2) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
        msg_remoter.left_y = (static_cast<float>(Dr16_Data().ch_3) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;

        // 鼠标
        msg_remoter.mouse_x  = static_cast<float>(Dr16_Data().mouse_x);
        msg_remoter.mouse_y  = static_cast<float>(Dr16_Data().mouse_y);
        msg_remoter.mouse_z  = static_cast<float>(Dr16_Data().mouse_z);
        msg_remoter.mouse_left  = Dr16_Data().mouse_left != 0;
        msg_remoter.mouse_right = Dr16_Data().mouse_right != 0;

        // 键盘位域可以直接 memcpy
        memcpy(&msg_remoter.key, &Dr16_Data().key, sizeof(msg_remoter.key));
        // Update();
        om_publish(remoter_topic, &msg_remoter, sizeof(msg_remoter), true, false);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, data_rx, DR16_DATA_SIZE);
        tx_thread_sleep(1);
    }
}

volatile uint16_t size_test = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart5) {
        SCB_InvalidateDCache_by_Addr((uint32_t*)data_rx, DR16_DATA_SIZE);
        size_test = Size;
        tx_semaphore_put(&RemoterThreadSem);
    }
}
