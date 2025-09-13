#include "bsp_can.hpp"

// #include "GMMotorhandler.hpp"
// #include "LKMotorhandler.hpp"
// #include "BoardConnectivity.hpp"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void CAN_Init(void)
{
    FDCAN_FilterTypeDef FDCAN_FilterConfig;

    FDCAN_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN_FilterConfig.FilterIndex = 0;
    FDCAN_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN_FilterConfig.FilterID1 = 0x00000000;
    FDCAN_FilterConfig.FilterID2 = 0x00000000;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_FilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);

    HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_FilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);

    HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_FilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan3);
}

void CAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t Id, uint8_t *msg, uint16_t len)
{
    FDCAN_TxHeaderTypeDef tx_header; ///< 定义发送数据结构体

    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    tx_header.Identifier = Id;
    tx_header.DataLength = len;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, msg); ///< 发送数据
}

/**
 * @todo 待完成整个函数
 * @brief 更新从CAN接收的电机数据。
 * @param index 电机的索引，用于识别特定的电机。
 * @param rx_header 指向接收到的数据的指针。
 * @param can_receive_data 指向电机测量数据结构的指针，用于存储更新的数据。
 */
void CAN_Receive()
{
}


