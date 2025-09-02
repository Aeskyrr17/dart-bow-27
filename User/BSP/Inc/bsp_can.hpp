//
// Created by cosmosmount on 2025/8/30.
//

#ifndef RM26_BSP_CAN_HPP
#define RM26_BSP_CAN_HPP

#include "fdcan.h"

/**
 * @brief  初始化
 */
void CAN_Init(void);

/**
 * @brief 发送CAN数据。
 * @param hcan 指向CAN句柄的指针，用于配置CAN传输。
 * @param StdId CAN消息的标准标识符
 * @param msg 发送的数据
 * @param len 数据长度
 * @note 该函数用于发送CAN数据，目前只支持标准帧
 */
void CAN_Transmit(FDCAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *msg, uint16_t len);

#endif //RM26_BSP_CAN_HPP