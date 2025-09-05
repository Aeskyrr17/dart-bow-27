#pragma once
#ifndef SERVICE_REMOTER_H
#define SERVICE_REMOTER_H
#ifdef __cplusplus
#include "tx_api.h"
#include "Dr16.hpp"
#include "bsp_cache.hpp"

#define SBUS_RX_BUF_NUM 18u ///< SBUS接收缓冲区大小
#define RC_FRAME_LENGTH 18u ///< 遥控器数据帧长度
#define NULL 0 ///< 空指针

#ifndef RM26_H7_SERVICEREMOTER_HPP
#define RM26_H7_SERVICEREMOTER_HPP

#endif //RM26_H7_SERVICEREMOTER_HPP
#ifdef __cplusplus
extern "C" {
#endif
    void USER_USART5_RxHandler(UART_HandleTypeDef *huart, uint16_t Size);
#ifdef __cplusplus
}
#endif

#endif

#endif