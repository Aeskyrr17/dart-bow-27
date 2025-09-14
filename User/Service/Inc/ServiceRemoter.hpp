#pragma once
#ifndef SERVICE_REMOTER_H
#define SERVICE_REMOTER_H
#ifdef __cplusplus
#include "tx_api.h"
#include "main.h"
#include "Dr16.hpp"
#include "main.h"

#define SBUS_RX_BUF_NUM 18u

#ifdef __cplusplus
extern "C" {
#endif

    void USER_USART5_RxHandler(UART_HandleTypeDef *huart, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif
#endif