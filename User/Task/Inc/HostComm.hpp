#pragma once

#include <stdint.h>

#include "main.h"
#include "tx_api.h"

extern TX_THREAD HostCommThread;
extern uint8_t HostCommThreadStack[1024];

struct HostCommStats
{
    volatile uint32_t rx_bytes;
    volatile uint32_t rx_frames_ok;
    volatile uint32_t rx_crc_errors;
    volatile uint32_t rx_length_errors;
    volatile uint32_t rx_ring_overflow;
    volatile uint32_t rx_dma_errors;
    volatile uint32_t tx_frames_ok;
    volatile uint32_t tx_busy_count;
    volatile uint32_t tx_errors;
    volatile uint32_t parser_timeouts;
};

extern HostCommStats host_comm_stats;

[[noreturn]] void HostCommThreadFun(ULONG initial_input);
void HostComm_RxEventCallback(uint16_t size);
void HostComm_TxCpltCallback(UART_HandleTypeDef* huart);
void HostComm_ErrorCallback(UART_HandleTypeDef* huart);
