#include "HostComm.hpp"

#include <stdint.h>
#include <string.h>

#include "crc.hpp"
#include "HostProtocol.hpp"
#include "om.h"
#include "usart.h"

TX_THREAD HostCommThread;
uint8_t HostCommThreadStack[1024] = {0};
HostCommStats host_comm_stats{};

extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;
extern DMA_HandleTypeDef hdma_usart10_tx;

namespace
{
constexpr uint8_t FRAME_OVERHEAD = 8;
constexpr uint16_t RX_DMA_BUFFER_SIZE = 256;
constexpr uint16_t RX_RING_SIZE = 512;
constexpr uint32_t CACHE_LINE_SIZE = 32;
constexpr ULONG HOSTCOMM_PARSER_TIMEOUT_TICKS = 50;

__attribute__((section(".RAM_D1"), aligned(32))) uint8_t host_rx_dma_buffer[RX_DMA_BUFFER_SIZE] = {0};
__attribute__((section(".RAM_D1"), aligned(32))) uint8_t host_tx_dma_buffer[FRAME_OVERHEAD + HOSTCOMM_MAX_PAYLOAD_SIZE] = {0};

uint8_t host_rx_ring[RX_RING_SIZE] = {0};
volatile uint16_t host_rx_ring_write = 0;
volatile uint16_t host_rx_ring_read = 0;
volatile bool host_tx_busy = false;

TX_SEMAPHORE HostCommRxSem;
TX_SEMAPHORE HostCommTxSem;

enum class ParseState : uint8_t
{
    WaitSof1,
    WaitSof2,
    WaitHeader,
    WaitPayload,
    WaitCrcLo,
    WaitCrcHi,
};

enum class DecodeResult : uint8_t
{
    Ok,
    UnknownType,
    LengthError,
};

enum class TxStartResult : uint8_t
{
    Started,
    Busy,
    Error,
};

struct HostFrame
{
    uint8_t version = 0;
    uint8_t type = 0;
    uint8_t seq = 0;
    uint8_t len = 0;
    uint8_t payload[HOSTCOMM_MAX_PAYLOAD_SIZE] = {0};
};

ParseState parser_state = ParseState::WaitSof1;
HostFrame parser_frame{};
uint8_t parser_header_index = 0;
uint8_t parser_payload_index = 0;
uint8_t parser_crc_lo = 0;
ULONG parser_last_tick = 0;

uint32_t AlignDown(uint32_t value)
{
    return value & ~(CACHE_LINE_SIZE - 1U);
}

uint32_t AlignUp(uint32_t value)
{
    return (value + CACHE_LINE_SIZE - 1U) & ~(CACHE_LINE_SIZE - 1U);
}

void InvalidateDCache(const void* address, uint32_t size)
{
    if (size == 0)
    {
        return;
    }

    const uint32_t start = AlignDown(reinterpret_cast<uint32_t>(address));
    const uint32_t end = AlignUp(reinterpret_cast<uint32_t>(address) + size);
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(start), static_cast<int32_t>(end - start));
}

void CleanDCache(const void* address, uint32_t size)
{
    if (size == 0)
    {
        return;
    }

    const uint32_t start = AlignDown(reinterpret_cast<uint32_t>(address));
    const uint32_t end = AlignUp(reinterpret_cast<uint32_t>(address) + size);
    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(start), static_cast<int32_t>(end - start));
}

bool RingPush(uint8_t byte)
{
    const uint16_t next = static_cast<uint16_t>((host_rx_ring_write + 1U) % RX_RING_SIZE);
    if (next == host_rx_ring_read)
    {
        host_comm_stats.rx_ring_overflow++;
        return false;
    }

    host_rx_ring[host_rx_ring_write] = byte;
    host_rx_ring_write = next;
    return true;
}

bool RingPop(uint8_t* byte)
{
    if (host_rx_ring_read == host_rx_ring_write)
    {
        return false;
    }

    *byte = host_rx_ring[host_rx_ring_read];
    host_rx_ring_read = static_cast<uint16_t>((host_rx_ring_read + 1U) % RX_RING_SIZE);
    return true;
}

void ResetParser()
{
    parser_state = ParseState::WaitSof1;
    parser_frame = HostFrame{};
    parser_header_index = 0;
    parser_payload_index = 0;
    parser_crc_lo = 0;
    parser_last_tick = tx_time_get();
}

void TouchParserTimer()
{
    if (parser_state != ParseState::WaitSof1)
    {
        parser_last_tick = tx_time_get();
    }
}

void CheckParserTimeout()
{
    if (parser_state == ParseState::WaitSof1)
    {
        return;
    }

    if (tx_time_get() - parser_last_tick >= HOSTCOMM_PARSER_TIMEOUT_TICKS)
    {
        host_comm_stats.parser_timeouts++;
        ResetParser();
    }
}

void PutU16(uint8_t* out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value & 0xFFU);
    out[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void PublishError(om_topic_t* hosttx_topic, uint8_t ref_type, uint8_t ref_seq,
                  uint8_t error_code, uint16_t detail)
{
    if (hosttx_topic == nullptr)
    {
        return;
    }

    msg_hosttx_t tx{};
    tx.valid = 1;
    tx.type = HOST_TYPE_ERROR_RSP;
    tx.seq = ref_seq;
    tx.len = 6;
    tx.payload[0] = ref_type;
    tx.payload[1] = ref_seq;
    tx.payload[2] = error_code;
    tx.payload[3] = 0;
    PutU16(&tx.payload[4], detail);
    om_publish(hosttx_topic, &tx, sizeof(tx), true, false);
}

float ReadFloat(const uint8_t* payload, uint8_t offset)
{
    float value = 0.0f;
    memcpy(&value, payload + offset, sizeof(value));
    return value;
}

DecodeResult DecodeFrame(const HostFrame& frame, msg_hostreq_t* req)
{
    if (req == nullptr)
    {
        return DecodeResult::LengthError;
    }

    memset(req, 0, sizeof(*req));
    req->valid = 1;
    req->seq = frame.seq;
    req->type = frame.type;

    switch (frame.type)
    {
        case HOST_TYPE_HEARTBEAT_REQ:
            if (frame.len != 0) { return DecodeResult::LengthError; }
            req->kind = HOST_REQ_HEARTBEAT;
            return DecodeResult::Ok;

        case HOST_TYPE_GET_DART_TABLE:
            if (frame.len != 0) { return DecodeResult::LengthError; }
            req->kind = HOST_REQ_GET_DART_TABLE;
            return DecodeResult::Ok;

        case HOST_TYPE_SET_DART_PARAMS_BATCH:
        {
            if (frame.len < 12) { return DecodeResult::LengthError; }
            const uint8_t dart_count = frame.payload[2];
            if (dart_count == 0 || dart_count > 8 || frame.len != static_cast<uint8_t>(11U + dart_count))
            {
                return DecodeResult::LengthError;
            }

            req->kind = HOST_REQ_SET_DART_PARAMS_BATCH;
            req->target_type = frame.payload[0];
            req->update_mask = frame.payload[1];
            req->dart_count = dart_count;
            memcpy(req->dart_ids, &frame.payload[3], dart_count);
            req->yaw_offset = ReadFloat(frame.payload, static_cast<uint8_t>(3U + dart_count));
            req->tension = ReadFloat(frame.payload, static_cast<uint8_t>(7U + dart_count));
            return DecodeResult::Ok;
        }

        case HOST_TYPE_GET_SEQUENCE:
            if (frame.len != 0) { return DecodeResult::LengthError; }
            req->kind = HOST_REQ_GET_SEQUENCE;
            return DecodeResult::Ok;

        case HOST_TYPE_SET_SEQUENCE:
            if (frame.len != 4) { return DecodeResult::LengthError; }
            req->kind = HOST_REQ_SET_SEQUENCE;
            memcpy(req->sequence, frame.payload, sizeof(req->sequence));
            return DecodeResult::Ok;

        case HOST_TYPE_GET_PRE_TENSION:
            if (frame.len != 0) { return DecodeResult::LengthError; }
            req->kind = HOST_REQ_GET_PRE_TENSION;
            return DecodeResult::Ok;

        case HOST_TYPE_SET_PRE_TENSION:
            if (frame.len != 4) { return DecodeResult::LengthError; }
            req->kind = HOST_REQ_SET_PRE_TENSION;
            req->pre_tension = ReadFloat(frame.payload, 0);
            return DecodeResult::Ok;

        case HOST_TYPE_RESET_TEST_ROUND:
            if (frame.len != 0) { return DecodeResult::LengthError; }
            req->kind = HOST_REQ_RESET_TEST_ROUND;
            return DecodeResult::Ok;

        default:
            req->kind = HOST_REQ_NONE;
            return DecodeResult::UnknownType;
    }
}

void HandleFrame(const HostFrame& frame, om_topic_t* hostreq_topic, om_topic_t* hosttx_topic)
{
    host_comm_stats.rx_frames_ok++;

    if (frame.version != HOSTCOMM_PROTOCOL_VERSION)
    {
        PublishError(hosttx_topic, frame.type, frame.seq, HOST_ERR_VERSION_UNSUPPORTED, frame.version);
        return;
    }

    msg_hostreq_t req{};
    const DecodeResult result = DecodeFrame(frame, &req);
    if (result != DecodeResult::Ok)
    {
        if (result == DecodeResult::LengthError)
        {
            host_comm_stats.rx_length_errors++;
            PublishError(hosttx_topic, frame.type, frame.seq, HOST_ERR_LENGTH_ERROR, frame.len);
        }
        else
        {
            PublishError(hosttx_topic, frame.type, frame.seq, HOST_ERR_UNKNOWN_TYPE, frame.type);
        }
        return;
    }

    om_publish(hostreq_topic, &req, sizeof(req), true, false);
}

void AssignHeaderByte(uint8_t index, uint8_t byte)
{
    switch (index)
    {
        case 0:
            parser_frame.version = byte;
            break;
        case 1:
            parser_frame.type = byte;
            break;
        case 2:
            parser_frame.seq = byte;
            break;
        case 3:
            parser_frame.len = byte;
            break;
        default:
            break;
    }
}

void ParseByte(uint8_t byte, om_topic_t* hostreq_topic, om_topic_t* hosttx_topic)
{
    switch (parser_state)
    {
        case ParseState::WaitSof1:
            if (byte == HOSTCOMM_SOF1)
            {
                parser_state = ParseState::WaitSof2;
                parser_frame = HostFrame{};
                parser_header_index = 0;
                parser_payload_index = 0;
            }
            break;

        case ParseState::WaitSof2:
            if (byte == HOSTCOMM_SOF2)
            {
                parser_state = ParseState::WaitHeader;
                parser_header_index = 0;
                parser_frame = HostFrame{};
            }
            else if (byte == HOSTCOMM_SOF1)
            {
                parser_state = ParseState::WaitSof2;
            }
            else
            {
                ResetParser();
            }
            break;

        case ParseState::WaitHeader:
            AssignHeaderByte(parser_header_index, byte);
            parser_header_index++;
            if (parser_header_index >= 4)
            {
                if (parser_frame.len > HOSTCOMM_MAX_PAYLOAD_SIZE)
                {
                    host_comm_stats.rx_length_errors++;
                    PublishError(hosttx_topic, parser_frame.type, parser_frame.seq,
                                 HOST_ERR_LENGTH_ERROR, parser_frame.len);
                    ResetParser();
                    break;
                }
                parser_payload_index = 0;
                parser_state = (parser_frame.len == 0) ? ParseState::WaitCrcLo : ParseState::WaitPayload;
            }
            break;

        case ParseState::WaitPayload:
            parser_frame.payload[parser_payload_index++] = byte;
            if (parser_payload_index >= parser_frame.len)
            {
                parser_state = ParseState::WaitCrcLo;
            }
            break;

        case ParseState::WaitCrcLo:
            parser_crc_lo = byte;
            parser_state = ParseState::WaitCrcHi;
            break;

        case ParseState::WaitCrcHi:
        {
            uint8_t crc_data[4 + HOSTCOMM_MAX_PAYLOAD_SIZE] = {0};
            crc_data[0] = parser_frame.version;
            crc_data[1] = parser_frame.type;
            crc_data[2] = parser_frame.seq;
            crc_data[3] = parser_frame.len;
            memcpy(&crc_data[4], parser_frame.payload, parser_frame.len);
            const uint16_t expected = Get_CRC16_Modbus_Check_Sum(crc_data, 4U + parser_frame.len, 0xFFFF);
            const uint16_t received = static_cast<uint16_t>(parser_crc_lo | (static_cast<uint16_t>(byte) << 8U));
            if (expected == received)
            {
                HandleFrame(parser_frame, hostreq_topic, hosttx_topic);
            }
            else
            {
                host_comm_stats.rx_crc_errors++;
                PublishError(hosttx_topic, parser_frame.type, parser_frame.seq, HOST_ERR_CRC_ERROR, received);
            }
            ResetParser();
            break;
        }
    }

    TouchParserTimer();
}

void DrainRxRing(om_topic_t* hostreq_topic, om_topic_t* hosttx_topic)
{
    uint8_t byte = 0;
    while (RingPop(&byte))
    {
        ParseByte(byte, hostreq_topic, hosttx_topic);
    }
}

HAL_StatusTypeDef StartUart10RxDma()
{
    InvalidateDCache(host_rx_dma_buffer, RX_DMA_BUFFER_SIZE);
    const HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart10, host_rx_dma_buffer, RX_DMA_BUFFER_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
        __HAL_DMA_ENABLE_IT(&hdma_usart10_rx, DMA_IT_TC);
    }
    else
    {
        host_comm_stats.rx_dma_errors++;
    }
    return status;
}

TxStartResult StartTxFrame(const msg_hosttx_t& tx)
{
    if (host_tx_busy)
    {
        host_comm_stats.tx_busy_count++;
        return TxStartResult::Busy;
    }

    if (!tx.valid || tx.len > HOSTCOMM_MAX_PAYLOAD_SIZE)
    {
        return TxStartResult::Error;
    }

    host_tx_dma_buffer[0] = HOSTCOMM_SOF1;
    host_tx_dma_buffer[1] = HOSTCOMM_SOF2;
    host_tx_dma_buffer[2] = HOSTCOMM_PROTOCOL_VERSION;
    host_tx_dma_buffer[3] = tx.type;
    host_tx_dma_buffer[4] = tx.seq;
    host_tx_dma_buffer[5] = tx.len;
    if (tx.len > 0)
    {
        memcpy(&host_tx_dma_buffer[6], tx.payload, tx.len);
    }

    const uint16_t crc = Get_CRC16_Modbus_Check_Sum(&host_tx_dma_buffer[2], 4U + tx.len, 0xFFFF);
    host_tx_dma_buffer[6 + tx.len] = static_cast<uint8_t>(crc & 0xFFU);
    host_tx_dma_buffer[7 + tx.len] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);

    const uint16_t frame_len = static_cast<uint16_t>(FRAME_OVERHEAD + tx.len);
    CleanDCache(host_tx_dma_buffer, frame_len);

    host_tx_busy = true;
    const HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart10, host_tx_dma_buffer, frame_len);
    if (status == HAL_OK)
    {
        return TxStartResult::Started;
    }

    host_tx_busy = false;
    if (status == HAL_BUSY)
    {
        host_comm_stats.tx_busy_count++;
        return TxStartResult::Busy;
    }

    host_comm_stats.tx_errors++;
    return TxStartResult::Error;
}

void ProcessTxQueue(om_fifo_t* hosttx_fifo)
{
    if (hosttx_fifo == nullptr || !om_fifo_readable(hosttx_fifo))
    {
        return;
    }

    if (host_tx_busy)
    {
        host_comm_stats.tx_busy_count++;
        return;
    }

    msg_hosttx_t tx{};
    if (om_fifo_peek(hosttx_fifo, &tx) != OM_OK)
    {
        return;
    }

    const TxStartResult result = StartTxFrame(tx);
    if (result == TxStartResult::Started || result == TxStartResult::Error)
    {
        om_fifo_pop(hosttx_fifo);
    }
}
}

void HostComm_RxEventCallback(uint16_t size)
{
    if (size > RX_DMA_BUFFER_SIZE)
    {
        size = RX_DMA_BUFFER_SIZE;
    }

    InvalidateDCache(host_rx_dma_buffer, size);
    for (uint16_t i = 0; i < size; i++)
    {
        if (RingPush(host_rx_dma_buffer[i]))
        {
            host_comm_stats.rx_bytes++;
        }
    }

    StartUart10RxDma();
    tx_semaphore_put(&HostCommRxSem);
}

void HostComm_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart != &huart10)
    {
        return;
    }

    if (host_tx_busy)
    {
        host_comm_stats.tx_frames_ok++;
    }
    host_tx_busy = false;
    tx_semaphore_put(&HostCommTxSem);
}

void HostComm_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (huart != &huart10)
    {
        return;
    }

    host_comm_stats.rx_dma_errors++;
    if (host_tx_busy)
    {
        host_tx_busy = false;
        host_comm_stats.tx_errors++;
        tx_semaphore_put(&HostCommTxSem);
    }
    StartUart10RxDma();
}

[[noreturn]] void HostCommThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    tx_semaphore_create(&HostCommRxSem, const_cast<char*>("HostCommRxSem"), 0);
    tx_semaphore_create(&HostCommTxSem, const_cast<char*>("HostCommTxSem"), 0);

    om_topic_t* hostreq_topic = om_config_topic(nullptr, "ca", "hostreq", sizeof(msg_hostreq_t));
    om_topic_t* hosttx_topic = om_config_topic(nullptr, "ca", "hosttx", sizeof(msg_hosttx_t));
    om_fifo_t* hosttx_fifo = om_queue_add(hosttx_topic, 32);

    __HAL_DMA_DISABLE_IT(&hdma_usart10_tx, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(&hdma_usart10_tx, DMA_IT_TC);
    __HAL_UART_SEND_REQ(&huart10, UART_RXDATA_FLUSH_REQUEST);
    ResetParser();
    StartUart10RxDma();

    for (;;)
    {
        tx_semaphore_get(&HostCommRxSem, 1);
        DrainRxRing(hostreq_topic, hosttx_topic);
        CheckParserTimeout();
        ProcessTxQueue(hosttx_fifo);
        tx_thread_sleep(1);
    }
}
