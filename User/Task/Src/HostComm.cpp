#include "HostComm.hpp"

#include <string.h>

#include "main.h"
#include "tx_api.h"
#include "usart.h"

#include "crc.hpp"
#include "magicmsgs.hpp"
#include "om.h"

TX_THREAD HostCommThread;
uint8_t HostCommThreadStack[1024] = {0};

extern UART_HandleTypeDef huart10;

namespace
{
constexpr uint8_t FRAME_OVERHEAD = 8;
constexpr uint32_t UART_TIMEOUT_MS = 5;
constexpr uint32_t UART_TX_TIMEOUT_MS = 20;

enum class ParseState : uint8_t
{
    WaitSof1,
    WaitSof2,
    WaitHeader,
    WaitPayload,
    WaitCrcLo,
    WaitCrcHi,
};

struct HostFrame
{
    uint8_t version = 0;
    uint8_t type = 0;
    uint8_t seq = 0;
    uint8_t len = 0;
    uint8_t payload[HOSTCOMM_MAX_PAYLOAD_SIZE] = {0};
};

void PutU16(uint8_t* out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value & 0xFFU);
    out[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void PublishError(om_topic_t* hosttx_topic, uint8_t ref_type, uint8_t ref_seq,
                  uint8_t error_code, uint16_t detail)
{
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

void SendFrame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len)
{
    uint8_t frame[FRAME_OVERHEAD + HOSTCOMM_MAX_PAYLOAD_SIZE] = {0};
    frame[0] = HOSTCOMM_SOF1;
    frame[1] = HOSTCOMM_SOF2;
    frame[2] = HOSTCOMM_PROTOCOL_VERSION;
    frame[3] = type;
    frame[4] = seq;
    frame[5] = len;
    if (payload != nullptr && len > 0)
    {
        memcpy(&frame[6], payload, len);
    }

    const uint16_t crc = Get_CRC16_Modbus_Check_Sum(&frame[2], 4U + len, 0xFFFF);
    frame[6 + len] = static_cast<uint8_t>(crc & 0xFFU);
    frame[7 + len] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);
    HAL_UART_Transmit(&huart10, frame, static_cast<uint16_t>(FRAME_OVERHEAD + len), UART_TX_TIMEOUT_MS);
}

float ReadFloat(const uint8_t* payload, uint8_t offset)
{
    float value = 0.0f;
    memcpy(&value, payload + offset, sizeof(value));
    return value;
}

bool DecodeFrame(const HostFrame& frame, msg_hostreq_t* req)
{
    if (req == nullptr)
    {
        return false;
    }

    memset(req, 0, sizeof(*req));
    req->valid = 1;
    req->seq = frame.seq;
    req->type = frame.type;

    switch (frame.type)
    {
        case HOST_TYPE_HEARTBEAT_REQ:
            if (frame.len != 0) { return false; }
            req->kind = HOST_REQ_HEARTBEAT;
            return true;

        case HOST_TYPE_GET_DART_TABLE:
            if (frame.len != 0) { return false; }
            req->kind = HOST_REQ_GET_DART_TABLE;
            return true;

        case HOST_TYPE_SET_DART_PARAMS_BATCH:
        {
            if (frame.len < 12) { return false; }
            const uint8_t dart_count = frame.payload[2];
            if (dart_count == 0 || dart_count > 8 || frame.len != static_cast<uint8_t>(11U + dart_count))
            {
                return false;
            }

            req->kind = HOST_REQ_SET_DART_PARAMS_BATCH;
            req->target_type = frame.payload[0];
            req->update_mask = frame.payload[1];
            req->dart_count = dart_count;
            memcpy(req->dart_ids, &frame.payload[3], dart_count);
            req->yaw_offset = ReadFloat(frame.payload, static_cast<uint8_t>(3U + dart_count));
            req->tension = ReadFloat(frame.payload, static_cast<uint8_t>(7U + dart_count));
            return true;
        }

        case HOST_TYPE_GET_SEQUENCE:
            if (frame.len != 0) { return false; }
            req->kind = HOST_REQ_GET_SEQUENCE;
            return true;

        case HOST_TYPE_SET_SEQUENCE:
            if (frame.len != 4) { return false; }
            req->kind = HOST_REQ_SET_SEQUENCE;
            memcpy(req->sequence, frame.payload, sizeof(req->sequence));
            return true;

        case HOST_TYPE_GET_PRE_TENSION:
            if (frame.len != 0) { return false; }
            req->kind = HOST_REQ_GET_PRE_TENSION;
            return true;

        case HOST_TYPE_SET_PRE_TENSION:
            if (frame.len != 4) { return false; }
            req->kind = HOST_REQ_SET_PRE_TENSION;
            req->pre_tension = ReadFloat(frame.payload, 0);
            return true;

        case HOST_TYPE_RESET_TEST_ROUND:
            if (frame.len != 0) { return false; }
            req->kind = HOST_REQ_RESET_TEST_ROUND;
            return true;

        default:
            req->kind = HOST_REQ_NONE;
            return false;
    }
}

void HandleFrame(const HostFrame& frame, om_topic_t* hostreq_topic, om_topic_t* hosttx_topic)
{
    if (frame.version != HOSTCOMM_PROTOCOL_VERSION)
    {
        PublishError(hosttx_topic, frame.type, frame.seq, HOST_ERR_VERSION_UNSUPPORTED, frame.version);
        return;
    }

    msg_hostreq_t req{};
    if (!DecodeFrame(frame, &req))
    {
        const uint8_t error = req.kind == HOST_REQ_NONE ? HOST_ERR_UNKNOWN_TYPE : HOST_ERR_LENGTH_ERROR;
        PublishError(hosttx_topic, frame.type, frame.seq, error, frame.len);
        return;
    }

    om_publish(hostreq_topic, &req, sizeof(req), true, false);
}

void ParseByte(uint8_t byte, om_topic_t* hostreq_topic, om_topic_t* hosttx_topic)
{
    static ParseState state = ParseState::WaitSof1;
    static HostFrame frame;
    static uint8_t header_index = 0;
    static uint8_t payload_index = 0;
    static uint8_t crc_lo = 0;

    switch (state)
    {
        case ParseState::WaitSof1:
            if (byte == HOSTCOMM_SOF1)
            {
                state = ParseState::WaitSof2;
            }
            break;

        case ParseState::WaitSof2:
            state = (byte == HOSTCOMM_SOF2) ? ParseState::WaitHeader : ParseState::WaitSof1;
            header_index = 0;
            frame = HostFrame{};
            break;

        case ParseState::WaitHeader:
        {
            uint8_t* header = reinterpret_cast<uint8_t*>(&frame);
            header[header_index++] = byte;
            if (header_index >= 4)
            {
                if (frame.len > HOSTCOMM_MAX_PAYLOAD_SIZE)
                {
                    PublishError(hosttx_topic, frame.type, frame.seq, HOST_ERR_LENGTH_ERROR, frame.len);
                    state = ParseState::WaitSof1;
                    break;
                }
                payload_index = 0;
                state = (frame.len == 0) ? ParseState::WaitCrcLo : ParseState::WaitPayload;
            }
            break;
        }

        case ParseState::WaitPayload:
            frame.payload[payload_index++] = byte;
            if (payload_index >= frame.len)
            {
                state = ParseState::WaitCrcLo;
            }
            break;

        case ParseState::WaitCrcLo:
            crc_lo = byte;
            state = ParseState::WaitCrcHi;
            break;

        case ParseState::WaitCrcHi:
        {
            uint8_t crc_data[4 + HOSTCOMM_MAX_PAYLOAD_SIZE] = {0};
            crc_data[0] = frame.version;
            crc_data[1] = frame.type;
            crc_data[2] = frame.seq;
            crc_data[3] = frame.len;
            memcpy(&crc_data[4], frame.payload, frame.len);
            const uint16_t expected = Get_CRC16_Modbus_Check_Sum(crc_data, 4U + frame.len, 0xFFFF);
            const uint16_t received = static_cast<uint16_t>(crc_lo | (static_cast<uint16_t>(byte) << 8U));
            if (expected == received)
            {
                HandleFrame(frame, hostreq_topic, hosttx_topic);
            }
            else
            {
                PublishError(hosttx_topic, frame.type, frame.seq, HOST_ERR_CRC_ERROR, received);
            }
            state = ParseState::WaitSof1;
            break;
        }
    }
}
}

[[noreturn]] void HostCommThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    om_topic_t* hostreq_topic = om_config_topic(nullptr, "ca", "hostreq", sizeof(msg_hostreq_t));
    om_topic_t* hosttx_topic = om_config_topic(nullptr, "ca", "hosttx", sizeof(msg_hosttx_t));
    om_fifo_t* hosttx_fifo = om_queue_add(hosttx_topic, 32);
    msg_hosttx_t tx{};

    for (;;)
    {
        uint8_t rx_byte = 0;
        if (HAL_UART_Receive(&huart10, &rx_byte, 1, UART_TIMEOUT_MS) == HAL_OK)
        {
            ParseByte(rx_byte, hostreq_topic, hosttx_topic);
        }

        while (om_fifo_readable(hosttx_fifo))
        {
            if (om_fifo_read(hosttx_fifo, &tx) == OM_OK &&
                tx.valid &&
                tx.len <= HOSTCOMM_MAX_PAYLOAD_SIZE)
            {
                SendFrame(tx.type, tx.seq, tx.payload, tx.len);
            }
        }

        tx_thread_sleep(1);
    }
}
