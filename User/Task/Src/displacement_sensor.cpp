#include "displacement_sensor.hpp"
#include "bsp_can.hpp"
#include "tx_api.h"

DisplacementSensor *DisplacementSensor::Instance(uint8_t node_id)
{
    static DisplacementSensor sensors[2];
    return (node_id >= 1U && node_id <= 2U) ? &sensors[node_id - 1U] : nullptr;
}

bool DisplacementSensor::Init(const Config &config)
{
    if (config.node_id ==0 || config.resolution_mm_per_count <=0.0f || config.timeout_ms < config.poll_period_ms)
    {
        return false;
    }
    cfg = config;
    raw_count = 0;
    raw_velocity = 0;
    stamp_ms = 0;
    rx_count = 0;
    pos_rx_count = 0;
    vel_rx_count = 0;
    err_count = 0;
    miss_count = 0;
    last_ack_func = 0xFF;
    last_status = 0xFF;
    state = SENSOR_OFFLINE;
    position_mm = 0.0f;
    velocity_mm_s = 0.0f;
    velocity_diff_mm_s = 0.0f;
    prev_rx_count_ = 0;
    prev_pos_rx_count_ = 0;
    prev_vel_rx_count_ = 0;
    prev_stamp_ms_ = 0;
    prev_position_mm_ = 0.0f;
    prev_raw_count_ = 0;
    velocity_sample_time = 30U;
    return true;
}

bool DisplacementSensor::Init()
{
    return Init(Config());
}

void DisplacementSensor::Poll()
{
    if (!SendInstruction(InstructionType::ReadPosition))
        if (miss_count < 0xFFFFFFFFU)
            miss_count++;
}

void DisplacementSensor::PollVelocity()
{
    if (!SendInstruction(InstructionType::ReadVelocity))
        if (miss_count < 0xFFFFFFFFU)
            miss_count++;
}

bool DisplacementSensor::SendInstruction(InstructionType func, const uint8_t *param, uint8_t param_len)
{
    if (param_len > 4U)
        return false;
    uint8_t msg[kMaxDlc];
    uint8_t msg_len;
    if (param == nullptr || param_len == 0U)
    {
        msg[0] = kReqLen;
        msg[1] = cfg.node_id;
        msg[2] = static_cast<uint8_t>(func);
        msg[3] = 0x00;
        msg_len = kReqLen;
    }
    else
    {
        msg[0] = static_cast<uint8_t>(param_len + kIdxData);
        msg[1] = cfg.node_id;
        msg[2] = static_cast<uint8_t>(func);
        for (uint8_t i = 0; i < param_len; i++)
            msg[kIdxData + i] = param[i];
        msg_len = static_cast<uint8_t>(param_len + kIdxData);
    }
    FDCAN_TxHeaderTypeDef header{};
    header.Identifier = cfg.node_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = static_cast<uint32_t>(msg_len);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(cfg.hcan, &header, msg) == HAL_OK;
}

bool DisplacementSensor::SetMode(SensorMode mode)
{
    const uint8_t param = static_cast<uint8_t>(mode);
    return SendInstruction(InstructionType::SetMode, &param, 1U);
}

bool DisplacementSensor::SetDirection(SensorDirection dir)
{
    const uint8_t param = static_cast<uint8_t>(dir);
    return SendInstruction(InstructionType::SetDirection, &param, 1U);
}

bool DisplacementSensor::SetReportPeriod(uint16_t new_time)
{
    const uint8_t param[2] = {static_cast<uint8_t>(new_time & 0xFF), static_cast<uint8_t>(new_time >> 8)};
    return SendInstruction(InstructionType::SetReportPeriod, param, 2U);
}

bool DisplacementSensor::SetVelocitySampleTime(uint16_t new_time)
{
    const uint8_t param[2] = {static_cast<uint8_t>(new_time >> 8), static_cast<uint8_t>(new_time & 0xFF)};
    if (SendInstruction(InstructionType::SetVelocitySampleTime, param, 2U))
    {
        velocity_sample_time = new_time;
        return true;
    }
    else
        return false;
}

bool DisplacementSensor::SetBaudRate(SensorBaudCode code)
{
    const uint8_t param = static_cast<uint8_t>(code);
    return SendInstruction(InstructionType::SetBaudRate, &param, 1U);
}

bool DisplacementSensor::SetCurrentNum(uint32_t new_count)
{
    const uint8_t param[4] = {static_cast<uint8_t>(new_count >> 24), static_cast<uint8_t>((new_count >> 16) & 0xFF),
                              static_cast<uint8_t>((new_count >> 8) & 0xFF), static_cast<uint8_t>(new_count & 0xFF)};
    if (SendInstruction(InstructionType::SetCurrentNum, param, 4U))
    {
        cfg.zero_count = new_count;
        return true;
    }
    else
        return false;
}

void DisplacementSensor::OnFrame(const uint8_t *data, uint16_t len)
{
    if (data == nullptr || len < 4U)
    {
        err_count++;
        return;
    }
    if (data[kIdxLen] < 4U || data[kIdxLen] > len)
    {
        err_count++;
        return;
    }
    if (data[kIdxAddr] != cfg.node_id)
    {
        err_count++;
        return;
    }
    const uint32_t stamp = tx_time_get();

    switch (static_cast<InstructionType>(data[kIdxFunc]))
    {
    case InstructionType::ReadPosition:
        if (data[kIdxLen] != kRespLenPos || len < kRespLenPos)
        {
            err_count++;
            return;
        }
        raw_count = static_cast<uint32_t>(data[kIdxData + 0]) |
                    (static_cast<uint32_t>(data[kIdxData + 1]) << 8) |
                    (static_cast<uint32_t>(data[kIdxData + 2]) << 16) |
                    (static_cast<uint32_t>(data[kIdxData + 3]) << 24);
        pos_rx_count++;
        break;
    case InstructionType::ReadVelocity:
        if (data[kIdxLen] != kRespLenPos || len < kRespLenPos)
        {
            err_count++;
            return;
        }
        raw_velocity = static_cast<int32_t>(
                      static_cast<uint32_t>(data[kIdxData + 0]) |
                      (static_cast<uint32_t>(data[kIdxData + 1]) << 8) |
                      (static_cast<uint32_t>(data[kIdxData + 2]) << 16) |
                      (static_cast<uint32_t>(data[kIdxData + 3]) << 24));
        vel_rx_count++;
        break;
    default:  //目前其他指令都是设置指令，最后一位都是0x00表示成功，其他表示失败
        // 已经确认一下自动回传对应的指令码是0x04还是对应的0x01、0x0A
        //TODO :将非ID更改的设置移到接收后完成
        if (data[kIdxLen] != kRespLenAck || len < kRespLenAck)
        {
            err_count++;
            return;
        }
        last_status = data[kIdxData];
        last_ack_func = data[kIdxFunc];
        break;
    }
    stamp_ms = stamp;       //现在是任何有效帧都可以被判断成活着
    rx_count++;
    miss_count = 0;
}

void DisplacementSensor::Update()
{
    const uint32_t pos_n = pos_rx_count;
    const uint32_t vel_n = vel_rx_count;
    const uint32_t stamp = stamp_ms;
    if (pos_n != prev_pos_rx_count_)
    {
        position_mm = Position();
        const uint32_t dt = stamp - prev_stamp_ms_;
        if (prev_stamp_ms_ != 0U && dt > 0U)
            velocity_diff_mm_s = (position_mm-prev_position_mm_) * 1000.0f / static_cast<float>(dt);
        prev_position_mm_ = position_mm;
        prev_stamp_ms_ = stamp;
        prev_pos_rx_count_ = pos_n;
    }
    if (vel_n != prev_vel_rx_count_)
    {
        // 公式是速度 = 角速度*轮周长(m)/单圈分辨率/转速采样时间(s)，而这里的数据mm_per_count单位mm，sample_time单位ms
        velocity_mm_s = static_cast<float>(raw_velocity) * static_cast<float>(cfg.direction) * cfg.resolution_mm_per_count / static_cast<float>(velocity_sample_time) * 1000.0f;
        prev_vel_rx_count_ = vel_n;
    }
    if (rx_count == prev_rx_count_)
    {
        if (miss_count < 0xFFFFFFFFU)
            miss_count++;
    }
    else
        prev_rx_count_ = rx_count;
    AliveCheck();
}

DisplacementSensor::SensorStateTypeDef DisplacementSensor::AliveCheck()
{
    const uint32_t now = tx_time_get();
    if (rx_count == 0U || now - stamp_ms > cfg.timeout_ms)
    {
        state = SENSOR_OFFLINE;
    }
    else
    {
        state = SENSOR_ONLINE;
    }
    return state;
}

float DisplacementSensor::Position() const
{
    const int64_t delta = static_cast<int64_t>(raw_count) - static_cast<int64_t>(cfg.zero_count);
    return delta * static_cast<int32_t>(cfg.direction) * cfg.resolution_mm_per_count;
}

void DisplacementSensor::SetZeroHard()
{
    if (SendInstruction(InstructionType::SetZero))
    {
        cfg.zero_count = 0U;
        raw_count = 0U;
    }
}

void DisplacementSensor::SetZeroSoft()
{
    cfg.zero_count = raw_count;
}