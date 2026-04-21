#include "DM4310_MultiPos.hpp"

#include "math.hpp"

using namespace Numeric;

DM4310_MultiPos::DM4310_MultiPos()
{
    controlMode = RELAX_MODE;
    speedSet = 0.0f;
    positionSet = 0.0f;
    torqueSet = 0.0f;

    motorFeedback.ID = 0;
    motorFeedback.ERR = ERR_DISABLE;
    motorFeedback.speedFdb = 0.0f;
    motorFeedback.positionFdb = 0.0f;
    motorFeedback.torqueFdb = 0.0f;
    motorFeedback.temMOS = 0.0f;
    motorFeedback.temRotor = 0.0f;

    rawPositionFdb = 0.0f;
    lastRawPositionFdb = 0.0f;
    positionRoundCount = 0;
    positionInited = false;

    P_MAX = 12.56637061f;
    P_MIN = -12.56637061f;

    V_MAX = 15.0f;
    V_MIN = -15.0f;

    T_MAX = 10.0f;
    T_MIN = -10.0f;

    AliveFlag = 0;
    Pre_Flag = 0;
    motorState = MOTOR_OFFLINE;

    LowerPosLimit = P_MIN;
    UpperPosLimit = P_MAX;
    offset = 0.0f;
    KP = 0.0f;
    KD = 0.0f;
}

DM4310_MultiPos::MotorStateTypeDef DM4310_MultiPos::AliveCheck()
{
    if (AliveFlag == Pre_Flag)
    {
        motorState = MOTOR_OFFLINE;
    }
    else
    {
        Pre_Flag = AliveFlag;
        motorState = MOTOR_ONLINE;
    }
    return motorState;
}

void DM4310_MultiPos::SetOutput()
{
    uint8_t OutputData[8] = {0};
    uint32_t CAN_ID = this->canId;

    this->positionSet = FloatConstrain(this->positionSet, this->LowerPosLimit, this->UpperPosLimit);
    switch (this->controlMode)
    {
    case DMMotor::RELAX_MODE:
    {
        memcpy(OutputData, Disable_Frame, 8);
        break;
    }
    case DMMotor::MIT_MODE:
    {
        float position_set = LoopFloatConstrain(this->positionSet + this->offset, -Pi, Pi);
        uint16_t pos_tmp = float_to_uint(position_set, this->Get_P_MIN(), this->Get_P_MAX(), 16);
        uint16_t vel_tmp = float_to_uint(this->speedSet, this->Get_V_MIN(), this->Get_V_MAX(), 12);
        uint16_t kp_tmp = float_to_uint(this->KP, KP_MIN, KP_MAX, 12);
        uint16_t kd_tmp = float_to_uint(this->KD, KD_MIN, KD_MAX, 12);
        uint16_t tor_tmp = float_to_uint(this->torqueSet, this->Get_T_MIN(), this->Get_T_MAX(), 12);
        OutputData[0] = (pos_tmp >> 8);
        OutputData[1] = pos_tmp;
        OutputData[2] = (vel_tmp >> 4);
        OutputData[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
        OutputData[4] = kp_tmp;
        OutputData[5] = (kd_tmp >> 4);
        OutputData[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
        OutputData[7] = tor_tmp;
        break;
    }
    case DMMotor::POS_SPD_MODE:
    {
        CAN_ID += 0x100;
        // float position_set = LoopFloatConstrain(this->positionSet + this->offset, -Pi, Pi);
        // uint8_t *pbuf = reinterpret_cast<uint8_t *>(&position_set);
        // uint8_t *vbuf = reinterpret_cast<uint8_t *>(&this->speedSet);
        //
        // OutputData[0] = *pbuf;
        // OutputData[1] = *(pbuf + 1);
        // OutputData[2] = *(pbuf + 2);
        // OutputData[3] = *(pbuf + 3);
        // OutputData[4] = *vbuf;
        // OutputData[5] = *(vbuf + 1);
        // OutputData[6] = *(vbuf + 2);
        // OutputData[7] = *(vbuf + 3);
        memset(OutputData, 0, sizeof(OutputData));
        break;
    }
    case DMMotor::SPD_MODE:
    {
        CAN_ID += 0x200;
        uint8_t *vbuf = reinterpret_cast<uint8_t *>(&this->speedSet);
        OutputData[0] = *vbuf;
        OutputData[1] = *(vbuf + 1);
        OutputData[2] = *(vbuf + 2);
        OutputData[3] = *(vbuf + 3);
        break;
    }
    default:
    {
        memcpy(OutputData, Disable_Frame, 8);
        break;
    }
    }

    CAN_Transmit(this->hcan, CAN_ID, OutputData, 8);
}

void DM4310_MultiPos::ReceiveData(uint8_t *buffer)
{
    this->AliveFlag++;
    this->motorFeedback.ID = buffer[0] & 0x0F;
    this->motorFeedback.ERR = static_cast<DMMotor::MotorErrorTypeDef>(buffer[0] >> 4);

    const uint16_t p_int = (buffer[1] << 8) | buffer[2];
    const uint16_t v_int = (buffer[3] << 4) | (buffer[4] >> 4);
    const uint16_t t_int = ((buffer[4] & 0x0F) << 8) | buffer[5];

    lastRawPositionFdb = rawPositionFdb;
    rawPositionFdb = LoopFloatConstrain(uint_to_float(p_int, this->Get_P_MIN(), this->Get_P_MAX(), 16), -Pi, Pi);

    if (!positionInited)
    {
        lastRawPositionFdb = rawPositionFdb;
        positionRoundCount = 0;
        positionInited = true;
    }
    else
    {
        const float delta_pos = rawPositionFdb - lastRawPositionFdb;
        if (delta_pos > Pi)
        {
            positionRoundCount--;
        }
        else if (delta_pos < -Pi)
        {
            positionRoundCount++;
        }
    }

    const float continuous_position = rawPositionFdb + positionRoundCount * 2.0f * Pi;
    this->motorFeedback.positionFdb = continuous_position;
    this->motorFeedback.speedFdb = uint_to_float(v_int, this->Get_V_MIN(), this->Get_V_MAX(), 12);
    this->motorFeedback.torqueFdb = uint_to_float(t_int, this->Get_T_MIN(), this->Get_T_MAX(), 12);
    this->motorFeedback.temMOS = buffer[6];
    this->motorFeedback.temRotor = buffer[7];
}
