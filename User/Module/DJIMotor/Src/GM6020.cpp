#include "GM6020.hpp"
#include "bsp_can.hpp"

const float GM6020::RawPos2Rad = 0.0007669903939f; // （7.6699039394282061485904379474597e-4）位置值转换为弧度的转换因子，编码器为十三位，2^13 = 8192, 2 * PI / 8192 (rad)，这样，当编码器的值增加或减少 1 时，它表示电机轴旋转了 2 * PI / 8192 (rad)
const float GM6020::RawRpm2Rps = 0.1047197551196f; // 0.1047197551f;  // RPM 到弧度每秒的转换因子, 2 * PI / 60 (s)

/**
 * @brief GM6020类的构造函数。
 * 初始化电机的控制模式、各种设定值和PID控制器。
 */
GM6020::GM6020()
{
    // 初始化为松开模式
    controlMode = RELAX_MODE;

    // 初始化设定值为0
    speedSet = 0;
    positionSet = 0;
    currentSet = 0;
    maxCurrent = 25000; // 电机最大电压设定

    // 初始化电机反馈数据
    motorFeedback.speedFdb = 0;
    motorFeedback.lastSpeedFdb = 0;
    motorFeedback.positionFdb = 0;
    motorFeedback.lastPositionFdb = 0;
    motorFeedback.temperatureFdb = 0;

    // PID控制器初始化
    speedPid.mode = PID_POSITION;
    speedPid.kp = 0.1;
    speedPid.ki = 0.0;
    speedPid.kd = 0.0;
    speedPid.maxOut = 25000;
    speedPid.maxIOut = 3;

    positionPid.mode = PID_POSITION;
    positionPid.kp = 0.1;
    positionPid.ki = 0.0;
    positionPid.kd = 0.0;
    positionPid.maxOut = 25000;
    positionPid.maxIOut = 3;
}

/**
 * @brief GM6020类的析构函数。
 */
GM6020::~GM6020()
{
}

/**
 * @brief 设置电机输出。
 * 根据当前控制模式，计算并设置电机的当前输出。
 */
void GM6020::setOutput()
{
    if (this->controlMode == RELAX_MODE)
    {
        this->currentSet = 0.0; // 松开模式下电流设定为0
        return;
    }
    else if (this->controlMode == SPD_MODE)
    {
        this->speedPid.ref = this->speedSet;
        this->speedPid.fdb = this->motorFeedback.speedFdb;
        this->speedPid.UpdateResult();

        this->currentSet = this->speedPid.result; // 根据速度PID结果设置电流
    }
    else if (this->controlMode == POS_MODE)
    {
        // 外环控制，位置环控制
        this->positionPid.ref = this->positionSet;
        this->positionPid.fdb = this->motorFeedback.positionFdb;
        this->positionPid.UpdateResult();

        // 内环控制，速度环控制
        this->speedPid.ref = this->positionPid.result;
        this->speedPid.fdb = this->motorFeedback.speedFdb;
        this->speedPid.UpdateResult();

        this->currentSet = this->speedPid.result; // 根据速度PID结果设置电流
    }
    // 其他控制模式下的电流设定逻辑同样待确定
    else if (this->controlMode == POS_FOR_NO_SPD_MODE || this->controlMode == IMU_MODE)
    {
        this->currentSet = 0; // 具体控制逻辑未定义
    }
    else
    {
        this->currentSet = 0; // 其他情况电流设定为0
    }

    // 限制电流输出不超过最大值
    if (currentSet > maxCurrent)
        currentSet = maxCurrent;
    else if (currentSet < -maxCurrent)
        currentSet = -maxCurrent;
}

void GM6020::UpdateSensorData(uint8_t *buffer_ptr)
{
    motorFeedback.ecd = (uint16_t)(buffer_ptr[0] << 8 | buffer_ptr[1]);
    motorFeedback.speed_rpm = (uint16_t)(buffer_ptr[2] << 8 | buffer_ptr[3]);
    motorFeedback.currentFdb = (uint16_t)(buffer_ptr[4] << 8 | buffer_ptr[5]);
    motorFeedback.temperatureFdb = (float)buffer_ptr[6];

    /* update last time speed and position -------------------------------------------*/
    motorFeedback.lastPositionFdb = motorFeedback.positionFdb;
    motorFeedback.lastSpeedFdb = motorFeedback.speedFdb;

    /* update member variables in MotorFeedback --------------------------------------*/
    motorFeedback.positionFdb = Math::LoopFloatConstrain((float)((motorFeedback.ecd - Offset) * RawPos2Rad), -Math::Pi, Math::Pi);
    motorFeedback.speedFdb = motorFeedback.speed_rpm * RawRpm2Rps;
}
