#include "GM3508.hpp"
#include "bsp_can.hpp"
#include "main.h"

// for M3508
const float GM3508::RawPos2Rad = 0.0007670840321f; /*!< 2Pi / 8191 */
const float GM3508::RawRpm2Rps = 0.1047197551f;   /*!< 2Pi / 60 */

const float GM3508::RawPos2RadDiv19 = 0.00003994561794f; /*!< 2Pi / 8191 / 3591 * 187 */
const float GM3508::RawRpm2RpsDiv19 = 0.005453242609f;   /*!< 2Pi / 60 * 187 / 3591 */
const float GM3508::PIDiv19 = 0.1635972783f;             /*!< PI / 3591 X 187 */

const float GM3508::RawPos2RadDiv1576 = 0.00004865831547f; /*!< 2Pi / 8191 / 268 * 17 */
const float GM3508::RawRpm2RpsDiv1576 = 0.006642671034f;   /*!< 2Pi / 60 * 17 / 268 */
const float GM3508::PIDiv1576 = 0.199280131f;             /*!< PI / 268 X 17 */

/**
 * @brief GM3508类的构造函数。
 * 初始化电机的控制模式、各种设定值和PID控制器。
 */
GM3508::GM3508()
{
    // 初始化为松开模式
    controlMode = RELAX_MODE;

    // 初始化为0
    speedSet = 0;
    positionSet = 0;
    currentSet = 0;
    maxCurrent = 16384; // 电机最大电流设定

    // 初始化电机反馈数据
    motorFeedback.speedFdb = 0;
    motorFeedback.lastSpeedFdb = 0;
    motorFeedback.positionFdb = 0;
    motorFeedback.lastPositionFdb = 0;
    motorFeedback.temperatureFdb = 0;

    // pid初始化
    speedPid.mode = PID_POSITION;
    speedPid.kp = 0.1;
    speedPid.ki = 0.0;
    speedPid.kd = 0.0;
    speedPid.maxOut = 15000;
    speedPid.maxIOut = 3;

    positionPid.mode = PID_POSITION;
    positionPid.kp = 0.1;
    positionPid.ki = 0.0;
    positionPid.kd = 0.0;
    positionPid.maxOut = 15000;
    positionPid.maxIOut = 3;
}

/**
 * @brief GM6020类的析构函数。
 */
GM3508::~GM3508()
{
}

/**
 * @brief 设置电机输出。23
 * 根据当前控制模式，计算并设置电机的当前输出。
 */
void GM3508::setOutput()
{
    if (this->controlMode == TOR_MODE)
    {
        // 电流控制,currentSet = (torqueSet * 16384) / (0.3 * (15.76/19) * 20), 16384: 电机最大电流设定;20A: 电机最大电流;268/17: 减速比; 0.3: 3508转矩常数
        this->currentSet = this->torqueSet * 2889.69f;//3292.047377326565f;
        this->currentSet = Math::FloatConstrain(currentSet, -maxCurrent, maxCurrent);
    }
    else if (this->controlMode == SPD_MODE)
    {
        // 内环控制，速度环控制
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
        this->speedSet = this->positionPid.result;

        // 内环控制，速度环控制
        this->speedPid.ref = speedSet;
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

    if (this->controlMode == RELAX_MODE)
    {
        this->currentSet = 0.0; // 松开模式下电流设定为0
    }

    // 限制电流输出不超过最大值
    if (currentSet > maxCurrent)
        currentSet = maxCurrent;
    else if (currentSet < -maxCurrent)
        currentSet = -maxCurrent;
}

/**
 * @brief 更新电机传感器数据。
 * @param buffer_ptr 从CAN总线接收到的数据指针。
 * @todo 更具是否有减速箱，更新电机的位置和速度。
 */
void GM3508::UpdateSensorData(uint8_t *buffer_ptr)
{
    motorFeedback.ecd = (uint16_t)(buffer_ptr[0] << 8 | buffer_ptr[1]);
    motorFeedback.speed_rpm = (uint16_t)(buffer_ptr[2] << 8 | buffer_ptr[3]);
    motorFeedback.currentFdb = (uint16_t)(buffer_ptr[4] << 8 | buffer_ptr[5]);
    motorFeedback.temperatureFdb = (float)buffer_ptr[6];

    motorFeedback.lastSpeedFdb = motorFeedback.speedFdb;

    if (!HaveGearbox)
    {
        RotorPosition = motorFeedback.ecd * RawPos2Rad;

        motorFeedback.positionFdb = Math::LoopFloatConstrain(RotorPosition, -Math::Pi, Math::Pi);
        motorFeedback.speedFdb = motorFeedback.speed_rpm * RawRpm2Rps;
    }
    else if (HaveGearbox && IsXRoll)//xroll电机
    {
        RotorPosition = motorFeedback.ecd * RawPos2RadDiv1576 - PIDiv1576;

        motorFeedback.positionFdb += Math::LoopFloatConstrain((RotorPosition - LastRotorPosition), -PIDiv1576, PIDiv1576);
        motorFeedback.speedFdb = motorFeedback.speed_rpm * RawRpm2RpsDiv1576;
    }
    else//有减速箱时，计算的是负载轴的转速、位置、扭矩
    {
        RotorPosition = motorFeedback.ecd * RawPos2RadDiv19 - PIDiv19;

        motorFeedback.positionFdb += Math::LoopFloatConstrain((RotorPosition - LastRotorPosition), -PIDiv19, PIDiv19);
        motorFeedback.speedFdb = motorFeedback.speed_rpm * RawRpm2RpsDiv19;
    }
}