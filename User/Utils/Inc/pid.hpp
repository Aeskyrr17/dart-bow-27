//
// Created by cosmosmount on 2025/8/29.
//

#ifndef RM26_PID_HPP
#define RM26_PID_HPP

#include "math.hpp"
using namespace Numeric;

/**
 * @brief PID模式
 */
enum PidModeType
{
    PID_POSITION = 0x01,                  // 位置式PID，0000 0000
    PID_DELTA = 0x02,                     // 增量式PID，0000 0001
    PID_Trapezoid_Intergral = 0x04,       // 梯形积分，0000 0010
    PID_Changing_Integral_Rate = 0x08,    // 变速积分，0000 0100
    PID_Integral_Separation = 0x10,       // 积分分离，0000 1000
    PID_Derivative_On_Measurement = 0x20, // 微分先行，0001 0000
    PID_Integral_Limit = 0x40,             // 积分限幅，0010 0000
    PID_Derivative_Incomplete = 0x80,       // 不完全微分，0100 0000
};

/**
 * @brief PID类
 */
class PID
{
public:
    uint8_t mode;

    float kp;
    float ki;
    float kd;

    float ScalarA; // 变速积分分离系数
    float ScalarB; // 变速积分分离系数

    float tau;

    float fdf;
    float ref;
    float fdb;
    float last_fbd;
    float last_ref;
    float err[3]{};

    float pResult;
    float iResult;
    float dResult;
    float iTerm;
    float result;

    float maxOut;
    float maxIOut;

    float deadband;

    // 计数，防止过快响应作出错误判断
    int errorcount; // 堵转计数
    int rightcount; // 正常工作计数

    // 标志
    bool Motorblocked;
    bool Motornormal;

    PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode = PID_POSITION);
    void Tuning(float tuning_kp, float tuning_ki, float tuning_kd);
    void UpdateResult();
    void Clear();
};

static void f_Trapezoid_Intergral(PID *pid)
{
    pid->iTerm = pid->ki * ((pid->err[0] + pid->err[1]) / 2.0f);
}

static void f_Derivative_On_Measurement(PID *pid)
{
    pid->dResult = pid->kd * (pid->last_fbd - pid->fdb);
}

static void f_Integral_Separation(PID *pid)
{
    // 首先判断该周期内积分项是否为积累趋势
    // 只有本次积分为积累趋势才会取消积分项
    if (pid->err[0] * pid->iResult > 0)
    {
        if (fabs(pid->err[0]) <= pid->maxOut)
            return; // 完整积分
        else
            pid->iTerm = 0; // 取消积分环节
    }
}

static void f_Changing_Integral_Rate(PID *pid)
{
    if (pid->err[0] * pid->iResult > 0)
    {
        if (fabs(pid->err[0]) <= pid->ScalarB)
            return; // 完整积分
        if (fabs(pid->err[0]) <= (pid->ScalarA + pid->ScalarB))
            // 使用线性函数过渡
            pid->iTerm *= (pid->ScalarA - fabs(pid->err[0]) + pid->ScalarB) / pid->ScalarA;
        else
            pid->iTerm = 0; // 取消积分环节
    }
}

static void f_Integral_Limit(PID *pid)
{
    float temp_Output, temp_Iout;
    temp_Iout = pid->iResult + pid->iTerm;
    temp_Output = pid->pResult + pid->iResult + pid->dResult;
    if (fabs(temp_Output) > pid->maxOut)
    {
        if (pid->err[0] * pid->iResult > 0)
        {
            // 在取消积分作用前，同样需要先判断当前周期内积分是否积累
            // 如果积分为减小趋势则不需要限制其减小
            // 原因与（三）中相同。
            pid->iTerm = 0;
        }
    }

    if (temp_Iout > pid->maxIOut)
    {
        pid->iTerm = 0;
        pid->iResult = pid->maxIOut;
    }
    if (temp_Iout < -pid->maxIOut)
    {
        pid->iTerm = 0;
        pid->iResult = -pid->maxIOut;
    }

    // pid->iTerm = FloatConstrain(pid->iTerm, -pid->maxIOut, pid->maxIOut);
}

#endif //RM26_PID_HPP