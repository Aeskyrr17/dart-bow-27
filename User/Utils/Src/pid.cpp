//
// Created by cosmosmount on 2025/8/30.
//

#include "pid.hpp"

PID::PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode)
    : mode(mode), kp(kp), ki(ki), kd(kd), maxOut(maxOut), maxIOut(maxIOut)
{
    fdb = last_fbd = 0.0f;
    ref = last_ref = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    ScalarA = ScalarB = 0.0f;
    fdf = 0.0f;
    pResult = iResult = dResult = 0.0f;
    iTerm = 0.0f;
    tau = 0.0f;
    result = 0.0f;
    Motorblocked = false;
    Motornormal = true;
    errorcount = rightcount = 0;
    deadband = 0.0f;
}

void PID::Tuning(float tuning_kp, float tuning_ki, float tuning_kd) {
    kp = tuning_kp;
    ki = tuning_ki;
    kd = tuning_kd;
}


void PID::UpdateResult(void)
{
    // 更新误差缓存
    err[2] = err[1];
    err[1] = err[0];
    err[0] = ref - fdb;

    // if (mode == PID_POSITION)
    // {
    //     // 计算比例输出
    //     pResult = kp * err[0];

    //     // 计算微分输出
    //     dResult = kd * (err[0] - err[1]);
    // }
    // else if (mode == PID_DELTA)
    // {
    //     pResult = kp * (err[0] - err[1]);

    //     iResult = ki * err[0];
    //     iResult = LimitABS(iResult, maxIOut);

    //     dResult = kd * (err[0] - 2.0f * err[1] + err[2]);
    // }

    pResult = kp * err[0];
    dResult = kd * (err[0] - err[1]);
    iTerm = ki * err[0];

    if (mode & PID_Trapezoid_Intergral)
        f_Trapezoid_Intergral(this); // 梯形积分
    if (mode & PID_Derivative_On_Measurement)
        f_Derivative_On_Measurement(this); // 微分先行
    if (mode & PID_Integral_Separation)
        f_Integral_Separation(this); // 积分分离
    if (mode & PID_Changing_Integral_Rate)
        f_Changing_Integral_Rate(this); // 变速积分
    if (mode & PID_Integral_Limit)
        f_Integral_Limit(this); // 积分限幅

    iResult += iTerm;
    iResult = LimitABS(iResult, maxIOut);

    // 更新反馈值缓存
    last_fbd = fdb;
    // 计算输出
    result = pResult + iResult + dResult;
    result = LimitABS(result, maxOut);

    // // 更新微分输出缓存
    // last_dResult = dResult;
}

void PID::Clear()
{
    last_fbd = fdb = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
    ref = fdb = 0.0f;
    iResult = 0.0f;
    pResult = 0.0f;
    dResult = 0.0f;

    ScalarA = 1.0f;
    ScalarB = 1.0f;
}
