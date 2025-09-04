//
// Created by cosmosmount on 2025/8/30.
//

#include "filter.hpp"

namespace Filter
{
    FirstOrderFilter::FirstOrderFilter()
    {
        Input = 0.0f;
        OutPut = 0.0f;
        Tau = 0.0f;
        UpdatePeriod = 0.01f; // 默认10ms
    }

    float FirstOrderFilter::GetTau()
    {
        return Tau;
    }

    void FirstOrderFilter::Clear()
    {
        Input = 0.0f;
        OutPut = 0.0f;
        Tau = 0.0f;
        UpdatePeriod = 0.01f; // 默认10ms
    }

    void FirstOrderFilter::Update()
    {
        if (Tau <= 0.0f)
            return; // 防止除以零

        float alpha = UpdatePeriod / (Tau + UpdatePeriod);
        OutPut = OutPut + alpha * (Input - OutPut);
    }

    float FirstOrderFilter::GetResult()
    {
        return OutPut;
    }

    KalmanFilter::KalmanFilter()
    {
        LastP = 0.02f;
        NowP = 0.0f;
        result = 0.0f;
        Kg = 0.0f;
        Q = 0.001f;
        R = 0.543f;
    }

    void KalmanFilter::Clear()
    {
        LastP = 0.02f;
        NowP = 0.0f;
        result = 0.0f;
        Kg = 0.0f;
        Q = 0.001f;
        R = 0.543f;
    }

    void KalmanFilter::SetKg(float kg)
    {
        this->Kg = kg;
    }

    void KalmanFilter::SetQ(float q)
    {
        this->Q = q;
    }

    void KalmanFilter::SetR(float r)
    {
        this->R = r;
    }

    float KalmanFilter::Update(float input)
    {
        // 预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
        this->NowP = this->LastP + this->Q;

        // 卡尔曼增益方程：卡尔曼增益 = k1-1时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
        this->Kg = this->NowP * (1.0f / (this->NowP + this->R));
        // 更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
        this->result = this->result + this->Kg * (input - this->result); // 因为这一次的预测值就是上一次的输出值
        // 更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
        this->LastP = (1.0f - this->Kg) * this->NowP;

        return this->result;
    }
} // namespace Filter