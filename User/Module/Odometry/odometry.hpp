//
// Created by ASUS on 2025/10/15.
//

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "kalman_filter.h"

#define t  0.001f
#define t2 0.000001f
#define t3 0.000000001f
#define t4 0.000000000001f
#define t5 0.000000000000001f

struct odometry_info_t
{
    float x;
    float v;
    float a_z;
};

class cVelFusionKF
{
protected:
    const float qq = 5.0f;//10
    const float rv = 0.1f;
    const float ra = 50.0f;//25.0f

    const float A_Init[9] = {1, t, t2 / 2, 0, 1, t, 0, 0, 1};
    const float Q_Init[9] = {t5 / 20 * qq, t4 / 8 * qq, t3 / 6 * qq, t4 / 8 * qq, t3 / 3 * qq, t2 / 2 * qq, t3 / 6 * qq,
                             t2 / 2 * qq, t * qq};
    const float H_Init[6] = {0, 1, 0, 0, 0, 1};
    const float P_Init[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    const float R_Init[4] = {rv, 0, 0, ra};

public:
    KalmanFilter_t KF;

    cVelFusionKF()
    {
        Kalman_Filter_Init(&this->KF, 3, 0, 2);//Inertia odome 3 State 2 observation
        memcpy(this->KF.P_data, P_Init, sizeof(P_Init));
        memcpy(this->KF.F_data, A_Init, sizeof(A_Init));
        memcpy(this->KF.Q_data, Q_Init, sizeof(Q_Init));
        memcpy(this->KF.H_data, H_Init, sizeof(H_Init));
        memcpy(this->KF.R_data, R_Init, sizeof(R_Init));
    }

    void ResetKF(KalmanFilter_t *kf)
    {
        memset(kf->xhat.pData, 0, sizeof(float) * kf->xhat.numRows);
        memset(kf->xhatminus.pData, 0, sizeof(float) * kf->xhatminus.numRows);
        //	memcpy(F->P.pData,Pminus_data,sizeof(Pminus_data));
        memset(kf->P.pData, 0, sizeof(float) * kf->P.numRows*kf->P.numRows);
    }

    void UpdateKalman(float Velocity, float AccelerationX)
    {
        this->KF.MeasuredVector[0] = Velocity;
        this->KF.MeasuredVector[1] = AccelerationX;
        Kalman_Filter_Update(&this->KF);
        //VelFusionKF_Update(&this->KF);
    }

    float GetXhat()
    {
        return this->KF.xhat.pData[0];
    }

    float GetVhat()
    {
        return this->KF.xhat.pData[1];
    }

};

odometry_info_t Odometry_Update(float *_quaternion, float _vel, float *_acc);

#endif //ODOMETRY_HPP