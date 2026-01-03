#pragma once

#include "kalmanfilter.hpp"
#include "arm_math.h"

class QuaternionEKF : public Filter::KalmanFilter
{
public:
    QuaternionEKF();
    
    void Init(float *init_quaternion, float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
    void Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    // Public data members to maintain compatibility with previous struct usage
    bool Initialized = false;
    bool ConvergeFlag = false;
    bool StableFlag = false;
    uint64_t ErrorCount = 0;
    uint64_t UpdateCount = 0;

    float q[4];        // Quaternion estimate
    float GyroBias[3]; // Gyro bias estimate

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef = 0;
    float gyro_norm = 0;
    float accl_norm = 0;
    float AdaptiveGainScale = 0;

    float Roll = 0;
    float Pitch = 0;
    float Yaw = 0;

    float YawTotalAngle = 0;
    float YawAngleLast = 0;
    int16_t YawRoundCount = 0;

    float Q1 = 0; // Quaternion process noise
    float Q2 = 0; // Gyro bias process noise
    float R = 0;  // Accel measure noise

    float dt = 0; // Update period
    
    float ChiSquare_Data[1];      // Chi-square test data
    float ChiSquareTestThreshold = 0; // Chi-square test threshold
    float lambda = 0;                 // Fading coefficient

protected:
    // Override virtual functions from KalmanFilter
    void xhatMinusUpdate() override;
    void setK() override;
    void xhatUpdate() override;

private:
    void F_Linearization_P_Fading();
    void SetH();
    static float invSqrt(float x);

    // Initial matrices
    static const float F_Init_Data[36];
    static float P_Init_Data[36];
};

extern QuaternionEKF QEKF_INS;
