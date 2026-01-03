#include "QuaternionEKF.hpp"
#include <cstring>

QuaternionEKF QEKF_INS;

const float QuaternionEKF::F_Init_Data[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

float QuaternionEKF::P_Init_Data[36] = {
    100000, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 100000, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 100000, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 100000, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 100, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 100
};

QuaternionEKF::QuaternionEKF() 
    : Filter::KalmanFilter(6, 0, 3, F_Init_Data, nullptr, nullptr, nullptr, nullptr, P_Init_Data)
{
}

void QuaternionEKF::Init(float *init_quaternion, float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf)
{
    Initialized = true;
    Q1 = process_noise1;
    Q2 = process_noise2;
    R = measure_noise;
    ChiSquareTestThreshold = 1e-8;
    ConvergeFlag = false;
    ErrorCount = 0;
    UpdateCount = 0;
    if (lambda > 1) lambda = 1;
    this->lambda = lambda;
    accLPFcoef = lpf;

    // Initialize quaternion
    for (int i = 0; i < 4; i++)
    {
        xhat_data[i] = init_quaternion[i];
        xhatminus_data[i] = init_quaternion[i];
        q[i] = init_quaternion[i];
    }
    
    // Reset P
    std::memcpy(P_data, P_Init_Data, sizeof(P_Init_Data));
}

void QuaternionEKF::Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    this->dt = dt;

    Gyro[0] = gx - GyroBias[0];
    Gyro[1] = gy - GyroBias[1];
    Gyro[2] = gz - GyroBias[2];

    // Pre-calculate F matrix elements (linearization point)
    float halfgxdt = 0.5f * Gyro[0] * dt;
    float halfgydt = 0.5f * Gyro[1] * dt;
    float halfgzdt = 0.5f * Gyro[2] * dt;

    std::memcpy(F_data, F_Init_Data, sizeof(F_Init_Data));

    F_data[1] = -halfgxdt;
    F_data[2] = -halfgydt;
    F_data[3] = -halfgzdt;

    F_data[6] = halfgxdt;
    F_data[8] = halfgzdt;
    F_data[9] = -halfgydt;

    F_data[12] = halfgydt;
    F_data[13] = -halfgzdt;
    F_data[15] = halfgxdt;

    F_data[18] = halfgzdt;
    F_data[19] = halfgydt;
    F_data[20] = -halfgxdt;

    // Accel LPF
    if (UpdateCount == 0)
    {
        Accel[0] = ax;
        Accel[1] = ay;
        Accel[2] = az;
    }
    Accel[0] = Accel[0] * accLPFcoef / (dt + accLPFcoef) + ax * dt / (dt + accLPFcoef);
    Accel[1] = Accel[1] * accLPFcoef / (dt + accLPFcoef) + ay * dt / (dt + accLPFcoef);
    Accel[2] = Accel[2] * accLPFcoef / (dt + accLPFcoef) + az * dt / (dt + accLPFcoef);

    // Normalize Accel to get measurement vector z
    float accelInvNorm = invSqrt(Accel[0] * Accel[0] + Accel[1] * Accel[1] + Accel[2] * Accel[2]);
    MeasuredVector[0] = Accel[0] * accelInvNorm;
    MeasuredVector[1] = Accel[1] * accelInvNorm;
    MeasuredVector[2] = Accel[2] * accelInvNorm;

    // Stability check
    gyro_norm = 1.0f / invSqrt(Gyro[0] * Gyro[0] + Gyro[1] * Gyro[1] + Gyro[2] * Gyro[2]);
    accl_norm = 1.0f / accelInvNorm;

    if (gyro_norm < 0.3f && accl_norm > 9.8f - 0.5f && accl_norm < 9.8f + 0.5f)
    {
        StableFlag = true;
    }
    else
    {
        StableFlag = false;
    }

    // Update Q and R
    Q_data[0] = Q1 * dt;
    Q_data[7] = Q1 * dt;
    Q_data[14] = Q1 * dt;
    Q_data[21] = Q1 * dt;
    Q_data[28] = Q2 * dt;
    Q_data[35] = Q2 * dt;
    R_data[0] = R;
    R_data[4] = R;
    R_data[8] = R;

    // Call base Update
    Filter::KalmanFilter::Update();

    // Post-process
    q[0] = FilteredValue[0];
    q[1] = FilteredValue[1];
    q[2] = FilteredValue[2];
    q[3] = FilteredValue[3];
    GyroBias[0] = FilteredValue[4];
    GyroBias[1] = FilteredValue[5];
    GyroBias[2] = 0; 

    // Euler angles
    Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    Pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.295779513f;
    Roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;

    // Yaw wrapping
    if (Yaw - YawAngleLast > 180.0f)
    {
        YawRoundCount--;
    }
    else if (Yaw - YawAngleLast < -180.0f)
    {
        YawRoundCount++;
    }
    YawTotalAngle = 360.0f * YawRoundCount + Yaw;
    YawAngleLast = Yaw;
    UpdateCount++;
}

void QuaternionEKF::xhatMinusUpdate()
{
    // Standard prediction: x- = F * x
    Filter::KalmanFilter::xhatMinusUpdate();
    
    // Custom linearization and fading
    F_Linearization_P_Fading();
}

void QuaternionEKF::F_Linearization_P_Fading()
{
    float q0 = xhatminus_data[0];
    float q1 = xhatminus_data[1];
    float q2 = xhatminus_data[2];
    float q3 = xhatminus_data[3];

    float qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    xhatminus_data[0] *= qInvNorm;
    xhatminus_data[1] *= qInvNorm;
    xhatminus_data[2] *= qInvNorm;
    xhatminus_data[3] *= qInvNorm;
    
    q0 = xhatminus_data[0];
    q1 = xhatminus_data[1];
    q2 = xhatminus_data[2];
    q3 = xhatminus_data[3];

    // Update F matrix for P update
    F_data[4] = q1 * dt / 2;
    F_data[5] = q2 * dt / 2;

    F_data[10] = -q0 * dt / 2;
    F_data[11] = q3 * dt / 2;

    F_data[16] = -q3 * dt / 2;
    F_data[17] = -q0 * dt / 2;

    F_data[22] = q2 * dt / 2;
    F_data[23] = -q1 * dt / 2;

    // Fading filter
    P_data[28] /= lambda;
    P_data[35] /= lambda;

    // Limit P
    if (P_data[28] > 10000) P_data[28] = 10000;
    if (P_data[35] > 10000) P_data[35] = 10000;
}

void QuaternionEKF::setK()
{
    SetH();
    Filter::KalmanFilter::setK();
}

void QuaternionEKF::SetH()
{
    float doubleq0 = 2 * xhatminus_data[0];
    float doubleq1 = 2 * xhatminus_data[1];
    float doubleq2 = 2 * xhatminus_data[2];
    float doubleq3 = 2 * xhatminus_data[3];

    std::memset(H_data, 0, sizeof(float) * zSize * xhatSize);

    H_data[0] = -doubleq2;
    H_data[1] = doubleq3;
    H_data[2] = -doubleq0;
    H_data[3] = doubleq1;

    H_data[6] = doubleq1;
    H_data[7] = doubleq0;
    H_data[8] = doubleq3;
    H_data[9] = doubleq2;

    H_data[12] = doubleq0;
    H_data[13] = -doubleq1;
    H_data[14] = -doubleq2;
    H_data[15] = doubleq3;
}

void QuaternionEKF::xhatUpdate()
{
    // Calculate h(x-)
    float q0 = xhatminus_data[0];
    float q1 = xhatminus_data[1];
    float q2 = xhatminus_data[2];
    float q3 = xhatminus_data[3];

    // Predicted gravity direction
    temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // Calculate residual y = z - h(x-)
    // z is in z_data (copied from MeasuredVector in measure())
    
    // Set dimensions for 3x1 operations
    temp_vector.numRows = 3;
    temp_vector1.numRows = 3;
    
    // temp_vector1 = z - h(x-)
    arm_mat_sub_f32(&z, &temp_vector, &temp_vector1);

    // Chi-square test
    // r' * invS * r
    // invS is in temp_matrix1 (from setK)
    // temp_vector = invS * r
    arm_mat_mult_f32(&temp_matrix1, &temp_vector1, &temp_vector);
    
    // result = r . (invS * r) = dot(r, temp_vector)
    float chiSquare = 0;
    arm_dot_prod_f32(temp_vector1.pData, temp_vector.pData, 3, &chiSquare);
    
    ChiSquare_Data[0] = chiSquare;

    if (chiSquare < ChiSquareTestThreshold || (StableFlag && chiSquare < 0.1f)) // Divergence protection
    {
        ConvergeFlag = true;
    }
    else
    {
        ConvergeFlag = false;
        if (chiSquare > 0.1f) ErrorCount++;
    }

    if (ConvergeFlag)
    {
        // x = x- + K * r
        // K is 6x3. r (temp_vector1) is 3x1.
        // Result is 6x1.
        
        temp_vector.numRows = 6;
        
        arm_mat_mult_f32(&K, &temp_vector1, &temp_vector);
        arm_mat_add_f32(&xhatminus, &temp_vector, &xhat);
    }
    else
    {
        // x = x-
        std::memcpy(xhat_data, xhatminus_data, sizeof(float) * xhatSize);
    }
    
    // Restore dimensions
    temp_vector.numRows = xhatSize;
    temp_vector1.numRows = xhatSize;
}

float QuaternionEKF::invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
