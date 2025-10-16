//
// Created by ASUS on 2025/10/15.
//

#include "odometry.hpp"
#include "quaternion_math_functions.h"

static cVelFusionKF vel_kf;

/**
 * @brief odemetry update function
 * @note all the params must be homography
 * @param _quaternion
 * @param _acc
 * @param _vel
 * @param _yaw in degree
 * @return odometry_info
 */
odometry_info_t Odometry_Update(float *_quaternion, float *_acc, float _vel, float _yaw)
{
    odometry_info_t odometry_info;
    odometry_info.x = 0.0f;
    odometry_info.v = 0.0f;
    odometry_info.a_z = 0.0f;

    float temp[4] = {0};
    float a_world[4] = {0};

    arm_quaternion_product_f32(_quaternion,_acc,temp,1);
    arm_quaternion_product_f32(temp,_quaternion, a_world, 1);

    float a_x = sqrtf(a_world[1] * a_world[1] + a_world[2] * a_world[2]) *
    arm_cos_f32(atan2f(a_world[2], a_world[1]) - _yaw*0.017453293f);

    vel_kf.UpdateKalman(_vel, a_x);

    odometry_info.v = vel_kf.GetVhat();
    odometry_info.x = vel_kf.GetXhat();
    odometry_info.a_z = a_world[3];

    return odometry_info;
}