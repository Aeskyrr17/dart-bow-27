//
// Created by ASUS on 2025/10/15.
//

#include "odometry.hpp"
#include "arm_qua"

static cVelFusionKF vel_kf;

/**
 * @brief odemetry update function
 * @note all the params must be homography
 * @param _quaternion
 * @param _vel
 * @param _acc
 * @return
 */
odometry_info_t Odometry_Update(float *_quaternion, float _vel, float *_acc)
{
    odometry_info_t odometry_info;
    odometry_info.x = 0.0f;
    odometry_info.v = 0.0f;
    odometry_info.a_z = 0.0f;

    float temp[4] = {0};
    float a_world[4] = {0};

    // arm_quaternion

    return odometry_info;
}