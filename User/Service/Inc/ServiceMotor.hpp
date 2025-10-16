#pragma once
#ifndef SERVICE_MOTOR_HPP
#define SERVICE_MOTOR_HPP

#include "cstdint"
#include "tx_api.h"


#include "DJIMotorHandler.hpp"
#include "LKMotorHandler.hpp"
#include "LK9025.hpp"
#include "LK8016.hpp"

class ServiceMotors
{
public:
    LK9025 RWheel;
    LK9025 LWheel;

    LK8016 RD;
    LK8016 RU;

    LK8016 LD;
    LK8016 LU;

    void MotorRegister();
    void AllMotorSetOutput();
    void SetModeAndPidParam();
};


#endif