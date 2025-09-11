#pragma once
#ifndef TASKWHEEL_HPP
#define TASKWHEEL_HPP

#include "cstdint"
#include "tx_api.h"


#include "GMMotorHandler.hpp"
#include "LKMotorHandler.hpp"
#include "LK9025.hpp"
#include "LK8016.hpp"
#include "GM3508.hpp"
#include "GM6020.hpp"


class ServiceMotors
{
public:
    // LK9025 RMotor;
    // LK9025 LMotor;

    // LK8016 RD;
    // LK8016 RU;

    // LK8016 LD;
    // LK8016 LU;

    GM3508 LWheel;
    GM3508 RWheel;

    void MotorRegister();
    void AllMotorSetOutput();
    void SetModeAndPidParam();
};


#endif