//
// Created by cosmosmount on 2025/9/10.
//

#include "GimbalStates.hpp"
#include "../Inc/GimbalStates.hpp"

namespace GimbalStates {
    void Relax::init() {
        YawMotor->controlMode = GM6020::RELAX_MODE;
        YawMotor->speedPid.Clear();
        YawMotor->positionPid.Clear();

        PitchMotor->controlMode = GM6020::RELAX_MODE;
        PitchMotor->speedPid.Clear();
        PitchMotor->positionPid.Clear();
    }

    void Relax::enter() {
        YawMotor->controlMode = GM6020::RELAX_MODE;
        PitchMotor->controlMode = GM6020::RELAX_MODE;
    }

    void Run::init() {
        PitchPosFilter.SetQ(0.001f);
        PitchPosFilter.SetR(0.543f);
        YawPosFilter.SetQ(0.001f);
        YawPosFilter.SetR(0.543f);
    }

    void Run::enter() {
        /*-----------------------------------------云台Yaw轴电机-----------------------------------------*/
        YawMotor->controlMode = GM6020::SPD_MODE;
        // 位置环PID参数
        YawMotor->positionPid.mode = PID_POSITION;
        YawMotor->positionPid.kp = 60.0f;
        YawMotor->positionPid.ki = 1.2f;
        YawMotor->positionPid.kd = 3000.0f;
        YawMotor->positionPid.ScalarA = 6.0f;
        YawMotor->positionPid.ScalarB = 0.01f;
        YawMotor->positionPid.maxIOut = 1.0f;
        YawMotor->positionPid.maxOut = 25000.0f;
        // 速度环PID参数
        YawMotor->speedPid.mode = PID_POSITION;
        YawMotor->speedPid.kp = 4000.0f;
        YawMotor->speedPid.ki = 0.0f;
        YawMotor->speedPid.kd = 0.0f;
        YawMotor->speedPid.maxIOut = 3000.0f;
        YawMotor->speedPid.maxOut = 25000.0f;
        /*----------------------------------------云台Pitch轴电机----------------------------------------*/
        PitchMotor->controlMode = GM6020::SPD_MODE;
        // 位置环PID参数
        PitchMotor->positionPid.mode = PID_POSITION;
        PitchMotor->positionPid.kp = 30.0f;
        PitchMotor->positionPid.ki = 0.0f;
        PitchMotor->positionPid.kd = 2000.0f;
        PitchMotor->positionPid.maxIOut = 3.0f;
        PitchMotor->positionPid.maxOut = 25000.0f;
        // 速度环PID参数
        PitchMotor->speedPid.mode = PID_DELTA;
        PitchMotor->speedPid.kp = 3000.0f;
        PitchMotor->speedPid.ki = 5.0f;
        PitchMotor->speedPid.kd = 0.0f;
        PitchMotor->speedPid.maxIOut = 1000.0f;
        PitchMotor->speedPid.maxOut = 25000.0f;
    }

    void Run::execute() {

    }


    void Run::exit() {

    }

}
