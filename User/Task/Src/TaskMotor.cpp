#include "DJIMotorHandler.hpp"
#include "main.h"
#include "pid.hpp"
#include "stm32h7xx_hal_tim.h"
#include "tx_api.h"
#include "bsp_can.hpp"

#include "om.h"
#include "config_motor.hpp"
#include "magicmsgs.hpp"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

TX_THREAD MotorThread;
uint8_t MotorThreadStack[2048] = {0};
DJIMotorHandler* DJIMotorhandler = DJIMotorHandler::Instance();

TaskMotors taskmotors;

PID coilSpringMotorL_spd_pid(100.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID coilSpringMotorR_spd_pid(100.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

void TaskMotors::MotorRegister() 
{
    //左右卷簧电机
    DJIMotorhandler->registerMotor(&CoilSpringMotorL, &hfdcan1, 0x201); //todo:CAN ID,挂载的CAN口
    DJIMotorhandler->registerMotor(&CoilSpringMotorR, &hfdcan1, 0x202);
    CoilSpringMotorL.controlMode = M3508::RELAX_MODE;
    CoilSpringMotorR.controlMode = M3508::RELAX_MODE;
    CoilSpringMotorL.setOutput();
    CoilSpringMotorR.setOutput();
    CoilSpringMotorL.gearBox = GearBox::GearBox_M3508;
    CoilSpringMotorR.gearBox = GearBox::GearBox_M3508;

    //yaw轴电机
    YawMotor.Init(&hfdcan1, 0x301); //todo:CAN ID,挂载的CAN口
    //副弦步进电机
    StringMotorL.Init(&hfdcan1, 0x302); 
    StringMotorR.Init(&hfdcan1, 0x303); 

    //扳机电机
    TriggerMotor.Init(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief Set the Mode And Pid Param object
 * @todo 设置电机模式和PID参数
 */
void TaskMotors::SetModeAndPidParam()
{
    CoilSpringMotorL.controlMode = M3508::SPD_MODE;
    CoilSpringMotorR.controlMode = M3508::SPD_MODE;
    YawMotor.Set_Mode();
    StringMotorL.Set_Mode();
    StringMotorR.Set_Mode();
}


void TaskMotors::AllMotorSetOutput() //todo:不一定使用，可以直接发
{
    CoilSpringMotorL.setOutput();
    CoilSpringMotorR.setOutput();
    YawMotor.SendControlData();
    StringMotorL.SendControlData();
    StringMotorR.SendControlData();
}

[[noreturn]] void MotorThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_suber_t *motorctrl_suber = om_subscribe(om_find_topic("motorctrl", UINT32_MAX));
    struct msg_motor_ctrl_t motorctrl{};


    taskmotors.MotorRegister();
    taskmotors.SetModeAndPidParam();

    for (;;)
    {
        om_suber_export(motorctrl_suber, &motorctrl, false);

        //计算卷簧电机PID
        if (motorctrl.Coil_mode == SPD)    
        {
            coilSpringMotorL_spd_pid.ref = motorctrl.Coil_speed;
            coilSpringMotorL_spd_pid.fdb = taskmotors.CoilSpringMotorL.motorFeedback.speedFdb;
            coilSpringMotorL_spd_pid.UpdateResult();
            taskmotors.CoilSpringMotorL.currentSet = static_cast<int16_t>(coilSpringMotorR_spd_pid.result);
            coilSpringMotorR_spd_pid.ref = motorctrl.Coil_speed;
            coilSpringMotorR_spd_pid.fdb = taskmotors.CoilSpringMotorR.motorFeedback.speedFdb;
            coilSpringMotorR_spd_pid.UpdateResult();
            taskmotors.CoilSpringMotorR.currentSet = static_cast<int16_t>(coilSpringMotorR_spd_pid.result);
        }
        else
        {
            taskmotors.CoilSpringMotorL.currentSet = static_cast<int16_t>(motorctrl.Coil_torque*100);
            taskmotors.CoilSpringMotorR.currentSet = static_cast<int16_t>(motorctrl.Coil_torque*100);
        }

        //发送控制指令给电机
        DJIMotorhandler->sendControlData();

        tx_thread_sleep(1);
    }
}



 