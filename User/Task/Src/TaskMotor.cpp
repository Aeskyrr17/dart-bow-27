#include "DJIMotorHandler.hpp"
#include "main.h"
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
    TriggerMotor.Init();
}

/**
 * @brief Set the Mode And Pid Param object
 * @todo 设置电机模式和PID参数
 */
void TaskMotors::SetModeAndPidParam()
{

}


void TaskMotors::AllMotorSetOutput()
{
    CoilSpringMotorL.setOutput();
    CoilSpringMotorR.setOutput();
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

        
    }
}



 