#include "DJIMotorHandler.hpp"
#include "main.h"
#include "pid.hpp"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_tim.h"
#include "tx_api.h"
#include "bsp_can.hpp"

#include "om.h"
#include "config_motor.hpp"
#include "magicmsgs.hpp"
#include <cstdint>

#include "gpio.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

TX_THREAD MotorThread;
uint8_t MotorThreadStack[2048] = {0};
DJIMotorHandler* DJIMotorhandler = DJIMotorHandler::Instance();

TaskMotors motors;

PID coilSpringMotorL_spd_pid(100.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID coilSpringMotorR_spd_pid(100.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

void TaskMotors::MotorRegister() 
{
    //左右卷簧电机
    DJIMotorhandler->registerMotor(&CoilSpringMotorL, &hfdcan1, 0x201);
    DJIMotorhandler->registerMotor(&CoilSpringMotorR, &hfdcan1, 0x202);
    CoilSpringMotorL.gearBox = GearBox::GearBox_M3508;
    CoilSpringMotorR.gearBox = GearBox::GearBox_M3508;



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

}


// void TaskMotors::AllMotorSetOutput() //todo:不一定使用，可以直接发
// {
//     CoilSpringMotorL.setOutput();
//     CoilSpringMotorR.setOutput();
//     YawMotor.SendControlData();
//     StringMotorL.SendControlData();
//     StringMotorR.SendControlData();
// }

[[noreturn]] void MotorThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_suber_t *motorctrl_suber = om_subscribe(om_find_topic("motorctrl", UINT32_MAX));
    struct msg_motor_ctrl_t motorctrl{};

    // motors.MotorRegister();
    // motors.SetModeAndPidParam();

     motors.StringMotorR_Init();

    bool hasStarted = false;
    bool isrev = false;

    

    for (;;)
    {
        om_suber_export(motorctrl_suber, &motorctrl, false);

        // //撒放机构处理逻辑
        // if ( motorctrl.trigger_lock)
        //     motors.TriggerMotor.Trigger_Lock();
        // else if ( !motorctrl.trigger_lock)
        //     motors.TriggerMotor.Trigger_Open();

        // //计算卷簧电机PID
        // if (motorctrl.Coil_mode == SPD)    
        // {
        //     coilSpringMotorL_spd_pid.ref = motorctrl.Coil_speed;
        //     coilSpringMotorL_spd_pid.fdb = motors.CoilSpringMotorL.motorFeedback.speedFdb;
        //     coilSpringMotorL_spd_pid.UpdateResult();
        //     motors.CoilSpringMotorL.currentSet = static_cast<int16_t>(coilSpringMotorL_spd_pid.result);
        //     coilSpringMotorR_spd_pid.ref = motorctrl.Coil_speed;
        //     coilSpringMotorR_spd_pid.fdb = motors.CoilSpringMotorR.motorFeedback.speedFdb;
        //     coilSpringMotorR_spd_pid.UpdateResult();
        //     motors.CoilSpringMotorR.currentSet = static_cast<int16_t>(coilSpringMotorR_spd_pid.result);
        // }
        // else
        // {
        //     motors.CoilSpringMotorL.currentSet = static_cast<int16_t>(motorctrl.Coil_torque*100);
        //     motors.CoilSpringMotorR.currentSet = static_cast<int16_t>(motorctrl.Coil_torque*100);
        // }

        // //发送控制指令给电机
        // DJIMotorhandler->sendControlData();

#define StepperTest
#ifdef StepperTest

        // motors.StringMotorR.targetSpeed = 3000.0f;
        // motors.StringMotorR.SendControlData();


        // tx_thread_sleep(5000);
        // motors.StringMotorR.targetSpeed =6000.0f;
        // motors.StringMotorR.SendControlData();

            if (hasStarted == false)
            {
            for(int i =500; i <= 2000; i += 100) 
            {
                motors.StringMotorR.targetSpeed = (float)i;
                motors.StringMotorR.SendControlData();
                tx_thread_sleep(10); // 每 10ms 加速一点点
            }
            hasStarted = true;
            isrev = false;
            }

            // if (isrev == true)
            // {
            //     for (int i = -4000; i <= 4000; i += 100)
            //     {
            //     motors.StringMotorR.targetSpeed = float(i);
            //     motors.StringMotorR.SendControlData();
            //     tx_thread_sleep(10);
            //     }
            //     isrev = false;
            // }   

        // motors.StringMotorR.targetSpeed = -2000.0f;
        // motors.StringMotorR.SendControlData();
        //         tx_thread_sleep(1000); 
        motors.StringMotorR.targetSpeed = 2000.0f;
        motors.StringMotorR.SendControlData();
        //         tx_thread_sleep(1000); 

        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        // tx_thread_sleep(1000);
        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        // tx_thread_sleep(1000); 
        // tx_thread_sleep(3000);
    
        // motors.StringMotorR.targetSpeed =-3000.0f;
        // motors.StringMotorR.SendControlData();
        // tx_thread_sleep(3000);
        

        //     if (isrev == false)
        //     {
        //         for (int i = 4000; i >= -4000; i -= 100)
        //         {
        //         motors.StringMotorR.targetSpeed = float(i);
        //         motors.StringMotorR.SendControlData();
        //         tx_thread_sleep(10);
        //         }
        //         isrev = true;
        //     }   
        // motors.StringMotorR.targetSpeed = -4000.0f;
        // motors.StringMotorR.SendControlData();
        // tx_thread_sleep(5000);

#endif

        tx_thread_sleep(1);
    }
}



 