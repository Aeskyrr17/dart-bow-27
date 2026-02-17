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

TaskMotors* motors = TaskMotors::Instance();

PID coilSpringMotorL_spd_pid(2000.0f, 10.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID coilSpringMotorR_spd_pid(2000.0f, 10.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

PID StringMotorL_spd_pid(100.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID StringMotorR_spd_pid(100.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

debug_motor_t coil_L_debug{};
debug_motor_t coil_R_debug{};
debug_motor_t String_L_debug{};
debug_motor_t String_R_debug{};
msg_motor_ctrl_t debug_motorctrl{};

void TaskMotors::MotorInit() 
{
    //左右卷簧电机
    DJIMotorhandler->registerMotor(&CoilSpringMotorL, &hfdcan1, 0x201);
    DJIMotorhandler->registerMotor(&CoilSpringMotorR, &hfdcan1, 0x202);
    CoilSpringMotorL.gearBox = GearBox::GearBox_M3508;
    CoilSpringMotorR.gearBox = GearBox::GearBox_M3508;

    //扳机电机
    TriggerMotor.Init(&htim1, TIM_CHANNEL_3);

    motors->YawMotor_Init();

    motors->StringMotorL.Init(&hfdcan3, 2);
    motors->StringMotorR.Init(&hfdcan3, 1);
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
    msg_motor_ctrl_t motorctrl{};

    motors->MotorInit();
    motors->SetModeAndPidParam();
    motors->YawMotor.SetTargetSpeed(0);

    float yaw_target_hz ;

    motors->TriggerMotor.Trigger_Lock();

    float string_L_spd;
    uint8_t string_L_dir;
    float string_R_spd;
    uint8_t string_R_dir;

    float coil_L_spd;
    float coil_R_spd;
    for (;;)
    {
        om_suber_export(motorctrl_suber, &motorctrl, false);

        //撒放机构处理逻辑
        if ( motorctrl.trigger_lock)
            motors->TriggerMotor.Trigger_Lock();
        else if ( !motorctrl.trigger_lock)
            motors->TriggerMotor.Trigger_Open();
        // motorctrl.Coil_speed = 20.0f;

        // if (motorctrl.Coil_L_speed > 0.03)
        //     coil_L_spd = 15.0f;
        // else if (motorctrl.Coil_L_speed < -0.03)
        //     coil_L_spd = -15.0f;
        // else 
        //     coil_L_spd = 0;     

        // if (motorctrl.Coil_R_speed > 0.03)
        //     coil_R_spd = 15.0f;
        // else if (motorctrl.Coil_R_speed < -0.03)
        //     coil_R_spd = -15.0f;
        // else 
        //     coil_R_spd = 0;        


        
        //计算卷簧电机PID
        if (motorctrl.Coil_mode == SPD)    
        {
            coilSpringMotorL_spd_pid.ref = motorctrl.Coil_L_speed;
            coilSpringMotorL_spd_pid.fdb = motors->CoilSpringMotorL.motorFeedback.speedFdb;
            coilSpringMotorL_spd_pid.UpdateResult();
            motors->CoilSpringMotorL.currentSet = static_cast<int16_t>(coilSpringMotorL_spd_pid.result);

            coilSpringMotorR_spd_pid.ref = motorctrl.Coil_R_speed;;
            coilSpringMotorR_spd_pid.fdb = motors->CoilSpringMotorR.motorFeedback.speedFdb;
            coilSpringMotorR_spd_pid.UpdateResult();
            motors->CoilSpringMotorR.currentSet = static_cast<int16_t>(coilSpringMotorR_spd_pid.result);
        }
        else
        {
            motors->CoilSpringMotorL.currentSet = static_cast<int16_t>(motorctrl.Coil_torque*100);
            motors->CoilSpringMotorR.currentSet = static_cast<int16_t>(motorctrl.Coil_torque*100);
        }
        
        DJIMotorhandler->sendControlData();//发送控制指令给电机

        //yaw轴步进电机简单控制逻辑
        if (motorctrl.target_yaw > 0.03)
            yaw_target_hz = 1000;
        else if (motorctrl.target_yaw < -0.03)
            yaw_target_hz = -1000;
        else 
            yaw_target_hz = 0;     

        motors->YawMotor.SetTargetSpeed(yaw_target_hz);

        //副弦步进电机
        // motors->StringMotorL.X_V2_Vel_LC_Control(motors->StringMotorL._id, 1, 200, 500.0f , false, 3000);
        // motors->StringMotorL.X_V2_Read_Sys_Params(2, S_VEL);
        // motors->StringMotorR.X_V2_Vel_LC_Control(motors->StringMotorR._id, 0, 200, 500.0f , false, 3000);
        // motors->StringMotorR.X_V2_Read_Sys_Params(1, S_VEL);
        if (motorctrl.String_L_speed > 0.03)
        {
            string_L_dir = 0;
            string_L_spd = 500.0f;
        }
        else if (motorctrl.String_L_speed < -0.03)
        {
            string_L_dir = 1;
            string_L_spd = 500.0f;
        }
        else 
            string_L_spd = 0.0f;     


        if (motorctrl.String_R_speed > 0.03)
        {
            string_R_dir = 1;
            string_R_spd = 500.0f;
        }
        else if (motorctrl.String_R_speed < -0.03)
        {
            string_R_dir = 0;
            string_R_spd = 500.0f;
        }
        else 
            string_R_spd = 0.0f;     

        motors->StringMotorL.X_V2_Vel_LC_Control(motors->StringMotorL._id, string_L_dir, 500, string_L_spd , false, 3000);
        motors->StringMotorR.X_V2_Vel_LC_Control(motors->StringMotorR._id, string_R_dir, 500 , string_R_spd , false, 3000);



        memcpy(&debug_motorctrl, &motorctrl,sizeof(motorctrl));
        tx_thread_sleep(1);
    }
}



 /**
 * @brief PWM中断回调函数
 * 
 * @param htim tim句柄
 */
 void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == motors->YawMotor.pwmTim) 
    {
        motors->YawMotor.HandleInterrupt();
    }
}