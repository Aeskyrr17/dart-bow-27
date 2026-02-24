#include "main.h"
#include "tx_api.h"
#include "om.h"

#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"
#include "pid.hpp"
#include "magicmsgs.hpp"

#include "config_motor.hpp"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

TX_THREAD MotorThread;
uint8_t MotorThreadStack[2048] = {0};
DJIMotorHandler* DJIMotorhandler = DJIMotorHandler::Instance();

TaskMotors* motors = TaskMotors::Instance();

PID coilSpringMotorL_spd_pid(2000.0f, 10.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID coilSpringMotorR_spd_pid(2000.0f, 10.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

PID GantryMotor_spd_pid(100.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID GantryMotor_pos_pid(100.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

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

    //龙门架装填电机
    DJIMotorhandler->registerMotor(&GantryMotor, &hfdcan1, 0x203);
    GantryMotor.gearBox = GearBox::GearBox_M2006;
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
    GantryMotor.controlMode = M2006::SPD_MODE;
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
    motors->TriggerMotor.Trigger_Lock();

    float gantry_target_pos = 0.0f;

    float yaw_target_hz = 0.0f;

    float string_L_spd = 0.0f;
    uint8_t string_L_dir;
    float string_R_spd = 0.0f;
    uint8_t string_R_dir;

    for (;;)
    {
        om_suber_export(motorctrl_suber, &motorctrl, false);

        //撒放机构处理逻辑
        if ( motorctrl.trigger_lock)
            motors->TriggerMotor.Trigger_Lock();
        else if ( !motorctrl.trigger_lock)
            motors->TriggerMotor.Trigger_Open();
        // motorctrl.Coil_spd = 20.0f;

        //卷簧电机摇杆控制死区模式
        // if (motorctrl.Coil_L_spd > 0.03)
        //     coil_L_spd = 15.0f;
        // else if (motorctrl.Coil_L_spd < -0.03)
        //     coil_L_spd = -15.0f;
        // else 
        //     coil_L_spd = 0;     

        // if (motorctrl.Coil_R_spd > 0.03)
        //     coil_R_spd = 15.0f;
        // else if (motorctrl.Coil_R_spd < -0.03)
        //     coil_R_spd = -15.0f;
        // else 
        //     coil_R_spd = 0;        


                
        if (motorctrl.gantry_reset)
        {
            gantry_target_pos = motors->gantry_pos.reset;
        }
        else if (motorctrl.gantry_open)
        {
            gantry_target_pos = motors->gantry_pos.open;
        }
        else if (motorctrl.gantry_lock)
        {
            gantry_target_pos = motors->gantry_pos.lock;
        }
        //龙门架电机PID
        GantryMotor_pos_pid.ref = gantry_target_pos;
        GantryMotor_pos_pid.fdb = motors->GantryMotor.motorFeedback.positionFdb;
        GantryMotor_pos_pid.UpdateResult();

        GantryMotor_spd_pid.ref = GantryMotor_pos_pid.result;
        GantryMotor_spd_pid.fdb = motors->GantryMotor.motorFeedback.speedFdb;
        GantryMotor_spd_pid.UpdateResult();
        motors->GantryMotor.currentSet = static_cast<int16_t>(GantryMotor_spd_pid.result);
        
        //卷簧电机PID
        if (motorctrl.Coil_mode == SPD)    
        {
            coilSpringMotorL_spd_pid.ref = motorctrl.Coil_L_spd;
            coilSpringMotorL_spd_pid.fdb = motors->CoilSpringMotorL.motorFeedback.speedFdb;
            coilSpringMotorL_spd_pid.UpdateResult();
            motors->CoilSpringMotorL.currentSet = static_cast<int16_t>(coilSpringMotorL_spd_pid.result);

            coilSpringMotorR_spd_pid.ref = motorctrl.Coil_R_spd;
            coilSpringMotorR_spd_pid.fdb = motors->CoilSpringMotorR.motorFeedback.speedFdb;
            coilSpringMotorR_spd_pid.UpdateResult();
            motors->CoilSpringMotorR.currentSet = static_cast<int16_t>(coilSpringMotorR_spd_pid.result);
        }
        else
        {
            motors->CoilSpringMotorL.currentSet = static_cast<int16_t>(motorctrl.Coil_L_tq*100);
            motors->CoilSpringMotorR.currentSet = static_cast<int16_t>(motorctrl.Coil_R_tq*100);
        }

        DJIMotorhandler->sendControlData();//发送控制指令给电机



        //yaw轴步进电机简单控制逻辑
        if (motorctrl.yaw_spd > 0.03)
            yaw_target_hz = 1000;
        else if (motorctrl.yaw_spd < -0.03)
            yaw_target_hz = -1000;
        else 
            yaw_target_hz = 0;     

        motors->YawMotor.SetTargetSpeed(yaw_target_hz);

        //副弦步进电机
        // motors->StringMotorL.X_V2_Vel_LC_Control(motors->StringMotorL._id, 1, 200, 500.0f , false, 3000);
        // motors->StringMotorL.X_V2_Read_Sys_Params(2, S_VEL);
        // motors->StringMotorR.X_V2_Vel_LC_Control(motors->StringMotorR._id, 0, 200, 500.0f , false, 3000);
        // motors->StringMotorR.X_V2_Read_Sys_Params(1, S_VEL);
        if (motorctrl.String_L_spd > 0.03)
        {
            string_L_dir = 0;
            string_L_spd = 600.0f;
        }
        else if (motorctrl.String_L_spd < -0.03)
        {
            string_L_dir = 1;
            string_L_spd = 600.0f;
        }
        else 
            string_L_spd = 0.0f;     


        if (motorctrl.String_R_spd > 0.03)
        {
            string_R_dir = 1;
            string_R_spd = 600.0f;
        }
        else if (motorctrl.String_R_spd < -0.03)
        {
            string_R_dir = 0;
            string_R_spd = 600.0f;
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