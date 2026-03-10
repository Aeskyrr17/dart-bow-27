/**
 * @file TaskMotor.cpp
 * @author Aeskyrr17
 * @brief 
 * @version 0.1
 * @date 2026-02-27
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "om.h"
#include "X_V2.hpp"
#include "main.h"
#include "tx_api.h"

#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"
#include "pid.hpp"
#include "magicmsgs.hpp"
#include "math.hpp"

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

PID StringMotorL_tq_pid(5.0f, 0.0f, 0.0f, 1000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID StringMotorR_tq_pid(5.0f, 0.0f, 0.0f, 1000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

#define STRING_HAND_CONTROL                                                                                                                                                                                           

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
    DJIMotorhandler->registerMotor(&GantryMotor, &hfdcan1, 0x204);
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
    GantryMotor.controlMode = M2006::POS_MODE;

    StringMotorL.X_V2_Auto_Return_Sys_Params_Timed(motors->StringMotorL._id, S_VEL, 1);
    StringMotorR.X_V2_Auto_Return_Sys_Params_Timed(motors->StringMotorR._id, S_VEL, 1);
}


[[noreturn]] void MotorThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_suber_t *motorctrl_suber = om_subscribe(om_find_topic("motorctrl", UINT32_MAX));
    msg_motor_ctrl_t motorctrl{};
    om_suber_t *sensor_suber = om_subscribe(om_find_topic("sensor", UINT32_MAX));
    msg_sensor_t sensor{};

    motors->MotorInit();
    motors->SetModeAndPidParam();
    motors->YawMotor.SetTargetSpeed(0);
    motors->TriggerMotor.Trigger_Lock();

    //todo:后续考虑整理局部变量
    float gantry_target_pos = 0.52f; //30度

    float yaw_target_hz = 0.0f;

    float string_L_spd = 0.0f;
    uint8_t string_L_dir;
    float string_R_spd = 0.0f;
    uint8_t string_R_dir;

    for (;;)
    {
        om_suber_export(motorctrl_suber, &motorctrl, false);
        om_suber_export(sensor_suber, &sensor, false);

        //撒放机构处理逻辑
        if ( motorctrl.trigger_lock)
            motors->TriggerMotor.Trigger_Lock();
        else if ( !motorctrl.trigger_lock)
            motors->TriggerMotor.Trigger_Open();
        

        //龙门架电机        
        if      (motorctrl.gantry_reset)    {gantry_target_pos = motors->gantry_pos.reset;}
        else if (motorctrl.gantry_open)     {gantry_target_pos = motors->gantry_pos.open;}
        else if (motorctrl.gantry_lock)     {gantry_target_pos = motors->gantry_pos.lock;};

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

        coil_L_debug.position = motors->CoilSpringMotorL.motorFeedback.positionFdb;
        coil_R_debug.position = motors->CoilSpringMotorR.motorFeedback.positionFdb;

        //yaw轴步进电机简单控制逻辑
        if (motorctrl.yaw_spd > 0.03)
            yaw_target_hz = 1000;
        else if (motorctrl.yaw_spd < -0.03)
            yaw_target_hz = -1000;
        else 
            yaw_target_hz = 0;     

        motors->YawMotor.SetTargetSpeed(yaw_target_hz);

#ifdef STRING_HAND_CONTROL

        if (motorctrl.String_L_spd > 0.03)
        {
            string_L_dir = 0;
            string_L_spd = 1000.0f;
        }
        else if (motorctrl.String_L_spd < -0.03)
        {
            string_L_dir = 1;
            string_L_spd = 1000.0f;
        }
        else 
            string_L_spd = 0.0f;     


        if (motorctrl.String_R_spd > 0.03)
        {
            string_R_dir = 1;
            string_R_spd = 1000.0f;
        }
        else if (motorctrl.String_R_spd < -0.03)
        {
            string_R_dir = 0;
            string_R_spd = 1000.0f;
        }
        else 
            string_R_spd = 0.0f;     

        motors->StringMotorL.X_V2_Vel_LC_Control(motors->StringMotorL._id, string_L_dir, 1000, string_L_spd , false, 3000);
        motors->StringMotorR.X_V2_Vel_LC_Control(motors->StringMotorR._id, string_R_dir, 1000, string_R_spd , false, 3000);
#else 

//! for test
        motorctrl.Coil_L_tq = 130000;
        motorctrl.Coil_R_tq = 130000;
        StringMotorL_tq_pid.ref = motorctrl.Coil_L_tq;
        StringMotorL_tq_pid.fdb = sensor.string_L_force;
        StringMotorL_tq_pid.UpdateResult();

        string_L_dir = (StringMotorL_tq_pid.result > 0) ? 1 : 0;

        float cmd_spd_L = Numeric::abs(StringMotorL_tq_pid.result);
        if (cmd_spd_L > 800.0f) cmd_spd_L = 800.0f; 

        StringMotorR_tq_pid.ref = motorctrl.Coil_R_tq;
        StringMotorR_tq_pid.fdb = sensor.string_R_force;
        StringMotorR_tq_pid.UpdateResult();

        string_R_dir = (StringMotorR_tq_pid.result > 0) ? 0 : 1;

        float cmd_spd_R = Numeric::abs(StringMotorR_tq_pid.result);
        if (cmd_spd_R > 800.0f) cmd_spd_R = 800.0f;

        motors->StringMotorL.X_V2_Vel_LC_Control(motors->StringMotorL._id, string_L_dir, 65535, cmd_spd_L , false, 3000);
        motors->StringMotorR.X_V2_Vel_LC_Control(motors->StringMotorR._id, string_R_dir, 65535, cmd_spd_R , false, 3000);

#endif
        memcpy(&debug_motorctrl, &motorctrl,sizeof(motorctrl));
        String_L_debug.speed = motors->StringMotorL.speed;
        String_R_debug.speed = motors->StringMotorR.speed;
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
