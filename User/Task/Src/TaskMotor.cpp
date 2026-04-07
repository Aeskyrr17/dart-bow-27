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
#include "DMMotorHandler.hpp"
#include "om.h"
#include "X_V2.hpp"
#include "main.h"
#include "om_fmt.h"
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


float gantry_pos_fdb = 0.0f;

namespace
{
constexpr ULONG kGantrySlotSwitchPeriodMs = 3000;

DART_SLOT GetGantryTestSlot()
{
    switch ((HAL_GetTick() / kGantrySlotSwitchPeriodMs) % 4U)
    {
    case 0:
        return DART_SLOT_NONE;
    case 1:
        return DART_SLOT_1;
    case 2:
        return DART_SLOT_2;
    default:
        return DART_SLOT_3;
    }
}
}

TX_THREAD MotorThread;
uint8_t MotorThreadStack[2048] = {0};
DJIMotorHandler* DJIMotorhandler = DJIMotorHandler::Instance();

TaskMotors* motors = TaskMotors::Instance();

float Find_gantry_pos(DART_SLOT slot);

PID coilSpringMotorL_spd_pid(2000.0f, 10.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID coilSpringMotorR_spd_pid(2000.0f, 10.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

// PID coilSpringMotorL_spd_pid(100.0f, 10.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
// PID coilSpringMotorR_spd_pid(100.0f, 10.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID coilSpringMotorL_pos_pid(3.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID coilSpringMotorR_pos_pid(3.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

// PID GantryMotor_spd_pid(100.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID GantryMotor_pos_pid(5.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

// PID StringMotorL_spd_pid(100.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID StringMotorL_tq_pid(0.12f, 0.0f, 0.0f, 1000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
// PID StringMotorR_spd_pid(100.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID StringMotorR_tq_pid(0.12f, 0.0f, 0.0f, 1000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);


float Gantry_Kp = 0.5f;
float Gantry_Kd = 1.0f;

// #define STRING_HAND_CONTROL                                                                                                                                                                                           

debug_motor_t coil_L_debug{};
debug_motor_t coil_R_debug{};
debug_motor_t String_L_debug{};
debug_motor_t String_R_debug{};
msg_motor_ctrl_t debug_motorctrl{};

float kp = 1000;
float ki = 26;

void TaskMotors::MotorInit() 
{
    //左右卷簧电机
    DJIMotorhandler->registerMotor(&CoilSpringMotorL, &hfdcan1, 0x201);
    DJIMotorhandler->registerMotor(&CoilSpringMotorR, &hfdcan1, 0x202);
    CoilSpringMotorL.gearBox = GearBox::GearBox_M3508;
    CoilSpringMotorR.gearBox = GearBox::GearBox_M3508;
    DJIMotorhandler->ResetMotorPosFeedback(&CoilSpringMotorL);
    DJIMotorhandler->ResetMotorPosFeedback(&CoilSpringMotorR);

    //龙门架装填电机
    DMMotorHandler::Instance()->registerMotor(&this->GantryMotor, &hfdcan2, 0x01);

    GantryMotor.controlMode = DMMotor::POS_SPD_MODE;
    GantryMotor.torqueSet = 0.0f;
    GantryMotor.positionSet = DART_SLOT_NONE;
    GantryMotor.speedSet = 0.0f;
    GantryMotor.KP = Gantry_Kp;
    GantryMotor.KD = Gantry_Kd;

    DMMotorHandler::Instance()->EnableMotor_Block(&this->GantryMotor);
    // DMMotorHandler::Instance()->DisableMotor(&GantryMotor);


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


    StringMotorL.X_V2_Auto_Return_Sys_Params_Timed(motors->StringMotorL._id, S_VEL, 1);
    StringMotorR.X_V2_Auto_Return_Sys_Params_Timed(motors->StringMotorR._id, S_VEL, 1);
}


[[noreturn]] void MotorThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *motorfdb_topic = om_config_topic(nullptr, "ca", "motorfdb", sizeof(msg_motorfdb_t));
    msg_motorfdb_t motorfdb{};

    om_suber_t *motorctrl_suber = om_subscribe(om_find_topic("motorctrl", UINT32_MAX));
    msg_motor_ctrl_t motorctrl{};
    om_suber_t *sensor_suber = om_subscribe(om_find_topic("sensor", UINT32_MAX));
    msg_sensor_t sensor{};

    motors->MotorInit();
    motors->SetModeAndPidParam();
    motors->YawMotor.SetTargetSpeed(0);
    motors->TriggerMotor.Trigger_Lock();

    motors->GantryMotor.positionPid = GantryMotor_pos_pid;

    //todo:后续考虑整理局部变量

    float yaw_target_hz = 0.0f;

    float string_L_spd = 0.0f;
    uint8_t string_L_dir;
    float string_R_spd = 0.0f;
    uint8_t string_R_dir;

    for (;;)
    {
        om_suber_export(motorctrl_suber, &motorctrl, false);
        om_suber_export(sensor_suber, &sensor, false);

    // DMMotorHandler::Instance()->SaveZeroPosition(&motors->GantryMotor);
    //     DMMotorHandler::Instance()->sendControlData();


        //撒放机构处理逻辑
        if ( motorctrl.trigger_lock)
            motors->TriggerMotor.Trigger_Lock();
        else if ( !motorctrl.trigger_lock)
            motors->TriggerMotor.Trigger_Open();
        


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
        else if (motorctrl.Coil_mode == POS)
        {
            coilSpringMotorL_pos_pid.ref = motorctrl.Coil_L_pos;
            coilSpringMotorL_pos_pid.fdb = motors->CoilSpringMotorL.motorFeedback.positionFdb;
            coilSpringMotorL_pos_pid.UpdateResult();
            coilSpringMotorL_spd_pid.ref = coilSpringMotorL_pos_pid.result;
            coilSpringMotorL_spd_pid.fdb = motors->CoilSpringMotorL.motorFeedback.speedFdb;
            coilSpringMotorL_spd_pid.UpdateResult();
            motors->CoilSpringMotorL.currentSet = static_cast<int16_t>(coilSpringMotorL_spd_pid.result);

            coilSpringMotorR_pos_pid.ref = motorctrl.Coil_R_pos;
            coilSpringMotorR_pos_pid.fdb = motors->CoilSpringMotorR.motorFeedback.positionFdb;
            coilSpringMotorR_pos_pid.UpdateResult();
            coilSpringMotorR_spd_pid.ref = coilSpringMotorR_pos_pid.result;
            coilSpringMotorR_spd_pid.fdb = motors->CoilSpringMotorR.motorFeedback.speedFdb;
            coilSpringMotorR_spd_pid.UpdateResult();
            motors->CoilSpringMotorR.currentSet = static_cast<int16_t>(coilSpringMotorR_spd_pid.result);
        }
        else
        {
            motors->CoilSpringMotorL.currentSet = static_cast<int16_t>(motorctrl.String_L_tq*100);
            motors->CoilSpringMotorR.currentSet = static_cast<int16_t>(motorctrl.String_R_tq*100);
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
        motors->StringMotorR.X_V2_Vel_LC_Control(motors->StringMotorR._id, string_R_dir, 1000 , string_R_spd , false, 3000);
#else 

//! for test

        // motorctrl.String_L_tq = 50000;
        // motorctrl.String_R_tq = 50000;


        if (motorctrl.String_able)
        {
            StringMotorL_tq_pid.ref = motorctrl.String_L_tq;
            StringMotorL_tq_pid.fdb = sensor.string_L_force;
            // if (Numeric::abs(StringMotorL_tq_pid.ref - StringMotorL_tq_pid.fdb) <= 50.0f) 
            // {
            //     StringMotorL_tq_pid.fdb = StringMotorL_tq_pid.ref;
            // }
            StringMotorL_tq_pid.UpdateResult();

            string_L_dir = (StringMotorL_tq_pid.result > 0) ? 1 : 0;

            float cmd_spd_L = Numeric::abs(StringMotorL_tq_pid.result);
            // if (cmd_spd_L > 800.0f) cmd_spd_L = 800.0f; 

            StringMotorR_tq_pid.ref = motorctrl.String_R_tq;
            StringMotorR_tq_pid.fdb = sensor.string_R_force;
            // if (Numeric::abs(StringMotorR_tq_pid.ref - StringMotorR_tq_pid.fdb) <= 50.0f) 
            // {
            //     StringMotorR_tq_pid.fdb = StringMotorR_tq_pid.ref;
            // }
            StringMotorR_tq_pid.UpdateResult();

            string_R_dir = (StringMotorR_tq_pid.result > 0) ? 0 : 1;

            float cmd_spd_R = Numeric::abs(StringMotorR_tq_pid.result);
            // if (cmd_spd_R > 800.0f) cmd_spd_R = 800.0f;

            motors->StringMotorL.X_V2_Vel_LC_Control(motors->StringMotorL._id, string_L_dir, 1000, cmd_spd_L , false, 3000);
            motors->StringMotorR.X_V2_Vel_LC_Control(motors->StringMotorR._id, string_R_dir, 1000, cmd_spd_R , false, 3000);
        }
        else 
        {
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
            motors->StringMotorR.X_V2_Vel_LC_Control(motors->StringMotorR._id, string_R_dir, 1000 , string_R_spd , false, 3000);
        }
        // motors->StringMotorL.X_V2_Torque_Control(motors->StringMotorL._id, string_L_dir, uint16_t t_ramp, uint16_t torque, bool snF)

#endif
//111debug
        // motors->TriggerMotor.Trigger_Lock();
        // tx_thread_sleep(5000);
        // motors->TriggerMotor.Trigger_Lock();
        // tx_thread_sleep(5000);
        // // motors->TriggerMotor.Trigger_Open();


        // motors->TriggerMotor.Trigger_Lock();
        // motors->TriggerMotor.Trigger_1();

        
        //达妙电机
        // 达妙电机位控测试：按固定节拍轮换四个 slot
        
        gantry_pos_fdb = motors->GantryMotor.motorFeedback.positionFdb;


        // const DART_SLOT gantry_target_slot = GetGantryTestSlot();

        const DART_SLOT gantry_target_slot = motorctrl.gantry_target_slot;
        const float gantry_target_pos = Find_gantry_pos(gantry_target_slot);
        motors->GantryMotor.offset = 0.0f;
        motors->GantryMotor.positionSet = gantry_target_pos;
        motors->GantryMotor.speedSet = 10.0f;
        DMMotorHandler::Instance()->sendControlData();

        motorfdb.Lcoil_pos_fdb = motors->CoilSpringMotorL.motorFeedback.positionFdb;
        motorfdb.Rcoil_pos_fbd = motors->CoilSpringMotorR.motorFeedback.positionFdb;
        motorfdb.gantry_pos_fdb = motors->GantryMotor.motorFeedback.positionFdb;
        motorfdb.gantry_spd_fdb = motors->GantryMotor.motorFeedback.speedFdb;
        motorfdb.gantry_pos_set = gantry_target_pos;
        motorfdb.gantry_pos_abserr = Numeric::abs(gantry_target_pos - motors->GantryMotor.motorFeedback.positionFdb);
        om_publish(motorfdb_topic, &motorfdb, sizeof(msg_motorfdb_t), true, false);


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

float Find_gantry_pos(DART_SLOT slot)
{

        switch (slot)
    {
        case DART_SLOT_NONE: //原点
            return Numeric::Pi;
        case DART_SLOT_1:
            return -Numeric::Pi*0.5f;
        case DART_SLOT_2:
            return 0.0f;
        case DART_SLOT_3:
            return Numeric::Pi*0.5f;
            // return 0;
        default:
            return 0.4999237f;
    }
}
