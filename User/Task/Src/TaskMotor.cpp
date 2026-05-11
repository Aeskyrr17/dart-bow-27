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


TX_THREAD MotorThread;
uint8_t MotorThreadStack[2048] = {0};
DJIMotorHandler* DJIMotorhandler = DJIMotorHandler::Instance();

TaskMotors motor;

#define MOTOR_DEBUG

float Find_gantry_pos(DART_SLOT slot);

PID str_L_tqpid(0.010f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID str_R_tqpid(0.010f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

// PID syn_spd_pid(1.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID syn_pos_pid(5.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

// PID gantry_spd_pid(0.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
// PID gantry_pos_pid(0.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

debug_motor_t coil_L_debug{};
debug_motor_t coil_R_debug{};
debug_motor_t string_L_debug{};
debug_motor_t string_R_debug{};
msg_motor_ctrl_t debug_motorctrl{};

float debug_syn_tq;

[[noreturn]] void MotorThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *motorfdb_topic = om_config_topic(nullptr, "ca", "motorfdb", sizeof(msg_motorfdb_t));
    msg_motorfdb_t motorfdb{};

    om_suber_t *motorctrl_suber = om_subscribe(om_find_topic("motorctrl", UINT32_MAX));
    msg_motor_ctrl_t motorctrl{};
    om_suber_t *sensor_suber = om_subscribe(om_find_topic("sensor", UINT32_MAX));
    msg_sensor_t sensor{};

    motor.MotorsInit();

    motor.triggerMotor.Lock();

    motor.synbeltMotor.positionPid = syn_pos_pid;
    // motor.synbeltMotor.speedPid = syn_spd_pid;
    // motor.gantryMotor.positionPid = gantry_pos_pid;
    // motor.gantryMotor.speedPid = gantry_spd_pid;


    //todo:后续考虑整理局部变量


    float string_L_spd = 0.0f;
    float string_R_spd = 0.0f;

    for (;;)
    {
        om_suber_export(motorctrl_suber, &motorctrl, false);
        om_suber_export(sensor_suber, &sensor, false);

        //撒放机构处理逻辑
        if ( motorctrl.trigger_lock)        motor.triggerMotor.Lock();
        else if ( !motorctrl.trigger_lock)  motor.triggerMotor.Open();


        motor.yawMotor.X_V2_Vel_LC_Control(motor.yawMotor.id, motor.yawMotor.dir, 1000,
                                            motor.yawMotor.ParseSpeed(motorctrl.yaw_spd * 100), 
                                            false, 5000);


        if (motorctrl.string_able)
        {
            str_L_tqpid.ref = motorctrl.string_L_tq;
            str_L_tqpid.fdb = sensor.string_L_force;

            str_L_tqpid.UpdateResult();
            float cmd_spd_L = Numeric::abs(str_L_tqpid.result);
            if (str_L_tqpid.result > 0.0f)
                string_L_spd = -cmd_spd_L;
            else
                string_L_spd = cmd_spd_L;

            str_R_tqpid.ref = motorctrl.string_R_tq;
            str_R_tqpid.fdb = sensor.string_R_force;

            str_R_tqpid.UpdateResult();
            float cmd_spd_R = Numeric::abs(str_R_tqpid.result);
            if (str_R_tqpid.result > 0.0f)
                string_R_spd = -cmd_spd_R;
            else
                string_R_spd = cmd_spd_R;

            motor.stringMotorL.X_V2_Vel_LC_Control(motor.stringMotorL.id, motor.stringMotorL.dir, 3000,
                                                    motor.stringMotorL.ParseSpeed(string_L_spd),
                                                    false, 5000);
            motor.stringMotorR.X_V2_Vel_LC_Control(motor.stringMotorR.id, motor.stringMotorR.dir, 3000,
                                                    motor.stringMotorR.ParseSpeed(string_R_spd),
                                                    false, 5000);
        }
        else 
        {
            if (motorctrl.string_L_spd > 0.03)
            {
                string_L_spd = 400.0f;
            }
            else if (motorctrl.string_L_spd < -0.03)
            {
                string_L_spd = -400.0f;
            }
            else 
                string_L_spd = 0.0f;     


            if (motorctrl.string_R_spd > 0.03)
            {
                string_R_spd = 400.0f;
            }
            else if (motorctrl.string_R_spd < -0.03)
            {
                string_R_spd = -400.0f;
            }
            else 
                string_R_spd = 0.0f;     

            motor.stringMotorL.X_V2_Vel_LC_Control(motor.stringMotorL.id, motor.stringMotorL.dir, 3000,
                                                    motor.stringMotorL.ParseSpeed(string_L_spd),
                                                    false, 3000);
            motor.stringMotorR.X_V2_Vel_LC_Control(motor.stringMotorR.id, motor.stringMotorR.dir, 3000,
                                                    motor.stringMotorR.ParseSpeed(string_R_spd),
                                                    false, 3000);
        }


        DART_SLOT gantry_target_slot = motorctrl.gantry_target_slot;
        float gantry_target_pos = Find_gantry_pos(gantry_target_slot);
        motor.gantryMotor.offset = 0.0f;
        motor.gantryMotor.positionSet = gantry_target_pos;
        motor.gantryMotor.speedSet = motor.gantry_max_spd;


        switch (motorctrl.synbelt_mode)//pos为4310的SPD模式+外部pos闭环
        {
        case POS: 
            motor.synbeltMotor.positionPid.ref = motorctrl.synbelt_pos;
            motor.synbeltMotor.positionPid.fdb = motor.synbeltMotor.motorFeedback.positionFdb;
            motor.synbeltMotor.positionPid.UpdateResult();
            motor.synbeltMotor.speedSet = motor.synbeltMotor.positionPid.result;
            break;
        case SPD:
            motor.synbeltMotor.speedSet = motorctrl.synbelt_spd;
            break;
        case TORQUE: //? 暂时未完成
        default:
            motor.synbeltMotor.speedSet = 0.0f;
            break;
        }
        DMMotorHandler::Instance()->sendControlData();



        motorfdb.gantry_pos_fdb = motor.gantryMotor.motorFeedback.positionFdb;
        motorfdb.gantry_spd_fdb = motor.gantryMotor.motorFeedback.speedFdb;
        motorfdb.gantry_pos_set = gantry_target_pos;
        motorfdb.syn_pos_fdb = motor.synbeltMotor.motorFeedback.positionFdb;
        om_publish(motorfdb_topic, &motorfdb, sizeof(msg_motorfdb_t), true, false);

#ifdef MOTOR_DEBUG
        memcpy(&debug_motorctrl, &motorctrl,sizeof(motorctrl));
        debug_syn_tq = motor.synbeltMotor.motorFeedback.torqueFdb;
        string_L_debug.speed = motor.stringMotorL.speed;
        string_R_debug.speed = motor.stringMotorR.speed;
#endif

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
    // if (htim == motor.YawMotor.pwmTim) 
    // {
    //     motor.YawMotor.HandleInterrupt();
    // }
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
            return Numeric::Pi;
    }
}
