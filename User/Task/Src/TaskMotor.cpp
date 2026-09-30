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
#include "bsp_dwt.hpp"
#include <cmath>

#include "config_launcher.hpp"
#include "config_motor.hpp"
#include "YawIdentDebug.hpp"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;


TX_THREAD MotorThread;
uint8_t MotorThreadStack[2048] = {0};
TX_SEMAPHORE MotorAlive;
TX_SEMAPHORE GantryMotorErrorSem;
DJIMotorHandler* DJIMotorhandler = DJIMotorHandler::Instance();

TaskMotors motor;

#define MOTOR_DEBUG

PID str_L_tension_pid(60.0f, 0.0f, 0.0f, 5000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID str_R_tension_pid(60.0f, 0.0f, 0.0f, 5000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

// PID syn_spd_pid(1.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
PID syn_pos_pid(10.0f, 0.0f, 10.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

// Yaw cascade PID: position loop outputs speed, speed loop outputs MIT torque.
PID yaw_pos_pid(15.0f, 0.0f, 0.0f, 0.5f, 0.0f, PID_POSITION);
PID yaw_spd_pid(20.0f, 0.0f, 0.0f, 10.0f, 1.0f,
                PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
// PID gantry_spd_pid(0.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);
// PID gantry_pos_pid(0.0f, 0.0f, 0.0f, 10000.0f, 1000.0f, PID_POSITION | PID_Integral_Limit | PID_Trapezoid_Intergral);

constexpr float yaw_target_position_rate_rad_s = 0.5f;
constexpr float yaw_control_period_s = 0.001f;
constexpr float yaw_cmd_deadzone = 0.01f;
constexpr float yaw_imu_spd_sign = 1.0f;

constexpr float yaw_static_torque_pos_nm = 1.2f;
constexpr float yaw_static_torque_neg_nm = 1.2f;
constexpr float yaw_ff_spd_deadzone_rad_s = 0.01f;
constexpr float yaw_max_torque_nm = 10.0f;

// Static-friction trials use only the DM8009P MIT torque field.
// The stored absolute-encoder zero is yaw=0.
constexpr float yaw_friction_start_rad = 20.0f * Pi / 180.0f;
constexpr float yaw_friction_cutoff_rad = 45.0f * Pi / 180.0f;
constexpr float yaw_friction_max_torque_nm = 8.0f;
constexpr float yaw_friction_ramp_nm_s = 0.5f;
constexpr float yaw_friction_still_speed_rad_s = 0.04f;
constexpr float yaw_friction_motion_speed_threshold_rad_s = 0.05f;
constexpr float yaw_friction_still_position_rad = 0.002f;
constexpr float yaw_friction_motion_position_threshold_rad = 0.003f;
constexpr uint32_t yaw_friction_still_us = 500000U;
constexpr uint32_t yaw_friction_onset_us = 20000U;
constexpr uint8_t yaw_friction_trial_count = 6; // Three trials per direction.

enum : uint8_t
{
    YAW_FRICTION_IDLE = 0,
    YAW_FRICTION_SETTLE = 1,
    YAW_FRICTION_RAMP = 2,
    YAW_FRICTION_COAST = 3,
    YAW_FRICTION_DONE = 4,
    YAW_FRICTION_ABORTED = 5,
};

volatile YawIdentDebug yaw_ident_debug{};

struct yaw_debug_t
{
    float pos_ref;
    float pos_fdb;
    float spd_ref;
    float spd_fdb;
    float spd_fdb_motor;
    float spd_fdb_imu;
    float torque_ref;
    float torque_fdb;
};

debug_motor_t coil_L_debug{};
debug_motor_t coil_R_debug{};
debug_motor_t string_L_debug{};
debug_motor_t string_R_debug{};
yaw_debug_t yaw_debug{};
msg_motor_ctrl_t debug_motorctrl{};

float debug_syn_tq;

msg_ins_t ins{};


[[noreturn]] void MotorThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *motorfdb_topic = om_config_topic(nullptr, "ca", "motorfdb", sizeof(msg_motorfdb_t));
    msg_motorfdb_t motorfdb{};

    om_suber_t *motorctrl_suber = om_subscribe(om_find_topic("motorctrl", UINT32_MAX));
    msg_motor_ctrl_t motorctrl{};
    om_suber_t *sensor_suber = om_subscribe(om_find_topic("sensor", UINT32_MAX));
    msg_sensor_t sensor{};
    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    // msg_ins_t ins{};
    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};

    motor.MotorsInit();

    motor.synbeltMotor.positionPid = syn_pos_pid;
    motor.yawMotor.positionSet = motor.yawMotor.motorFeedback.positionFdb;
    // motor.synbeltMotor.speedPid = syn_spd_pid;
    // motor.gantryMotor.positionPid = gantry_pos_pid;
    // motor.gantryMotor.speedPid = gantry_spd_pid;


    //todo:后续考虑整理局部变量


    float string_L_spd = 0.0f;
    float string_R_spd = 0.0f;
    const uint16_t string_max_current = 5000;
    const float string_open_loop_max_spd = 2000.0f;
    const float string_force_limit_kg = 100.0f;
    const uint32_t gantry_alive_check_period = 100;
    const uint8_t gantry_alive_lost_limit = 3;
    uint32_t gantry_alive_check_count = 0;
    uint8_t gantry_alive_lost_count = 0;
    bool gantry_motor_error = false;

    bool trigger_latched = false;
    bool last_trigger_release = false;
    bool yaw_hold_latched = false;
    uint8_t yaw_friction_state = YAW_FRICTION_IDLE;
    uint8_t yaw_friction_trial = 0;
    uint8_t yaw_friction_abort_reason = 0;
    int8_t yaw_friction_direction = 0;
    uint32_t yaw_friction_ramp_start_us = 0;
    uint32_t yaw_friction_quiet_since_us = 0;
    uint32_t yaw_friction_motion_since_us = 0;
    uint32_t yaw_ident_last_rx_us = 0;
    uint32_t yaw_ident_last_rx_seq = 0;
    uint32_t yaw_friction_onset_seq = 0;
    uint32_t yaw_friction_onset_timestamp_us = 0;
    float yaw_friction_quiet_anchor_rad = 0.0f;
    float yaw_friction_trial_start_rad = 0.0f;
    float yaw_friction_baseline_feedback_nm = 0.0f;
    float yaw_friction_pre_command_nm = 0.0f;
    float yaw_friction_pre_feedback_nm = 0.0f;
    float yaw_friction_onset_command_nm = 0.0f;
    float yaw_friction_onset_feedback_nm = 0.0f;
    float yaw_friction_onset_position_rad = 0.0f;
    float yaw_friction_onset_speed_rad_s = 0.0f;
    bool yaw_friction_switch_seen = false;

    for (;;)
    {
        om_suber_export(motorctrl_suber, &motorctrl, false);
        om_suber_export(sensor_suber, &sensor, false);
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(remoter_suber, &remoter, false);

        //撒放机构处理逻辑
        if (motorctrl.trigger_release && !last_trigger_release) 
        {
            trigger_latched = true;
        }

        if (trigger_latched)
        {
            trigger_latched = !motor.triggerMotor.OpenAndReset();
        }
        last_trigger_release = motorctrl.trigger_release;




        if (motorctrl.string_able)
        {
            if (sensor.string_L_force_kg == 0.0f)
            {
                string_L_spd = 0.0f;
            }
            else
            {
                str_L_tension_pid.ref = motorctrl.string_L_tension_kg;
                str_L_tension_pid.fdb = sensor.string_L_force_kg;

                str_L_tension_pid.UpdateResult();
                float cmd_spd_L = Numeric::abs(str_L_tension_pid.result);
                if (str_L_tension_pid.result > 0.0f)
                    string_L_spd = -cmd_spd_L;
                else
                    string_L_spd = cmd_spd_L;
            }

            if (sensor.string_R_force_kg == 0.0f)
            {
                string_R_spd = 0.0f;
            }
            else
            {
                str_R_tension_pid.ref = motorctrl.string_R_tension_kg;
                str_R_tension_pid.fdb = sensor.string_R_force_kg;

                str_R_tension_pid.UpdateResult();
                float cmd_spd_R = Numeric::abs(str_R_tension_pid.result);
                if (str_R_tension_pid.result > 0.0f)
                    string_R_spd = -cmd_spd_R;
                else
                    string_R_spd = cmd_spd_R;
            }

        }
        else 
        {
            if (motorctrl.string_L_spd > 0.03)
            {
                string_L_spd = Numeric::LimitABS(motorctrl.string_L_spd, 1.0f) * string_open_loop_max_spd;
            }
            else if (motorctrl.string_L_spd < -0.03)
            {
                string_L_spd = Numeric::LimitABS(motorctrl.string_L_spd, 1.0f) * string_open_loop_max_spd;
            }
            else 
                string_L_spd = 0.0f;     


            if (motorctrl.string_R_spd > 0.03)
            {
                string_R_spd = Numeric::LimitABS(motorctrl.string_R_spd, 1.0f) * string_open_loop_max_spd;
            }
            else if (motorctrl.string_R_spd < -0.03)
            {
                string_R_spd = Numeric::LimitABS(motorctrl.string_R_spd, 1.0f) * string_open_loop_max_spd;
            }
            else 
                string_R_spd = 0.0f;     

        }

        // 
        if (sensor.string_L_force_kg > string_force_limit_kg && string_L_spd < 0.0f)
        {
            string_L_spd = 0.0f;
        }
        if (sensor.string_R_force_kg > string_force_limit_kg && string_R_spd < 0.0f)
        {
            string_R_spd = 0.0f;
        }

        motor.stringMotorL.X_V2_Vel_LC_Control(motor.stringMotorL.id, motor.stringMotorL.dir, 3000,
                                                motor.stringMotorL.ParseSpeed(string_L_spd),
                                                false, string_max_current);
        motor.stringMotorR.X_V2_Vel_LC_Control(motor.stringMotorR.id, motor.stringMotorR.dir, 3000,
                                                motor.stringMotorR.ParseSpeed(string_R_spd),
                                                false, string_max_current);


        DART_SLOT gantry_target_slot = motorctrl.gantry_target_slot;
        float gantry_target_pos = Get_Gantry_Target_Pos(gantry_target_slot);
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

        const uint32_t yaw_ident_now_us = static_cast<uint32_t>(DWT_GetTimeline_us());
        const float yaw_position_rad = motor.yawMotor.motorFeedback.positionFdb;
        const uint32_t yaw_motor_rx_seq = motor.yawMotor.AliveFlag;
        if (yaw_motor_rx_seq != yaw_ident_last_rx_seq)
        {
            yaw_ident_last_rx_seq = yaw_motor_rx_seq;
            yaw_ident_last_rx_us = yaw_ident_now_us;
        }
        const bool yaw_feedback_fresh = yaw_motor_rx_seq != 0 &&
            yaw_ident_now_us - yaw_ident_last_rx_us <= 20000U;
        const float yaw_motor_speed_rad_s = motor.yawMotor.motorFeedback.speedFdb;
        const float yaw_motor_feedback_nm = motor.yawMotor.motorFeedback.torqueFdb;
        const bool yaw_friction_switch = !remoter.offline &&
                                         remoter.left_sw == Up &&
                                         remoter.right_sw == Down;
        if (yaw_friction_switch && !yaw_friction_switch_seen &&
            yaw_friction_state == YAW_FRICTION_IDLE &&
            yaw_feedback_fresh &&
            motor.yawMotor.motorFeedback.ERR == DMMotor::ERR_ENABLE &&
            std::isfinite(yaw_position_rad) &&
            std::isfinite(yaw_motor_speed_rad_s) &&
            std::abs(yaw_position_rad) <= yaw_friction_start_rad &&
            std::abs(yaw_motor_speed_rad_s) <= yaw_friction_still_speed_rad_s)
        {
            yaw_friction_state = YAW_FRICTION_SETTLE;
            yaw_friction_trial = 1;
            yaw_friction_direction = 1;
            yaw_friction_abort_reason = 0;
            yaw_friction_quiet_since_us = yaw_ident_now_us;
            yaw_friction_quiet_anchor_rad = yaw_position_rad;
            yaw_friction_onset_seq = 0;
            yaw_friction_onset_timestamp_us = 0;
            yaw_friction_trial_start_rad = yaw_position_rad;
            yaw_friction_baseline_feedback_nm = yaw_motor_feedback_nm;
            yaw_friction_pre_command_nm = 0.0f;
            yaw_friction_pre_feedback_nm = 0.0f;
            yaw_friction_onset_command_nm = 0.0f;
            yaw_friction_onset_feedback_nm = 0.0f;
            yaw_friction_onset_position_rad = 0.0f;
            yaw_friction_onset_speed_rad_s = 0.0f;
            yaw_pos_pid.Clear();
            yaw_spd_pid.Clear();
        }
        yaw_friction_switch_seen = yaw_friction_switch;
        bool yaw_friction_just_released = false;
        if (!yaw_friction_switch && yaw_friction_state != YAW_FRICTION_IDLE)
        {
            yaw_friction_state = YAW_FRICTION_IDLE;
            yaw_friction_just_released = true;
            motor.yawMotor.torqueSet = 0.0f;
            motor.yawMotor.positionSet = yaw_position_rad;
            motor.yawMotor.speedSet = 0.0f;
            yaw_hold_latched = true;
            yaw_pos_pid.Clear();
            yaw_spd_pid.Clear();
        }

        if (yaw_friction_state == YAW_FRICTION_IDLE && !yaw_friction_just_released &&
            motorctrl.yaw_mode == TORQUE)
        {
            motor.yawMotor.positionSet = motor.yawMotor.motorFeedback.positionFdb;
            motor.yawMotor.speedSet = 0.0f;
            motor.yawMotor.torqueSet = motorctrl.yaw_tq;
            yaw_pos_pid.Clear();
            yaw_spd_pid.Clear();
            yaw_hold_latched = true;
        }
        else if (yaw_friction_state == YAW_FRICTION_IDLE && !yaw_friction_just_released)
        {
            const bool yaw_cmd_active = std::abs(motorctrl.yaw_spd) > yaw_cmd_deadzone;
            if (yaw_cmd_active)
            {
                yaw_hold_latched = false;
                motor.yawMotor.positionSet -= motorctrl.yaw_spd *
                                              yaw_target_position_rate_rad_s *
                                              yaw_control_period_s;
            }
            else
            {
                if (!yaw_hold_latched)
                {
                    motor.yawMotor.positionSet = motor.yawMotor.motorFeedback.positionFdb;
                    yaw_pos_pid.Clear();
                    yaw_spd_pid.Clear();
                    yaw_hold_latched = true;
                }
            }

        // 速度环
            yaw_pos_pid.ref = motor.yawMotor.positionSet;
            yaw_pos_pid.fdb = motor.yawMotor.motorFeedback.positionFdb;
            yaw_pos_pid.UpdateResult();
            motor.yawMotor.speedSet = yaw_pos_pid.result;

            const float yaw_imu_spd_fdb = yaw_imu_spd_sign * ins.gyro_y;
            yaw_spd_pid.ref = motor.yawMotor.speedSet;
            yaw_spd_pid.fdb = yaw_imu_spd_fdb;
            yaw_spd_pid.UpdateResult();

            float yaw_friction_ff = 0.0f;
            if (motor.yawMotor.speedSet > yaw_ff_spd_deadzone_rad_s)
            {
                yaw_friction_ff = yaw_static_torque_pos_nm;
            }
            else if (motor.yawMotor.speedSet < -yaw_ff_spd_deadzone_rad_s)
            {
                yaw_friction_ff = -yaw_static_torque_neg_nm;
            }

            motor.yawMotor.torqueSet =
                FloatConstrain(yaw_spd_pid.result + yaw_friction_ff,
                               -yaw_max_torque_nm,
                               yaw_max_torque_nm);
        }

        float yaw_friction_elapsed_s = 0.0f;
        if (yaw_friction_state != YAW_FRICTION_IDLE)
        {
            // Zero MIT torque in every phase except ramp. No yaw PID runs here.
            motor.yawMotor.torqueSet = 0.0f;
            if (yaw_friction_state <= YAW_FRICTION_COAST &&
                (!yaw_feedback_fresh ||
                 motor.yawMotor.motorFeedback.ERR != DMMotor::ERR_ENABLE ||
                 !std::isfinite(yaw_position_rad) ||
                 !std::isfinite(yaw_motor_speed_rad_s)))
            {
                yaw_friction_state = YAW_FRICTION_ABORTED;
                yaw_friction_abort_reason = 3;
            }
            else if (yaw_friction_state <= YAW_FRICTION_COAST &&
                     std::abs(yaw_position_rad) >= yaw_friction_cutoff_rad)
            {
                yaw_friction_state = YAW_FRICTION_ABORTED;
                yaw_friction_abort_reason = 1;
            }

            if (yaw_friction_state == YAW_FRICTION_SETTLE)
            {
                if (std::abs(yaw_position_rad) > yaw_friction_start_rad)
                {
                    yaw_friction_state = YAW_FRICTION_ABORTED;
                    yaw_friction_abort_reason = 4;
                }
                else if (std::abs(yaw_motor_speed_rad_s) > yaw_friction_still_speed_rad_s ||
                         std::abs(yaw_position_rad - yaw_friction_quiet_anchor_rad) >
                             yaw_friction_still_position_rad)
                {
                    yaw_friction_quiet_since_us = yaw_ident_now_us;
                    yaw_friction_quiet_anchor_rad = yaw_position_rad;
                }
                else if (yaw_ident_now_us - yaw_friction_quiet_since_us >= yaw_friction_still_us)
                {
                    yaw_friction_state = YAW_FRICTION_RAMP;
                    yaw_friction_ramp_start_us = yaw_ident_now_us;
                    yaw_friction_trial_start_rad = yaw_position_rad;
                    yaw_friction_baseline_feedback_nm = yaw_motor_feedback_nm;
                    yaw_friction_pre_command_nm = 0.0f;
                    yaw_friction_pre_feedback_nm = yaw_motor_feedback_nm;
                    yaw_friction_onset_command_nm = 0.0f;
                    yaw_friction_onset_feedback_nm = 0.0f;
                    yaw_friction_onset_position_rad = 0.0f;
                    yaw_friction_onset_speed_rad_s = 0.0f;
                    yaw_friction_motion_since_us = 0;
                }
            }
            else if (yaw_friction_state == YAW_FRICTION_RAMP)
            {
                yaw_friction_elapsed_s =
                    static_cast<float>(yaw_ident_now_us - yaw_friction_ramp_start_us) * 1.0e-6f;
                const float ramp_nm = yaw_friction_ramp_nm_s * yaw_friction_elapsed_s;
                if (ramp_nm >= yaw_friction_max_torque_nm)
                {
                    yaw_friction_state = YAW_FRICTION_ABORTED;
                    yaw_friction_abort_reason = 2;
                }
                else
                {
                    motor.yawMotor.torqueSet = yaw_friction_direction * ramp_nm;
                    const bool moving =
                        yaw_friction_direction * yaw_motor_speed_rad_s >
                            yaw_friction_motion_speed_threshold_rad_s &&
                        yaw_friction_direction *
                            (yaw_position_rad - yaw_friction_trial_start_rad) >
                            yaw_friction_motion_position_threshold_rad;
                    if (!moving)
                    {
                        yaw_friction_motion_since_us = 0;
                        yaw_friction_pre_command_nm = motor.yawMotor.torqueSet;
                        yaw_friction_pre_feedback_nm = yaw_motor_feedback_nm;
                    }
                    else if (yaw_friction_motion_since_us == 0)
                    {
                        yaw_friction_motion_since_us = yaw_ident_now_us;
                    }
                    else if (yaw_ident_now_us - yaw_friction_motion_since_us >=
                             yaw_friction_onset_us)
                    {
                        yaw_friction_onset_timestamp_us = yaw_ident_now_us;
                        yaw_friction_onset_command_nm = motor.yawMotor.torqueSet;
                        yaw_friction_onset_feedback_nm = yaw_motor_feedback_nm;
                        yaw_friction_onset_position_rad = yaw_position_rad;
                        yaw_friction_onset_speed_rad_s = yaw_motor_speed_rad_s;
                        ++yaw_friction_onset_seq;
                        yaw_friction_state = YAW_FRICTION_COAST;
                        yaw_friction_quiet_since_us = yaw_ident_now_us;
                        yaw_friction_quiet_anchor_rad = yaw_position_rad;
                        motor.yawMotor.torqueSet = 0.0f;
                    }
                }
            }
            else if (yaw_friction_state == YAW_FRICTION_COAST)
            {
                if (std::abs(yaw_motor_speed_rad_s) > yaw_friction_still_speed_rad_s ||
                    std::abs(yaw_position_rad - yaw_friction_quiet_anchor_rad) >
                        yaw_friction_still_position_rad)
                {
                    yaw_friction_quiet_since_us = yaw_ident_now_us;
                    yaw_friction_quiet_anchor_rad = yaw_position_rad;
                }
                else if (yaw_ident_now_us - yaw_friction_quiet_since_us >=
                         yaw_friction_still_us)
                {
                    if (yaw_friction_trial >= yaw_friction_trial_count)
                    {
                        yaw_friction_state = YAW_FRICTION_DONE;
                    }
                    else
                    {
                        ++yaw_friction_trial;
                        yaw_friction_direction = (yaw_friction_trial & 1U) ? 1 : -1;
                        yaw_friction_state = YAW_FRICTION_SETTLE;
                        yaw_friction_quiet_since_us = yaw_ident_now_us;
                        yaw_friction_quiet_anchor_rad = yaw_position_rad;
                    }
                }
            }

            motor.yawMotor.controlMode = DMMotor::MIT_MODE;
            motor.yawMotor.positionSet = yaw_position_rad;
            motor.yawMotor.speedSet = 0.0f;
        }

        DMMotorHandler::Instance()->sendControlData();
        ++yaw_ident_debug.update_seq;
        yaw_ident_debug.timestamp_us = yaw_ident_now_us;
        yaw_ident_debug.motor_rx_seq = yaw_motor_rx_seq;
        yaw_ident_debug.imu_sample_seq = ins.gyro_sample_seq;
        yaw_ident_debug.state = yaw_friction_state;
        yaw_ident_debug.imu_status = ins.imu_status |
            (ins.gyro_sample_seq == 0U ? 4U : 0U);
        yaw_ident_debug.trial = yaw_friction_trial;
        yaw_ident_debug.direction = yaw_friction_direction;
        yaw_ident_debug.abort_reason = yaw_friction_abort_reason;
        yaw_ident_debug.onset_seq = yaw_friction_onset_seq;
        yaw_ident_debug.onset_timestamp_us = yaw_friction_onset_timestamp_us;
        yaw_ident_debug.elapsed_s = yaw_friction_elapsed_s;
        yaw_ident_debug.frequency_hz = 0.0f;
        yaw_ident_debug.position_rad = yaw_position_rad;
        yaw_ident_debug.gyro_roll_rad_s = ins.gyro_r;
        yaw_ident_debug.gyro_pitch_rad_s = ins.gyro_p;
        yaw_ident_debug.gyro_yaw_rad_s = ins.gyro_y;
        yaw_ident_debug.motor_speed_rad_s = motor.yawMotor.motorFeedback.speedFdb;
        yaw_ident_debug.torque_command_nm = motor.yawMotor.torqueSet;
        yaw_ident_debug.torque_feedback_nm = motor.yawMotor.motorFeedback.torqueFdb;
        yaw_ident_debug.trial_start_position_rad = yaw_friction_trial_start_rad;
        yaw_ident_debug.baseline_feedback_nm = yaw_friction_baseline_feedback_nm;
        yaw_ident_debug.pre_onset_command_nm = yaw_friction_pre_command_nm;
        yaw_ident_debug.pre_onset_feedback_nm = yaw_friction_pre_feedback_nm;
        yaw_ident_debug.onset_command_nm = yaw_friction_onset_command_nm;
        yaw_ident_debug.onset_feedback_nm = yaw_friction_onset_feedback_nm;
        yaw_ident_debug.onset_position_rad = yaw_friction_onset_position_rad;
        yaw_ident_debug.onset_speed_rad_s = yaw_friction_onset_speed_rad_s;
        ++yaw_ident_debug.sample_seq;
        ++yaw_ident_debug.update_seq;

        // gantry电机状态error check
        gantry_alive_check_count++;
        if (gantry_alive_check_count >= gantry_alive_check_period)
        {
            gantry_alive_check_count = 0;
            bool gantry_offline = (motor.gantryMotor.AliveCheck() == DMMotor::MOTOR_OFFLINE);
            bool gantry_state_error = motor.gantryMotor.Enable_Failed ||
                                      (motor.gantryMotor.motorFeedback.ERR != DMMotor::ERR_ENABLE);

            if (gantry_offline)
            {
                if (gantry_alive_lost_count < gantry_alive_lost_limit)
                    gantry_alive_lost_count++;
            }
            else
            {
                gantry_alive_lost_count = 0;
            }

            gantry_motor_error = (gantry_alive_lost_count >= gantry_alive_lost_limit) || gantry_state_error;
        }

        if (gantry_motor_error)
        {
            tx_semaphore_ceiling_put(&GantryMotorErrorSem, 1);
        }

        motorfdb.gantry_pos_fdb = motor.gantryMotor.motorFeedback.positionFdb;
        motorfdb.gantry_spd_fdb = motor.gantryMotor.motorFeedback.speedFdb;
        motorfdb.gantry_pos_set = gantry_target_pos;
        motorfdb.syn_pos_fdb = motor.synbeltMotor.motorFeedback.positionFdb;
        motorfdb.syn_tq_fdb = motor.synbeltMotor.motorFeedback.torqueFdb;
        motorfdb.yaw_pos_fdb = motor.yawMotor.motorFeedback.positionFdb; 
        om_publish(motorfdb_topic, &motorfdb, sizeof(msg_motorfdb_t), true, false);

#ifdef MOTOR_DEBUG
        memcpy(&debug_motorctrl, &motorctrl,sizeof(motorctrl));
        debug_syn_tq = motor.synbeltMotor.motorFeedback.torqueFdb;
        string_L_debug.speed = motor.stringMotorL.speed;
        string_R_debug.speed = motor.stringMotorR.speed;
        yaw_debug.pos_ref = motor.yawMotor.positionSet;
        yaw_debug.pos_fdb = motor.yawMotor.motorFeedback.positionFdb;
        yaw_debug.spd_ref = motor.yawMotor.speedSet;
        yaw_debug.spd_fdb = yaw_imu_spd_sign * ins.gyro_y;
        yaw_debug.spd_fdb_motor = motor.yawMotor.motorFeedback.speedFdb;
        yaw_debug.spd_fdb_imu = ins.gyro_y;
        yaw_debug.torque_ref = motor.yawMotor.torqueSet;
        yaw_debug.torque_fdb = motor.yawMotor.motorFeedback.torqueFdb;
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

