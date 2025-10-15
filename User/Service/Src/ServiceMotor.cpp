#include "ServiceMotor.hpp"
#include "bsp_can.hpp"
#include "main.h"
#include "om.h"
#include "magicmsgs.hpp"
#include "filter.hpp"
#include "math.hpp"
#include "bsp_dwt.hpp"

using namespace Numeric;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

TX_SEMAPHORE MotorCANRecvSem;

TX_THREAD MotorThread;
uint8_t MotorThreadStack[4096] = {0};
DJIMotorHandler* DJIMotorhandler = DJIMotorHandler::Instance();

motor_debug_t motor_debug;
pid_tuning_t motor_pos_pid;
pid_tuning_t motor_spd_pid;
int debug_cur = 0;
float vel_ratio = 0;

void ServiceMotors::MotorRegister() {
    // //注册电机
    DJIMotorhandler->registerMotor(&YawMotor, &hfdcan1, 0x205);
    YawMotor.currentSet = 0;
    YawMotor.gearBox = GearBox_None;

    DJIMotorhandler->registerMotor(&PitchMotor, &hfdcan1, 0x206);
    PitchMotor.currentSet = 0;
}

void ServiceMotors::AllMotorSetOutput()
{
    // LWheel.setOutput();
    // RWheel.setOutput();
}

void ServiceMotors::SetModeAndPidParam()
{
    YawMotor.speedPid.kp = 300.0f;
    YawMotor.speedPid.ki = 0.01f;
    YawMotor.speedPid.kd = 1.0f;

    PitchMotor.speedPid.kp = 100.0f;
}

double signal(double t)
{
    const double T = 0.9;           // 周期
    const double step_value = 0.19; // 最大幅值

    const double buffer_ratio = 1.0 / 5.0;
    const double rise_ratio = 4.0 / 5.0;

    double t_mod = std::fmod(t, T);

    double buffer_time = buffer_ratio * T;
    double rise_time = rise_ratio * T;

    if (t_mod < buffer_time)
    {
        // 前1/5缓冲段
        return 0.0;
    }
    else if (t_mod < T)
    {
        // 后4/5平滑上升段
        double rise_t = t_mod - buffer_time;
        double progress = rise_t / rise_time; // 0 ~ 1

        // 平滑函数：cosine ease-in
        double smooth = (1 - std::cos(Pi * progress)) / 2.0;

        return step_value * smooth;
    }
    else
    {
        return 0.0;
    }
}

[[noreturn]] void MotorThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    ULONG time;

    //注册电机
    ServiceMotors serviceMotors;
    serviceMotors.MotorRegister();

    om_suber_t *gimbal_suber = om_subscribe(om_find_topic("gimbalctrl", UINT32_MAX));
    msg_gimbal_ctrl_t gimbal_ctrl{};
    serviceMotors.SetModeAndPidParam();

    Filter::KalmanFilter test_spd_filer;
    test_spd_filer.SetQ(0.030f);
    test_spd_filer.SetR(1.200f);

    motor_pos_pid.kp = 10.0f;
    motor_pos_pid.ki = 0.0f;
    motor_pos_pid.kd = 0.0f;

    motor_spd_pid.kp = 300.0f;
    motor_spd_pid.ki = 0.01f;
    motor_spd_pid.kd = 1.0f;
    float yaw_init = 0.0f;

    for (;;) {
        om_suber_export(gimbal_suber, &gimbal_ctrl, false);
        if (gimbal_ctrl.pitch_mode == SPD)
        {
            serviceMotors.PitchMotor.speedPid.ref = gimbal_ctrl.pitch_speed;
            serviceMotors.PitchMotor.speedPid.fdb = serviceMotors.PitchMotor.motorFeedback.speedFdb;
            serviceMotors.PitchMotor.speedPid.UpdateResult();
            serviceMotors.PitchMotor.currentSet = static_cast<int16_t>(serviceMotors.PitchMotor.speedPid.result);
        }
        else
        {
            serviceMotors.PitchMotor.currentSet = static_cast<int16_t>(gimbal_ctrl.pitch_torque*100);
        }
        if (gimbal_ctrl.yaw_mode == SPD)
        {
            if (yaw_init == 0.0f)
            {
                yaw_init = serviceMotors.YawMotor.motorFeedback.positionFdb;
            }
            serviceMotors.YawMotor.positionPid.ref = yaw_init + signal(DWT_GetTimeline_s());
            serviceMotors.YawMotor.positionPid.fdb = serviceMotors.YawMotor.motorFeedback.positionFdb;
            serviceMotors.YawMotor.positionPid.UpdateResult();
            // serviceMotors.YawMotor.currentSet = static_cast<int16_t>(serviceMotors.YawMotor.positionPid.result*900);

            serviceMotors.YawMotor.speedPid.ref = serviceMotors.YawMotor.positionPid.result;
            serviceMotors.YawMotor.speedPid.fdb = test_spd_filer.Update(serviceMotors.YawMotor.motorFeedback.speedFdb);
            serviceMotors.YawMotor.speedPid.UpdateResult();
            serviceMotors.YawMotor.currentSet = static_cast<int16_t>(serviceMotors.YawMotor.speedPid.result);
        }
        else
        {
            serviceMotors.YawMotor.currentSet = static_cast<int16_t>(gimbal_ctrl.yaw_torque*100);
        }
        // debug_cur += 1;
        // if (debug_cur > 15000)
        //     debug_cur = 5000;
        // serviceMotors.YawMotor.currentSet = debug_cur;
        // if (serviceMotors.YawMotor.motorFeedback.speedFdb != 0.0f)
        //     vel_ratio = debug_cur / serviceMotors.YawMotor.motorFeedback.speedFdb;

        motor_debug.pos_set = serviceMotors.YawMotor.positionPid.ref;
        motor_debug.pos_fdb = serviceMotors.YawMotor.positionPid.fdb;
        motor_debug.spd_set = serviceMotors.YawMotor.speedPid.ref;
        motor_debug.spd_fdb = serviceMotors.YawMotor.speedPid.fdb;
        motor_debug.cur_set = serviceMotors.YawMotor.currentSet;
        motor_debug.cur_fdb = serviceMotors.YawMotor.motorFeedback.currentFdb;

        serviceMotors.YawMotor.speedPid.kp = motor_spd_pid.kp;
        serviceMotors.YawMotor.speedPid.ki = motor_spd_pid.ki;
        serviceMotors.YawMotor.speedPid.kd = motor_spd_pid.kd;

        serviceMotors.YawMotor.positionPid.kp = motor_pos_pid.kp;
        serviceMotors.YawMotor.positionPid.ki = motor_pos_pid.ki;
        serviceMotors.YawMotor.positionPid.kd = motor_pos_pid.kd;

        //发送控制指令给电机
        DJIMotorhandler->sendControlData();

        tx_thread_sleep(1);
    }
}

/**
 * @brief CAN接收中断回调函数，所有反馈在can上的数据会在这里根据ID进行分类并处理。
 * @param hfdcan CAN句柄
 * @note 该函数用于处理CAN接收中断，根据ID分类处理接收到的数据。但是这中方法可能会在回调里浪费时间，因为这里的处理是阻塞的。考虑是否需要将数据存储到一个缓冲区，然后在主循环中处理。
 */


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

   FDCAN_RxHeaderTypeDef rx_header;
   uint8_t rx_data[8];
   HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
    /*-------------------------------------------------大疆电机数据-------------------------------------------------*/
    if (rx_header.Identifier >= 0x201 && rx_header.Identifier <= 0x208)
    {
        if (hfdcan == &hfdcan1)
        {
            DJIMotorHandler::Instance()->updateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x201));
        }
        else if (hfdcan == &hfdcan2) // 处理CAN2的数据
        {
            DJIMotorHandler::Instance()->updateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x201));
        }
    }
   /*--------------------------------------------------LK电机数据--------------------------------------------------*/
   else if (rx_header.Identifier >= 0x140 && rx_header.Identifier <= 0x160)
   {
       if (hfdcan == &hfdcan1)
       {
           LKMotorHandler::instance()->updateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x141));
       }
       else if (hfdcan == &hfdcan2) // 处理CAN2的数据
       {
           // LKMotorHandler::instance()->processZeroPointData(hfdcan, rx_data, int(rx_header.Identifier - 0x141));
           LKMotorHandler::instance()->updateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x141));
       }
       else if (hfdcan == &hfdcan3) // 处理CAN3的数据
       {
           // LKMotorHandler::instance()->processZeroPointData(hfdcan, rx_data, int(rx_header.Identifier - 0x141));
           LKMotorHandler::instance()->updateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x141));
       }
   }
}