//
// Created by cosmosmount on 2025/9/2.
//
#include "BMI088.hpp"
#include "tx_api.h"
#include "bsp_pwm.hpp"
#include "ServiceIMU.hpp"
#include "AHRS.hpp"


using namespace BMI088;

cBMI088 *bmi088 = cBMI088::Instance();

AHRS *ahrs = AHRS::Instance();


TX_THREAD IMUThread;
uint8_t IMUThreadStack[4096] = {0};

TX_SEMAPHORE IMUThreadSem;

ULONG IMU_time;

[[noreturn]] void IMUThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    IMU_time = tx_time_get();

    /* INS Topic */
    // om_topic_t *ins_topic = om_config_topic(nullptr, "CA", "INS", sizeof(Msg_INS_t));

    bmi088->bmi088_selfTest.ACC_CHIP_ID_ERR = true;       // 加速度计ID错误则为true
    bmi088->bmi088_selfTest.ACC_DATA_ERR = true;          // 加速度计数据错误则为true
    bmi088->bmi088_selfTest.GYRO_CHIP_ID_ERR = true;      // 陀螺仪ID错误则为true
    bmi088->bmi088_selfTest.GYRO_DATA_ERR = true;         // 陀螺仪数据错误则为true
    bmi088->bmi088_selfTest.INIT_ERR = true;       // BMI088初始化错误则为true
    bmi088->bmi088_selfTest.CALIBRATE_ERR = false; // BMI088标定错误则为true
    bmi088->bmi088_selfTest.TEMP_CTRL_ERR = false; // BMI088温度控制错误则为true

    bmi088->BMI088Config(); //< 初始化配置

    bmi088->VerifyAccChipID();  //< 验证加速度计ID
    bmi088->VerifyGyroChipID(); //< 验证陀螺仪ID

    while (bmi088->bmi088_data.acc_data.temperature < 45.0f) {
        tx_thread_sleep(100);
    }
    tx_thread_sleep(2000);

    bmi088->CalibrateIMU(); //< 标定IMU

    bmi088->bmi088_selfTest.INIT_ERR = false;

    ahrs->INS_Init();

    for (;;) {

        if (!bmi088->bmi088_selfTest.INIT_ERR) {
            bmi088->ReadAccData(&bmi088->bmi088_data.acc_data);
            bmi088->ReadGyroData(&bmi088->bmi088_data.gyro_data);
        }
        ahrs->AHRS_Update();

        tx_semaphore_put(&IMUThreadSem);

        uint8_t time_to_delay = tx_time_get() - IMU_time;
        if (time_to_delay < 1) {
            tx_thread_sleep(1 - time_to_delay);
        }
        // om_publish(ins_topic, &msg_ins, sizeof(msg_ins), true, false);
    }
}

TX_THREAD IMUTempThread;
uint8_t IMUTempThreadStack[2048] = {0};
// TX_SEMAPHORE IMUTempThreadSem;

[[noreturn]] void IMUTempThreadFun(ULONG initial_input) {
    UNUSED(initial_input);

    bmi088->TempPid.mode = PID_POSITION | PID_Integral_Limit | PID_Changing_Integral_Rate | PID_Derivative_On_Measurement; // 位置式PID，积分限幅
    bmi088->TempPid.kp = 650.0f;
    bmi088->TempPid.ki = 0.06f;
    bmi088->TempPid.kd = 0.1f;
    bmi088->TempPid.maxOut = 300.0f;
    bmi088->TempPid.maxIOut = 300.0f;
    bmi088->TempPid.ScalarA = 3.5f;
    bmi088->TempPid.ScalarB = 0.08f;

    bmi088->TempFdbFilter.SetTau(0.1f);       // 设置滤波时间常数
    bmi088->TempFdbFilter.SetUpdatePeriod(1); // 设置更新周期

    bmi088->SetTargetTemp(45.0f);                              //< 设置目标温度，一般为40度以上
    PWM_Start(&HEATING_RESISTANCE_TIM, TIM_CHANNEL_4); //< 启动加热电阻PWM

    float tmp_last = bmi088->bmi088_data.acc_data.temperature;
    tx_thread_sleep(1000);
    bmi088->ReadAccTemperature(&bmi088->bmi088_data.acc_data.temperature);
    if (tmp_last == bmi088->bmi088_data.acc_data.temperature) {
        //error in temp
        tx_thread_suspend(&IMUTempThread);
    }

    for (;;) {
        // tx_semaphore_get(&IMUTempThreadSem, TX_WAIT_FOREVER);
        bmi088->ReadAccTemperature(&bmi088->bmi088_data.acc_data.temperature);
        bmi088->TemperatureControl(bmi088->TargetTemp);

        uint8_t time_to_delay = tx_time_get() - IMU_time;
        if (time_to_delay < 1) {
            tx_thread_sleep(1 - time_to_delay);
        }
    }
}
