//
// Created by cosmosmount on 2025/9/2.
//
#include "bsp_dwt.hpp"
#include "BMI088.hpp"
#include "ServiceIMU.hpp"

using namespace BMI088;

static void BMI088Config(cBMI088 &bmi088);
static cIMU *imu_handle = nullptr;

TX_THREAD IMUThread;
TX_SEMAPHORE IMUThreadSem;
uint8_t IMUThreadStack[4096] = {0};

[[noreturn]] void IMUThreadFun(ULONG initial_input)
{
    TX_PARAMETER_NOT_USED(initial_input);
    /* INS Topic */
    // om_topic_t *ins_topic = om_config_topic(nullptr, "CA", "INS", sizeof(Msg_INS_t));
    /* BMI088 handle*/
    cBMI088 bmi088;
    imu_handle = &bmi088;

    float ACC_Coef = IMU_ACCEL_3G_SEN;
    float gyroMax[3], gyroMin[3];        // 保存标定过程中读取到的数据最大值判断是否满足标定环境
    float gNormTemp, gNormMax, gNormMin; // 同上,计算矢量范数(模长)
    float gyroDiff[3], gNormDiff;        // 每个轴的最大角速度跨度及其模长

    float startTime = DWT_GetTimeline_s();

    do {
        gNormDiff = 0.0f;

    }while (gNormDiff > 0.5f ||
             // fabsf(gNorm - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f
             // fabsf(Gyro_offset[0]) > 0.01f ||
             // fabsf(Gyro_offset[1]) > 0.01f ||
             // fabsf(Gyro_offset[2]) > 0.01f
             );

    BMI088Config(bmi088); //< 配置BMI088


    for (;;)
    {
        tx_thread_sleep(1);

        tx_semaphore_put(&IMUThreadSem);

        // om_publish(ins_topic, &msg_ins, sizeof(msg_ins), true, false);
    }
}

static void BMI088Config(cBMI088 &bmi088)
{
    {
        tx_thread_sleep(10); //< 等待系统稳定
        //< 加速度计初始化
        //< 先软重启，清空所有寄存器
        uint8_t pTxData;
        pTxData = ACC_SOFTRESET_VAL;
        bmi088.WriteReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        //< 打开加速度计电源
        pTxData = ACC_PWR_CTRL_ON;
        bmi088.WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(10); //< 延时10ms

        //< 加速度计变成正常模式
        pTxData = ACC_PWR_CONF_ACT;
        bmi088.WriteReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(10); //< 延时10ms

        //< 测量范围
        pTxData = ACC_RANGE_3G;
        bmi088.WriteReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0xAC;
        bmi088.WriteReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x08;
        bmi088.WriteReg(BMI088_CS_ACC, INT1_IO_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x04;
        bmi088.WriteReg(BMI088_CS_ACC, INT_MAP_DATA_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        /*-------------------------------------陀螺仪初始化-------------------------------------*/
        //< 先软重启，清空所有寄存器
        pTxData = GYRO_SOFTRESET_VAL;
        bmi088.WriteReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        pTxData = GYRO_RANGE_1000_DEG_S;
        bmi088.WriteReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x02;//GYRO_ODR_1000Hz_BANDWIDTH_116Hz | GYRO_LPM1_SUS;
        bmi088.WriteReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_LPM1_NOR;
        bmi088.WriteReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x80;
        bmi088.WriteReg(BMI088_CS_GYRO, GYRO_INT_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x0C;
        bmi088.WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x01;
        bmi088.WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_MAP_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms
    }
}