//
// Created by cosmosmount on 2025/9/2.
//
#include "BMI088.hpp"
#include "ServiceIMU.hpp"

using namespace BMI088;

static void BMI088Config(cBMI088 &bmi088) {
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