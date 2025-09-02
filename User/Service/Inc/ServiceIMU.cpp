//
// Created by cosmosmount on 2025/9/2.
//
#include "BMI088.hpp"

using namespace BMI088;

static void BMI088Config(cBMI088 &bmi088) {
{
    //< 加速度计初始化
    //< 先软重启，清空所有寄存器
    uint8_t pTxData;
    pTxData = ACC_SOFTRESET_VAL;
    bmi088.WriteReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
    DWT_Delay(0.150); //< 延时50ms,重启需要时间

    //< 打开加速度计电源
    pTxData = ACC_PWR_CTRL_ON;
    bmi088.WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms,重启需要时间

    //< 加速度计变成正常模式
    pTxData = ACC_PWR_CONF_ACT;
    bmi088.WriteReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms,重启需要时间

    pTxData = ((0x2 << 0x4) | (0xB << 0x0) | 0x80);
    bmi088.WriteReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    //< 测量范围
    pTxData = ACC_RANGE_3G;
    bmi088.WriteReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = ((0x1 << 0x3) | (0x0 << 0x2) | (0x0 << 0x1));
    bmi088.WriteReg(BMI088_CS_ACC, INT1_IO_CTRL_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = ((0x1 << 0x2));
    bmi088.WriteReg(BMI088_CS_ACC, INT_MAP_DATA_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    /*-------------------------------------陀螺仪初始化-------------------------------------*/
    //< 先软重启，清空所有寄存器
    pTxData = GYRO_SOFTRESET_VAL;
    bmi088.WriteReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
    DWT_Delay(0.150); //< 延时50ms,重启需要时间

    pTxData = GYRO_RANGE_2000_DEG_S;
    bmi088.WriteReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms,重启需要时间

    pTxData = GYRO_ODR_1000Hz_BANDWIDTH_116Hz | GYRO_LPM1_SUS;
    bmi088.WriteReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = GYRO_LPM1_NOR;
    bmi088.WriteReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = 0x80;
    bmi088.WriteReg(BMI088_CS_GYRO, GYRO_INT_CTRL_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = ((0x0 << 0x1) | (0x0 << 0x0));
    bmi088.WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_CONF_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = 0x01;
    bmi088.WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_MAP_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms
}