//
// Created by cosmosmount on 2025/9/2.
//

#include "BMI088.hpp"
#include "bsp_spi.hpp"

namespace BMI088
{
    void cBMI088::ReadReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        //< 片选，考虑以后进行bsp_gpio封装
        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
        else if (cs == BMI088_CS_GYRO)
            HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);

        uint8_t pTxData = (addr | BMI088_SPI_READ_CODE); //< 处理地址为读地址

        SPI_Transmit(&BMI088_SPI, &pTxData, 1, SPI_BLOCK_MODE); //< 发送地址
        SPI_Receive(&BMI088_SPI, data, len, SPI_BLOCK_MODE);    //< 读取数据

        //< 取消片选
        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
        else if (cs == BMI088_CS_GYRO)
            HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
    }

    void cBMI088::WriteReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        //< 片选，考虑以后进行bsp_gpio封装
        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
        else if (cs == BMI088_CS_GYRO)
            HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);

        uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE); //< 处理地址为写地址

        SPI_Transmit(&BMI088_SPI, &pTxData, 1, SPI_BLOCK_MODE); //< 发送地址
        SPI_Transmit(&BMI088_SPI, data, len, SPI_BLOCK_MODE);   //< 发送数据

        //< 取消片选
        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
        else if (cs == BMI088_CS_GYRO)
            HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
    }



}

