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

    void cBMI088::ReadAccData(acc_data_t *data)
    {
        uint8_t buf[ACC_XYZ_LEN + 1];                                         //< 读取数据缓存
        int16_t acc[3];                                                       //< 加速度计数据暂存
        ReadReg(BMI088_CS_ACC, ACC_X_LSB_ADDR, buf, ACC_XYZ_LEN + 1); //< 读取加速度计数据
        //< 拼接和转换数据
        acc[0] = ((int16_t)buf[1 + 1] << 8) + (int16_t)buf[0 + 1];
        acc[1] = ((int16_t)buf[3 + 1] << 8) + (int16_t)buf[2 + 1];
        acc[2] = ((int16_t)buf[5 + 1] << 8) + (int16_t)buf[4 + 1];
        data->x = SensorFilter[0].calculate((float)acc[0] * Acc_coef);
        data->y = SensorFilter[1].calculate((float)acc[1] * Acc_coef);
        data->z = SensorFilter[2].calculate((float)acc[2] * Acc_coef);
    }

    void cBMI088::ReadGyroData(gyro_data_t *data)
    {
        uint8_t buf[GYRO_XYZ_LEN]; //, range; //< 读取数据缓存, range其实是写入的配置
        int16_t gyro[3];

        ReadReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR, buf, GYRO_XYZ_LEN);
        //< 拼接和转换数据
        gyro[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
        gyro[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
        gyro[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
        // 注意这里就不要又除法又乘法的了，直接乘以一个常数，根据配置，单位是16.384，所以直接乘以1/16.384 * DEG2SEC即可
        // 滤波
        data->roll = SensorFilter[3].calculate((float)gyro[0] * IMU_GYRO_2000_SEN);
        data->pitch = SensorFilter[4].calculate((float)gyro[1] * IMU_GYRO_2000_SEN);
        data->yaw = SensorFilter[5].calculate((float)gyro[2] * IMU_GYRO_2000_SEN);
    }


}

