//
// Created by cosmosmount on 2025/9/2.
//

#include "BMI088.hpp"
#include "bsp_spi.hpp"

namespace BMI088
{
    void cBMI088::Init() {
        gNorm = 9.805f;

        bmi088_selfTest.ACC_CHIP_ID_ERR = true;       // 加速度计ID错误则为true
        bmi088_selfTest.ACC_DATA_ERR = true;          // 加速度计数据错误则为true
        bmi088_selfTest.GYRO_CHIP_ID_ERR = true;      // 陀螺仪ID错误则为true
        bmi088_selfTest.GYRO_DATA_ERR = true;         // 陀螺仪数据错误则为true
        bmi088_selfTest.INIT_ERR = true;       // BMI088初始化错误则为true
        bmi088_selfTest.CALIBRATE_ERR = false; // BMI088标定错误则为true
        bmi088_selfTest.TEMP_CTRL_ERR = false; // BMI088温度控制错误则为true

        Acc_coef = IMU_ACCEL_3G_SEN; // 标定完后要乘以9.805/gNorm，注意这里需要和配置的范围对应

        BMI088_CONF_INIT(); //< 初始化配置

        VerifyAccChipID();  //< 验证加速度计ID
        VerifyGyroChipID(); //< 验证陀螺仪ID

        TempPid.mode = PID_POSITION | PID_Integral_Limit | PID_Changing_Integral_Rate | PID_Derivative_On_Measurement; // 位置式PID，积分限幅
        TempPid.kp = 650.0f;
        TempPid.ki = 0.06f;
        TempPid.kd = 0.1f;
        TempPid.maxOut = 300.0f;
        TempPid.maxIOut = 300.0f;
        TempPid.ScalarA = 3.5f;
        TempPid.ScalarB = 0.08f;

        TempFdbFilter.SetTau(0.1f);       // 设置滤波时间常数
        TempFdbFilter.SetUpdatePeriod(1); // 设置更新周期

    SetTargetTemp(45.0f);                              //< 设置目标温度，一般为40度以上
    PWM_Start(&HEATING_RESISTANCE_TIM, TIM_CHANNEL_4); //< 启动加热电阻PWM

   float startTime; // 开始升温时间,用于确定是否超时
   startTime = DWT_GetTimeline_s();
   while (fabs(bmi088_data.acc_data.temperature - TargetTemp) > 0.1f)
   {
       if (DWT_GetTimeline_s() - startTime > 1.00) // 超时则直接进入下一步
       {
           bmi088_selfTest.BMI088_TEMP_CTRL_ERR = true;
           startTime = DWT_GetTimeline_s();
           break;
       }
       ReadAccTemperature(&bmi088_data.acc_data.temperature);
       TemperatureControl(TargetTemp);
   }
   startTime = DWT_GetTimeline_s();
   while (DWT_GetTimeline_s() - startTime < 2.01)
   {
       ReadAccTemperature(&bmi088_data.acc_data.temperature);
       TemperatureControl(TargetTemp);
   }
    CalibrateIMU(); //< 标定IMU
    }

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

