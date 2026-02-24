/**
 * @file TaskSensor.cpp
 * @author Aeskyrr17
 * @brief 处理飞镖系统中传感器状态（Hall/光电门/力传感器/TOF），发布sensor_topic
 * HALL：RESET触发
 * LIGHT：SET触发
 * @todo 激光测距TOF未接入
 * @todo 力传感器ads1120：string_force目前未用
 */
#include "main.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"

#include "ADS1120.hpp"

TX_THREAD SensorThread;
uint8_t SensorThreadStack[2048] = {0};

#define HALL_R_PORT                GPIOE
#define HALL_R_PIN                 GPIO_PIN_1

#define HALL_L_PORT                GPIOE
#define HALL_L_PIN                 GPIO_PIN_0

#define LIGHT_PORT                 GPIOE
#define LIGHT_PIN                  GPIO_PIN_14

ADS1120_params ADS1120;

struct HALL
{
    bool is_L_reset;
    bool is_R_reset;
}hall;

float Val_Ch0 = 0.0f;
float Val_Ch1 = 0.0f;

[[nonreturn]] void SensorThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *sensor_topic = om_config_topic(nullptr, "ca", "sensor", sizeof(msg_sensor_t));
    msg_sensor_t sensor{};

    hall.is_L_reset = false;
    hall.is_R_reset = false;

    sensor.is_fire_done = false;
    sensor.is_coil_reset = false;
    sensor.is_door_open = false;
    sensor.is_string_tight = false;
    sensor.string_L_force = 0.0f;
    sensor.string_R_force = 0.0f;

    // ADS1120_init(&ADS1120);

    // ADS1120_setGain(&ADS1120, ADS1120_GAIN_1);

    for (;;)
    {

        //传感器数据
        hall.is_R_reset = (HAL_GPIO_ReadPin(HALL_R_PORT, HALL_R_PIN) == GPIO_PIN_RESET);
        hall.is_L_reset = (HAL_GPIO_ReadPin(HALL_L_PORT, HALL_L_PIN) == GPIO_PIN_RESET);
        sensor.is_coil_reset = (hall.is_L_reset && hall.is_R_reset);

        sensor.is_launchplat_return = (HAL_GPIO_ReadPin(LIGHT_PORT, LIGHT_PIN) == GPIO_PIN_SET);


    //    ADS1120_setCompareChannels(&ADS1120, ADS1120_MUX_2_AVSS);
        
    //     // 2. 读数据
    //     Val_Ch1 = ADS1120_getVoltage_mV(&ADS1120);

    //     debug111 = Val_Ch1;

        

        om_publish(sensor_topic, &sensor, sizeof(msg_sensor_t), true, false);
        tx_thread_sleep(1);
    }
} 
  



/**
 * @brief 外部中断回调,处理霍尔传感器和光电门的信息
 * 
 */
extern "C"
{
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        // if (GPIO_Pin == GPIO_PIN_0)//hall_L
        // {
        //     hall.L_reset = true;
        // }
        // else if (GPIO_Pin == GPIO_PIN_1)//hall_R
        // {
        //     hall.R_reset = true;
        // }
        // else if (GPIO_Pin == GPIO_PIN_14)//light gate
        // {
        //     sensor.launchplat_return = true;
        // }
    }
}