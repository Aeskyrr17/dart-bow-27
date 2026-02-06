#include "main.h"
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

#define LIGHT_PORT             GPIOE
#define LIGHT_PIN              GPIO_PIN_14

ADS1120_params ADS1120;
msg_sensor_t msg_sensor{};

struct HALL
{
    bool is_L_reset;
    bool is_R_reset;
}hall;

float Val_Ch0 = 0.0f;
float Val_Ch1 = 0.0f;

float debug111;

[[nonreturn]] void SensorThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *sensor_topic = om_config_topic(nullptr, "ca", "sensor", sizeof(msg_sensor_t));

    hall.is_L_reset = false;
    hall.is_R_reset = false;
    msg_sensor.is_fire_done = false;
    msg_sensor.is_coil_reset = false;
    msg_sensor.is_door_open = false;
    msg_sensor.is_string_tight = false;
    msg_sensor.string_L_force = 0.0f;
    msg_sensor.string_R_force = 0.0f;

    ADS1120_init(&ADS1120);

    ADS1120_setGain(&ADS1120, ADS1120_GAIN_1);

    for (;;)
    {

        //传感器数据
        if (HAL_GPIO_ReadPin(HALL_R_PORT, HALL_R_PIN))
            hall.is_R_reset = true;
        else 
            hall.is_R_reset = false;

        if (HAL_GPIO_ReadPin(HALL_L_PORT, HALL_L_PIN))
            hall.is_L_reset = true;
        else 
            hall.is_L_reset = false;

        if (HAL_GPIO_ReadPin(LIGHT_PORT, LIGHT_PIN))
            msg_sensor.is_launchplat_return = true;
        else 
            msg_sensor.is_launchplat_return = false;

        if (hall.is_L_reset && hall.is_R_reset)
            msg_sensor.is_coil_reset = true;
        else 
            msg_sensor.is_coil_reset = false;


       ADS1120_setCompareChannels(&ADS1120, ADS1120_MUX_2_AVSS);
        
        // 2. 读数据
        Val_Ch1 = ADS1120_getVoltage_mV(&ADS1120);

        debug111 = Val_Ch1;

        

        om_publish(sensor_topic, &msg_sensor, sizeof(msg_sensor_t), true, false);
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
        if (GPIO_Pin == GPIO_PIN_0)//hall_L
        {
            hall.is_L_reset = true;
        }
        else if (GPIO_Pin == GPIO_PIN_1)//hall_R
        {
            hall.is_R_reset = true;
        }
        else if (GPIO_Pin == GPIO_PIN_14)//light gate
        {
            msg_sensor.is_launchplat_return = true;
        }
    }
}