#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"

#include "ADS1256.hpp"

TX_THREAD SensorThread;
uint8_t SensorThreadStack[2048] = {0};

TX_SEMAPHORE ads_drdy_sem;

[[nonreturn]] void SensorThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *sensor_topic = om_config_topic(nullptr, "ca", "sensor", sizeof(msg_sensor_t));
    msg_sensor_t msg_sensor{};

    ADS1256_Init();

    for (;;)
    {

        om_publish(sensor_topic, &msg_sensor, sizeof(msg_sensor_t), true, false);
        tx_thread_sleep(1);

    }
}




/**
 * @brief 外部中断回调
 * 由于while死等ads的DRDY耗费时间，利用外部中断并发送sema
 */
extern "C"
{
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        if (GPIO_Pin == GPIO_PIN_1)
        {
            tx_semaphore_put(&ads_drdy_sem);
        }
    }
}