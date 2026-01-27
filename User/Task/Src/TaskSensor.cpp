#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"

TX_THREAD SensorThread;
uint8_t SensorThreadStack[2048] = {0};

[[nonreturn]] void SensorThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *sensor_topic = om_config_topic(nullptr, "ca", "sensor", sizeof(msg_sensor_t));
    msg_sensor_t msg_sensor{};

    for (;;)
    {

        om_publish(sensor_topic, &msg_sensor, sizeof(msg_sensor_t), true, false);
        tx_thread_sleep(1);

    }
}