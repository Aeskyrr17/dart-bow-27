/**
 * @file TaskSensor.cpp
 * @author Aeskyrr17
 * @brief Process sensor status and publish sensor_topic
 */
#include "main.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_uart.h"
#include "tx_api.h"
#include "om.h"

#include "bsp_usart.hpp"

#include "magicmsgs.hpp"
#include <cstdint>

#include "crc.hpp"
// #include "config_sensor.hpp"

TX_THREAD SensorThread;
uint8_t SensorThreadStack[2048] = {0};

#define HALL_R_PORT                GPIOE
#define HALL_R_PIN                 GPIO_PIN_1

#define HALL_L_PORT                GPIOE
#define HALL_L_PIN                 GPIO_PIN_0

#define LIGHT_PORT                 GPIOE
#define LIGHT_PIN                  GPIO_PIN_14

#define FORCE_DATA_RX_SIZE         9

// usart2 force_left
static uint8_t u2_rx_buffer[FORCE_DATA_RX_SIZE];
static uint8_t u2_rx_done = 0;

// usart3 force_right
static uint8_t u3_rx_buffer[FORCE_DATA_RX_SIZE];
static uint8_t u3_rx_done = 0;

struct FORCE
{
    int32_t force_L;
    int32_t force_R;
} force;

struct HALL
{
    bool is_L_reset;
    bool is_R_reset;
};

void Force_L_Request080();
void Force_R_Request080();

[[nonreturn]] void SensorThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    om_topic_t *sensor_topic = om_config_topic(nullptr, "ca", "sensor", sizeof(msg_sensor_t));
    msg_sensor_t sensor{};

    HALL hall{};

    hall.is_L_reset = false;
    hall.is_R_reset = false;

    sensor.is_fire_done = false;
    sensor.is_coil_reset = false;
    sensor.is_door_open = false;
    sensor.is_string_tight = false;
    sensor.string_L_force = 0.0f;
    sensor.string_R_force = 0.0f;

    HAL_UART_Receive_IT(&huart2, u2_rx_buffer, FORCE_DATA_RX_SIZE);
    HAL_UART_Receive_IT(&huart3, u3_rx_buffer, FORCE_DATA_RX_SIZE);

    for (;;)
    {
        hall.is_R_reset = (HAL_GPIO_ReadPin(HALL_R_PORT, HALL_R_PIN) == GPIO_PIN_RESET);
        hall.is_L_reset = (HAL_GPIO_ReadPin(HALL_L_PORT, HALL_L_PIN) == GPIO_PIN_RESET);
        sensor.is_coil_reset = (hall.is_L_reset && hall.is_R_reset);

        sensor.is_launchplat_return = (HAL_GPIO_ReadPin(LIGHT_PORT, LIGHT_PIN) == GPIO_PIN_SET);

        Force_L_Request080();
        Force_R_Request080();

        sensor.string_L_force = force.force_L;
        sensor.string_R_force = force.force_R;

        om_publish(sensor_topic, &sensor, sizeof(msg_sensor_t), true, false);
        tx_thread_sleep(5);
    }
}

// read force value(080): 01 03 00 50 00 02 + crc16（C4 1A)
void Force_L_Request080()
{
    uint8_t cmd[8] = {0x01, 0x03, 0x00, 0x50, 0x00, 0x02, 0x00, 0x00};
    Append_CRC16_Modbus_Check_Sum(cmd, 8);
    u2_rx_done = 0;
    HAL_UART_Transmit(&huart2, cmd, 8, 100);
}

void Force_R_Request080()
{
    uint8_t cmd[8] = {0x01, 0x03, 0x00, 0x50, 0x00, 0x02, 0x00, 0x00};
    Append_CRC16_Modbus_Check_Sum(cmd, 8);
    u3_rx_done = 0;
    HAL_UART_Transmit(&huart3, cmd, 8, 100);
}

int32_t Decode_Force(const uint8_t* rx_buf)
{
    int32_t force = (rx_buf[3] << 24) | (rx_buf[4] << 16) | (rx_buf[5] << 8) | rx_buf[6];
    return force;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        if (u2_rx_buffer[0] == 0x01 &&
            u2_rx_buffer[1] == 0x03 &&
            u2_rx_buffer[2] == 0x04 &&
            Verify_CRC16_Modbus_Check_Sum(u2_rx_buffer, FORCE_DATA_RX_SIZE))
        {
            force.force_L = Decode_Force(u2_rx_buffer);
            u2_rx_done = 1;
        }
        else
        {
            u2_rx_done = 0;
        }
        HAL_UART_Receive_IT(&huart2, u2_rx_buffer, FORCE_DATA_RX_SIZE);
    }

    if (huart == &huart3)
    {
        if (u3_rx_buffer[0] == 0x01 &&
            u3_rx_buffer[1] == 0x03 &&
            u3_rx_buffer[2] == 0x04 &&
            Verify_CRC16_Modbus_Check_Sum(u3_rx_buffer, FORCE_DATA_RX_SIZE))
        {
            force.force_R = Decode_Force(u3_rx_buffer);
            u3_rx_done = 1;
        }
        else
        {
            u3_rx_done = 0;
        }
        HAL_UART_Receive_IT(&huart3, u3_rx_buffer, FORCE_DATA_RX_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        HAL_UART_Receive_IT(&huart2, u2_rx_buffer, FORCE_DATA_RX_SIZE);
    }

    if (huart == &huart3)
    {
        HAL_UART_Receive_IT(&huart3, u3_rx_buffer, FORCE_DATA_RX_SIZE);
    }
}
