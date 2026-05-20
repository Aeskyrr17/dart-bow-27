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
#include "config_sensor.hpp"

TX_THREAD SensorThread;
uint8_t SensorThreadStack[2048] = {0};

// usart2 force_left
uint8_t u2_rx_buffer[FORCE_DATA_RX_SIZE];
static uint8_t u2_rx_done = 0;

// usart3 force_right
uint8_t u3_rx_buffer[FORCE_DATA_RX_SIZE];
static uint8_t u3_rx_done = 0;

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define USING_G4_FORCE_SENSOR


struct FORCE
{
    int32_t force_L;
    int32_t force_R;
} force;

struct HALL
{
    bool is_L_reset;
    bool is_R_reset;
}hall;

#define G4_FORCE_RX_TIMEOUT 100
#define G4_FORCE_RX_RESTART_DELAY 3

struct G4Force
{
    uint32_t L;
    uint32_t R;
}g4_force;

__attribute__((section(".RAM_D1"), aligned(32))) uint8_t g4_rx_buffer[G4_FORCE_RX_BUFFER_SIZE];
TX_SEMAPHORE G4ForceGot;

static uint32_t ForceSensor_DecodeU24(const uint8_t* buf)
{
    return ((uint32_t)buf[0]) |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16);
}

void Force_L_Request080();
void Force_R_Request080();

[[nonreturn]] void SensorThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    om_topic_t *sensor_topic = om_config_topic(nullptr, "ca", "sensor", sizeof(msg_sensor_t));
    msg_sensor_t sensor{};

    // HALL hall{};

    hall.is_L_reset = false;
    hall.is_R_reset = false;

    sensor.is_fire_done = false;
    sensor.is_coil_reset = false;
    sensor.is_door_open = false;
    sensor.is_string_tight = false;
    sensor.string_L_force = 0.0f;
    sensor.string_R_force = 0.0f;

    uint32_t g4_force_lost_ticks = 0;

    HAL_UART_Receive_IT(&huart2, u2_rx_buffer, FORCE_DATA_RX_SIZE);
    HAL_UART_Receive_IT(&huart3, u3_rx_buffer, FORCE_DATA_RX_SIZE);

    for (;;)
    {
        // hall.is_R_reset = (HAL_GPIO_ReadPin(HALL_R_PORT, HALL_R_PIN) == GPIO_PIN_RESET);
        // hall.is_L_reset = (HAL_GPIO_ReadPin(HALL_L_PORT, HALL_L_PIN) == GPIO_PIN_RESET);
        // sensor.is_coil_reset = (hall.is_L_reset && hall.is_R_reset);
        sensor.is_coil_R_reset = (HAL_GPIO_ReadPin(HALL_R_PORT, HALL_R_PIN) == GPIO_PIN_RESET);
        sensor.is_coil_L_reset = (HAL_GPIO_ReadPin(HALL_L_PORT, HALL_L_PIN) == GPIO_PIN_RESET);

        sensor.is_launchplat_return = (HAL_GPIO_ReadPin(LIGHT_PORT, LIGHT_PIN) == GPIO_PIN_SET);


#ifdef USING_G4_FORCE_SENSOR
        bool got_g4_rx_done = false;
        while (tx_semaphore_get(&G4ForceGot, 0) == TX_SUCCESS)
        {
            got_g4_rx_done = true;
        }

        if (got_g4_rx_done && g4_rx_buffer[0] == 'L' && g4_rx_buffer[4] == 'R')
        {
            g4_force_lost_ticks = 0;
            g4_force.L = ForceSensor_DecodeU24(&g4_rx_buffer[1]);
            g4_force.R = ForceSensor_DecodeU24(&g4_rx_buffer[5]);
        }
        else
        {
            g4_force_lost_ticks++;
            if (g4_force_lost_ticks >= G4_FORCE_RX_TIMEOUT)
            {
                HAL_UART_Abort(&huart7);
                tx_thread_sleep(G4_FORCE_RX_RESTART_DELAY);
                HAL_UART_Receive_DMA(&huart7, g4_rx_buffer, G4_FORCE_RX_DATA_SIZE);
                g4_force_lost_ticks = 0;
            }
        }
        sensor.string_L_force = (float)g4_force.L;
        sensor.string_R_force = (float)g4_force.R;

        om_publish(sensor_topic, &sensor, sizeof(msg_sensor_t), true, false);
        tx_thread_sleep(1);

#else

        Force_L_Request080();
        Force_R_Request080();

        sensor.string_L_force = force.force_L;
        sensor.string_R_force = force.force_R;

        om_publish(sensor_topic, &sensor, sizeof(msg_sensor_t), true, false);

        tx_thread_sleep(5);
#endif

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

void ForceSensor_RxCpltCallback(UART_HandleTypeDef *huart)
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

void ForceSensor_ErrorCallback(UART_HandleTypeDef *huart)
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
