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
#include "crc.hpp"

#include "bsp_usart.hpp"
#include "config_sensor.hpp"

#include "magicmsgs.hpp"


TX_THREAD SensorThread;
uint8_t SensorThreadStack[2048] = {0};

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

__attribute__((section(".RAM_D1"))) uint8_t u2_rx_buffer[FORCE_DATA_RX_SIZE] = {0};
__attribute__((section(".RAM_D1"))) uint8_t u3_rx_buffer[FORCE_DATA_RX_SIZE] = {0};
static __attribute__((section(".RAM_D1"))) uint8_t u2_tx_buffer[8] = {0x01, 0x03, 0x00, 0x50, 0x00, 0x02, 0xC4, 0x1A};
static __attribute__((section(".RAM_D1"))) uint8_t u3_tx_buffer[8] = {0x01, 0x03, 0x00, 0x50, 0x00, 0x02, 0xC4, 0x1A};

struct FORCE
{
    int32_t force_L;
    int32_t force_R;
} force;





struct HALL
{
    bool is_L_reset;
    bool is_R_reset;
} hall;

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

    for (;;)
    {
        sensor.is_coil_R_reset = (HAL_GPIO_ReadPin(HALL_R_PORT, HALL_R_PIN) == GPIO_PIN_RESET);
        sensor.is_coil_L_reset = (HAL_GPIO_ReadPin(HALL_L_PORT, HALL_L_PIN) == GPIO_PIN_RESET);

        sensor.is_launchplat_return = (HAL_GPIO_ReadPin(LIGHT_PORT, LIGHT_PIN) == GPIO_PIN_SET);

        ForceSensor_RequestAll();

        sensor.string_L_force = force.force_L;
        sensor.string_R_force = force.force_R;

        om_publish(sensor_topic, &sensor, sizeof(msg_sensor_t), true, false);
        tx_thread_sleep(2);
    }
}

inline int32_t DecodeForce(const uint8_t* rx_buf)
{
    return (rx_buf[3] << 24) | (rx_buf[4] << 16) | (rx_buf[5] << 8) | rx_buf[6];
}

void ForceSensor_RequestAll(void)
{
    SCB_CleanDCache_by_Addr((uint32_t*)u2_tx_buffer, sizeof(u2_tx_buffer));
    HAL_UART_Transmit_DMA(&huart2, u2_tx_buffer, sizeof(u2_tx_buffer));

    SCB_CleanDCache_by_Addr((uint32_t*)u3_tx_buffer, sizeof(u3_tx_buffer));
    HAL_UART_Transmit_DMA(&huart3, u3_tx_buffer, sizeof(u3_tx_buffer));
}

void ForceSensor_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart2)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t*)u2_rx_buffer, FORCE_DATA_RX_SIZE);
        if (size == FORCE_DATA_RX_SIZE &&
            u2_rx_buffer[0] == 0x01 &&
            u2_rx_buffer[1] == 0x03 &&
            u2_rx_buffer[2] == 0x04 &&
            Verify_CRC16_Modbus_Check_Sum(u2_rx_buffer, FORCE_DATA_RX_SIZE))
        {
            force.force_L = DecodeForce(u2_rx_buffer);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, u2_rx_buffer, FORCE_DATA_RX_SIZE);
    }
    else if (huart == &huart3)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t*)u3_rx_buffer, FORCE_DATA_RX_SIZE);
        if (size == FORCE_DATA_RX_SIZE &&
            u3_rx_buffer[0] == 0x01 &&
            u3_rx_buffer[1] == 0x03 &&
            u3_rx_buffer[2] == 0x04 &&
            Verify_CRC16_Modbus_Check_Sum(u3_rx_buffer, FORCE_DATA_RX_SIZE))
        {
            force.force_R = DecodeForce(u3_rx_buffer);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, u3_rx_buffer, FORCE_DATA_RX_SIZE);
    }
}

void ForceSensor_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, u2_rx_buffer, FORCE_DATA_RX_SIZE);
    }
    else if (huart == &huart3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, u3_rx_buffer, FORCE_DATA_RX_SIZE);
    }
}
