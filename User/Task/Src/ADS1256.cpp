#include "main.h"
#include "stm32h723xx.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_it.h"

#include "bsp_spi.hpp"
#include "bsp_dwt.hpp"
#include "tx_api.h"

#include "ADS1256.hpp"

#define PORT_DRDY               GPIOE
#define PIN_DRDY                GPIO_PIN_1

#define PORT_CS                 GPIOE
#define PIN_CS                  GPIO_PIN_0

#define ADS_CS_LOW()            HAL_GPIO_WritePin(PORT_CS, PIN_CS, GPIO_PIN_RESET)
#define ADS_CS_HIGH()           HAL_GPIO_WritePin(PORT_CS, PIN_CS, GPIO_PIN_SET)

#define ADS_DRDY_PIN_STATE()    HAL_GPIO_ReadPin(PORT_DRDY, PIN_DRDY)

#define ADS_SPI_HANDLER         &hspi3

extern TX_SEMAPHORE ads_drdy_sem;

uint8_t ads1256_spi_tx_buf[10] = {0};
uint8_t ads1256_spi_rx_buf[10] = {0};

/**
 * @brief ADS1256写数据 write register
 * 向ADS1256中地址为regaddr的寄存器写入一个字节databyte
 * @param reg_addr 寄存器地址
 * @param databyte 需要写的1byte数据
 */
void ADS1256WREG(unsigned char reg_addr,unsigned char databyte)
{
    ADS_CS_LOW();
    while(ADS_DRDY_PIN_STATE() == GPIO_PIN_SET); 

    ads1256_spi_tx_buf[0] = ADS1256_CMD_WREG | (reg_addr & 0x0F);
    ads1256_spi_tx_buf[1] = 0x00; 
    ads1256_spi_tx_buf[2] = databyte;

    SPI_Transmit(ADS_SPI_HANDLER, ads1256_spi_tx_buf, 3, SPI_BLOCK_MODE);
    ADS_CS_HIGH();
}

/**
 * @brief ADS1256读数据
 * @param channel
 * @return sum
 */
int32_t ADS1256ReadData(uint8_t channel)  
{
    int32_t sum = 0;

    ADS_CS_LOW(); 

    // 写MUX寄存器切换通道
    ads1256_spi_tx_buf[0] = ADS1256_CMD_WREG | 0x01; 
    ads1256_spi_tx_buf[1] = 0x00;                    
    ads1256_spi_tx_buf[2] = channel;                 
    SPI_Transmit(ADS_SPI_HANDLER, ads1256_spi_tx_buf, 3, SPI_BLOCK_MODE);
 
    // 重启ADC(SYNC+WAKEUP)
    ads1256_spi_tx_buf[0] = ADS1256_CMD_SYNC;
    SPI_Transmit(ADS_SPI_HANDLER, ads1256_spi_tx_buf, 1, SPI_BLOCK_MODE);
    DWT_Delay_us(10); // t11
    
    ads1256_spi_tx_buf[0] = ADS1256_CMD_WAKEUP;
    SPI_Transmit(ADS_SPI_HANDLER, ads1256_spi_tx_buf, 1, SPI_BLOCK_MODE);
    

    ADS_CS_HIGH(); 

    tx_semaphore_get(&ads_drdy_sem, TX_WAIT_FOREVER);

    ADS_CS_LOW();

    ads1256_spi_tx_buf[0] = ADS1256_CMD_RDATA;
    SPI_Transmit(ADS_SPI_HANDLER, ads1256_spi_tx_buf, 1, SPI_BLOCK_MODE);
    

    DWT_Delay_us(10);     //硬件延时

    SPI_Receive(ADS_SPI_HANDLER, ads1256_spi_rx_buf, 3, SPI_BLOCK_MODE);

    ADS_CS_HIGH(); 

    sum = ((int32_t)ads1256_spi_rx_buf[0] << 16) | 
          ((int32_t)ads1256_spi_rx_buf[1] << 8)  | 
          (int32_t)ads1256_spi_rx_buf[2];

    if (sum > 0x7FFFFF) 
    {
        sum -= 0x1000000; 
    }
    
    return sum;
}

/**
 * @brief 向ads1256发送1byte的简单指令
 * @param cmd 命令
 */
void ADS1256_SendCmd(uint8_t cmd)
{
    ADS_CS_LOW();
    ads1256_spi_tx_buf[0] = cmd;

    SPI_Transmit(ADS_SPI_HANDLER, ads1256_spi_tx_buf , 1, SPI_BLOCK_MODE);
    ADS_CS_HIGH();
}

/**
 * @brief 初始化 (保持不变)
 */
void ADS1256_Init(void)
{
    // 初始化里可以用死等，因为只会执行一次
    while(ADS_DRDY_PIN_STATE() == GPIO_PIN_SET);
    ADS1256_SendCmd(ADS1256_CMD_SELFCAL);
    while(ADS_DRDY_PIN_STATE() == GPIO_PIN_SET);

    ADS1256WREG(ADS1256_STATUS,0x06);               
    ADS1256WREG(ADS1256_ADCON,ADS1256_GAIN_1);      
    ADS1256WREG(ADS1256_DRATE,ADS1256_DRATE_10SPS); 
    ADS1256WREG(ADS1256_IO,0x00);               

    while(ADS_DRDY_PIN_STATE() == GPIO_PIN_SET);
    ADS1256_SendCmd(ADS1256_CMD_SELFCAL);
    while(ADS_DRDY_PIN_STATE() == GPIO_PIN_SET);
}

