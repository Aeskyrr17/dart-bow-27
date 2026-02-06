/**
 * 此工程中将DOUT/DRDY都接到MISO,若后续需要修改，需需要修改ADS1120_init
 * 
 */

#include "ADS1120.hpp"
#include "main.h"
#include "tx_api.h" 

//在此处修改引脚定义,此工程中将DOUT/DRDY都接到MISO
#define PORT_DRDY               GPIOC
#define PIN_DRDY                GPIO_PIN_11

//此工程中CS直接接GND，所以此处引脚填一个没有使用的
#define PORT_CS                 GPIOE
#define PIN_CS                  GPIO_PIN_5

extern SPI_HandleTypeDef hspi3; 

#define ADS_SPI_HANDLER         &hspi3

// extern TX_SEMAPHORE ads_drdy_sem;



void ADS1120_User_Setup(ADS1120_params *adsParam) {
    adsParam->adsSpi = ADS_SPI_HANDLER;
    adsParam->csPort = PORT_CS;
    adsParam->csPin  = PIN_CS;
    adsParam->drdyPort = PORT_DRDY;
    adsParam->drdyPin  = PIN_DRDY;
    
    // 初始化时先拉高 CS (取消选中)
    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_SET);
}



uint8_t ADS1120_init(ADS1120_params *adsParam){

    ADS1120_User_Setup(adsParam);

    adsParam->vRef = 2.048;
    adsParam->gain = 1;
    adsParam->refMeasurement = false;
    adsParam->convMode = ADS1120_SINGLE_SHOT;

    ADS1120_reset(adsParam);

    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_3, 0x02);//若需要修改DRDY引脚则需要修改
    tx_thread_sleep(1);

    // 检查芯片是否在线
    uint8_t ctrlVal = 0;
    ADS1120_bypassPGA(adsParam, true); 
    ctrlVal = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    
    bool isConnected = (ctrlVal & 0x01);
    
    ADS1120_bypassPGA(adsParam, false); // 恢复默认
    return isConnected;
}

void ADS1120_start(ADS1120_params *adsParam){
    ADS1120_command(adsParam, ADS1120_START);
}

void ADS1120_reset(ADS1120_params *adsParam){
    ADS1120_command(adsParam, ADS1120_RESET);
    // [修改] 使用 ThreadX 睡眠代替 HAL_Delay
    tx_thread_sleep(2); // 2ms 足够复位
}

void ADS1120_powerDown(ADS1120_params *adsParam){
    ADS1120_command(adsParam, ADS1120_PWRDOWN);
}

// =========================================================
// 寄存器操作函数 (Read-Modify-Write 安全逻辑)
// =========================================================

void ADS1120_setCompareChannels(ADS1120_params *adsParam, ads1120Mux mux){
    // 特殊情况处理：某些 MUX 模式强制 Gain=1
    if((mux == ADS1120_MUX_REFPX_REFNX_4) || (mux == ADS1120_MUX_AVDD_M_AVSS_4)){
        adsParam->gain = 1;   
        adsParam->refMeasurement = true;
    }
    else{
        // 否则我们需要读取当前的 Gain 设置，以免覆盖
        adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
        adsParam->regValue = adsParam->regValue & 0x0E; // 提取 Gain 位
        adsParam->regValue = adsParam->regValue >> 1;
        adsParam->gain = 1 << adsParam->regValue;       // 简单的位移计算增益倍数
        adsParam->refMeasurement = false;
    }
    
    // 读回 Reg0
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    // 清除 MUX 位 (高4位) 和 PGA bypass 位 (最低位, 这里逻辑有点怪，但保留原作者意图)
    adsParam->regValue &= ~0xF1; 
    // 写入新 MUX
    adsParam->regValue |= mux;
    // 恢复 PGA Bypass 状态
    adsParam->regValue |= !(adsParam->doNotBypassPgaIfPossible & 0x01);
    
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_0, adsParam->regValue);
    
    // 如果在单端模式下，增益不能超过4，这里做限制
    if((mux >= 0x80) && (mux <=0xD0)){
        if(adsParam->gain > 4){
            adsParam->gain = 4;           
        }
        ADS1120_forcedBypassPGA(adsParam); // 单端通常建议 Bypass PGA
    }
}

void ADS1120_setGain(ADS1120_params *adsParam, ads1120Gain enumGain){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    ads1120Mux mux = (ads1120Mux)(adsParam->regValue & 0xF0); // 记住当前的 MUX
    
    adsParam->regValue &= ~0x0E; // 清除 Gain 位
    adsParam->regValue |= enumGain;
    
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_0, adsParam->regValue);

    adsParam->gain = 1 << (enumGain >> 1); // 更新结构体里的 gain 变量
    
    // 同样的单端限制检查
    if((mux >= 0x80) && (mux <=0xD0)){
        if(adsParam->gain > 4){
            adsParam->gain = 4;   
        }
        ADS1120_forcedBypassPGA(adsParam);
    }
}

void ADS1120_bypassPGA(ADS1120_params *adsParam, bool bypass){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    adsParam->regValue &= ~0x01;
    adsParam->regValue |= bypass;
    adsParam->doNotBypassPgaIfPossible = !(bypass & 0x01);
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_0, adsParam->regValue);
}

bool ADS1120_isPGABypassed(ADS1120_params *adsParam){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    return adsParam->regValue & 0x01;
}

// ---------------- Register 1 ----------------
void ADS1120_setDataRate(ADS1120_params *adsParam, ads1120DataRate rate){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_1);
    adsParam->regValue &= ~0xE0;
    adsParam->regValue |= rate;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_1, adsParam->regValue);
}

void ADS1120_setOperatingMode(ADS1120_params *adsParam, ads1120OpMode mode){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_1);
    adsParam->regValue &= ~0x18;
    adsParam->regValue |= mode;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_1, adsParam->regValue);
}

void ADS1120_setConversionMode(ADS1120_params *adsParam, ads1120ConvMode mode){
    adsParam->convMode = mode;
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_1);
    adsParam->regValue &= ~0x04;
    adsParam->regValue |= mode;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_1, adsParam->regValue);
}

void ADS1120_enableTemperatureSensor(ADS1120_params *adsParam, bool enable){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_1);
    if(enable) adsParam->regValue |= 0x02;
    else       adsParam->regValue &= ~0x02;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_1, adsParam->regValue);
}

void ADS1120_enableBurnOutCurrentSources(ADS1120_params *adsParam, bool enable){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_1);
    if(enable) adsParam->regValue |= 0x01;
    else       adsParam->regValue &= ~0x01;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_1, adsParam->regValue);
}

// ---------------- Register 2 ----------------
void ADS1120_setVRefSource(ADS1120_params *adsParam, ads1120VRef vRefSource){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_2);
    adsParam->regValue &= ~0xC0;
    adsParam->regValue |= vRefSource;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_2, adsParam->regValue);
}

void ADS1120_setFIRFilter(ADS1120_params *adsParam, ads1120FIR fir){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_2);
    adsParam->regValue &= ~0x30;
    adsParam->regValue |= fir;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_2, adsParam->regValue);
}

void ADS1120_setLowSidePowerSwitch(ADS1120_params *adsParam, ads1120PSW psw){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_2);
    adsParam->regValue &= ~0x08;
    adsParam->regValue |= psw;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_2, adsParam->regValue);
}

void ADS1120_setIdacCurrent(ADS1120_params *adsParam, ads1120IdacCurrent current){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_2);
    adsParam->regValue &= ~0x07;
    adsParam->regValue |= current;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_2, adsParam->regValue);
    // [修改] 延时
    tx_thread_sleep(1);
}

// ---------------- Register 3 ----------------
void ADS1120_setIdac1Routing(ADS1120_params *adsParam, ads1120IdacRouting route){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_3);
    adsParam->regValue &= ~0xE0;
    adsParam->regValue |= (route<<5);
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_3, adsParam->regValue);
}

void ADS1120_setIdac2Routing(ADS1120_params *adsParam, ads1120IdacRouting route){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_3);
    adsParam->regValue &= ~0x1C;
    adsParam->regValue |= (route<<2);
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_3, adsParam->regValue);
}

void ADS1120_setDrdyMode(ADS1120_params *adsParam, ads1120DrdyMode mode){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_3);
    adsParam->regValue &= ~0x02;
    adsParam->regValue |= mode;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_3, adsParam->regValue);
}

// =========================================================
// 结果读取函数
// =========================================================

int16_t ADS1120_getData(ADS1120_params *adsParam){
    union Data{
        uint16_t rawResult;
        int16_t result;
    };
    union Data data;
    data.rawResult = ADS1120_readResult(adsParam);
    return data.result;
}

uint16_t ADS1120_readResult(ADS1120_params *adsParam){
    uint8_t buf[2];
    uint16_t rawResult = 0;

    // 1. 如果是 Single Shot 模式，需要手动触发 START
    if(adsParam->convMode == ADS1120_SINGLE_SHOT){
        ADS1120_start(adsParam);
    }
    
    //使用信号量等待 DRDY，设置超时 200 ticks
    // if (tx_semaphore_get(&ads_drdy_sem, 200) != TX_SUCCESS) {
    //     return 0; // 超时返回 0
    // }

    // 若不使用semaphore，可替换为while等待，直到DRDY变低（阻塞）
    // 注意：ADS1120 数据准备好时 DRDY 会拉低

    while(HAL_GPIO_ReadPin(adsParam->drdyPort, adsParam->drdyPin) == GPIO_PIN_SET);

    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_RESET);    // ADS1120 只有 16 位，读 2 个字节
    HAL_SPI_Receive(adsParam->adsSpi, buf, 2, 100); 
    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_SET);

    rawResult = buf[0];
    rawResult = (rawResult << 8) | buf[1];

    return rawResult;
}

// =========================================================
// 辅助计算函数
// =========================================================

void ADS1120_setIntVRef(ADS1120_params *adsParam){
    ADS1120_setVRefSource(adsParam, ADS1120_VREF_INT);
    adsParam->vRef = 2.048;
}

float ADS1120_getVoltage_mV(ADS1120_params *adsParam){
    int32_t rawData = ADS1120_getData(adsParam);
    float resultInMV = 0.0;
    if(adsParam->refMeasurement){
        resultInMV = (rawData / ADS1120_RANGE) * 2.048 * 1000.0 / (adsParam->gain * 1.0);
    }
    else{
        resultInMV = (rawData / ADS1120_RANGE) * adsParam->vRef * 1000.0 / (adsParam->gain * 1.0);
    }
    return resultInMV;
}

float ADS1120_getVoltage_muV(ADS1120_params *adsParam){
    return ADS1120_getVoltage_mV(adsParam) * 1000.0;
}

int16_t ADS1120_getRawData(ADS1120_params *adsParam){
    return ADS1120_getData(adsParam);
}

float ADS1120_getTemperature(ADS1120_params *adsParam){
    ADS1120_enableTemperatureSensor(adsParam, true);
    // 读取一次数据
    int16_t rawResult = (int16_t)ADS1120_readResult(adsParam); 
    ADS1120_enableTemperatureSensor(adsParam, false);

    uint16_t result = (rawResult >> 2);
    if(result >> 13){ // 处理 14位 补码
        result = ~(result-1) & 0x3777;
        return result * (-0.03125);
    }
    return result * 0.03125;
}

// =========================================================
// 底层私有函数
// =========================================================

void ADS1120_forcedBypassPGA(ADS1120_params *adsParam){
    adsParam->regValue = ADS1120_readRegister(adsParam, ADS1120_CONF_REG_0);
    adsParam->regValue |= 0x01;
    ADS1120_writeRegister(adsParam, ADS1120_CONF_REG_0, adsParam->regValue);
}

uint8_t ADS1120_readRegister(ADS1120_params *adsParam, uint8_t reg){
    adsParam->regValue = 0;
    
    uint8_t buf[1] = { (uint8_t)(ADS1120_RREG | (reg << 2)) };

    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(adsParam->adsSpi, buf, 1, 100);
    HAL_SPI_Receive(adsParam->adsSpi, &adsParam->regValue, 1, 100);
    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_SET);

    return adsParam->regValue;
}

void ADS1120_writeRegister(ADS1120_params *adsParam, uint8_t reg, uint8_t val){
    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_RESET);
    
    uint8_t buf[1] = { (uint8_t)(ADS1120_WREG | (reg << 2)) };
    
    HAL_SPI_Transmit(adsParam->adsSpi, buf, 1, 100000000);
    HAL_SPI_Transmit(adsParam->adsSpi, &val, 1, 100);

    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_SET);
}

void ADS1120_command(ADS1120_params *adsParam, uint8_t cmd){
    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(adsParam->adsSpi, &cmd, 1, 100);
    HAL_GPIO_WritePin(adsParam->csPort, adsParam->csPin, GPIO_PIN_SET);
}

