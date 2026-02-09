// #include "config_motor.hpp"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal_gpio.h"
#include "math.h"
#include "slope.hpp"
#include <cstdint>

// 宏定义
#define ACCEL_STEP          4.0f          //<每次中断增加的频率(Hz)，值越大加速越快
#define TIM_CLOCK_FREQ      1000000.0f    //<定时器计数频率,APB1主频192MHz，PSC为192-1
#define SYSTEM_MAX_FREQ     10000.0f      //最大频率
#define MIN_START_FREQ      500.0f         //<最小启动频率

class Stepper
{
public:
    float targetSpeedHz;      
    SLOPE indexRamp;//斜坡

    TIM_HandleTypeDef* pwmTim;      //<挂载pwm波的定时器
    uint32_t pwmChannel;            //<定时器通道
    GPIO_TypeDef* dirPort;          //<控制方向的gpio
    uint16_t dirPin;                //<方向引脚pin

    // 内部状态
    bool isReversed;         
    uint32_t lastWrittenARR;

    Stepper()
    {
        this->pwmTim = nullptr;
        this->targetSpeedHz = 0.0f;
        this->isReversed = false;
    
        this->indexRamp.SetPath(ACCEL_STEP); 
    }                
/**
 * @brief 初始化函数
 * @param pwmTim pwm挂载的定时器
 * @param pwmChannel pwmchannel
 * @param dirPort 方向gpio
 * @param dirPin 方向gpio
 * @param reverse 方向是否反转 
 */
    void Init(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel, 
                GPIO_TypeDef* dirPort, uint16_t dirPin, bool reverse = false)
    {
        this->pwmTim = pwmTim;
        this->pwmChannel = pwmChannel;
        this->dirPort = dirPort;
        this->dirPin = dirPin;
        this->isReversed = reverse;

        indexRamp.SetDefault(0.0f);     // 归零

        if (this->pwmTim != nullptr) 
            HAL_TIM_PWM_Stop_IT(pwmTim, pwmChannel);
    }

/**
 * @brief Set the Target Speed object
 * @param speedHz 目标速度（Hz）
 */
    void SetTargetSpeed(float speedHz)
    {
        if (this->pwmTim == nullptr) 
            return;

        if (this->targetSpeedHz == speedHz) 
            return;

        float absSpeed = (speedHz > 0) ? speedHz : -speedHz;
        if (absSpeed > SYSTEM_MAX_FREQ) absSpeed = SYSTEM_MAX_FREQ;
        if (speedHz < 0) absSpeed = -absSpeed;

        this->targetSpeedHz = absSpeed;

        //在当前是停止状态下执行
        if ((int32_t)indexRamp.GetVal() == 0 && this->targetSpeedHz != 0)
        {
            bool dir = (this->targetSpeedHz > 0);
            if (this->isReversed) dir = !dir; 
            HAL_GPIO_WritePin(dirPort, dirPin, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
            
            __HAL_TIM_SET_COUNTER(this->pwmTim, 0);//清零计数器，确保dir信号建立（硬件存在延时）

            float startVal = (this->targetSpeedHz > 0) ? MIN_START_FREQ : -MIN_START_FREQ;
            indexRamp.SetDefault(startVal);

            UpdateARR(MIN_START_FREQ); 
            HAL_TIM_PWM_Start_IT(pwmTim, pwmChannel);
        }
    }

/**
 * @brief HAL_TIM_PWM_PulseFinishedCallback中调用
 */
    void HandleInterrupt()
    {
        float currentFreq = indexRamp.UpdateVal(this->targetSpeedHz);
        
        // 判断是否掉进了死区 (绝对值小于最小频率)
        if ((currentFreq > 0 && currentFreq < MIN_START_FREQ) || 
            (currentFreq < 0 && currentFreq > -MIN_START_FREQ) ||
            currentFreq == 0)
        {
            // 情况1：目标真的是 0，说明要停车 -> 关中断，停车
            if (this->targetSpeedHz == 0.0f)
            {
                HAL_TIM_PWM_Stop_IT(pwmTim, pwmChannel);
                indexRamp.SetDefault(0.0f);
                return;
            }
            // 情况2：目标不是 0，说明正在反向过零 -> 强制跳过死区，继续跑
            else
            {
                // 如果目标是正，就跳到 +50；如果目标是负，就跳到 -50
                float jumpVal = (this->targetSpeedHz > 0) ? MIN_START_FREQ : -MIN_START_FREQ;
                
                // 更新斜坡和当前频率
                indexRamp.SetDefault(jumpVal);
                currentFreq = jumpVal;
                
                // 不 return，继续往下执行，更新硬件产生下一个波
            }
        }

        float absFreq = (currentFreq > 0) ? currentFreq : -currentFreq;
        uint32_t nextARR = GetARR(absFreq);
        
        bool newDir = (currentFreq > 0);
        if (this->isReversed) newDir = !newDir;

        // 读取当前 GPIO 电平来判断是否需要换向
        GPIO_PinState currentPin = HAL_GPIO_ReadPin(dirPort, dirPin);
        GPIO_PinState targetPin = newDir ? GPIO_PIN_RESET : GPIO_PIN_SET;

        if (currentPin != targetPin)
        {
            HAL_GPIO_WritePin(dirPort, dirPin, targetPin);
            __HAL_TIM_SET_COUNTER(this->pwmTim, 0);
        }
      
        //只有arr改变时才写寄存器
        if (nextARR != this->lastWrittenARR)
        {
            __HAL_TIM_SET_AUTORELOAD(this->pwmTim, nextARR);
            __HAL_TIM_SET_COMPARE(this->pwmTim, this->pwmChannel, nextARR/2);
            this->lastWrittenARR = nextARR;
        }
    }

/**
 * @brief 修改更新ARR寄存器
 * @param freq 
 */
    inline void UpdateARR(float freq)
    {
        if(freq < 1.0f) freq = 1.0f;
        uint32_t arr = (uint32_t)(TIM_CLOCK_FREQ / freq) - 1;
        if(arr > 65535) arr = 65535;
        
        __HAL_TIM_SET_AUTORELOAD(this->pwmTim, arr);
        __HAL_TIM_SET_COMPARE(this->pwmTim, this->pwmChannel,arr/2);
    }

/**
 * @brief 计算ARR值
 * @param freq 
 * @return uint32_t arr值
 */
    inline uint32_t GetARR(float freq)
    {
        if(freq < 1.0f) freq = 1.0f;
        uint32_t arr = (uint32_t)(TIM_CLOCK_FREQ / freq) - 1;
        if(arr > 65535) arr = 65535;
        return arr;
    }
};