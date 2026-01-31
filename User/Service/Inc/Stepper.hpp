// #include "config_motor.hpp"
#include "main.h"
#include "stm32_hal_legacy.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_tim.h"
#include <cstddef>
#include <cstdint>


/**
 * @brief Stepper步进电机类
 * 
 */
class Stepper
{
public:
    TIM_HandleTypeDef* pwmTim;          //发波的定时器
    uint32_t pwmChannel;                //通道
    TIM_HandleTypeDef* countTim;        //用于计数的计数器
    GPIO_TypeDef* dirPort;              //控制方向的gpio
    uint16_t dirPin;                    //方向引脚pin

    //控制参数
    float targetSpeed;                  //目标速度（Hz）//todo：考虑是否转化为更直观的速度值
    bool isReversed;                    //是否反转方向  //todo：确定正方向
    int32_t currentSteps;               //当前读到的步数

    Stepper()
    {
        this->pwmTim = nullptr;
        this->countTim = nullptr;
        this->targetSpeed = 0.0f;
        this->isReversed = false;
        this->currentSteps = 0;
    }                
    
    void Init(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel, TIM_HandleTypeDef* countTim, 
                GPIO_TypeDef* dirPort, uint16_t dirPin, bool reverse = false)
    {
        this->pwmTim = pwmTim;
        this->pwmChannel = pwmChannel;
        this->countTim = countTim;
        this->dirPort = dirPort;
        this->dirPin = dirPin;
        this->isReversed = reverse;

        if (this->countTim != nullptr)
        {
            HAL_TIM_Base_Start(this->countTim);
        }

        if (this->pwmTim != nullptr)
        {
            HAL_TIM_PWM_Start(this->pwmTim, this->pwmChannel);
        }
    }

    //todo:待测试
    void UpdateFeedback()
    {
        if (this->countTim != nullptr)
        {
            this->currentSteps = (uint32_t)__HAL_TIM_GetCounter(this->countTim);
        }
    }

    /**
     * @brief SendControlData 直接控制步进电机
     * 通过直接修改ARR的值，改变电机速度。速度可以直接使用负值
     */
    void SendControlData()
    {
        if (this->pwmTim == nullptr) return;

        bool dir = (this->targetSpeed >0); //速度大于零则为正方向

        if (this->isReversed)
            dir = !dir;

        if (dir)
            HAL_GPIO_WritePin(this->dirPort, this->dirPin, GPIO_PIN_RESET);//引脚低电平为正方向
        else
            HAL_GPIO_WritePin(this->dirPort, this->dirPin, GPIO_PIN_SET);
        
        float absSpeed = this->targetSpeed > 0 ? this->targetSpeed : -this->targetSpeed;//速度取绝对值

        if (absSpeed > 20000.0f)
            absSpeed = 20000.0f;//todo:计算一下最大速度对应频率

        if (absSpeed < 1.0f)  //防止在计算arr时除以0
        {
        __HAL_TIM_SET_AUTORELOAD(this->pwmTim, 0xFFFFFFFF);                 //给一个最大值arr，让电机停止
        __HAL_TIM_SET_COMPARE(this->pwmTim, this->pwmChannel, 0);           // 占空比设为0
        return; //直接返回
        }
            
        uint32_t arr = (uint32_t)(1000000.0f / absSpeed) - 1; //arr = (1MHz/freq) -1  (PSC为192-1)

        __HAL_TIM_SET_AUTORELOAD(this->pwmTim, arr);
        __HAL_TIM_SET_COMPARE(this->pwmTim, this->pwmChannel, arr/2); //保持为50%占空比
    }

};

