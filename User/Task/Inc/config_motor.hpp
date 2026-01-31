#pragma once
#ifndef TASK_MOTOR_HPP
#define TASK_MOTOR_HPP

#include <cstdint>

#include "stm32h7xx_hal_gpio.h"
#include "gpio.h"
#include "tim.h"
#include "bsp_pwm.hpp"

#include "tx_api.h"

#include "DJIMotorHandler.hpp"
#include "M3508.hpp"

#include "Stepper.hpp"

//todo:看需不需要改成更加通用的setangle，目前扳机和夹爪应该都是只需要起始和结束两个脉冲值
/**
 * @brief 舵机类，实现两点间移动
 */
class ServoMotors
{
    public:
    TIM_HandleTypeDef* htim; 
    uint32_t channel;

    float open_pulse; //打开时脉冲
    float lock_pulse; //锁定时的脉冲

    ServoMotors()
    {
        this->htim = nullptr;
        this->channel = 0;
        this->open_pulse = 750;
        this->lock_pulse = 1750;
    }

    void Init(TIM_HandleTypeDef* htim, uint32_t channel) //初始化舵机,配置挂载的定时器和通道
    {
        this->htim = htim;
        this->channel = channel;
        PWM_Start(this->htim, this->channel);
        PWM_SetDutyRatio(this->htim, lock_pulse, this->channel);
    }

    void Trigger_Open()
    {
        PWM_SetDutyRatio(this->htim, open_pulse, this->channel); //默认闭合
    }

    void Trigger_Lock()
    {
        PWM_SetDutyRatio(this->htim, lock_pulse, this->channel);
    }
};


class TaskMotors
{
    public:
    M3508 CoilSpringMotorL;         //卷簧电机L
    M3508 CoilSpringMotorR;         //卷簧电机R

    Stepper StringMotorL;           //弓弦调节步进电机L
    Stepper StringMotorR;           //弓弦调节步进电机R

    ServoMotors TriggerMotor;       //扳机电机

    void MotorRegister();           //DJI电机注册与初始化
    void SetModeAndPidParam();
    void AllMotorSetOutput();
    void Init();

    //todo:再检查有没有引脚冲突
    void StringMotorL_Init()
    {
        this->StringMotorL.Init(
            &htim4,            
            TIM_CHANNEL_3,    
            &htim5,          
            GPIOE,                  // 方向引脚 (L_DIR)
            GPIO_PIN_14,             // PE14 
            false                   //todo:确定方向
        );
    }

    void StringMotorR_Init()
    {
        this->StringMotorR.Init(
            &htim2,            
            TIM_CHANNEL_3,    
            &htim24,          
            GPIOA,                  // 方向引脚 (R_DIR)
            GPIO_PIN_0,              // PA0
            false                   //todo:确定方向
        );
    }

};

#endif // TASK_MOTOR_HPP