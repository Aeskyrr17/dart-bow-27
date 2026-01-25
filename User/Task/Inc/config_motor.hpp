#pragma once
#include <cstdint>
#ifndef TASK_MOTOR_HPP
#define TASK_MOTOR_HPP

#include "cstdint"
#include "tx_api.h"

#include "DJIMotorHandler.hpp"
#include "M3508.hpp"

#include "bsp_pwm.hpp"

#define JH_CMD_SET_MODE    0x23
#define JH_CMD_SPEED_MODE  0x33
#define JH_CMD_SET_SPEED   0x33
#define JH_ERROR_FLAG      0xFF

/**
 * @brief StepperMotors类，步进电机
 */
class StepperMotors
{
public:
    FDCAN_HandleTypeDef* hcan; //挂载的CAN线
    uint16_t can_id;    //can ID
    uint8_t tx_data[8] = {0};   //发送数据缓存

    //tx_data
    float target_speed; //目标速度，使用速度环控制
    float limit_current; //限流值
    float target_accel; //加速度

    //todo:rx_data

    StepperMotors()
    {
        this->hcan = nullptr;
        this->can_id = 0;
        memset(tx_data, 0, 8);

        this->target_speed = 0.0f;
        this->limit_current = 0.0f;//todo:修改默认限流值
        this->target_accel = 0.0f;
    }

    /**
     * @brief 初始化步进电机，绑定CAN线和ID
     * @param hcan_ptr 挂载的can线
     * @param id id
     */
    void Init(FDCAN_HandleTypeDef* hcan_ptr, uint16_t id)
    {
        this->hcan = hcan_ptr;
        this->can_id = id;
    }

    /**
     * @brief 设置电机模式--本工程中使用速度模式
     */
    void Set_Mode()//设置电机模式--使用速度模式 //todo:可能不调用，直接提前写入电机驱动板，不知道是否安全
    {
        memset(tx_data,0,8);
        this->tx_data[0] = JH_CMD_SET_MODE;
        this->tx_data[1] = JH_CMD_SPEED_MODE;
    }

    /**
    * @brief 打包速度指令到this->tx_data中
    */
    void Pack_Target_Speed()//打包速度指令,封装在SendControlData中
    {
        memset(this->tx_data, 0, 8);
        this->tx_data[0] = JH_CMD_SET_SPEED;
        this->tx_data[1] = 0x00;

        this->tx_data[2] = (int16_t)(this->target_accel) >> 8  & 0xFF;
        this->tx_data[3] = (int16_t)(this->target_accel)       & 0xFF;

        int16_t cur_int = (int16_t)(this->limit_current * 100.0f);
        this->tx_data[4] = (cur_int >> 8)                & 0xFF;
        this->tx_data[5] = (cur_int)                     & 0xFF;
        int16_t spd_int = (int16_t)this->target_speed;
        this->tx_data[6] = (spd_int >> 8) & 0xFF;
        this->tx_data[7] = (spd_int)      & 0xFF;
    }
    
    /**
     * @brief 更新反馈数据
     * @param data 接收到的原始数据
     */
    void UpdateFeedback(uint8_t* data)    //todo:解析反馈数据,bsp_can.cpp中HAL_FDCAN_RxFifo0Callback调用,data为接收到的rawdata
    {

    } 

    /**
     * @brief 发送控制数据
     * @param hcan CAN句柄
     */
    void SendControlData()
    {
        Pack_Target_Speed();
        if (this->hcan != nullptr)
            CAN_Transmit(this->hcan, this->can_id, this->tx_data, 8);
    }
};


//todo:记得重新看PWM，不会写
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
    M3508 CoilSpringMotorL; //卷簧电机L
    M3508 CoilSpringMotorR; //卷簧电机R

    StepperMotors YawMotor; //yaw轴步进电机
    StepperMotors StringMotorL; //弓弦调节步进电机L
    StepperMotors StringMotorR; //弓弦调节步进电机R

    ServoMotors TriggerMotor; //扳机电机

    void MotorRegister();   //DJI电机注册与初始化
    void SetModeAndPidParam();
    void AllMotorSetOutput();
    void Init();

    // void Can_Send_StepperMotors(uint32_t id, float spd); //发送步进电机数据
    // void Decode_StepperMotors(uint32_t id, uint8_t* data); //解析步进电机数据
};

#endif // TASK_MOTOR_HPP