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
#include "M2006.hpp"

#include "Stepper.hpp"
#include "X_V2.hpp"


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
        this->open_pulse = 1100  / 20000.0f;
        this->lock_pulse = 1630 / 20000.0f;//50Hz //1750
    }

    void Init(TIM_HandleTypeDef* htim, uint32_t channel) //初始化舵机,配置挂载的定时器和通道
    {
        this->htim = htim;
        this->channel = channel;
        PWM_Start(this->htim, this->channel);
        PWM_SetDutyRatio(this->htim, lock_pulse, this->channel);
    }

    void Trigger_1()
    {
         PWM_SetDutyRatio(this->htim, 1500 / 20000.0f, this->channel); //默认闭合
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

/**
 * @brief 张大头步进电机类
 * @details 具体的实现功能来源于张大头步进电机例程，需修改static变量
 * 
 */
class ZDTStepper
{
    public:
    ZDTStepper(){}

    uint16_t torque;
    uint16_t speed;
    uint32_t position;
    float current;

    uint8_t _id;
    FDCAN_HandleTypeDef* _hcan;
    uint8_t dir;

    void Init(FDCAN_HandleTypeDef* hcan, uint8_t id)
    {
        this->_hcan = hcan;
        this->_id = id;
    }

/**
 * @brief 在canrxcallback中调用，筛选不同功能码id并给参数赋值
 * 
 * @param hfdcan 
 * @param rx_data uint8_t[8]的rxdata
 * @param rx_id rx_header.Identifier
 */
    void updateFeedback(FDCAN_HandleTypeDef *hfdcan, uint8_t *rx_data, uint32_t rx_id)
    {
        uint8_t target_id = (uint8_t)(rx_id >> 8) & 0xFF;

        if (target_id != this->_id) return;

        switch (rx_data[0])
        {
            case 0x27:// 读取相电流
                this->torque = (uint16_t)(rx_data[2] << 8 | rx_data[3]);
                break;
            case 0x35:// 读取电机实时转速
                this->speed = (uint16_t)(rx_data[2] << 8 | rx_data[3]);
                break;
            case 0x36:// 读取电机实时位置
                this->position = (uint32_t)((rx_data[2] << 24) | (rx_data[3] << 16) |
                                             (rx_data[4] << 8) | rx_data[5]);
                break;
        };

    }

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型，填入对应的宏
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
    void X_V2_Read_Sys_Params(uint8_t addr, SysParams_t s)
    {
        uint8_t i = 0;
        uint8_t cmd[16] = {0};
        
        // 装载命令
        cmd[i] = addr; ++i;                   // 地址

        switch(s)                             // 功能码
        {
            case S_VBUS : cmd[i] = 0x24; ++i; break;	// 读取总线电压
            case S_CBUS : cmd[i] = 0x26; ++i; break;	// 读取总线电流
            case S_CPHA : cmd[i] = 0x27; ++i; break;	// 读取相电流
            case S_ENCO : cmd[i] = 0x29; ++i; break;	// 读取编码器原始值
            case S_CLKC : cmd[i] = 0x30; ++i; break;	// 读取实时脉冲数
            case S_ENCL : cmd[i] = 0x31; ++i; break;	// 读取经过线性化校准后的编码器值
            case S_CLKI : cmd[i] = 0x32; ++i; break;	// 读取输入脉冲数
            case S_TPOS : cmd[i] = 0x33; ++i; break;	// 读取电机目标位置
            case S_SPOS : cmd[i] = 0x34; ++i; break;	// 读取电机实时设定的目标位置
            case S_VEL  : cmd[i] = 0x35; ++i; break;	// 读取电机实时转速
            case S_CPOS : cmd[i] = 0x36; ++i; break;	// 读取电机实时位置
            case S_PERR : cmd[i] = 0x37; ++i; break;	// 读取电机位置误差
            case S_VBAT : cmd[i] = 0x38; ++i; break;	// 读取多圈编码器电池电压（Y42）
            case S_TEMP : cmd[i] = 0x39; ++i; break;	// 读取电机实时温度（X42S/Y42）
            case S_FLAG : cmd[i] = 0x3A; ++i; break;	// 读取电机状态标志位
            case S_OFLAG: cmd[i] = 0x3B; ++i; break;	// 读取回零状态标志位
            case S_OAF  : cmd[i] = 0x3C; ++i; break;	// 读取电机状态标志位 + 回零状态标志位（X42S/Y42）
            case S_PIN  : cmd[i] = 0x3D; ++i; break;	// 读取引脚状态（X42S/Y42）
            case S_SYS  : cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;	// 读取系统状态参数
            default: break;
        }

        cmd[i] = 0x6B; ++i;                   // 校验字节
        
        // 发送命令
        can_SendCmd(this->_hcan, cmd, i);
    }

/**
* @brief    力矩模式
* @param    addr  	：电机地址
* @param    sign  	：符号（方向）		，0为正，1为负
* @param    t_ramp	：电流斜率(Ma/s)	，范围0 - 65535Ma/s
* @param    torque	：力矩电流(Ma)		，范围0 - 6000Ma
* @param    snF   	：多机同步标志		，false为不启用，true为启用
* @retval   地址 + 功能码 + 命令状态 + 校验字节
*/
    void X_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, bool snF)
    {
        uint8_t cmd[16] = {0};
    
        // 装载命令
        cmd[0] =  addr;                       // 地址
        cmd[1] =  0xF5;                       // 功能码
        cmd[2] =  sign;                       // 符号（方向）
        cmd[3] =  (uint8_t)(t_ramp >> 8);     // 电流斜率(Ma/s)
        cmd[4] =  (uint8_t)(t_ramp >> 0);
        cmd[5] =  (uint8_t)(torque >> 8);     // 力矩电流(Ma)
        cmd[6] =  (uint8_t)(torque >> 0);
        cmd[7] =  snF;                        // 多机同步标志
        cmd[8] =  0x6B;                       // 校验字节
        
        // 发送命令
        can_SendCmd(this->_hcan, cmd, 9);
    }


/**
* @brief    力矩模式限速控制（X42S/Y42）
* @param    addr  	：电机地址
* @param    sign  	：符号（方向）		，0为正，1为负
* @param    t_ramp	：电流斜率(Ma/s)	，范围0 - 65535Ma/s
* @param    torque	：力矩电流(Ma)		，范围0 - 6000Ma
* @param    snF   	：多机同步标志		，false为不启用，true为启用
* @param    maxVel	：最大速度(RPM)	，范围0.0 - 3000.0RPM
* @retval   地址 + 功能码 + 命令状态 + 校验字节
*/
    void X_V2_Torque_LV_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, bool snF, float maxVel)
    {
        uint8_t cmd[16] = {0}; uint16_t v = 0;

        // 将速度放大10倍发送过去
        v = (uint16_t)ABS(maxVel * 10.0f);
        
        // 装载命令
        cmd[0]  =  this->_id;                     // 地址
        cmd[1]  =  0xC5;                      // 功能码
        cmd[2]  =  sign;                      // 符号（方向）
        cmd[3]  =  (uint8_t)(t_ramp >> 8);    // 电流斜率(Ma/s)
        cmd[4]  =  (uint8_t)(t_ramp >> 0);
        cmd[5]  =  (uint8_t)(torque >> 8);    // 力矩电流(Ma)
        cmd[6]  =  (uint8_t)(torque >> 0);
        cmd[7]  =  snF;                       // 多机同步标志
        cmd[8]  =  (uint8_t)(v >> 8);    	  // 最大速度(RPM)
        cmd[9]  =  (uint8_t)(v >> 0);    
        cmd[10] =  0x6B;                      // 校验字节
        
        // 发送命令
        can_SendCmd(this->_hcan, cmd, 11);
    }


/**
  * @brief    速度模式限电流控制（X42S/Y42）
  * @param    addr  ：电机地址
  * @param    dir   ：方向						，0为CW，1为CCW
  * @param    acc   ：加速度(RPM/s)	，范围0 - 65535RPM/s
  * @param    vel		：速度(RPM)			，范围0.0 - 3000.0RPM
  * @param    snF   ：多机同步标志		，false为不启用，true为启用
	* @param    maxCur：最大电流(mA)		，范围0 - 6000mA
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
    void X_V2_Vel_LC_Control(uint8_t addr, uint8_t dir, uint16_t acc, float vel, bool snF, uint16_t maxCur)
    {
        uint8_t cmd[16] = {0}; uint16_t v = 0;

        // 将速度放大10倍发送过去
        v = (uint16_t)ABS(vel * 10.0f);

        // 装载命令
        cmd[0]  =  this->_id;                         // 地址
        cmd[1]  =  0xC6;                      // 功能码
        cmd[2]  =  dir;                       // 符号（方向）
        cmd[3]  =  (uint8_t)(acc >> 8);     	// 加速度(RPM/s)
        cmd[4]  =  (uint8_t)(acc >> 0);
        cmd[5]  =  (uint8_t)(v >> 8);        	// 速度(RPM)
        cmd[6]  =  (uint8_t)(v >> 0);
        cmd[7]  =  snF;                       // 多机同步运动标志
        cmd[8]  =  (uint8_t)(maxCur >> 8);    // 最大电流(mA)
        cmd[9]  =  (uint8_t)(maxCur >> 0);
        cmd[10] =  0x6B;                      // 校验字节
        
        // 发送命令
        can_SendCmd(this->_hcan, cmd, 11);
    }


/**
  * @brief    定时返回信息命令（X42S/Y42）
  * @param    addr  	：电机地址
  * @param    s     	：系统参数类型
  * @param    time_ms ：定时时间
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
    void X_V2_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms)
    {
        uint8_t i = 0; 
        uint8_t cmd[16] = {0};
    
        // 装载命令
        cmd[i] = this->_id; ++i;                   // 地址

        cmd[i] = 0x11; ++i;                   // 功能码

        cmd[i] = 0x18; ++i;                   // 辅助码

        switch(s)                             // 信息功能码
        {
            case S_VBUS : cmd[i] = 0x24; ++i; break;	// 读取总线电压
            case S_CBUS : cmd[i] = 0x26; ++i; break;	// 读取总线电流
            case S_CPHA : cmd[i] = 0x27; ++i; break;	// 读取相电流
            case S_ENCO : cmd[i] = 0x29; ++i; break;	// 读取编码器原始值
            case S_CLKC : cmd[i] = 0x30; ++i; break;	// 读取实时脉冲数
            case S_ENCL : cmd[i] = 0x31; ++i; break;	// 读取经过线性化校准后的编码器值
            case S_CLKI : cmd[i] = 0x32; ++i; break;	// 读取输入脉冲数
            case S_TPOS : cmd[i] = 0x33; ++i; break;	// 读取电机目标位置
            case S_SPOS : cmd[i] = 0x34; ++i; break;	// 读取电机实时设定的目标位置
            case S_VEL  : cmd[i] = 0x35; ++i; break;	// 读取电机实时转速
            case S_CPOS : cmd[i] = 0x36; ++i; break;	// 读取电机实时位置
            case S_PERR : cmd[i] = 0x37; ++i; break;	// 读取电机位置误差
            case S_VBAT : cmd[i] = 0x38; ++i; break;	// 读取多圈编码器电池电压（Y42）
            case S_TEMP : cmd[i] = 0x39; ++i; break;	// 读取电机实时温度（X42S/Y42）
            case S_FLAG : cmd[i] = 0x3A; ++i; break;	// 读取电机状态标志位
            case S_OFLAG: cmd[i] = 0x3B; ++i; break;	// 读取回零状态标志位
            case S_OAF  : cmd[i] = 0x3C; ++i; break;	// 读取电机状态标志位 + 回零状态标志位（X42S/Y42）
            case S_PIN  : cmd[i] = 0x3D; ++i; break;	// 读取引脚IO状态（X42S/Y42）
            default: break;
        }
        
        cmd[i] = (uint8_t)(time_ms >> 8);  ++i;	// 定时时间
        cmd[i] = (uint8_t)(time_ms >> 0);  ++i;

        cmd[i] = 0x6B; ++i;                   	// 校验字节
        
        // 发送命令
        can_SendCmd(this->_hcan, cmd, i);
        }


};



class TaskMotors
{
    public:
    M3508 CoilSpringMotorL;         //卷簧电机L
    M3508 CoilSpringMotorR;         //卷簧电机R

    // Stepper StringMotorL;           //弓弦调节步进电机L
    // Stepper StringMotorR;           //弓弦调节步进电机R
    Stepper YawMotor;

    ZDTStepper StringMotorL;
    ZDTStepper StringMotorR;

    ServoMotors TriggerMotor;       //扳机电机

    M2006 GantryMotor;              //龙门架装填电机

    struct GantryMotorPosition
    {
        float open = 0.52f;
        float reset = 0; //待测试
        float lock = -0.52; 
    };
    GantryMotorPosition gantry_pos;

    void MotorInit();           //DJI电机注册与初始化
    void SetModeAndPidParam();
    void AllMotorSetOutput();
    void Init();


    void YawMotor_Init()
    {
        this->YawMotor.Init(
            &htim2,            
            TIM_CHANNEL_3,        
            GPIOA,                  // 方向引脚 (R_DIR)
            GPIO_PIN_0,              // PA0
            true                   //todo:确定方向
        );
    }

    static TaskMotors* Instance()
    {
        static TaskMotors instance;
        return &instance;
    }

};


#endif // TASK_MOTOR_HPP