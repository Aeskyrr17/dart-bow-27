#ifndef GM3508_HPP
#define GM3508_HPP

// #include "stm32f4xx_hal.h"
// #include "stm32f4xx_ll_rcc.h"
// #include "stm32f4xx_ll_bus.h"
// #include "stm32f4xx_ll_system.h"
// #include "stm32f4xx_ll_exti.h"
// #include "stm32f4xx_ll_cortex.h"
// #include "stm32f4xx_ll_utils.h"
// #include "stm32f4xx_ll_pwr.h"
// #include "stm32f4xx_ll_dma.h"
// #include "stm32f4xx_ll_gpio.h"
#include "main.h"
#include "GMMotor.hpp"

/**
 * @class GM3508
 * @brief GM3508电机的控制类，继承自Motor类。
 *
 * 这个类实现了GM3508电机的控制方法，包括输出设置等。
 */
class GM3508 : public GMMotor
{
public:
    /**
     * @brief 构造函数，初始化GM6020电机控制类。
     */
    GM3508();

    /**
     * @brief 构造函数，初始化GM6020电机控制类。
     */
    ~GM3508();

    // for M3508
    const static float RawPos2Rad; /*!< 2Pi / 8191 / 3591 * 187 */
    const static float RawRpm2Rps; /*!< 2Pi / 60 * 187 / 3591 */

    const static float RawPos2RadDiv19;
    const static float RawRpm2RpsDiv19;
    const static float PIDiv19;

    const static float RawPos2RadDiv1576;
    const static float RawRpm2RpsDiv1576;
    const static float PIDiv1576;

    float RotorPosition;     // 电机转子位置
    float LastRotorPosition; // 上次记录的电机转子位置
    
    bool HaveGearbox;        // 是否有减速箱
    bool IsXRoll;            // 是否为xroll改3508


    /**
     * @brief 实现电机输出设置。
     * 根据当前的控制模式和PID反馈调整电机输出。
     * 必须在派生类中具体实现此方法以适应具体电机。
     */
    void setOutput() override;

    void UpdateSensorData(uint8_t *buffer_ptr) override;
};

#endif // GM3508_HPP
