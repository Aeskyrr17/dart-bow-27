#ifndef REMOTE_CONTROL_HPP
#define REMOTE_CONTROL_HPP

#include "bsp_usart.hpp"
#include "bsp_dwt.hpp"

/**
 * @def RC_CH_VALUE_MIN
 * @brief 遥控器通道值的最小值
 */
#define RC_CH_VALUE_MIN 364

/**
 * @def RC_CH_VALUE_OFFSET
 * @brief 遥控器通道值的中间值
 */
#define RC_CH_VALUE_OFFSET 1024

/**
 * @def RC_CH_VALUE_MAX
 * @brief 遥控器通道值的最大值
 */
#define RC_CH_VALUE_MAX 1684

/**
 * @def RC_CH_OFFSET_MAX
 * @brief 遥控器通道最大偏移值
 */
#define RC_CH_OFFSET_MAX 660

/**
 * @def MOUSE_OFFSET_MAX
 * @brief 鼠标最大偏移值
 */
#define MOUSE_OFFSET_MAX 32767

class RemoteControl
{
public:
    virtual void Init() = 0; ///< 初始化遥控器
    virtual void Update() = 0; ///< 更新遥控器状态
    virtual void AliveCheck() = 0;
    virtual bool IsAlive() = 0;

    uint64_t AliveFlag = 0;
    uint64_t Pre_AliveFlag = 0;

    /**
     * @brief 映射遥控器的摇杆值。
     * 將遥控器的摇杆值映射到-1到1之间。
     */
    template <typename T>
    float MapAvix(T ch);

    /**
     * @brief 更新遥控器摇杆和开关状态。
     */
    virtual void UpdateRcStatus() = 0;

    /**
     * @brief 获取右侧摇杆X轴的值。
     * @return 右侧摇杆X轴的值。
     */
    virtual float GetRightX() = 0;

    /**
     * @brief 获取右侧摇杆Y轴的值。
     * @return 右侧摇杆Y轴的值。
     */
    virtual float GetRightY() = 0;

    /**
     * @brief 获取左侧摇杆X轴的值。
     * @return 左侧摇杆X轴的值。
     */
    virtual float GetLeftX() = 0;

    /**
     * @brief 获取左侧摇杆Y轴的值。
     * @return 左侧摇杆Y轴的值。
     */
    virtual float GetLeftY() = 0;

    /**
     * @brief 获取拨轮的值。
     * @return 拨轮的值。
     */
    virtual float GetWheel() = 0;

    template <typename T>
    bool QueryButtonPressed(T button);

    template <typename T1, typename T2>
    T1 QuerySwStatus(T2 sw);

    /*---------------------------------------------处理键鼠数据------------------------------------------------------*/
    /**
     * @enum PC_KEY_TYPE
     * @brief 定义电脑按键的类型。
     */
    typedef enum
    {
        PC_KEY_W = ((uint16_t)1 << 0),     ///< 按键W
        PC_KEY_S = ((uint16_t)1 << 1),     ///< 按键S
        PC_KEY_A = ((uint16_t)1 << 2),     ///< 按键A
        PC_KEY_D = ((uint16_t)1 << 3),     ///< 按键D
        PC_KEY_SHIFT = ((uint16_t)1 << 4), ///< 按键Shift
        PC_KEY_CTRL = ((uint16_t)1 << 5),  ///< 按键Ctrl
        PC_KEY_Q = ((uint16_t)1 << 6),     ///< 按键Q
        PC_KEY_E = ((uint16_t)1 << 7),     ///< 按键E
        PC_KEY_R = ((uint16_t)1 << 8),     ///< 按键R
        PC_KEY_F = ((uint16_t)1 << 9),     ///< 按键F
        PC_KEY_G = ((uint16_t)1 << 10),    ///< 按键G
        PC_KEY_Z = ((uint16_t)1 << 11),    ///< 按键Z
        PC_KEY_X = ((uint16_t)1 << 12),    ///< 按键X
        PC_KEY_C = ((uint16_t)1 << 13),    ///< 按键C
        PC_KEY_V = ((uint16_t)1 << 14),    ///< 按键V
        PC_KEY_B = ((uint16_t)1 << 15),    ///< 按键B
    } PC_KEY_TYPE;

    /**
     * @enum PC_KEY_STATE
     * @brief 定义电脑按键的状态类型。
     */
    typedef enum
    {
        PC_KEY_DOWN, ///< 按键保持按下
        PC_KEY_UP,   ///< 按键保持松开
        PC_KEY_FALL, ///< 按键下降沿
        PC_KEY_RISE  ///< 按键上升沿
    } PC_KEY_STATE;

    /**
     * @brief 映射鼠标XY轴值。
     * 將鼠标XY轴值映射到-1到1之间。
     */
    template <typename T>
    float MapMouse(T m);

    /**
     * @brief 更新电脑按键状态。
     */
    virtual void UpdateKeyStatus() = 0;

    /**
     * @brief 获取鼠标X轴的值。
     * @return 鼠标X轴的值。
     */
    virtual float GetMouseX() = 0;

    /**
     * @brief 获取鼠标Y轴的值。
     * @return 鼠标Y轴的值。
     */
    virtual float GetMouseY() = 0;

    /**
     * @brief 获取鼠标滚轮的值。
     * @return 鼠标滚轮的值。
     */
    virtual float GetMouseZ() = 0;

    /**
     * @brief 鼠标左键是否按下。
     * @return 按下为1，未按下为0。
     */
    virtual uint8_t GetMouseL() = 0;

    /**
     * @brief 鼠标右键是否按下。
     * @return 按下为1，未按下为0。
     */
    virtual uint8_t GetMouseR() = 0;

    /**
     * @brief 鼠标中键是否按下。
     * @return 按下为1，未按下为0。
     */
    virtual uint8_t GetMouseM() = 0;

    /**
     * @brief 查询电脑按键的状态。
     * @param key 电脑按键的类型。
     * @return 电脑按键的状态。
     */
    virtual PC_KEY_STATE QueryPcKeyStatus(PC_KEY_TYPE key) = 0;
};

#endif