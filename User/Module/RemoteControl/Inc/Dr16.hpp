#ifndef DR16_HPP
#define DR16_HPP

#include "RemoteControl.hpp"

#define DR16_DATA_SIZE 18u ///< Dr16数据帧长度

/**
 * @brief 遥控器数据结构。
 *
 * 此结构用于存储遥控器的完整状态信息，包括操纵杆数据、按钮状态和附加开关。
 */
struct dr16_data_t
{
    uint16_t ch_0 : 11;
    uint16_t ch_1 : 11;
    uint16_t ch_2 : 11;
    uint16_t ch_3 : 11;
    uint16_t s1 : 2;
    uint16_t s2 : 2;

    int16_t mouse_x : 16;
    int16_t mouse_y : 16;
    int16_t mouse_z : 16;
    uint8_t mouse_left : 8;
    uint8_t mouse_right : 8;
    uint16_t key : 16;
    uint16_t reserve : 16;
} __attribute__((packed));

union dr16_packet_t
{
    dr16_data_t _data;
    uint8_t ReceiveBuffer[DR16_DATA_SIZE];
};

class Dr16 : public RemoteControl
{
public:
    dr16_packet_t Dr16_Data;

    void Init() override;
    void Update() override;

    /*--------------------------------------------在线检测------------------------------------------------------*/
    void AliveCheck() override;
    bool IsAlive() override;

    /*--------------------------------------------处理遥控器数据------------------------------------------------------*/

    /**
     * @enum RC_SWITCH_TYPE
     * @brief 定义遥控器开关的类型。
     * @note 该枚举类型定义了遥控器上的两个开关。
     * @param LEFT_SWITCH 左侧开关状态
     * @param RIGHT_SWITCH 右侧开关状态
     */
    typedef enum
    {
        LEFT_SWITCH = 0,
        RIGHT_SWITCH,
        LEFT_SWITCH_CHANGE,
        RIGHT_SWITCH_CHANGE
    } RC_SWITCH_TYPE;

    /**
     * @enum RC_SWITCH_STATE
     * @brief 定义遥控器开关的状态类型。
     * @param RC_SW_UP 开关上
     * @param RC_SW_MID 开关中
     * @param RC_SW_DOWN 开关下
     * @param RC_SWITCH_M2D 从中到下
     * @param RC_SWITCH_M2U 从中到上
     * @param RC_SWITCH_D2M 从下到中
     * @param RC_SWITCH_U2M 从上到中
     */
    typedef enum
    {
        RC_SW_UP = 1,   ///< 开关上
        RC_SW_MID = 3,  ///< 开关中
        RC_SW_DOWN = 2, ///< 开关下

        RC_SWITCH_M2D = 4, ///< 从中到下
        RC_SWITCH_M2U = 5, ///< 从中到上
        RC_SWITCH_D2M = 6, ///< 从下到中
        RC_SWITCH_U2M = 7  ///< 从上到中
    } RC_SWITCH_STATE;

    /**
     * @brief 映射遥控器的摇杆值。
     * 將遥控器的摇杆值映射到-1到1之间。
     */
    float MapAvix(uint16_t ch);

    /**
     * @brief 更新遥控器摇杆和开关状态。
     */
    void UpdateRcStatus() override;

    /**
     * @brief 获取右侧摇杆X轴的值。
     * @return 右侧摇杆X轴的值。
     */
    float GetRightX() override;

    /**
     * @brief 获取右侧摇杆Y轴的值。
     * @return 右侧摇杆Y轴的值。
     */
    float GetRightY() override;

    /**
     * @brief 获取左侧摇杆X轴的值。
     * @return 左侧摇杆X轴的值。
     */
    float GetLeftX() override;

    /**
     * @brief 获取左侧摇杆Y轴的值。
     * @return 左侧摇杆Y轴的值。
     */
    float GetLeftY() override;

    /**
     * @brief 查询遥控器开关的状态。
     * @param sw 遥控器开关的类型。
     * @return 遥控器开关的状态。
     */
    RC_SWITCH_STATE QuerySwStatus(RC_SWITCH_TYPE sw);


    /*---------------------------------------------处理键鼠数据------------------------------------------------------*/
    /**
     * @brief 映射鼠标XY轴值。
     * 將鼠标XY轴值映射到-1到1之间。
     */
    float MapMouse(int16_t m);

    /**
     * @brief 更新电脑按键状态。
     */
    void UpdateKeyStatus() override;

    /**
     * @brief 获取鼠标X轴的值。
     * @return 鼠标X轴的值。
     */
    float GetMouseX() override;

    /**
     * @brief 获取鼠标Y轴的值。
     * @return 鼠标Y轴的值。
     */
    float GetMouseY() override;

    /**
     * @brief 获取鼠标滚轮的值。
     * @return 鼠标滚轮的值。
     */
    float GetMouseZ() override;

    /**
     * @brief 鼠标左键是否按下。
     * @return 按下为1，未按下为0。
     */
    uint8_t GetMouseL() override;

    /**
     * @brief 鼠标右键是否按下。
     * @return 按下为1，未按下为0。
     */
    uint8_t GetMouseR() override;

    /**
     * @brief 查询电脑按键的状态。
     * @param key 电脑按键的类型。
     * @return 电脑按键的状态。
     */
    PC_KEY_STATE QueryPcKeyStatus(PC_KEY_TYPE key) override;

    /*---------------------------------------------统一函数，Dr16不使用------------------------------------------------------*/
    float GetWheel() override;
    uint8_t GetMouseM() override;
    bool QueryButtonPressed();

    RC_SWITCH_STATE GetCurrentLeftSwStatus();

    RC_SWITCH_STATE GetPreviousLeftSwStatus();

private:
    RC_SWITCH_STATE Left_CurrentSw;  ///< 左侧开关当前状态。
    RC_SWITCH_STATE Left_PreviousSw; ///< 左侧开关上一次状态。
    RC_SWITCH_STATE Left_SwChange;   ///< 左侧开关状态变化。

    RC_SWITCH_STATE Right_CurrentSw;  ///< 右侧开关当前状态。
    RC_SWITCH_STATE Right_PreviousSw; ///< 右侧开关上一次状态。
    RC_SWITCH_STATE Right_SwChange;   ///< 右侧开关状态变化。

    uint16_t CurrentKeyState;  ///< 当前键盘按键状态。
    uint16_t PreviousKeyState; ///< 上一次键盘按键状态。

    bool Alive = false;
};

#endif // REMOTECONTROL_H
