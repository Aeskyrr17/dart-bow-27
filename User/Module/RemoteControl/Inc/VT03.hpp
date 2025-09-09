#ifndef VT03_HPP
#define VT03_HPP

#include "crc.hpp"
#include "RemoteControl.hpp"

#define VT03_DATA_SIZE 21u

struct vt03_data_t
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0 : 11;
    uint64_t ch_1 : 11;
    uint64_t ch_2 : 11;
    uint64_t ch_3 : 11;
    uint64_t mode_sw : 2;
    uint64_t pause : 1;
    uint64_t fn_1 : 1;
    uint64_t fn_2 : 1;
    uint64_t wheel : 11;
    uint64_t trigger : 1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left : 2;
    uint8_t mouse_right : 2;
    uint8_t mouse_middle : 2;
    uint16_t key;
    uint16_t crc16;
} __attribute__((packed));

#define VT03_SOF_1 0xA9
#define VT03_SOF_2 0x53

union vt03_packet_t
{
    vt03_data_t _data;
    uint8_t ReceiveBuffer[VT03_DATA_SIZE];
};

class VT03 : public RemoteControl
{
public:
    vt03_packet_t VT03_Data;

    void Init() override;
    void Update() override;

    /*--------------------------------------------在线检测------------------------------------------------------*/
    bool IsAlive() override;
    void AliveCheck() override;

    /*--------------------------------------------处理遥控器数据------------------------------------------------------*/
    /**
     * @enum MODE_SW_STATE
     * @brief 定义遥控器挡位的状态类型。
     * @param MODE_SW_C 挡位C
     * @param MODE_SW_N 挡位N
     * @param MODE_SW_S 挡位S
     * @param MODE_SW_C2N 挡位C到N
     * @param MODE_SW_N2C 挡位N到C
     * @param MODE_SW_S2N 挡位S到N
     * @param MODE_SW_N2S 挡位N到S
     */
    typedef enum
    {
        MODE_SW_C = 0,
        MODE_SW_N = 1,
        MODE_SW_S = 2,

        MODE_SW_C2N = 3,
        MODE_SW_N2C = 4,
        MODE_SW_S2N = 5,
        MODE_SW_N2S = 6
    } MODE_SW_STATE;


    typedef enum
    {
        BUTTON_PAUSE,
        BUTTON_TRIGGER,
        BUTTON_LEFT,
        BUTTON_RIGHT
    } BUTTON_TYPE;

    /**
     * @brief 映射遥控器的摇杆值。
     * 將遥控器的摇杆值映射到-1到1之间。
     */
    float MapAvix(uint64_t ch);

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
     * @brief 获取拨轮的值。
     * @return 拨轮的值。
     */
    float GetWheel() override;

    bool QueryButtonPressed(BUTTON_TYPE button);

    VT03::MODE_SW_STATE QuerySwStatus();

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
     * @brief 鼠标中键是否按下。
     * @return 按下为1，未按下为0。
     */
    uint8_t GetMouseM() override;

    /**
     * @brief 查询电脑按键的状态。
     * @param key 电脑按键的类型。
     * @return 电脑按键的状态。
     */
    PC_KEY_STATE QueryPcKeyStatus(PC_KEY_TYPE key) override;

    /*--------------------------------------------统一函数，VT03不使用------------------------------------------------------*/

    static VT03 *Instance()
    {
        static VT03 instance;
        return &instance;
    }
private:
    uint16_t CurrentKeyState;  ///< 当前键盘按键状态。
    uint16_t PreviousKeyState; ///< 上一次键盘按键状态。
    uint8_t PrevVT03Data[VT03_DATA_SIZE]; ///< 上一次接收的数据。

    MODE_SW_STATE CurrentSw;
    MODE_SW_STATE PreviousSw;
    MODE_SW_STATE SwChange;

    bool Alive = false;
};

#endif