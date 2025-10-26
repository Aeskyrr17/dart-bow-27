#include "TaskChassis.hpp"
#include "odometry.hpp"
#include "vmcsolver.hpp"
#include <sys/types.h>

#include "../../Module/Odometry/odometry.hpp"

using namespace Filter;

TX_THREAD ChassisThread;
uint8_t ChassisThreadStack[4096] = {0};

uint8_t xyAndRefAngleMsg[8] = {0};
uint8_t chassisStateMsg[8] = {0};

[[noreturn]] void ChassisThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);
    PID lleg_len_pd(4700.0f, 0.0f, 700.0f, 3000.0f, 0.0f, PID_POSITION);
    PID rleg_len_pd(4700.0f, 0.0f, 700.0f, 3000.0f, 0.0f, PID_POSITION);
    IIRFilter leg_len_filter(2,LOWPASS,1);
    SLOPE leg_len_updater(0.001f,0.001f,0.18f);

    PID roll_pd(0.7f, 0.0f, 0.01f, 3.0f, 0.0f, PID_POSITION);
    SLOPE roll_updater(0.0002f,0.0002f,0.0f);

    PID yaw_pd(18.0f, 0.0f, 4.5f, 5.0f, 0.0f, PID_POSITION);
    PID yaw_dot_pd(0.8f, 0.0f, 0.6f, 4.5f, 0.0f, PID_POSITION);
    SLOPE yaw_updater(0.01f, 0.01f, 0.0f);

    chassis_mode_t mode;
    uint16_t vlen_rx;
    uint16_t vy_rx;
    float vlen;
    float vy;
    float relativeangle;

    om_topic_t *chassisctrl_topic =om_config_topic(nullptr, "ca", "chassisctrl", sizeof(msg_chassis_ctrl_t));
    msg_chassis_ctrl_t chassis_ctrl{};
    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};

    for (;;)
    {
        uint8_t state_msg = chassisStateMsg[0];
        mode.chassis_mode = static_cast<chassis_mode_e>(state_msg & 0x03);
        mode.rotate_type = static_cast<rotate_ctrl_e>((state_msg >> 2) & 0x01);
        mode.jump_ctrl = static_cast<jump_ctrl_e>((state_msg >> 3) & 0x03);
        mode.fly_ctrl = static_cast<fly_ctrl_e>((state_msg >> 5) & 0x01);

        memcpy(&vlen_rx, xyAndRefAngleMsg, 2);               // 将接收到的数据拷贝到Vx
        memcpy(&vy_rx, xyAndRefAngleMsg + 2, 2);             // 将接收到的数据拷贝到Vy
        memcpy(&relativeangle, xyAndRefAngleMsg + 4, 4);     // 将接收到的数据拷贝到RelativeAngle

        //VxVy映射,[0,60000] -> [-2,2]
        vlen = ((float)vlen_rx) / 15000.0f - 2.0f;
        vy = ((float)vy_rx) / 15000.0f - 2.0f;

        if (isnan(vlen_rx) || isnan(vy_rx) || isnan(relativeangle) || (mode.chassis_mode > 3) || (mode.rotate_type > 1) || (mode.jump_ctrl > 2)) // 如果出现nan错误，将速度设定值设为0
        {
            vlen = 0;
            vy = 0;
            relativeangle = 0;
            mode.chassis_mode = NONE;
            mode.rotate_type = NORMAL_ROTATE;
            mode.jump_ctrl = DO_NOT_JUMP;
        }

        if (fabsf(vlen) < 0.0005f)
            vlen = 0;
        if (fabsf(vy) < 0.0005f)
            vy = 0;
        if (fabsf(relativeangle) < 0.0001f)
            relativeangle = 0;

        om_publish(chassisctrl_topic, &chassis_ctrl, sizeof(msg_chassis_ctrl_t), true, false);
        tx_thread_sleep(1);
    }
}