#include "bsp_dwt.hpp"
#include "config_remoter.hpp"
#include "math.hpp"
#include "pid.hpp"
#include "lqr.hpp"
#include "filter.hpp"
#include "slope.hpp"
#include "magicmsgs.hpp"
#include "config_chassis.hpp"
#include "tx_api.h"
#include "vmc.hpp"
#include "om.h"
#include <cstring>
#include "usart.h"

TX_THREAD FunctionThread;
uint8_t FunctionThreadStack[4096] = {0};
TX_SEMAPHORE TOFGot;

uint8_t xyAndRefAngleMsg[8] = {0};
uint8_t chassisStateMsg[8] = {0};
__attribute__((section(".RAM_D1"))) uint8_t tof_rx[TOF_DATA_SIZE];

extern TX_SEMAPHORE IMUThreadSem;

tof_data_t debug_tof;

inline tof_data_t& ToF_Data()
{
    return *reinterpret_cast<tof_data_t*>(tof_rx);
}

[[noreturn]] void FunctionThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    /* Control Signal Initialization */
    chassis_mode_t mode = {NONE, NORMAL_ROTATE, DO_NOT_JUMP, NOT_FLY_MODE};
    uint16_t vlen_rx;
    uint16_t v_rx;
    float relativeangle;
    float v;
    float vlen;

    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};

    for (;;)
    {
        /* Thread start time */
        ULONG thread_start_time = tx_time_get();

        om_suber_export(remoter_suber, &remoter, false);

        bool tof_valid = false;
        debug_tof = ToF_Data();

        if (ToF_Data().header[0]==0x59 && ToF_Data().header[1]==0x59)
        {
            uint8_t checksum = 0;
            for (int i = 0; i < 7; i++)
            {
                checksum += ((uint8_t*)&ToF_Data())[i];
            }
            if (checksum == ToF_Data().check_sum)
            {
                tof_valid = true;
            }
        }

        uint8_t state_msg = chassisStateMsg[0];
        mode.chassis_mode = static_cast<chassis_mode_e>(state_msg & 0x03);
        mode.rotate_type = static_cast<rotate_ctrl_e>((state_msg >> 2) & 0x01);
        mode.jump_ctrl = static_cast<jump_ctrl_e>((state_msg >> 3) & 0x03);
        mode.fly_ctrl = static_cast<fly_ctrl_e>((state_msg >> 5) & 0x01);

        memcpy(&vlen_rx, xyAndRefAngleMsg, 2);                  // 将接收到的数据拷贝到Vx
        memcpy(&v_rx, xyAndRefAngleMsg + 2, 2);                 // 将接收到的数据拷贝到Vy
        memcpy(&relativeangle, xyAndRefAngleMsg + 4, 4);        // 将接收到的数据拷贝到RelativeAngle

        // Vx Vy映射,[0,60000] -> [-2,2]
        vlen = ((float)vlen_rx) / 15000.0f - 2.0f;
        v = ((float)v_rx) / 15000.0f - 2.0f;

        if (isnan(vlen_rx) || isnan(v_rx) || isnan(relativeangle) || (mode.chassis_mode > 3) || (mode.rotate_type > 1) || (mode.jump_ctrl > 2)) // 如果出现nan错误，将速度设定值设为0
        {
            vlen = 0;
            v = 0;
            relativeangle = 0;
            mode.chassis_mode = NONE;
            mode.rotate_type = NORMAL_ROTATE;
            mode.jump_ctrl = DO_NOT_JUMP;
        }

        if (fabsf(vlen) < 0.0005f)
            vlen = 0;
        if (fabsf(v) < 0.0005f)
            v = 0;
        if (fabsf(relativeangle) < 0.0001f)
            relativeangle = 0;

        if (tx_semaphore_get(&IMUThreadSem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            // Functionality to be implemented
        }

        /* Thread periodic delay */
        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}