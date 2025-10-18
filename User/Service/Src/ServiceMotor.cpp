#include "ServiceMotor.hpp"
#include "bsp_can.hpp"
#include "main.h"
#include "om.h"
#include "magicmsgs.hpp"
#include "filter.hpp"
#include "math.hpp"
#include "bsp_dwt.hpp"

using namespace Numeric;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

TX_SEMAPHORE MotorCANRecvSem;

TX_THREAD MotorThread;
uint8_t MotorThreadStack[4096] = {0};
DJIMotorHandler* DJIMotorhandler = DJIMotorHandler::Instance();

extern TX_THREAD IMUThread;
extern TX_SEMAPHORE IMUThreadSem;
extern uint8_t IMUThreadStack[4096];
extern void IMUThreadFun(ULONG initial_input);

motor_debug_t motor_debug;
pid_tuning_t motor_pos_pid;
pid_tuning_t motor_spd_pid;
int debug_cur = 0;
float vel_ratio = 0;

void ServiceMotors::MotorRegister()
{
}

void ServiceMotors::AllMotorSetOutput()
{
    // LWheel.setOutput();
    // RWheel.setOutput();
}

void ServiceMotors::SetModeAndPidParam()
{

}

[[noreturn]] void MotorThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    constexpr float Tk_LK9025 = 195.3125f; // 2000 / (0.32f * 32.0f) 0.32：扭矩常数，32.0：电流实际最大值，2000.0：电流输入最大值
    constexpr float Tk_LK8016 = 43.4028f;  // 2000 / (0.24f * 32.0f * 6.0f) 0.24：扭矩常数，6：减速比，32.0：电流实际最大值，2000.0：电流数值范围

    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};

    for (;;)
    {
        om_suber_export(ins_suber, &ins, false);
        if (tx_semaphore_get(&IMUThreadSem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {

        }
        tx_thread_sleep(1);
    }
}
