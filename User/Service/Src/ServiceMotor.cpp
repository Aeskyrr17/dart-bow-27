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

motor_debug_t motor_debug;
pid_tuning_t motor_pos_pid;
pid_tuning_t motor_spd_pid;
int debug_cur = 0;
float vel_ratio = 0;

void ServiceMotors::MotorRegister() {
    // //注册电机
    DJIMotorhandler->registerMotor(&YawMotor, &hfdcan1, 0x205);
    YawMotor.currentSet = 0;
    YawMotor.gearBox = GearBox_None;

    DJIMotorhandler->registerMotor(&PitchMotor, &hfdcan1, 0x206);
    PitchMotor.currentSet = 0;
}

void ServiceMotors::AllMotorSetOutput()
{
    // LWheel.setOutput();
    // RWheel.setOutput();
}

void ServiceMotors::SetModeAndPidParam()
{
    YawMotor.speedPid.kp = 300.0f;
    YawMotor.speedPid.ki = 0.01f;
    YawMotor.speedPid.kd = 1.0f;

    PitchMotor.speedPid.kp = 100.0f;
}

[[noreturn]] void MotorThreadFun(ULONG initial_input) {
    UNUSED(initial_input);

    constexpr float Tk_LK9025 = 195.3125f; // 2000 / (0.32f * 32.0f) 0.32：扭矩常数，32.0：电流实际最大值，2000.0：电流输入最大值
    constexpr float Tk_LK8016 = 43.4028f;  // 2000 / (0.24f * 32.0f * 6.0f) 0.24：扭矩常数，6：减速比，32.0：电流实际最大值，2000.0：电流数值范围

    for (;;) {

        tx_thread_sleep(1);
    }
}
