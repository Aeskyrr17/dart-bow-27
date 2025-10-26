#include "main.h"

#include "om.h"
#include "bsp_can.hpp"
#include "bsp_dwt.hpp"

#include "LK9025.hpp"
#include "LK8016.hpp"
#include "DJIMotorHandler.hpp"
#include "LKMotorHandler.hpp"

#include "math.hpp"
#include "filter.hpp"
#include "odometry.hpp"
#include "vmcsolver.hpp"
#include "magicmsgs.hpp"

using namespace Numeric;
// using namespace Odometry;

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

[[noreturn]] void MotorThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    LK9025 RWheel;
    LK9025 LWheel;

    LK8016 RHip1;
    LK8016 RHip2;

    LK8016 LHip1;
    LK8016 LHip2;

    LKMotorHandler LKmotorhandler;
    LKmotorhandler.registerMotor(&LHip1, &hfdcan1, 0x141);
    LHip1.currentSet = 0;
    LKmotorhandler.registerMotor(&LHip2, &hfdcan1, 0x142);
    LHip2.currentSet = 0;
    LKmotorhandler.registerMotor(&RHip1, &hfdcan1, 0x143);
    RHip1.currentSet = 0;
    LKmotorhandler.registerMotor(&RHip2, &hfdcan1, 0x144);
    RHip2.currentSet = 0;
    LKmotorhandler.registerMotor(&LWheel, &hfdcan3, 0x141);
    LWheel.currentSet = 0;
    LKmotorhandler.registerMotor(&RWheel, &hfdcan3, 0x142);
    RWheel.currentSet = 0;

    constexpr float Tk_LK9025 = 195.3125f; // 2000 / (0.32f * 32.0f) 0.32：扭矩常数，32.0：电流实际最大值，2000.0：电流输入最大值
    constexpr float Tk_LK8016 = 43.4028f;  // 2000 / (0.24f * 32.0f * 6.0f) 0.24：扭矩常数，6：减速比，32.0：电流实际最大值，2000.0：电流数值范围

    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *chassisctrl_suber = om_subscribe(om_find_topic("chassisctrl", UINT32_MAX));
    msg_chassis_ctrl_t chassis_ctrl{};

    om_topic_t *rod_pub = om_config_topic(nullptr, "ca", "rod", sizeof(msg_rod_t));
    msg_rod_t rod{};
    om_topic_t *torque_pub = om_config_topic(nullptr, "ca", "torque", sizeof(msg_torque_t));
    msg_torque_t torque{};
    om_topic_t *odom_pub = om_config_topic(nullptr, "ca", "odom", sizeof(odometry_info_t));
    odometry_info_t odom_info{};

    cVMCSolver Lsolver;
    cVMCSolver Rsolver;

    float Lvq[2] = {0};
    float Rvq[2] = {0};

    float Lxdot[2] = {0};
    float Rxdot[2] = {0};

    float Lt[2] = {0};
    float Rt[2] = {0};

    float Lft[2] = {0};
    float Rft[2] = {0};

    for (;;)
    {
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(chassisctrl_suber, &chassis_ctrl, false);

        Lsolver.Resolve(LHip1.motorFeedback.positionFdb, LHip2.motorFeedback.positionFdb, 0);
        Rsolver.Resolve(RHip1.motorFeedback.positionFdb, RHip2.motorFeedback.positionFdb, 1);

        rod.leg_len = 0.5f * (Lsolver.GetPendulumLen() + Rsolver.GetPendulumLen());

        Lvq[0] = LHip1.motorFeedback.speedFdb;
        Lvq[1] = LHip2.motorFeedback.speedFdb;
        Rvq[0] = -RHip1.motorFeedback.speedFdb;
        Rvq[1] = -RHip2.motorFeedback.speedFdb;

        Lsolver.VMCVelCal(Lvq, Lxdot);
        Rsolver.VMCVelCal(Rvq, Rxdot);

        rod.leg_len_dot = 0.5f * (Lxdot[0] + Rxdot[0]);
        rod.leg_theta = 0.5f * (Lsolver.GetPendulumRadian() + Rsolver.GetPendulumRadian()) + ins.pitch * DegreeToRad - 0.5f*Pi;
        rod.leg_theta_dot = -0.5f * (Lxdot[1] + Rxdot[1]) + ins.gyro_p;

        float vel = 0.5f * (LWheel.motorFeedback.speedFdb + RWheel.motorFeedback.speedFdb);
        odom_info = Odometry_Update(ins.quaternion, ins.accel, vel, ins.yaw);

        torque.Tlwheel = static_cast<float>(LWheel.motorFeedback.currentFdb) * Tk_LK9025;
        torque.Trwheel = static_cast<float>(RWheel.motorFeedback.currentFdb) * Tk_LK9025;

        Lt[0] = static_cast<float>(LHip1.motorFeedback.currentFdb) * Tk_LK8016;
        Lt[1] = static_cast<float>(LHip2.motorFeedback.currentFdb) * Tk_LK8016;
        Rt[0] = static_cast<float>(RHip1.motorFeedback.currentFdb) * Tk_LK8016;
        Rt[1] = static_cast<float>(RHip2.motorFeedback.currentFdb) * Tk_LK8016;

        Lsolver.VMCRevCal(Lft, Lt);
        Rsolver.VMCRevCal(Rft, Rt);

        torque.F = Lft[0] + Rft[0];
        torque.Tp = Lft[1] + Rft[1];

        om_publish(rod_pub, &rod, sizeof(msg_rod_t), true, false);
        om_publish(odom_pub, &odom_info, sizeof(odometry_info_t), true, false);
        tx_thread_sleep(1);
    }
}
