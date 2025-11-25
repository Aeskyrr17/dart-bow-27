#include "GM6020.hpp"
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
#include "tx_api.h"
#include "utils.h"
#include "vmc.hpp"
#include "magicmsgs.hpp"

#include "config_chassis.hpp"

using namespace Numeric;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

TX_THREAD SolverThread;
uint8_t SolverThreadStack[4096] = {0};

LKMotorHandler *LKmotorhandler = LKMotorHandler::Instance();

#ifdef DEBUG
struct solver_debug_t
{
    float llength;
    float rlength;
    float lphi;
    float rphi;
    float llength_dot;
    float rlength_dot;
    float lphi_dot;
    float rphi_dot;
    float lphi1;
    float lphi4;
    float rphi1;
    float rphi4;
    float lhip1_tor;
    float lhip2_tor;
    float rhip1_tor;
    float rhip2_tor;
    float rwheel_tor_ref;
    float lwheel_tor_ref;
    float rwheel_tor_fdb;
    float lwheel_tor_fdb;
};
__attribute__((section(".RAM_D3"))) solver_debug_t solver_debug;
#endif

[[noreturn]] void SolverThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    LK9025 RWheel;
    LK9025 LWheel;

    LK8016 RHip1;
    LK8016 RHip2;

    LK8016 LHip1;
    LK8016 LHip2;

    LKMotorHandler::Instance()->registerMotor(&LHip1, &hfdcan1, 0x141);
    LHip1.currentSet = 0;
    LHip1.offset = LHIP1_OFFSET;
    LKMotorHandler::Instance()->registerMotor(&LHip2, &hfdcan1, 0x142);
    LHip2.currentSet = 0;
    LHip2.offset = LHIP2_OFFSET;
    LKMotorHandler::Instance()->registerMotor(&RHip1, &hfdcan1, 0x143);
    RHip1.currentSet = 0;
    RHip1.offset = RHIP1_OFFSET;
    LKMotorHandler::Instance()->registerMotor(&RHip2, &hfdcan1, 0x144);
    RHip2.currentSet = 0;
    RHip2.offset = RHIP2_OFFSET;
    LKMotorHandler::Instance()->registerMotor(&LWheel, &hfdcan3, 0x141);
    LWheel.currentSet = 0;
    LKMotorHandler::Instance()->registerMotor(&RWheel, &hfdcan3, 0x142);
    RWheel.currentSet = 0;

    constexpr float Tk_LK9025 = 195.3125f; // 2000 / (0.32f * 32.0f) 0.32：扭矩常数，32.0：电流实际最大值，2000.0：电流输入最大值
    constexpr float Tk_LK8016 = 43.4028f;  // 2000 / (0.24f * 32.0f * 6.0f) 0.24：扭矩常数，6：减速比，32.0：电流实际最大值，2000.0：电流数值范围

    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *pendulumctrl_suber = om_subscribe(om_find_topic("pendulumctrl", UINT32_MAX));
    msg_ctrl_t pendulumctrl{};

    om_topic_t *solverfdb_topic = om_config_topic(nullptr, "ca", "solverfdb", sizeof(msg_solver_t));
    msg_solver_t solverfdb{};
    om_topic_t *odom_pub = om_config_topic(nullptr, "ca", "odom", sizeof(msg_odometry_t));
    msg_odometry_t odom_data{};

    cVMCSolver Lsolver;
    cVMCSolver Rsolver;

    Odometry odom;

    float Lqdot[2] = {0};
    float Rqdot[2] = {0};

    float Lxdot[2] = {0};
    float Rxdot[2] = {0};

    float LTp[2] = {0};
    float RTp[2] = {0};

    float thread_start_time;

    for (;;)
    {
        thread_start_time = tx_time_get();
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(pendulumctrl_suber, &pendulumctrl, false);

        Lsolver.Resolve(LHip1.motorFeedback.positionFdb, LHip2.motorFeedback.positionFdb, 0);
        Rsolver.Resolve(RHip1.motorFeedback.positionFdb, RHip2.motorFeedback.positionFdb, 1);

        solverfdb.llen = Lsolver.GetPendulumLen();
        solverfdb.rlen = Rsolver.GetPendulumLen();

        Lqdot[0] = LHip1.motorFeedback.speedFdb;
        Lqdot[1] = LHip2.motorFeedback.speedFdb;
        Rqdot[0] = -RHip1.motorFeedback.speedFdb;
        Rqdot[1] = -RHip2.motorFeedback.speedFdb;

        Lsolver.VMCVelCal(Lqdot, Lxdot);
        Rsolver.VMCVelCal(Rqdot, Rxdot);

        solverfdb.llen_dot = Lxdot[0];
        solverfdb.rlen_dot = Rxdot[0];
        solverfdb.lphi = Lsolver.GetPendulumRadian();
        solverfdb.rphi = Rsolver.GetPendulumRadian();
        solverfdb.lphi_dot = Lxdot[1];
        solverfdb.rphi_dot = Rxdot[1];

        float vel = 0.5f * (LWheel.motorFeedback.speedFdb + RWheel.motorFeedback.speedFdb);
        odom_data = odom.Update(ins.quaternion, ins.accel, vel, ins.yaw);

        om_publish(solverfdb_topic, &solverfdb, sizeof(msg_solver_t), true, false);
        om_publish(odom_pub, &odom_data, sizeof(msg_odometry_t), true, false);

        Lsolver.VMCCal(pendulumctrl.Tl, LTp);
        Rsolver.VMCCal(pendulumctrl.Tr, RTp);

        LHip1.currentSet = 0;
        LHip2.currentSet = 0;
        RHip1.currentSet = 0;
        RHip2.currentSet = 0;

        LWheel.currentSet = 0;
        RWheel.currentSet = 0;

        LHip1.currentSet = LTp[0]*Tk_LK8016;
        LHip2.currentSet = LTp[1]*Tk_LK8016;
        RHip1.currentSet = -RTp[0]*Tk_LK8016;
        RHip2.currentSet = -RTp[1]*Tk_LK8016;

        LWheel.currentSet = pendulumctrl.Twl * Tk_LK9025;
        RWheel.currentSet = -pendulumctrl.Twr * Tk_LK9025;


    #ifdef DEBUG
        solver_debug.llength = solverfdb.llen;
        solver_debug.rlength = solverfdb.rlen;
        solver_debug.lphi = solverfdb.lphi;
        solver_debug.rphi = solverfdb.rphi;
        solver_debug.llength_dot = solverfdb.llen_dot;
        solver_debug.rlength_dot = solverfdb.rlen_dot;
        solver_debug.lphi_dot = solverfdb.lphi_dot;
        solver_debug.rphi_dot = solverfdb.rphi_dot;
        
        solver_debug.lphi1 = Lsolver.GetPhi1();
        solver_debug.lphi4 = Lsolver.GetPhi4();
        solver_debug.rphi1 = Rsolver.GetPhi1();
        solver_debug.rphi4 = Rsolver.GetPhi4();

        solver_debug.lhip1_tor = LTp[0];
        solver_debug.lhip2_tor = LTp[1];
        solver_debug.rhip1_tor = RTp[0];
        solver_debug.rhip2_tor = RTp[1];
        solver_debug.rwheel_tor_ref = -pendulumctrl.Twr;
        solver_debug.lwheel_tor_ref = pendulumctrl.Twl;
        solver_debug.rwheel_tor_fdb = RWheel.motorFeedback.torqueFdb;
        solver_debug.lwheel_tor_fdb = LWheel.motorFeedback.torqueFdb;
    #endif
        
        LKMotorHandler::Instance()->sendControlData();
        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}
