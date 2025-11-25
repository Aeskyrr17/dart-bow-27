#include "math.hpp"
#include "pid.hpp"
#include "lqr.hpp"
#include "filter.hpp"
#include "slope.hpp"
#include "magicmsgs.hpp"
#include "config_chassis.hpp"
#include "vmc.hpp"
#include "om.h"
#include <cstring>

using namespace Filter;

TX_THREAD PendulumThread;
uint8_t PendulumThreadStack[4096] = {0};

uint8_t xyAndRefAngleMsg[8] = {0};
uint8_t chassisStateMsg[8] = {0};

extern TX_SEMAPHORE IMUThreadSem;

#ifdef DEBUG
struct pendulum_debug_t
{
    float alpha;
    float alpha_dot;
    float x;
    float v;
    float l;
    float pitch;
    float pitch_dot;
    float T;
    float Tp;
    float Fl;
    float Fr;
};

__attribute__((section(".RAM_D3"))) msg_ins_t debug_ins;
__attribute__((section(".RAM_D3"))) msg_remoter_t debug_remoter;
__attribute__((section(".RAM_D3"))) pendulum_debug_t pendulum_debug;
#endif

[[noreturn]] void PendulumThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    /* Legs Params Initialization */
    PID rleg_len_pd(2000.0f, 0.0f, -1500.0f, 100.0f, 0.0f, PID_DVEL);
    PID lleg_len_pd(2000.0f, 0.0f, -1500.0f, 100.0f, 0.0f, PID_DVEL);
    
    IIRFilter leg_len_filter(2,LOWPASS,1);
    SLOPE leg_len_updater(0.001f,0.001f,0.18f);

    /* Roll Params Initialization */
    PID roll_pd(0.7f, 0.0f, 0.01f, 3.0f, 0.0f);
    SLOPE roll_updater(0.0002f,0.0002f,0.0f);

    /* Yaw Params Initialization */
    PID yaw_pd(18.0f, 0.0f, 4.5f, 5.0f, 0.0f);
    PID yaw_dot_pd(0.8f, 0.0f, 0.6f, 4.5f, 0.0f);
    SLOPE yaw_updater(0.01f, 0.01f, 0.0f);

    /* LQR Initialization */
    LQR lqr_controller;
    float observedX[6] = {0};
    float refX[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    arm_matrix_instance_f32 MatXRef = {6, 1, refX};
    arm_matrix_instance_f32 MatXObs = {6, 1, observedX};
    lqr_controller.InitMatX(&MatXRef, &MatXObs);
    float Tout[2] = {0};

    /* Control Signal Initialization */
    chassis_mode_t mode;
    uint16_t vlen_rx;
    uint16_t vy_rx;
    float vlen;
    float vy;
    float relativeangle;
    float lenfdb;

    /* One Message Initialization */
    om_topic_t *pendulumctrl_topic =om_config_topic(nullptr, "ca", "pendulumctrl", sizeof(msg_ctrl_t));
    msg_ctrl_t pendulum_ctrl{};

    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *solver_suber = om_subscribe(om_find_topic("solverfdb", UINT32_MAX));
    msg_solver_t solver_fdb{};
    om_suber_t *odom_suber = om_subscribe(om_find_topic("odom", UINT32_MAX));
    msg_odometry_t odom{};
    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};

    for (;;)
    {
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(solver_suber, &solver_fdb, false);
        om_suber_export(odom_suber, &odom, false);
        om_suber_export(remoter_suber, &remoter, false);

        if (tx_semaphore_get(&IMUThreadSem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            if (remoter.ctrl_sw == Relax || remoter.offline)
            {
                pendulum_ctrl.Tl[0] = 0;
                pendulum_ctrl.Tr[0] = 0;
                pendulum_ctrl.Tl[1] = 0;
                pendulum_ctrl.Tr[1] = 0;
                pendulum_ctrl.Twl = 0;
                pendulum_ctrl.Twr = 0;
            }
            
            else 
            {
                uint8_t state_msg = chassisStateMsg[0];
                mode.chassis_mode = static_cast<chassis_mode_e>(state_msg & 0x03);
                mode.rotate_type = static_cast<rotate_ctrl_e>((state_msg >> 2) & 0x01);
                mode.jump_ctrl = static_cast<jump_ctrl_e>((state_msg >> 3) & 0x03);
                mode.fly_ctrl = static_cast<fly_ctrl_e>((state_msg >> 5) & 0x01);

                memcpy(&vlen_rx, xyAndRefAngleMsg, 2);               // 将接收到的数据拷贝到Vx
                memcpy(&vy_rx, xyAndRefAngleMsg + 2, 2);             // 将接收到的数据拷贝到Vy
                memcpy(&relativeangle, xyAndRefAngleMsg + 4, 4);     // 将接收到的数据拷贝到RelativeAngle

                // Vx Vy映射,[0,60000] -> [-2,2]
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

                lleg_len_pd.ref = 0.20f;
                lleg_len_pd.fdb = solver_fdb.llen;
                lleg_len_pd.UpdateResult(solver_fdb.llen_dot);
                pendulum_ctrl.Tl[0] = lleg_len_pd.result;//0.0f;//

                rleg_len_pd.ref = 0.20f;
                rleg_len_pd.fdb = solver_fdb.rlen;
                rleg_len_pd.UpdateResult(solver_fdb.rlen_dot);
                pendulum_ctrl.Tr[0] = rleg_len_pd.result;//0.0f;//

                observedX[0] = 0.5f*(solver_fdb.lphi + solver_fdb.rphi - Pi) + ins.pitch*DegreeToRad;
                observedX[1] = 0.5f*(solver_fdb.lphi_dot + solver_fdb.rphi_dot) + ins.gyro_p;
                observedX[2] = 0.0f;//odom.x;
                observedX[3] = 0.0f;//odom.v;
                observedX[4] = ins.pitch*DegreeToRad;
                observedX[5] = ins.gyro_p;

                refX[0] = 0.0f;
                refX[1] = 0.0f;
                refX[2] = 0.0f;//odom.x;
                refX[3] = 0.0f;
                refX[4] = 0.0f;
                refX[5] = 0.0f;

                lenfdb = 0.5f * (solver_fdb.llen + solver_fdb.rlen);
                lqr_controller.refreshLQRK(lenfdb, mode.fly_ctrl == FLY_MODE);
                lqr_controller.LQRCal(Tout);
                pendulum_ctrl.Tl[1] = Tout[1]*0.5f;//0.0f;//
                pendulum_ctrl.Tr[1] = Tout[1]*0.5f;//0.0f;//
                pendulum_ctrl.Twl = Tout[0]*0.5f;
                pendulum_ctrl.Twr = Tout[0]*0.5f;
            }
        }

    #ifdef DEBUG
        debug_ins = ins;
        debug_remoter = remoter;
        pendulum_debug.alpha = observedX[0];
        pendulum_debug.alpha_dot = observedX[1];
        pendulum_debug.pitch = observedX[4];
        pendulum_debug.pitch_dot = observedX[5];
        pendulum_debug.T = Tout[0];
        pendulum_debug.Tp = Tout[1];
        pendulum_debug.Fl = pendulum_ctrl.Tl[0];
        pendulum_debug.Fr = pendulum_ctrl.Tr[0];
        pendulum_debug.l = lenfdb;
        pendulum_debug.x = odom.x;
        pendulum_debug.v = odom.v;
    #endif
        
        om_publish(pendulumctrl_topic, &pendulum_ctrl, sizeof(msg_ctrl_t), true, false);
        tx_thread_sleep(1);
    }
}