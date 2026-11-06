#pragma once

#include "HostProtocol.hpp"
#include "TaskSysCtrl.hpp"
#include "om.h"

class DartHostService
{
public:
    void Init();
    void Poll(DartLibrary& dart, const msg_launcher2sysctrl_t& lch2sys,
              bool test_control_allowed, ULONG test_mode_timeout_ticks,
              const msg_visionrx_t& vision_rx, bool vision_updated);
    bool IsTestModeEnabled() const;
    float GetTestTensionLeft() const;
    float GetTestTensionRight() const;
    HOST_TEST_ACTION GetTestAction() const;
    float GetTestYawCommand() const;
    CTRL_MODE GetTestYawMode() const;
    void ClearTestAction();
    void DisableTestMode();

private:
    bool EnsureTopicsReady();
    void UpdateForceTelemetrySources();
    void PublishForceTelemetry();
    void PublishTestState(const msg_launcher2sysctrl_t& lch2sys);
    void ProcessRequest(const msg_hostreq_t& req, const msg_launcher2sysctrl_t& lch2sys,
                        bool test_control_allowed, DartLibrary& dart);
    bool IsResetTestRoundAllowed(const msg_launcher2sysctrl_t& lch2sys) const;
    void UpdateTestAim(const msg_visionrx_t& vision_rx, const msg_launcher2sysctrl_t& lch2sys);

    om_fifo_t* hostreq_fifo_ = nullptr;
    om_topic_t* hosttx_topic_ = nullptr;
    om_suber_t* sensor_suber_ = nullptr;
    om_suber_t* motorctrl_suber_ = nullptr;
    msg_sensor_t latest_sensor_{};
    msg_motor_ctrl_t latest_motorctrl_{};
    ULONG last_force_telemetry_tick_ = 0;
    uint8_t force_telemetry_seq_ = 0;
    bool test_mode_enabled_ = false;
    bool test_control_allowed_ = false;
    float test_tension_left_ = 0.0f;
    float test_tension_right_ = 0.0f;
    HOST_TEST_ACTION test_action_ = HOST_TEST_ACTION_NONE;
    ULONG last_host_activity_tick_ = 0;
    ULONG last_test_state_tick_ = 0;
    uint8_t test_state_seq_ = 0;
    HOST_TEST_AIM_STATE test_aim_state_ = HOST_TEST_AIM_IDLE;
    float test_yaw_command_ = 0.0f;
    float vision_yaw_error_ = 0.0f;
    bool vision_valid_ = false;
    bool vision_seen_ = false;
    ULONG last_vision_tick_ = 0;
};
