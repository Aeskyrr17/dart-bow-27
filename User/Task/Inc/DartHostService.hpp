#pragma once

#include "HostProtocol.hpp"
#include "TaskSysCtrl.hpp"
#include "om.h"

class DartHostService
{
public:
    void Init();
    void Poll(DartLibrary& dart, const msg_launcher2sysctrl_t& lch2sys);

private:
    bool EnsureTopicsReady();
    void UpdateForceTelemetrySources();
    void PublishForceTelemetry();
    void ProcessRequest(const msg_hostreq_t& req, const msg_launcher2sysctrl_t& lch2sys,
                        DartLibrary& dart);
    bool IsResetTestRoundAllowed(const msg_launcher2sysctrl_t& lch2sys) const;

    om_fifo_t* hostreq_fifo_ = nullptr;
    om_topic_t* hosttx_topic_ = nullptr;
    om_suber_t* sensor_suber_ = nullptr;
    om_suber_t* motorctrl_suber_ = nullptr;
    msg_sensor_t latest_sensor_{};
    msg_motor_ctrl_t latest_motorctrl_{};
    ULONG last_force_telemetry_tick_ = 0;
    uint8_t force_telemetry_seq_ = 0;
};
