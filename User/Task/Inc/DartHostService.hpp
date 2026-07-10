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
    void ProcessRequest(const msg_hostreq_t& req, const msg_launcher2sysctrl_t& lch2sys,
                        DartLibrary& dart);
    bool IsResetTestRoundAllowed(const msg_launcher2sysctrl_t& lch2sys) const;

    om_fifo_t* hostreq_fifo_ = nullptr;
    om_topic_t* hosttx_topic_ = nullptr;
};
