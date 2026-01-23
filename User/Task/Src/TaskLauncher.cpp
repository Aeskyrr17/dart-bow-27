#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"
#include "config_launcher.hpp"
#include "config_motor.hpp"
#include "math.hpp"

TX_THREAD LauncherThread;
uint8_t LauncherThreadStack[2048] = {0};

[[nonreturn]] void LauncherThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_topic_t *motorctrl_topic = om_config_topic(nullptr, "ca", "motorctrl", sizeof(msg_motor_ctrl_t));
    msg_motor_ctrl_t motorctrl{};

    om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    msg_cmd_t cmd{};
    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    
    for (;;) 
    {
        om_suber_export(cmd_suber, &cmd, false);

        om_publish(motorctrl_topic, &motorctrl, sizeof(msg_motor_ctrl_t), true, false);
        tx_thread_sleep(1);
    }

}