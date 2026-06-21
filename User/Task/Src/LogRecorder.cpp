// #include "TaskBooster.hpp"
// #include "om_core.h"
#include "tx_api.h"
#include "om.h"
#include "magicmsgs.hpp"
#include "config_launcher.hpp"
#include "TaskSysCtrl.hpp"
#include "config_motor.hpp"
#include "config_sensor.hpp"
#include <sys/types.h>

TX_THREAD LogThread;
uint8_t LogThreadStack[1024] = {0};

extern DartLibrary dart_lib;
extern Launcher_Cxt_t launcher;

[[nonreturn]] void LogThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input); 

    om_suber_t *motorfdb_suber = om_subscribe(om_find_topic("motorfdb", UINT32_MAX));
    msg_motorfdb_t motorfdb{};
    om_suber_t* cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    msg_cmd_t cmd{};
    om_suber_t* sensor_suber = om_subscribe(om_find_topic("sensor", UINT32_MAX));
    msg_sensor_t sensor{};
    om_suber_t* lch2sys_suber = om_subscribe(om_find_topic("lch2sys", UINT32_MAX));
    msg_launcher2sysctrl_t lch2sys{};
    om_suber_t* referee_suber = om_subscribe(om_find_topic("referee", UINT32_MAX));
    msg_referee_t referee_pack{};
    om_suber_t* visionrx_suber = om_subscribe(om_find_topic("visionrx", UINT32_MAX)); 
    msg_visionrx_t vision_rx{};
    om_suber_t* visiontx_suber = om_subscribe(om_find_topic("visiontx", UINT32_MAX));
    msg_visiontx_t vision_tx{};
    om_suber_t* remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};
    om_suber_t* motorctrl_suber = om_subscribe(om_find_topic("motorctrl", UINT32_MAX));
    msg_motor_ctrl_t motorctrl{};

    om_topic_t* log_topic = om_config_topic(nullptr, "ca", "log", sizeof(logger_t));
    logger_t logger{};
    logger.header = 0xD5;

    while (1)
    {
        om_suber_export(cmd_suber, &cmd, false);
        om_suber_export(sensor_suber, &sensor, false);
        om_suber_export(motorfdb_suber, &motorfdb, false);
        om_suber_export(lch2sys_suber, &lch2sys, false);
        om_suber_export(referee_suber, &referee_pack, false);
        om_suber_export(visionrx_suber, &vision_rx, false);
        om_suber_export(visiontx_suber, &vision_tx, false);
        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(motorctrl_suber, &motorctrl, false);

        logger.header = 0xD5;
        logger.state = static_cast<uint8_t>(launcher.fsm_state);
        logger.prepare_state = static_cast<uint8_t>(launcher.prep_state);
        logger.launch_station_status = dart_lib.runtime.referee.launch_station_status;
        logger.is_fire_finished = launcher.is_fire_done;
        logger.fired_count_this_open = static_cast<uint8_t>(dart_lib.runtime.fired_count_this_open);
        logger.current_shot_number = static_cast<uint8_t>(dart_lib.runtime.current_shot_number);
        logger.current_dart_id = static_cast<uint8_t>(dart_lib.runtime.current_dart_id);
        logger.door_status = static_cast<uint8_t>(dart_lib.runtime.vision_door_status);
        logger.last_light_detected = 0;
        logger.vision_light_detected = vision_rx.light_detected;
        logger.vision_stable_state = vision_rx.stable_state;
        logger.door_session_active = 0;
        logger.autoaim_allow = dart_lib.runtime.autoAim.autoaim_allow;
        logger.door_close_inhibit_active = 0;
        logger.string_L_force_kg = sensor.string_L_force_kg;
        logger.string_R_force_kg = sensor.string_R_force_kg;

        om_publish(log_topic, &logger, sizeof(logger_t), true, false);
        tx_thread_sleep(1);
    };

}
