//
// Created by cosmosmount on 2025/9/2.
//
#include "BMI088.hpp"
#include "tx_api.h"
#include "ServiceIMU.hpp"
#include "AHRS.hpp"


using namespace BMI088;

cBMI088 *bmi088 = cBMI088::Instance();

AHRS *ahrs = AHRS::Instance();


TX_THREAD IMUThread;
uint8_t IMUThreadStack[4096] = {0};

TX_SEMAPHORE IMUThreadSem;

ULONG IMU_time;

[[noreturn]] void IMUThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    IMU_time = tx_time_get();

    /* INS Topic */
    // om_topic_t *ins_topic = om_config_topic(nullptr, "CA", "INS", sizeof(Msg_INS_t));

    bmi088->Init();
    ahrs->INS_Init();

    for (;;) {

        bmi088->Update();
        ahrs->AHRS_Update();

        tx_semaphore_put(&IMUThreadSem);

        uint8_t time_to_delay = tx_time_get() - IMU_time;
        if (time_to_delay < 1) {
            tx_thread_sleep(1 - time_to_delay);
        }
        // om_publish(ins_topic, &msg_ins, sizeof(msg_ins), true, false);
    }
}
