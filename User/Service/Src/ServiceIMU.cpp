#include "main.h"
#include "tx_api.h"
#include "om.h"

#include "bsp_dwt.hpp"
#include "quaternion_ekf.hpp"
#include "BMI088.hpp"
#include "magicmsgs.hpp"
#include <cstring>

using namespace BMI088;

cBMI088 bmi088;
cIMU *imu_handler = &bmi088;

TX_THREAD IMUThread;
uint8_t IMUThreadStack[4096] = {0};
TX_SEMAPHORE IMUThreadSem;

[[noreturn]] void IMUThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    om_topic_t *ins_topic = om_config_topic(nullptr, "ca", "ins", sizeof(msg_ins_t));
    msg_ins_t msg_ins{};

    imu_handler->self_test.ACC_CHIP_ID_ERR = true;
    imu_handler->self_test.ACC_DATA_ERR = true;
    imu_handler->self_test.GYRO_CHIP_ID_ERR = true;
    imu_handler->self_test.GYRO_DATA_ERR = true;
    imu_handler->self_test.INIT_ERR = true;
    imu_handler->self_test.CALIBRATE_ERR = false;
    imu_handler->self_test.TEMP_CTRL_ERR = false;

    imu_handler->Config();
    imu_handler->VerifyAccChipID();
    imu_handler->VerifyGyroChipID();
    imu_handler->self_test.INIT_ERR = imu_handler->self_test.ACC_CHIP_ID_ERR ||
                                      imu_handler->self_test.GYRO_CHIP_ID_ERR;

    if (!imu_handler->self_test.INIT_ERR)
    {
        // Keep the mechanism still during the approximately four-second calibration.
        imu_handler->Calibrate();
    }

    QuaternionEKF qekf;
    uint32_t ins_count = 0;
    DWT_GetDeltaT(&ins_count);

    for (;;)
    {
        if (!imu_handler->self_test.INIT_ERR)
        {
            imu_handler->ReadAccData(&imu_handler->acc_data);
            imu_handler->ReadGyroData(&imu_handler->gyro_data);
            qekf.UpdateKalman(
                imu_handler->gyro_data.x,
                imu_handler->gyro_data.y,
                imu_handler->gyro_data.z,
                imu_handler->acc_data.x,
                imu_handler->acc_data.y,
                imu_handler->acc_data.z,
                DWT_GetDeltaT(&ins_count));

            std::memcpy(msg_ins.quaternion, qekf.q, sizeof(qekf.q));
            msg_ins.yaw = qekf.yaw;
            msg_ins.pitch = qekf.pitch;
            msg_ins.roll = qekf.roll;
            msg_ins.total_yaw = qekf.total_yaw;
            msg_ins.gyro_r = imu_handler->gyro_data.x;
            msg_ins.gyro_p = imu_handler->gyro_data.y;
            msg_ins.gyro_y = imu_handler->gyro_data.z;
            msg_ins.accel[0] = imu_handler->acc_data.x;
            msg_ins.accel[1] = imu_handler->acc_data.y;
            msg_ins.accel[2] = imu_handler->acc_data.z;
        }

        om_publish(ins_topic, &msg_ins, sizeof(msg_ins), true, false);
        tx_semaphore_ceiling_put(&IMUThreadSem, 1);
        tx_thread_sleep(1);
    }
}
