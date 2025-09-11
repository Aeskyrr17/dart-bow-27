#include "GMMotorHandler.hpp"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

// 电机数据转换因子
const float GMMotorHandler::RawPosToRad = 0.0007669903939f;   // （7.6699039394282061485904379474597e-4）位置值转换为弧度的转换因子，编码器为十三位，2^13 = 8192, 2 * PI / 8192 (rad)，这样，当编码器的值增加或减少 1 时，它表示电机轴旋转了 2 * PI / 8192 (rad)
const float GMMotorHandler::RawRpmToRadps = 0.1047197551196f; // 0.1047197551f;  // RPM 到弧度每秒的转换因子, 2 * PI / 60 (s)

/**
 * @brief 构造函数，将所有值初始化
 */
GMMotorHandler::GMMotorHandler()
{
    for (int i = 0; i < 8; i++)
        GMMotorList[0][i] = nullptr;
    for (int i = 0; i < 8; i++)
        GMMotorList[1][i] = nullptr;
    for (int i = 0; i < 8; i++)
        GMMotorList[2][i] = nullptr;

    for (int i = 0; i < 8; i++)
        can1_send_data_0[i] = 0;
    for (int i = 0; i < 8; i++)
        can1_send_data_1[i] = 0;
    for (int i = 0; i < 8; i++)
        can2_send_data_0[i] = 0;
    for (int i = 0; i < 8; i++)
        can2_send_data_1[i] = 0;
    for (int i = 0; i < 8; i++)
        can3_send_data_0[i] = 0;
    for (int i = 0; i < 8; i++)
        can3_send_data_1[i] = 0;
}

GMMotorHandler::~GMMotorHandler()
{
}

/**
 * @brief 注册电机，将电机指针存入MotorList中
 * @param motor 电机指针
 * @param hcan CAN句柄
 * @param canId 电机ID
 */
void GMMotorHandler::registerMotor(GMMotor *GMmotor, FDCAN_HandleTypeDef *hcan, uint16_t canId)
{
    GMmotor->canId = canId;
    GMmotor->hcan = hcan;

    if (canId >= 0x201 && canId <= 0x208)
    {
        if (GMmotor->hcan == &hfdcan1)
        {
            GMMotorList[0][canId - 0x201] = GMmotor; // 将电机指针存入MotorList中
            // 判断电机的控制标识符，用于判断是否需要发送控制数据
            if (canId >= 0x201 && canId <= 0x204)
            {
                CAN1_0x200_Exist = true;
            }
            else if (canId >= 0x205 && canId <= 0x208)
            {
                CAN1_0x1FF_Exist = true;
            }
            return;
        }
        else if (GMmotor->hcan == &hfdcan2)
        {
            GMMotorList[1][canId - 0x201] = GMmotor; // 将电机指针存入MotorList中
            // 判断电机的控制标识符，用于判断是否需要发送控制数据
            if (canId >= 0x201 && canId <= 0x204)
            {
                CAN2_0x200_Exist = true;
            }
            else if (canId >= 0x205 && canId <= 0x208)
            {
                CAN2_0x1FF_Exist = true;
            }
        }
        else if (GMmotor->hcan == &hfdcan3)
        {
            GMMotorList[2][canId - 0x201] = GMmotor; // 将电机指针存入MotorList中
            // 判断电机的控制标识符，用于判断是否需要发送控制数据
            if (canId >= 0x201 && canId <= 0x204)
            {
                CAN3_0x200_Exist = true;
            }
            else if (canId >= 0x205 && canId <= 0x208)
            {
                CAN3_0x1FF_Exist = true;
            }
        }
    }
}

// void GMMotorHandler::processRawData(FDCAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
// {
//     if (hcan == &hfdcan1)
//     {
//         for (int i = 0; i < 8; i++)
//         {
//             can1_receive_data[index].last_ecd = can1_receive_data[index].ecd;                 ///< 上一次的编码器值
//             can1_receive_data[index].ecd = (uint16_t)(rx_data[0] << 8 | rx_data[1]);          ///< 编码器值
//             can1_receive_data[index].speed_rpm = (int16_t)(rx_data[2] << 8 | rx_data[3]);     ///< 速度值，单位rpm
//             can1_receive_data[index].given_current = (int16_t)(rx_data[4] << 8 | rx_data[5]); ///< 电流值，或者说是转矩值
//             can1_receive_data[index].temperate = rx_data[6];                                  ///< 温度值
//         }
//     }
//     else if (hcan == &hfdcan2)
//     {
//         for (int i = 0; i < 8; i++)
//         {
//             can2_receive_data[index].last_ecd = can2_receive_data[index].ecd;                 ///< 上一次的编码器值
//             can2_receive_data[index].ecd = (uint16_t)(rx_data[0] << 8 | rx_data[1]);          ///< 编码器值
//             can2_receive_data[index].speed_rpm = (int16_t)(rx_data[2] << 8 | rx_data[3]);     ///< 速度值，单位rpm
//             can2_receive_data[index].given_current = (int16_t)(rx_data[4] << 8 | rx_data[5]); ///< 电流值，或者说是转矩值
//             can2_receive_data[index].temperate = rx_data[6];                                  ///< 温度值
//         }
//     }

//     updateFeedback();
// }

/**
 * @brief 发送控制数据
 * @param hfdcan1 CAN1句柄
 * @param hfdcan2 CAN2句柄
 */
void GMMotorHandler::sendControlData()
{
    // 循环遍历所有的电机，将电机的控制数据存入can_send_data中
    //< 如果电机列表的最大数量发生变化，这里的循环次数需要修改！！！
    for (int i = 0; i < 8; i++)
    {
        // 处理挂载在CAN1的电机
        if (GMMotorList[0][i] != nullptr)
        {
            // 0x201-0x204，控制标识符为0x200
            if (GMMotorList[0][i]->canId >= 0x201 && GMMotorList[0][i]->canId <= 0x204)
            {
                int index = (GMMotorList[0][i]->canId - 0x200) * 2;
                can1_send_data_0[index - 2] = GMMotorList[0][i]->currentSet >> 8;
                can1_send_data_0[index - 1] = GMMotorList[0][i]->currentSet;
            }
            // 0x205-0x208，控制标识符为0x1FF
            if (GMMotorList[0][i]->canId >= 0x205 && GMMotorList[0][i]->canId <= 0x208)
            {
                int index = (GMMotorList[0][i]->canId - 0x204) * 2;
                can1_send_data_1[index - 2] = GMMotorList[0][i]->currentSet >> 8;
                can1_send_data_1[index - 1] = GMMotorList[0][i]->currentSet;
            }
        }
        // 处理挂载在CAN2的电机
        if (GMMotorList[1][i] != nullptr)
        {
            // 0x201-0x204，控制标识符为0x200
            if (GMMotorList[1][i]->canId >= 0x201 && GMMotorList[1][i]->canId <= 0x204)
            {
                int index = (GMMotorList[1][i]->canId - 0x200) * 2;
                can2_send_data_0[index - 2] = GMMotorList[1][i]->currentSet >> 8;
                can2_send_data_0[index - 1] = GMMotorList[1][i]->currentSet;
            }
            // 0x205-0x208，控制标识符为0x1FF
            if (GMMotorList[1][i]->canId >= 0x205 && GMMotorList[1][i]->canId <= 0x208)
            {
                int index = (GMMotorList[1][i]->canId - 0x204) * 2;
                can2_send_data_1[index - 2] = GMMotorList[1][i]->currentSet >> 8;
                can2_send_data_1[index - 1] = GMMotorList[1][i]->currentSet;
            }
        }
        // 处理挂载在CAN3的电机
        if (GMMotorList[2][i] != nullptr)
        {
            // 0x201-0x204，控制标识符为0x200
            if (GMMotorList[2][i]->canId >= 0x201 && GMMotorList[2][i]->canId <= 0x204)
            {
                int index = (GMMotorList[2][i]->canId - 0x200) * 2;
                can3_send_data_0[index - 2] = GMMotorList[2][i]->currentSet >> 8;
                can3_send_data_0[index - 1] = GMMotorList[2][i]->currentSet;
            }
            // 0x205-0x208，控制标识符为0x1FF
            if (GMMotorList[2][i]->canId >= 0x205 && GMMotorList[2][i]->canId <= 0x208)
            {
                int index = (GMMotorList[2][i]->canId - 0x204) * 2;
                can3_send_data_1[index - 2] = GMMotorList[2][i]->currentSet >> 8;
                can3_send_data_1[index - 1] = GMMotorList[2][i]->currentSet;
            }
        }
    }
    // 使用bsp_can中的函数发送数据，只应该发送有效数据，防止堵塞。
    if (CAN1_0x200_Exist)
        CAN_Transmit(&hfdcan1, 0x200, can1_send_data_0, 8); // 向CAN1发送数据，电机控制报文0x200
    if (CAN1_0x1FF_Exist)
        CAN_Transmit(&hfdcan1, 0x1FF, can1_send_data_1, 8); // 向CAN1发送数据，电机控制报文0x1FF
    if (CAN2_0x200_Exist)
        CAN_Transmit(&hfdcan2, 0x200, can2_send_data_0, 8); // 向CAN2发送数据，电机控制报文0x200
    if (CAN2_0x1FF_Exist)
        CAN_Transmit(&hfdcan2, 0x1FF, can2_send_data_1, 8); // 向CAN2发送数据，电机控制报文0x2FF
    if (CAN3_0x200_Exist)
        CAN_Transmit(&hfdcan3, 0x200, can3_send_data_0, 8); // 向CAN2发送数据，电机控制报文0x200
    if (CAN3_0x1FF_Exist)
        CAN_Transmit(&hfdcan3, 0x1FF, can3_send_data_1, 8); // 向CAN2发送数据，电机控制报文0x2FF
}

/**
 * @brief 处理并更新电机反馈数据
 * 将电机的反馈数据存入Motorlist中存在的电机的MotorFeedback中
 * @param hfdcan1 CAN1句柄
 * @param hfdcan2 CAN2句柄
 */
void GMMotorHandler::updateFeedback(FDCAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    if (hcan == &hfdcan1)
    {
        //< 如果电机列表的最大数量发生变化，这里的循环次数需要修改！！！
        for (int i = 0; i < 8; i++)
        {
            // 处理CAN1的电机
            if (GMMotorList[0][index] != nullptr)
            {
                Receive(GMMotorList[0][index], rx_data);
            }
        }
    }

    else if (hcan == &hfdcan2)
    {
        for (int i = 0; i < 8; i++)
        {
            // 处理CAN2的电机
            if (GMMotorList[1][index] != nullptr)
            {
                Receive(GMMotorList[1][index], rx_data);
            }
        }
    }

    else if (hcan == &hfdcan3)
    {
        for (int i = 0; i < 8; i++)
        {
            // 处理CAN2的电机
            if (GMMotorList[2][index] != nullptr)
            {
                Receive(GMMotorList[2][index], rx_data);
            }
        }
    }
}

/**
 * @brief 接收电机数据，将数据存入电机的反馈数据中
 * @param motor 电机指针
 * @param can_rx_buff CAN接收数据
 */
void GMMotorHandler::Receive(GMMotor *motor, uint8_t *can_receive_data)
{
    if (motor != nullptr)
    {
        motor->UpdateSensorData(can_receive_data);
    }
}
