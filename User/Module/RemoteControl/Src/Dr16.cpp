// #include "Dr16.hpp"
// extern UART_HandleTypeDef huart3;
//
// void Dr16::Init()
// {
//
// }
//
// void Dr16::Update()
// {
//     UpdateRcStatus();
//     UpdateKeyStatus();
//     AliveCheck();
// }
//
// /*--------------------------------------------在线检测------------------------------------------------------*/
// void Dr16::AliveCheck()
// {
//     if ((int)DWT_GetTimeline_ms() % 100 == 0)
//     {
//         if (AliveFlag == Pre_AliveFlag)
//         {
//             Alive = false;
//             AliveFlag = 0;
//             Pre_AliveFlag = 0;
//         }
//         else
//         {
//             Pre_AliveFlag = AliveFlag;
//             Alive = true;
//         }
//     }
// }
//
// bool Dr16::IsAlive()
// {
//     return Alive;
// }
//
// /*--------------------------------------------处理遥控器数据------------------------------------------------------*/
// float Dr16::MapAvix(uint16_t ch)
// {
//     return ((float)(ch) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
// }
//
// void Dr16::UpdateRcStatus()
// {
//     // 如果左侧开关状态发生变化，记录变化前的状态
//     if (Right_CurrentSw != Dr16_Data._data.s1)
//         Right_PreviousSw = Right_CurrentSw;
//     if (Left_CurrentSw != Dr16_Data._data.s2)
//         Left_PreviousSw = Left_CurrentSw;
//
//     // 更新左右侧开关状态
//     Right_CurrentSw = (RC_SWITCH_STATE)Dr16_Data._data.s1;
//     Left_CurrentSw = (RC_SWITCH_STATE)Dr16_Data._data.s2;
//
//     if (Left_PreviousSw == RC_SW_UP && Left_CurrentSw == RC_SW_MID)
//         Left_SwChange = RC_SWITCH_U2M;
//     else if (Left_PreviousSw == RC_SW_MID && Left_CurrentSw == RC_SW_UP)
//         Left_SwChange = RC_SWITCH_M2U;
//     else if (Left_PreviousSw == RC_SW_MID && Left_CurrentSw == RC_SW_DOWN)
//         Left_SwChange = RC_SWITCH_M2D;
//     else if (Left_PreviousSw == RC_SW_DOWN && Left_CurrentSw == RC_SW_MID)
//         Left_SwChange = RC_SWITCH_D2M;
//
//     if (Right_PreviousSw == RC_SW_UP && Right_CurrentSw == RC_SW_MID)
//         Right_SwChange = RC_SWITCH_U2M;
//     else if (Right_PreviousSw == RC_SW_MID && Right_CurrentSw == RC_SW_UP)
//         Right_SwChange = RC_SWITCH_M2U;
//     else if (Right_PreviousSw == RC_SW_MID && Right_CurrentSw == RC_SW_DOWN)
//         Right_SwChange = RC_SWITCH_M2D;
//     else if (Right_PreviousSw == RC_SW_DOWN && Right_CurrentSw == RC_SW_MID)
//         Right_SwChange = RC_SWITCH_D2M;
// }
//
// float Dr16::GetRightX()
// {
//     return MapAvix(Dr16_Data._data.ch_0);
// }
//
// float Dr16::GetRightY()
// {
//     return MapAvix(Dr16_Data._data.ch_1);
// }
//
// float Dr16::GetLeftX()
// {
//     return MapAvix(Dr16_Data._data.ch_2);
// }
//
// float Dr16::GetLeftY()
// {
//     return MapAvix(Dr16_Data._data.ch_3);
// }
//
// Dr16::RC_SWITCH_STATE Dr16::QuerySwStatus(RC_SWITCH_TYPE sw)
// {
//     switch (sw)
//     {
//     case LEFT_SWITCH:
//         return Left_CurrentSw;
//     case RIGHT_SWITCH:
//         return Right_CurrentSw;
//     case LEFT_SWITCH_CHANGE:
//         return Left_SwChange;
//     case RIGHT_SWITCH_CHANGE:
//         return Right_SwChange;
//     default:
//         return Left_SwChange;
//     }
// }
//
// /*--------------------------------------------处理键鼠数据------------------------------------------------------*/
// float Dr16::MapMouse(int16_t m)
// {
//     return (float)m / MOUSE_OFFSET_MAX;
// }
//
// void Dr16::UpdateKeyStatus()
// {
//     PreviousKeyState = CurrentKeyState;
//     // CurrentKeyState = Dr16_Data._data.key;
// }
//
// float Dr16::GetMouseX()
// {
//     return MapMouse(Dr16_Data._data.mouse_x);
// }
//
// float Dr16::GetMouseY()
// {
//     return MapMouse(Dr16_Data._data.mouse_y);
// }
//
// float Dr16::GetMouseZ()
// {
//     return Dr16_Data._data.mouse_z;
// }
//
// uint8_t Dr16::GetMouseL()
// {
//     return Dr16_Data._data.mouse_left;
// }
//
// uint8_t Dr16::GetMouseR()
// {
//     return Dr16_Data._data.mouse_right;
// }
//
// Dr16::PC_KEY_STATE Dr16::QueryPcKeyStatus(PC_KEY_TYPE key)
// {
//     uint16_t mask = (uint16_t)key;
//     uint16_t current = CurrentKeyState & mask;
//     uint16_t previous = PreviousKeyState & mask;
//
//     if (current && previous)
//     {
//         return PC_KEY_DOWN;
//     }
//     else if (!current && !previous)
//     {
//         return PC_KEY_UP;
//     }
//     else if (!previous && current)
//     {
//         return PC_KEY_FALL;
//     }
//     else
//     {
//         return PC_KEY_RISE;
//     }
// }
//
// /*--------------------------------------------统一函数，Dr16不使用------------------------------------------------------*/
// float Dr16::GetWheel()
// {
//     return 0;
// }
//
// bool Dr16::QueryButtonPressed()
// {
//     return false;
// }
//
// uint8_t Dr16::GetMouseM()
// {
//     return 0;
// }
//
// Dr16::RC_SWITCH_STATE Dr16::GetCurrentLeftSwStatus()
// {
//     return Left_CurrentSw;
// }
//
// Dr16::RC_SWITCH_STATE Dr16::GetPreviousLeftSwStatus()
// {
//     return Left_PreviousSw;
// }
