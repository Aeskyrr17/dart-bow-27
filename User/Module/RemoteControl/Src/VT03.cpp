// #include "../Inc/VT03.hpp"
//
// #include <cstring>
// extern UART_HandleTypeDef huart6;
//
// void VT03::Init()
// {
//     memset(VT03_Data.ReceiveBuffer, 0, VT03_DATA_SIZE);
// }
//
// void VT03::Update()
// {
//     UpdateKeyStatus();
//     UpdateRcStatus();
//     AliveCheck();
// }
//
// /*--------------------------------------------在线检测------------------------------------------------------*/
// void VT03::AliveCheck()
// {
//     if ((int)DWT_GetTimeline_ms() % 200 == 0)
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
// bool VT03::IsAlive()
// {
//     return Alive;
// }
//
// /*--------------------------------------------处理遥控器数据------------------------------------------------------*/
// float VT03::MapAvix(uint64_t ch)
// {
//     return ((float)(ch) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
// }
//
// void VT03::UpdateRcStatus()
// {
//     if (CurrentSw != VT03_Data._data.mode_sw)
//     {
//         PreviousSw = CurrentSw;
//         CurrentSw = (MODE_SW_STATE)VT03_Data._data.mode_sw;
//     }
//     if (PreviousSw == MODE_SW_C && CurrentSw == MODE_SW_N)
//         SwChange = MODE_SW_C2N;
//     else if (PreviousSw == MODE_SW_N && CurrentSw == MODE_SW_C)
//         SwChange = MODE_SW_N2C;
//     else if (PreviousSw == MODE_SW_N && CurrentSw == MODE_SW_S)
//         SwChange = MODE_SW_N2S;
//     else if (PreviousSw == MODE_SW_S && CurrentSw == MODE_SW_N)
//         SwChange = MODE_SW_S2N;
// }
//
// float VT03::GetRightX()
// {
//     return MapAvix(VT03_Data._data.ch_0);
// }
//
// float VT03::GetRightY()
// {
//     return MapAvix(VT03_Data._data.ch_1);
// }
//
// float VT03::GetLeftY()
// {
//     return MapAvix(VT03_Data._data.ch_2);
// }
//
// float VT03::GetLeftX()
// {
//     return MapAvix(VT03_Data._data.ch_3);
// }
//
// float VT03::GetWheel()
// {
//     return MapAvix(VT03_Data._data.wheel);
// }
//
// bool VT03::QueryButtonPressed(BUTTON_TYPE button)
// {
//     switch (button)
//     {
//     case BUTTON_PAUSE:
//         return VT03_Data._data.pause;
//     case BUTTON_TRIGGER:
//         return VT03_Data._data.trigger;
//     case BUTTON_LEFT:
//         return VT03_Data._data.fn_1;
//     case BUTTON_RIGHT:
//         return VT03_Data._data.fn_2;
//     default:
//         return false;
//     }
// }
//
// VT03::MODE_SW_STATE VT03::QuerySwStatus()
// {
//     return CurrentSw;
// }
//
// /*--------------------------------------------处理键鼠数据------------------------------------------------------*/
// float VT03::MapMouse(int16_t m)
// {
//     return (float)m / MOUSE_OFFSET_MAX;
// }
//
// void VT03::UpdateKeyStatus()
// {
//     PreviousKeyState = CurrentKeyState;
//     CurrentKeyState = VT03_Data._data.key;
// }
//
// float VT03::GetMouseX()
// {
//     return MapMouse(VT03_Data._data.mouse_x);
// }
//
// float VT03::GetMouseY()
// {
//     return MapMouse(VT03_Data._data.mouse_y);
// }
//
// float VT03::GetMouseZ()
// {
//     return VT03_Data._data.mouse_z;
// }
//
// uint8_t VT03::GetMouseL()
// {
//     return VT03_Data._data.mouse_left;
// }
//
// uint8_t VT03::GetMouseR()
// {
//     return VT03_Data._data.mouse_right;
// }
//
// uint8_t VT03::GetMouseM()
// {
//     return VT03_Data._data.mouse_middle;
// }
//
// VT03::PC_KEY_STATE VT03::QueryPcKeyStatus(PC_KEY_TYPE key)
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
