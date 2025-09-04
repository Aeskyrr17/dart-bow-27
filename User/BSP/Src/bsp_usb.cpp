//
// Created by cosmosmount on 2025/8/30.
//
#include "bsp_usb.hpp"
#include "ux_device_cdc_acm.h"

PortStatus USB_CDC_Send(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        UserTxBufferFS[UserTxBufPtrIn++] = data[i];

        if (UserTxBufPtrIn >= APP_TX_DATA_SIZE)
            UserTxBufPtrIn = 0;

        if (UserTxBufPtrIn == UserTxBufPtrOut)
            return PORT_BUFFER_FULL; // 缓冲区满
    }
    if (USB_TX_SUCCESS) {
        USB_TX_SUCCESS = 0;
        return PORT_SUCCESS;
    }
    else if (USB_TX_BUSY)
        return PORT_BUSY;
    else
        return PORT_FAILED;
}

PortStatus USB_CDC_Receive(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    if (UserRxBufPtrOut == UserRxBufPtrIn || !USB_RX_SUCCESS)
        return PORT_FAILED;
    while ((UserRxBufPtrOut != UserRxBufPtrIn) && (i < len))
    {
        data[i++] = UserRxBufferFS[UserRxBufPtrOut++];

        if (UserRxBufPtrOut >= APP_RX_DATA_SIZE)
        {
            UserRxBufPtrOut = 0;
        }
    }
    USB_RX_SUCCESS = 0;
    return PORT_SUCCESS; // 返回读取的字节数
}