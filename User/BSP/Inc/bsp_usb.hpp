//
// Created by cosmosmount on 2025/8/30.
//

#ifndef RM26_H7_BSP_USB_HPP
#define RM26_H7_BSP_USB_HPP

#include "tx_api.h"
#include "ux_api.h"
#include <cstdint>
#include "usertypes.hpp"

PortStatus USB_CDC_Send(uint8_t *data, uint16_t len);
PortStatus USB_CDC_Receive(uint8_t *data, uint16_t len);

#endif //RM26_H7_BSP_USB_HPP