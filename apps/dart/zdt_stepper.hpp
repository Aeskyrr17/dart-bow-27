#pragma once

#include "bsp_can.hpp"
#include <cmath>
#include <cstdint>

namespace dart
{
class zdt_stepper
{
public:
    zdt_stepper(bsp::can::handle bus, uint8_t id, uint8_t positive_direction)
        : bus_(bus), id_(id), positive_direction_(positive_direction)
    {
    }

    bool send_speed(float rpm, uint16_t acceleration, uint16_t max_current_ma);

    bool pending() const
    {
        return part_ != 0;
    }

    bool flush();

private:
    bsp::can::handle bus_;
    uint8_t id_;
    uint8_t positive_direction_;
    uint8_t payload_[9]{};
    uint8_t part_ = 0;
};

inline bool zdt_stepper::send_speed(float rpm, uint16_t acceleration, uint16_t max_current_ma)
{
    if (pending())
    {
        return false;
    }
    const auto speed = static_cast<uint16_t>(std::fabs(rpm) * 10.0f);
    payload_[0] = rpm >= 0.0f ? positive_direction_ : 1U - positive_direction_;
    payload_[1] = acceleration >> 8;
    payload_[2] = acceleration;
    payload_[3] = speed >> 8;
    payload_[4] = speed;
    payload_[5] = 0;
    payload_[6] = max_current_ma >> 8;
    payload_[7] = max_current_ma;
    payload_[8] = 0x6b;
    part_ = 1;
    return flush();
}

inline bool zdt_stepper::flush()
{
    if (!pending())
    {
        return true;
    }
    uint8_t frame[8]{0xc6};
    const uint8_t offset = part_ == 1 ? 0 : 7;
    const uint8_t size = part_ == 1 ? 7 : 2;
    for (uint8_t i = 0; i < size; ++i)
    {
        frame[i + 1] = payload_[offset + i];
    }
    const uint32_t id = (static_cast<uint32_t>(id_) << 8) | (part_ - 1);
    if (bsp::can::transmit(bus_, id, frame, size + 1) != pnxtypes::status::ok)
    {
        return false;
    }
    part_ = part_ == 1 ? 2 : 0;
    return true;
}

} // namespace dart
