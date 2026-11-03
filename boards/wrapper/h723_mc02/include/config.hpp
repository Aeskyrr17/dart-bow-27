#pragma once

// Configuration types for peripherals enabled in h723_mc02.ioc.

#include <cstdint>

namespace bsp
{

enum class active_level : uint8_t
{
    low,
    high
};

struct dma_config
{
    bool rx = false;
    bool tx = false;
};

namespace spi
{
enum class handle : uint8_t
{
    none,
    spi1,
    spi2,
    spi6
};

struct config
{
    handle instance = handle::none;
    dma_config dma{};
    bool irq = false;
};
} // namespace spi

namespace gpio
{
enum class port : uint8_t
{
    none,
    a,
    b,
    c,
    d,
    e,
    h
};

struct config
{
    port instance = port::none;
    uint8_t pin = 0; // Pin number 0..15, not a HAL bit mask.
    active_level active = active_level::high;
};
} // namespace gpio

namespace pwm
{
enum class handle : uint8_t
{
    none,
    tim1,
    tim3,
    tim12
};
enum class channel : uint8_t
{
    none,
    ch1,
    ch2,
    ch3,
    ch4
};

struct config
{
    uint32_t timer_clock_hz = 0; // Kernel clock before the timer prescaler.
    handle instance = handle::none;
    uint8_t channel_mask = 0; // Bits 0..3 describe enabled CH1..CH4.
};
} // namespace pwm

namespace adc
{
enum class handle : uint8_t
{
    none,
    adc1
};

enum class sampling_time : uint8_t
{
    cycles_1_5,
    cycles_2_5,
    cycles_8_5,
    cycles_16_5,
    cycles_32_5,
    cycles_64_5,
    cycles_387_5,
    cycles_810_5
};

struct config
{
    uint8_t channel = 0;
    uint8_t rank = 0;
    sampling_time sampling = sampling_time::cycles_1_5;
    handle instance = handle::none;
};
} // namespace adc

namespace usart
{
enum class handle : uint8_t
{
    none,
    uart5,
    uart7,
    usart1,
    usart10
};

struct config
{
    handle instance = handle::none;
    dma_config dma{};
};
} // namespace usart

} // namespace bsp

// Resource values mirror the board IOC; these declarations do not initialize hardware.
namespace bsp::peripherals
{
// DMA flags are {rx, tx}; irq is the SPI peripheral NVIC enable.
inline constexpr spi::config spi1{spi::handle::spi1, {false, false}, true};
inline constexpr spi::config spi2{spi::handle::spi2, {true, true}, true};
inline constexpr spi::config spi6{spi::handle::spi6, {false, false}, false};
inline constexpr usart::config uart5{usart::handle::uart5, {true, false}};
inline constexpr usart::config uart7{usart::handle::uart7, {true, true}};
inline constexpr usart::config usart1{usart::handle::usart1, {true, true}};
inline constexpr usart::config usart10{usart::handle::usart10, {true, false}};

inline constexpr adc::config adc1{4, 1, adc::sampling_time::cycles_1_5, adc::handle::adc1};

// Timer kernel clocks before prescaling, not PWM output frequencies.
// APB1 timers: RCC.Tim2OutputFreq_Value=260000000.
inline constexpr pwm::config tim1{260000000U, pwm::handle::tim1, 0x04U};   // CH3
inline constexpr pwm::config tim3{260000000U, pwm::handle::tim3, 0x08U};   // CH4
inline constexpr pwm::config tim12{260000000U, pwm::handle::tim12, 0x02U}; // CH2

// Unidentified GPIOs retain the default active level; initial output levels do not
// establish device polarity. PC13/14/15 have no device labels in the IOC.
inline constexpr gpio::config ps2_cs{gpio::port::e, 15, active_level::low};
inline constexpr gpio::config pc13{gpio::port::c, 13};
inline constexpr gpio::config pc14{gpio::port::c, 14};
inline constexpr gpio::config pc15{gpio::port::c, 15};

inline constexpr spi::config spi_configs[]{spi1, spi2, spi6};
inline constexpr usart::config usart_configs[]{uart5, uart7, usart1, usart10};
inline constexpr pwm::config pwm_configs[]{tim1, tim3, tim12};
} // namespace bsp::peripherals

// BMI088 pins labelled in h723_mc02.ioc; INT pins use rising-edge EXTI.
namespace board::bmi088
{
inline constexpr bsp::gpio::config acc_cs{bsp::gpio::port::c, 0, bsp::active_level::low};
inline constexpr bsp::gpio::config gyro_cs{bsp::gpio::port::c, 3, bsp::active_level::low};
inline constexpr bsp::gpio::config acc_drdy{bsp::gpio::port::e, 10};
inline constexpr bsp::gpio::config gyro_drdy{bsp::gpio::port::e, 12};
inline constexpr auto spi = bsp::spi::handle::spi2;
inline constexpr auto heater_timer = bsp::pwm::handle::tim3;
inline constexpr auto heater_channel = bsp::pwm::channel::ch4;
} // namespace board::bmi088

// WS2812 data is encoded over SPI6, not driven as three RGB GPIOs.
namespace board::led
{
inline constexpr auto spi = bsp::spi::handle::spi6;
} // namespace board::led
