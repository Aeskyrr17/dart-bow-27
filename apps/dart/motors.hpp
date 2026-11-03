#pragma once

#include "launcher.hpp"
#include <cstdint>

namespace dart::motors
{
bool start();
launcher_feedback read(uint32_t now);
void write(const launcher_output& output, uint32_t now);
} // namespace dart::motors
