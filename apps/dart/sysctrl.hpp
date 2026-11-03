#pragma once

#include "launcher.hpp"
#include "remoter.hpp"

namespace dart
{
class sysctrl
{
public:
    command update(const remotes::reading& input, bool online, launcher_state state);

};
} // namespace dart
