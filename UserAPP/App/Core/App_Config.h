#pragma once

#include <stdint.h>

#include "App/Core/App_BuildConfig.h"

namespace UserAPP
{

constexpr uint8_t APP_MAX_AXES = 1U;
constexpr uint8_t APP_COMMAND_INBOX_CAPACITY = 8U;
constexpr uint8_t APP_SYNC_FIFO_CAPACITY = 4U;

} // namespace UserAPP
