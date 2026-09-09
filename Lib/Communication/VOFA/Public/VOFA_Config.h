#pragma once

#include "Communication_Config.h"

#ifdef BSP_VOFA_ENABLE

#include <stdint.h>

namespace Lib_VOFA
{

#define VOFA_MAX_CHANNELS  12
#define VOFA_FRAME_SIZE    (VOFA_MAX_CHANNELS * 4 + 4)    // channels * sizeof(float) + tail
#define VOFA_TAIL_LEN      4

} // namespace Lib_VOFA

#endif /* BSP_VOFA_ENABLE */
