#pragma once

#include "Communication_Config.h"

#ifdef BSP_VOFA_ENABLE

#include <stdint.h>
#include "VOFA_Protocol.h"

namespace Lib_VOFA
{

void VOFA_SendFloat(const float* data, uint8_t channels);
void VOFA_Init(VOFA_Protocol protocol);

extern void (*VOFA_HardwareTransmit)(const uint8_t* data, uint16_t length);

} // namespace Lib_VOFA

#endif /* BSP_VOFA_ENABLE */
