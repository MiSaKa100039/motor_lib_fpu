#pragma once

#include "Communication_Config.h"

#ifdef BSP_VOFA_ENABLE

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_VOFA_Init(void);
bool BSP_VOFA_TxBusy(void);
void BSP_VOFA_Transmit(const uint8_t* data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* BSP_VOFA_ENABLE */
