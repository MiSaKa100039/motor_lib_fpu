#pragma once

#include "VOFA.h"

#ifdef __cplusplus
extern "C" {
#endif

void Platform_VOFA_Init(void);
void Platform_VOFA_SendFloat(const float* data, uint8_t channels);
void Platform_VOFA_TestFakeData_Init(void);
void Platform_VOFA_TestFakeData_Run(void);

#ifdef __cplusplus
}
#endif
