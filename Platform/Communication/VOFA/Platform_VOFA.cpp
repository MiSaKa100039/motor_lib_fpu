#include "Platform_VOFA.h"

#ifdef BSP_VOFA_ENABLE

#include "BSP_VOFA.h"
#include "main.h"

#include <cmath>

namespace
{
constexpr uint8_t kFakeDataChannels = 4;
constexpr uint32_t kFakeDataPeriodMs = 5;
constexpr float kPi = 3.1415927f;
constexpr float kTwoPi = 6.2831853f;

float g_fake_data_phase = 0.0f;
uint32_t g_fake_data_last_tick = 0;
} // namespace

void Platform_VOFA_Init(void)
{
    BSP_VOFA_Init();
    Lib_VOFA::VOFA_HardwareTransmit = BSP_VOFA_Transmit;
    Lib_VOFA::VOFA_Init(Lib_VOFA::VOFA_Protocol::JustFloat);
}

void Platform_VOFA_SendFloat(const float* data, uint8_t channels)
{
    Lib_VOFA::VOFA_SendFloat(data, channels);
}

void Platform_VOFA_TestFakeData_Init(void)
{
    g_fake_data_phase = 0.0f;
    g_fake_data_last_tick = 0;
    Platform_VOFA_Init();
}

void Platform_VOFA_TestFakeData_Run(void)
{
    const uint32_t now = g_heartbeat;
    if ((now - g_fake_data_last_tick) < kFakeDataPeriodMs)
    {
        return;
    }
    g_fake_data_last_tick = now;

    g_fake_data_phase += 0.1f;
    if (g_fake_data_phase > kTwoPi)
    {
        g_fake_data_phase -= kTwoPi;
    }

    float data[kFakeDataChannels];
    data[0] = sinf(g_fake_data_phase);
    data[1] = cosf(g_fake_data_phase);
    data[2] = g_fake_data_phase / kTwoPi;
    data[3] = (g_fake_data_phase < kPi) ? 1.0f : -1.0f;

    Platform_VOFA_SendFloat(data, kFakeDataChannels);
}

#endif /* BSP_VOFA_ENABLE */
