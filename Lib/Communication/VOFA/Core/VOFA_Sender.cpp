/*
 * VOFA_Sender.cpp — VOFA 发送器核心实现
 *
 * 调用链:
 *   VOFA_SendFloat(data, n)           // 外部入口
 *     → Buffer::writeFrame()          // 写入双缓冲
 *     → Buffer::getReadyBuffer()      // 取备妥帧 (同时复位 complete_)
 *     → VOFA_HardwareTransmit()       // 调用 BSP DMA 发送
 *         → HAL_UART_Transmit_DMA()   // 异步发送, 不阻塞
 *
 * 降采样:
 *   - 上层调用方负责降采样 (每 N 个 tick 调用一次)
 *   - 如果 DMA 仍在上一次发送中 (HAL_BUSY), 本帧被丢弃
 *   - getReadyBuffer() 已复位 complete_, 下次 writeFrame 可以写入
 *
 * 推荐调用频率:
 *   115200 bps: ~300 fps  → 降采样比 ≈ 60:1 (20kHz/300)
 *   921600 bps: ~2500 fps → 降采样比 ≈ 8:1
 */

#include "VOFA_Sender.h"

#ifdef BSP_VOFA_ENABLE

#include "VOFA_Buffer.h"

namespace Lib_VOFA
{

static VOFA_Buffer g_buffer;
static VOFA_Protocol g_protocol = VOFA_Protocol::JustFloat;

void (*VOFA_HardwareTransmit)(const uint8_t* data, uint16_t length) = nullptr;

void VOFA_Init(VOFA_Protocol protocol)
{
    g_protocol = protocol;
    g_buffer = VOFA_Buffer();
}

void VOFA_SendFloat(const float* data, uint8_t channels)
{
    if (VOFA_HardwareTransmit == nullptr) return;
    if (g_protocol != VOFA_Protocol::JustFloat) return;

    // 写入帧到双缓冲 (如果上一帧未被消费则丢弃)
    if (!g_buffer.writeFrame(data, channels))
        return;

    // 取备妥帧 ★ getReadyBuffer 会复位 complete_, 允许下次 writeFrame
    const uint8_t* buf = g_buffer.getReadyBuffer();
    if (buf == nullptr) return;

    // 启动 DMA 异步发送 (BSP_VOFA_Transmit 内部检查 DMA 是否空闲)
    uint16_t frame_size = g_buffer.getFrameSize(channels);
    VOFA_HardwareTransmit(buf, frame_size);
}

} // namespace Lib_VOFA

#endif /* BSP_VOFA_ENABLE */
