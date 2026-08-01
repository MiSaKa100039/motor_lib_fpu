/*
 * VOFA_Buffer.h — 双缓冲发送管理器
 *
 * VOFA+ JustFloat 协议每帧大小 = channels × 4 + 4(tail)
 * 双缓冲 (bufA/bufB) 目的: DMA 正在发送 bufA 时, ISR 可安全写入 bufB
 *
 * 状态机:
 *   writeFrame()     → complete_=true (帧已备妥)
 *   getReadyBuffer() → 返回备妥缓冲 + complete_=false ★ 复位
 *   只有 complete_=false 时才能写入新帧
 *
 * 为什么不用环形缓冲?
 *   环形缓冲有累积延迟, VOFA 需要实时波形, 旧数据没意义
 *   双缓冲只保留最新一帧 + 正在发送的一帧
 */

#pragma once

#include "Communication_Config.h"

#ifdef BSP_VOFA_ENABLE

#include <stdint.h>
#include "../Public/VOFA_Config.h"
#include "../Public/VOFA_Protocol.h"

namespace Lib_VOFA
{

class VOFA_Buffer
{
public:
    VOFA_Buffer();

    /*
     * 写入一帧数据到空闲缓冲区
     * @param data     float 数组指针
     * @param channels 通道数 (1~8)
     * @return true=写入成功, false=上一帧尚未被 consume (需等待)
     */
    bool writeFrame(const float* data, uint8_t channels);

    /*
     * 获取已就绪的帧缓冲区 (含帧尾), 同时将 complete_ 复位
     * ★ 调用后 complete_=false, 允许 writeFrame 写入下一帧
     * @return 就绪的缓冲区指针, 未就绪返回 nullptr
     */
    const uint8_t* getReadyBuffer();

    uint16_t getFrameSize(uint8_t channels) const;

    /* 检查是否有待发送帧 */
    bool isReady() const { return complete_; }

private:
    uint8_t bufA_[VOFA_FRAME_SIZE];    // 双缓冲 A
    uint8_t bufB_[VOFA_FRAME_SIZE];    // 双缓冲 B
    bool    useA_;                     // 当前写入哪个缓冲 (A=true, B=false)
    bool    complete_;                 // 备妥缓冲是否有完整帧
};

} // namespace Lib_VOFA

#endif /* BSP_VOFA_ENABLE */
