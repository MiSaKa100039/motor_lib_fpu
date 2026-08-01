/*
 * VOFA_Buffer.cpp — 双缓冲发送缓冲实现
 *
 * JustFloat 帧格式 (little-endian):
 *   | float0 | float1 | ... | floatN-1 | 0x00 0x00 0x80 0x7F |
 *   |←────────────────── 4N bytes ────→|←── tail (float +inf) ──→|
 *
 * 帧尾 = IEEE 754 float 正无穷大 (0x7F800000), VOFA+ 用此标记帧边界
 *
 * 双缓冲机制:
 *   writeFrame() 写入当前活跃缓冲 → 翻转 useA_ → complete_=true
 *   getReadyBuffer() 返回非活跃缓冲(刚写完的) → complete_=false ★
 *   ★ complete_ 在 getReadyBuffer() 时复位, 下次 writeFrame 才允许写入
 */

#include "VOFA_Buffer.h"

#ifdef BSP_VOFA_ENABLE

#include <cstring>

namespace Lib_VOFA
{

VOFA_Buffer::VOFA_Buffer()
    : useA_(true)
    , complete_(false)
{
    memset(bufA_, 0, sizeof(bufA_));
    memset(bufB_, 0, sizeof(bufB_));
}

bool VOFA_Buffer::writeFrame(const float* data, uint8_t channels)
{
    if (channels == 0 || channels > VOFA_MAX_CHANNELS)
        return false;

    // 上一帧还没被消费, 拒绝写入 (防止覆盖正在 DMA 的双缓冲搭档)
    if (complete_)
        return false;

    // 选择空闲缓冲写入
    uint8_t* buf = useA_ ? bufA_ : bufB_;
    uint16_t data_len = channels * 4;

    // 拷贝 float 数据 (字节级, 保留 IEEE 754 格式)
    memcpy(buf, data, data_len);

    // 追加帧尾: 0x00 0x00 0x80 0x7F = float +inf
    static const uint8_t tail[VOFA_TAIL_LEN] = { 0x00, 0x00, 0x80, 0x7F };
    memcpy(buf + data_len, tail, VOFA_TAIL_LEN);

    // 翻转: 下一帧写入另一个缓冲
    //   例: useA=true → 写入 bufA → useA=false → getReadyBuffer 返回 bufA
    useA_ = !useA_;
    complete_ = true;
    return true;
}

const uint8_t* VOFA_Buffer::getReadyBuffer()
{
    if (!complete_) return nullptr;

    // ★ 消费: 返回备妥缓冲, 同时复位 complete_ 允许写入下一帧
    complete_ = false;

    // 返回与当前写入缓冲相反的缓冲 (即刚写完的那个)
    return useA_ ? bufB_ : bufA_;
}

uint16_t VOFA_Buffer::getFrameSize(uint8_t channels) const
{
    return channels * 4 + VOFA_TAIL_LEN;
}

} // namespace Lib_VOFA

#endif /* BSP_VOFA_ENABLE */
