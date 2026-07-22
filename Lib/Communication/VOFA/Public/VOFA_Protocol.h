#pragma once

namespace Lib_VOFA
{

enum class VOFA_Protocol : uint8_t
{
    JustFloat = 0,   // 原始 float 字节 + 帧尾 0x00 0x00 0x80 0x7F
    Firewater = 1,   // 文本: "val1,val2,val3\n"
};

} // namespace Lib_VOFA
