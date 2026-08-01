#include "BSP_VOFA.h"

#ifdef BSP_VOFA_ENABLE

#include "main.h"

void BSP_VOFA_Init(void)
{
    Core_USART1_TxDmaResetRuntime();
}

bool BSP_VOFA_TxBusy(void)
{
    return Core_USART1_TxDmaBusy();
}

void BSP_VOFA_Transmit(const uint8_t* data, uint16_t length)
{
    if ((data == nullptr) || (length == 0U))
    {
        return;
    }

    /*
     * VOFA 只走 Core 层 USART1 TX DMA。
     * DMA 忙时直接丢帧，上位机会依靠 JustFloat 帧尾重新同步，快环内不等待串口。
     */
    if (Core_USART1_TxDmaBusy())
    {
        return;
    }

    (void)Core_USART1_TxDmaTryStart(data, length);
}

#endif /* BSP_VOFA_ENABLE */
