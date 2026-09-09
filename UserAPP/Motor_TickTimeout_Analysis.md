# 电机控制 Tick 超时问题分析

## 现象

调试过程中出现过两类表象：

1. `BSP_Motor_InjectedAdc1CallbackRateHz` 在空载时接近 `20000`，`cmd` 启动后掉到 `8000` 左右。
2. VOFA 波形卡顿，HFI 声音异常，甚至 ADC 注入回调中的 LED 翻转都不再执行。

这两个现象看起来像同一个问题，但实际是前后串联的两层问题。

## 第一层问题：ADC 注入回调直接丢失

最开始为了让三相功率输出在 STOP/COAST 时彻底关闭，BSP 层采用了直接关闭 `TIM1 MOE` 的方式。

但当前工程里：

- ADC 注入触发源配置为 `TIM1_CC4`
- TIM1 CH4 又承担了 ADC 注入触发参考

在这套配置下，直接关闭 `MOE` 会把 CH4 对 ADC 的注入触发链路一起影响掉。结果就是：

- `HAL_ADCEx_InjectedConvCpltCallback()` 不再进入
- `BSP_Motor_InjectedAdc1CallbackCount` 不再增长
- LED 翻转逻辑不执行
- `BSP_Motor_InjectedAdc1CallbackRateHz` 长时间为 `0`

### 第一层修复

修复思路是把“关输出”和“保留触发”分开：

- 保留 `TIM1 MOE` 和 `CH4` 触发链路
- 只通过 `CCER` 开关 `CH1/CH1N/CH2/CH2N/CH3/CH3N`
- 也就是只关闭三相功率通道，不去动 CH4 注入触发路径

修复后，ADC 注入回调恢复，LED 翻转恢复，`CallbackCount` 重新连续增长。

## 第二层问题：RUN 后 ISR 实际超时

回调恢复后，`BSP_Motor_InjectedAdc1CallbackRateHz` 仍然会在 `cmd` 启动后从 `20000` 掉到约 `8000`。这说明问题已经不是“回调不进”，而是“回调进了，但 ISR 处理不过来”。

进一步定位后，根因主要有两项：

1. `MotorManager::tick()` 在 RUN 状态下，每个 `20kHz` tick 都调用一次 `validateModeSelection(target_mode_)`
2. duty 限幅使用了 `std::fmax/std::fmin`

### 为什么这两项会出问题

`validateModeSelection()` 会继续走到配置安全校验逻辑，里面包含大量：

- `validateBase()`
- `validateFeedback()`
- 多个 `std::isfinite`
- 多个模式/能力分支判断

这些检查本来适合放在：

- `start()`
- `selectMode()`
- `debugXXX()` 入口

而不适合放在 `20kHz` ISR 里每 tick 重复执行。

`std::fmax/std::fmin` 单次开销不算大，但放在高频 ISR 里也没有必要，用普通分支更直接。

### 第二层修复

修复方式是：

1. 仅在 `mode_ != target_mode_` 时才执行 `validateModeSelection(target_mode_)`
2. 用普通 `if` 分支替换 duty 限幅里的 `std::fmax/std::fmin`

修复后：

- `BSP_Motor_InjectedAdc1CallbackRateHz` 在 RUN 后恢复稳定
- `BSP_Motor_TickCoreCyclesLast` / `BSP_Motor_TickIsrCyclesLast` 不再长期超预算
- VOFA 发送恢复连续
- HFI / VF / 手动 PWM 三种模式都恢复正常

## 结论

这次问题不是单一原因，而是两个问题叠加：

1. BSP 输出关断方式误伤了 ADC 注入触发链
2. RUN 态 ISR 内存在不该每 tick 重复执行的配置校验逻辑

真正稳定的做法是：

- 把“功率输出开关”和“ADC 注入触发链”明确拆开
- 把模式校验、配置合法性检查尽量前移到用户命令入口
- ISR 内只保留高频必需逻辑

## 当前建议

后续如果再次怀疑控制频率掉速，优先按下面顺序看：

1. `BSP_Motor_InjectedAdc1CallbackCount`
2. `BSP_Motor_InjectedAdc1CallbackRateHz`
3. `BSP_Motor_TickBudgetCycles`
4. `BSP_Motor_TickCoreCyclesLast`
5. `BSP_Motor_TickIsrCyclesLast`
6. `BSP_Motor_TickOverBudgetCount`

判断逻辑：

- `CallbackCount` 不增长：先查 ADC 注入触发链
- `CallbackCount` 增长但 `RateHz` 掉：查 ISR 是否超时
- `RateHz` 正常但控制异常：再查具体算法，如 HFI/SMO/FOC
