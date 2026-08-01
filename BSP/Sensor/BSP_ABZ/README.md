# BSP/Sensor/BSP_ABZ — ABZ 正交编码器 CubeMX 配置指南

## 硬件连接

| 编码器信号 | STM32G474 引脚 | 说明 |
|-----------|---------------|------|
| A         | PC6           | TIM3_CH1, 正交信号 A |
| B         | PC7           | TIM3_CH2, 正交信号 B |
| Z         | PC8           | TIM3_CH3, 可选索引脉冲 (通常每机械转一个) |

## Z 相的作用

`A/B` 两相已经能够提供相对角度、方向和速度；`Z` 相不是连续计数所必需的信号。
`Z` 是每转出现一次的索引/基准脉冲，可用于:

- 上电后转动寻找已知机械零点，建立绝对圈内参考位置。
- 每次经过索引点时校验或修正 A/B 累积计数。
- 编码器安装标定和电角度偏置标定。

若产品只需要相对位置/速度，或暂未实现索引校准逻辑，可以先不连接和不配置 Z。

## CubeMX 配置步骤

### 1. 启用 TIM3 编码器模式

- Pinout → 搜索 PC6 → 选择 **TIM3_CH1**
- Pinout → 搜索 PC7 → 选择 **TIM3_CH2**
- 如需要 Z 索引: Pinout → 搜索 PC8 → 选择 **TIM3_CH3**

### 2. TIM3 Mode 配置

`A/B` 必须使用硬件 Encoder Mode，不能仅将 CH1/CH2 设置为普通输入捕获。

| CubeMX Mode 项 | 不使用 Z | 使用 Z | 说明 |
|----------------|----------|--------|------|
| Slave Mode | Disable | Disable | 由 Encoder Mode 内部配置 |
| Trigger Source | Disable | Disable | 不使用外部触发启动 |
| Clock Source | Disable | Disable | 计数时钟来自 A/B 输入 |
| Channel1 | 由 Encoder Mode 使用 | 由 Encoder Mode 使用 | PC6 / A |
| Channel2 | 由 Encoder Mode 使用 | 由 Encoder Mode 使用 | PC7 / B |
| Channel3 | Disable | **Input Capture direct mode** | PC8 / Z |
| Channel4 | Disable | Disable | 未使用 |
| Combined Channels | **Encoder Mode** | **Encoder Mode** | 必须选择 |

### 3. TIM3 Parameter Settings

| 参数 | 值 | 说明 |
|------|-----|------|
| Encoder Mode | **TI1 and TI2** | A/B 两相计数，x4 正交解码 (`TIM_ENCODERMODE_TI12`) |
| Prescaler | `0` | 不对编码器边沿分频 |
| Counter Mode | Up | 计数方向 |
| Counter Period | `CPR - 1` | 已知分辨率且用于角度闭环时设置；如 1000 PPR → `3999` |
| Internal Clock Division | No Division | 输入数字滤波采样时钟不额外分频 |
| Auto-reload preload | Disable | 周期在运行中不动态修改，无需预装载 |
| Channel1/2 IC Selection | Direct | A/B 直接输入 |
| Channel1/2 Input Polarity | Rising Edge | 配合 TI1 and TI2 进行 x4 正交计数 |
| Channel1/2 Prescaler Division Ratio | No division | 不丢弃边沿 |
| Channel1/2 Input Filter | `0` 起步 | 有毛刺/长线干扰时再逐步增大并验证最高转速 |
| Channel3 Polarity | Rising Edge | 仅配置 Z 时需要；按编码器有效沿调整 |
| Channel3 IC Selection | Direct | 仅配置 Z 时需要 |

### 4. 编码器线数计算

- `PPR` 表示编码器每机械转每相输出的脉冲数。
- 本驱动中的 `CPR` 表示 x4 解码后的每机械转计数数: `CPR = 4 * PPR`。
- TIM3 周期设为: `Counter Period = CPR - 1 = 4 * PPR - 1`。
- 例如 1000 PPR 编码器: `CPR = 4000`，`Counter Period = 3999`。
- 在 [BSP_ABZ.h](BSP_ABZ.h) 中设置 `BSP_ABZ_CPR`；Platform 与 BSP 共用该值，
  必须与实际测得的 `CPR` 相同。

### 5. Counter Period 与未知分辨率处理

`Counter Period` 是 TIM3 自动重装载寄存器 (`ARR`) 的值。编码器模式下，计数器
`CNT` 根据 A/B 方向在 `0` 到 `ARR` 范围内加减计数，越界后回绕:

```text
ARR = Counter Period
每个计数循环的计数个数 = ARR + 1
```

本 BSP 以一机械转的计数值计算角度，因此闭环运行时应让一次计数循环等于一机械转:

```text
Counter Period = CPR - 1
```

如果暂时不知道编码器型号或 `PPR`:

1. 调试阶段先将 `Counter Period` 设置为 `65535`，仅观察原始计数方向和测量计数差，
   此时不要将读到的角度用于闭环控制。
2. 先小幅转动确认让 `CNT` 增加的旋转方向，并在该方向上测量一整圈；必要时测量前
   将 `CNT` 清零。若使用 Z 相，可在一个 Z 脉冲处清零并沿增计数方向转到下一个 Z
   脉冲，测量更可靠。
3. 将一圈结束时的计数值作为 `CPR`。例如一圈计数变化约为 `4000`，则配置
   `Counter Period = 3999`。
4. 在 `BSP_ABZ.h` 中将 `BSP_ABZ_CPR` 设置为该 `CPR`，再验证机械角度和闭环。

上述简化测量方法要求单圈计数小于 `65536` 且测量途中不反向回绕；若单圈计数可能
超出 16 位 TIM3 范围，应使用 32 位定时器或增加溢出累计逻辑。

### 6. 可选: Z 索引处理

如果需要在代码中使用 Z 脉冲:

- TIM3 CH3 配置为 **Input Capture direct mode**。
- NVIC 中使能 **TIM3 global interrupt**。
- 初始化后执行 `HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3)`。
- 在 `HAL_TIM_IC_CaptureCallback()` 中确认 `TIM_CHANNEL_3` 事件，再选择记录索引位置
  或执行 `__HAL_TIM_SET_COUNTER(&htim3, 0)`。

不要在尚未确认 Z 相机械零点与电角度偏置关系前，直接把每个 Z 脉冲当作 FOC 电角度零点。

### 7. GPIO 配置

- PC6/PC7/PC8 选择 TIM3 功能后自动配置为 AF2。
- 推挽编码器输出通常配置 `No pull`；开漏输出才使用匹配电气设计的上拉。
- 编码器输出电平必须兼容 STM32 输入电压；差分 A/B/Z 输出应经过差分接收器。
