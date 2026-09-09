# 方波 HFI 双角误差问题与修改方案

## 现象

当前方波 HFI 调试中出现了一个关键现象：

- 手动小范围左右拨动转子时，`observer_pll_error` 能明显随方向变化。
- 连续转动多圈时，`angle_elec_observer` 不能连续跟随，只出现小幅锯齿。
- 手转一圈机械角时，`observer_pll_error` 大约出现 14 次跳变。

如果电机极对数为 7，则：

```text
1 机械圈 = 7 电角圈
14 次跳变 = 2 * pole_pairs
```

这说明当前方波 HFI 前端更像是在输出带 `2 * theta_e` 周期性的凸极轴信息，而不是唯一的电角度 `theta_e`。

## 问题判断

这个问题不是简单的 `kp/ki`、`speed_limit` 或 LPF 参数问题。

已经验证过：

- 增大 `hfi_square_lpf_cutoff_ratio` 后，连续转动仍然只出现小幅锯齿。
- 关闭 `hfi_pll_speed_limit_rad_s` 后，连续跟随能力没有明显变化。
- `observer_pll_error` 在局部小范围扰动时有方向响应，但连续转动时呈现周期性重复。

因此问题重点不在高频分量提取本身，而在“如何把高频解调结果转换成送入 PLL 的误差”。

更准确地说：

```text
二阶差分提取高频分量: 方向基本对
同步整流得到 demod_alpha / demod_beta: 有可用角度信息
当前 error_sample 构造方式: 按单角 theta_e 建模，和实际 2theta_e 信息不匹配
```

## 当前代码的问题点

当前方波路径在 `Lib/Motor/Observer/Observer_HFI.h` 的 `updateSquareDemod()` 中使用如下误差：

```cpp
const float st = sinf(angle_est_);
const float ct = cosf(angle_est_);
const float error_sample = demod_alpha * st - demod_beta * ct;
```

这个公式假设 `demod_alpha / demod_beta` 直接对应单角 `theta_e`。

但从实测“一机械圈 14 次跳变”看，它更像对应双角信息：

```text
demod_alpha / demod_beta -> 2 * theta_e
```

所以直接用单角 PLL 会出现：

- 局部小范围可跟随。
- 跨过半个电角周期后误差信号重复。
- PLL 只能在局部支路附近摆动，不能全局连续跟踪。

## 修改方案

保留现有方波注入、二阶差分、高频同步整流流程，只修改方波模式下 `error_sample` 的构造方式。

建议把单角投影误差改成双角误差：

```cpp
const float meas_2theta = atan2f(demod_beta, demod_alpha);
const float est_2theta = 2.0f * angle_est_;
const float error_sample =
    0.5f * wrapSignedAngle(meas_2theta - est_2theta);
```

然后继续沿用现有的低通、PLL、速度限幅、积分限幅：

```cpp
demod_error_lpf_ += alpha * (error_sample - demod_error_lpf_);
demod_signal_lpf_ += alpha * (signal_sample - demod_signal_lpf_);
updatePll(demod_error_lpf_, dt);
```

还需要新增一个有符号角度归一化函数：

```cpp
static float wrapSignedAngle(float angle)
{
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}
```

其中 `PI` 可以定义为：

```cpp
static constexpr float PI = 3.14159265358979323846f;
```

## 方向极性

修改成双角误差后，如果发现：

- 正 `kp` 时仍然发散。
- 负 `kp` 时反而局部稳定。

不要长期依赖负 `kp`。

更好的做法是增加一个方波 HFI 极性参数，例如：

```cpp
float hfi_square_error_sign = 1.0f;
```

然后：

```cpp
const float error_sample =
    hfi_square_error_sign * 0.5f *
    wrapSignedAngle(meas_2theta - est_2theta);
```

这样 `kp` 仍然保持正值，极性由配置参数显式控制。

## 修改后的验证步骤

修改完成后，不要马上调 `kp/ki`。先验证前端和误差拓扑是否修正。

### 1. 开环验证

设置：

```cpp
hfi_pll_kp = 0.0f;
hfi_pll_ki = 0.0f;
```

观察：

- `observer_pll_error`
- `observer_signal_level`
- `sin(angle_elec_observer)`
- `cos(angle_elec_observer)`

用手慢速转动一机械圈。

期望现象：

- `observer_pll_error` 不再呈现明显的 14 次尖锐重复跳变。
- error 可以随转子位置平滑变化。
- `observer_signal_level` 在大部分角度保持可观测，不应周期性塌陷到很低。

### 2. P-only 验证

设置：

```cpp
hfi_pll_ki = 0.0f;
hfi_pll_speed_limit_rad_s = 0.0f; // 临时关闭速度限幅用于判断
```

逐步增加：

```cpp
hfi_pll_kp = 0.5f -> 1.0f -> 2.0f -> 5.0f
```

期望现象：

- 小范围左右拨动时，`sin/cos` 连续跟随。
- 慢速连续转动时，`sin/cos` 有连续趋势，而不是只出现局部小锯齿。
- 如果方向反了，优先调整 `hfi_square_error_sign`，不要用负 `kp` 当最终方案。

### 3. 限幅恢复

确认 P-only 可以跟随后，再恢复保护：

```cpp
hfi_pll_speed_limit_rad_s = 20.0f ~ 100.0f;
hfi_pll_integral_limit_rad_s = 5.0f;
```

判断：

- 如果角度被明显拴住，逐步放大 `hfi_pll_speed_limit_rad_s`。
- 如果一放大就乱跳，说明 error 仍然有尖峰，需要回到 LPF 或前端检查。

### 4. 最后加入积分

只有当 `kp` 单独能稳定跟随后，才加入很小的 `ki`：

```cpp
hfi_pll_ki = 0.5f -> 1.0f -> 2.0f
```

判断：

- `ki` 只是用于消除静态偏差。
- 如果加 `ki` 后静止自转，先减小 `ki` 或收紧 `hfi_pll_integral_limit_rad_s`。

## 如何判断问题已经解决

问题解决的标志不是“静止 error 很小”，而是下面几个条件同时满足：

- 一机械圈内不再出现固定的 `2 * pole_pairs` 次误差跳变。
- `kp > 0, ki = 0` 时，慢速连续转动能让 `sin/cos` 连续跟随。
- 关闭 `speed_limit` 和打开合理 `speed_limit` 时，差异只体现在保护强弱，不再决定是否能跟随。
- 调 `hfi_square_lpf_cutoff_ratio` 只影响噪声和响应速度，不再改变“能不能连续跟”的本质。

## 仍然存在的物理限制

HFI 基于凸极效应时，天然更容易得到“轴”信息，而不是永远唯一的磁极极性。

双角 PLL 可以解决当前单角 PLL 对 `2theta_e` 信息建模不匹配的问题，但仍可能存在 `theta` 与 `theta + pi` 的极性歧义。

后续如果需要绝对极性，可以考虑：

- 启动对齐给定初始支路。
- 注入额外极性判别脉冲。
- 用 IF 启动方向或 SMO 接管后消除歧义。

## 结论

当前问题不是高频二阶差分提取一定错误，而是方波 HFI 解调后送入 PLL 的误差模型不匹配。

实测一机械圈约 `2 * pole_pairs` 次跳变，说明前端输出带双角周期性。代码应从单角误差：

```cpp
demod_alpha * sin(theta) - demod_beta * cos(theta)
```

改为双角误差：

```cpp
0.5f * wrapSignedAngle(atan2(demod_beta, demod_alpha) - 2 * theta)
```

完成这个修改后，再重新从 `kp=0, ki=0` 开始验证前端，再进入 `kp`、限幅、`ki` 的调参流程。
