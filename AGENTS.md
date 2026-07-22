# 全局 Codex 协作规则

## 语言与编码

- 所有源码文件按 UTF-8 无 BOM 处理。
- 除非明确要求转换编码，否则保留现有文件编码。
- 除非明确要求，否则不要把 GBK/ANSI 文件转换为 UTF-8。
- `.c`、`.h`、`.cpp`、`.hpp`、`.s`、`.S`、`.ld`、`.cmake`、`CMakeLists.txt` 中的源码注释默认使用中文。
- 注释应使用简洁的中文技术表述，重点说明安全约束、时序假设、硬件限制和非显然控制逻辑。
- 仅在保留第三方代码、厂家代码、协议名、API 名，或英文术语明显更清晰时允许英文注释。
- Markdown 文档，例如 `README.md`、`docs/*.md`、设计说明，可以使用中文说明。
- 不要创建中文文件名、中文目录名、中文宏名或中文变量名。

## 嵌入式 C/C++ 规则

- 不做大范围无关重构。
- 修改公共 API 时必须说明兼容性影响。
- 电机控制代码需要保持 ISR、HAL、控制环、Manager、FSM、用户 API 层次清晰。
- 除非明确要求，不引入动态分配。
- ISR 或快速控制环内避免阻塞操作。
- STM32 工程中，除非必要，不修改 CubeMX 生成区域。
- 修改时序敏感代码时，需要说明对控制环或中断时序的影响。

## 注释规则

- 优先使用简洁的中文技术注释。
- 不添加“i 自增”“设置变量”这类显而易见的注释。
- 注释用于说明安全约束、时序假设、硬件限制和非显然控制逻辑。
- 项目自有源码中新增加的注释默认使用中文。


## P2 参数命名与配置语义约定

### 电机物理参数: *_measured vs *_identified (纯值链 0=未知)

- *_measured: 用户用仪器(万用表/LCR/反拖台架)测量, =0 表示未测量;
- *_identified: 库内自动辨识(RL 辨识/SMO 收敛)写入, RAM 维护, 关机丢失;
  - SMO 应用时仅 alid && speed > hfi_to_smo_rpm + hysteresis_rpm && valid_ticks >= 2*convergence_ticks 才更新 ke_v_per_rad_s_identified;
  - 非 SMO 应用或低速段保留 measured 不变, 工业做法由 VCU 温度补偿。
- helper s_effective()/ls_effective()/ke_effective_for_limit(): identified>0 优先, 否则 measured, 否则 0(被 limiter 跳过);
- 不再使用独立 bool 开关(ke_use_for_limit 等), 物理量 0=非法与 PID kp/ki=0 合法的语义差异天然区分。

### PID kp/ki: 用 auto_derive_current_pid bool 开关

- uto_derive_current_pid=true 时 init/applyAutoDeriveIfEnabled 用 R/L 推导覆盖 PIDParam.kp/ki;
- alse 时永远用手填 (因为 kp=0 纯 I 控制器是合法值, 不能用 0=未知哨兵)。

### 顺逆风 FlyingStartMode 三态语义

- DISABLED: 关闭飞启。SENSOR 启动安全。IF/HFI 启动需配 IF 静止保证机制 (if_stationary_guarantee != NONE);
- PHASE_VOLTAGE: 相电压反电动势 PLL 重构角度 → seedAngle SMO → SMO_ONLY 接管。强制要求 has_phase_voltage==true + BuildCfg MOTOR_BUILD_HAS_PHASE_VOLTAGE + MOTOR_BUILD_ENABLE_FLYING_START;
- AUTO_DETECT: P3 占位, ConfigCheck 直接拒未实现。

## P3.B 修订符: rotor_stationary_guarantee 与 flying_start_mode 决策矩阵

### 字段名变更 (与启动源解耦)

- 旧名 if_stationary_guarantee → 新名 otor_stationary_guarantee (放 cfg.observer)
- 旧名 if_explicit_bypass_confirmed → 新名 otor_explicit_bypass_confirmed
- 枚举 IFStationaryGuarantee → RotorStationaryGuarantee (值不变)

### 校验决策矩阵 (ConfigCheck 行为)

| flying_start_mode \ startup_source | SENSOR       | IF/HFI                                           |
|------------------------------------|--------------|---------------------------------------------------|
| DISABLED                           | 通过 (不查)  | rotor_stationary_guarantee != NONE 必填           |
|                                    |              |   - LOW_SIDE_BRAKE → HAL pwm_brake_lowside 校验   |
|                                    |              |   - PHASE_VOLTAGE_FLYING → has_phase_voltage 校验  |
|                                    |              |   - EXPLICIT_BYPASS → rotor_explicit_bypass_confirmed=true |
| PHASE_VOLTAGE                      | 强制 has_phase_voltage + BuildCfg PHASE_VOLTAGE + FLYING_START                            |
| AUTO_DETECT                        | P3 占位 → ConfigCheck 直接拒未实现                                                         |

### 飞启检测阈值

- cfg.observer.flying_start_speed_threshold_rpm 默认 100.0 RPM, 可调 (电机极对数不同对反电势信号阈值需不同)

### 用户操作指南

- "用户明确承诺转子不会被外力拉动" = lying_start_mode=DISABLED + otor_stationary_guarantee=EXPLICIT_BYPASS + otor_explicit_bypass_confirmed=true
- "扇叶可能被风吹转" = lying_start_mode=PHASE_VOLTAGE (需相电压采样硬件 + BuildCfg 开 FLYING_START + PHASE_VOLTAGE)