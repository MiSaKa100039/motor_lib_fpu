# Motor Safety

本目录用于放置可跨 profile 复用的安全保护模块。

建议拆分方向：

- `Basic`：过流、过压、欠压、过温、传感器基础健康检查。
- profile 内部：与启动链路或观测器组合强相关的故障处理。

约束：

- 安全模块可以输出 fault/event，但不应该到处直接改顶层 FSM。
- 顶层状态切换应由 profile 的 Manager/FSM 统一收口。
- ISR 快速路径中只做必要检查，复杂恢复策略放到 profile 层。
