# LCA039 J-Link/Ozone 使用说明

工程提供两条彼此独立的调试路径：

- `LCA039_Attach.jdebug`：通用 Cortex-M0，只连接、停止和读写内存，不执行 Flash 下载。
- `LCA039_Download.jdebug`：自定义 `LCM32F039` Device，使用厂商 Main Flash FLM 下载和调试。
- `LCA039_TargetTest.jdebug`：下载 `build/TargetTest/lca039_template.elf` 并观察独立测试状态。

排查问题时必须先验证 Attach。只有 Attach 能稳定连接后，才测试 Flash Loader。

## 当前验证结论

在 J-Link Software V9.46 下已经通过完整 DLL 日志确认：

- 用户目录中的 `Devices.xml` 能被找到并成功解析。
- `LCM32F039xx_64K_MainFlash.FLM` 的绝对路径正确。
- J-Link 能从 FLM 解析 512 Byte 扇区，并识别 `Init`、`UnInit`、`EraseChip`、`EraseSector` 和 `ProgramPage`。
- 当前失败发生在 `JLINK_Connect()`，不是 XML 路径或 FLM 格式错误。

厂商历史 Keil 配置使用通用 `Cortex-M0`，并由 Keil 单独加载同一个 FLM。历史日志中的成功基线为：

```text
J-Link S/N: 59605930
Hardware: V9.60
SWD speed: 5000 kHz
SW-DP ID: 0x0BB11477
CPUID: 0x410CC200
Core: Cortex-M0 r0p0
```

当前测试探针为另一台设备：

```text
J-Link S/N: 60978909
Hardware: V9.20
```

V9.46 日志还报告 `Bootloader: (FW returned invalid version)`。这是需要交叉验证的探针固件异常线索，不单独用于判定探针损坏或真伪。

因此需要对探针、转接线和目标板做交叉验证，不能用历史成功记录直接证明当前探针链路正常。

## 安装 J-Link Device

从模板根目录执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\Debug\Install-LCM32F039JLinkDevice.ps1
```

脚本安装以下文件：

```text
%APPDATA%\SEGGER\JLinkDevices\LCMicroelectronics\LCM32F039\
|- Devices.xml
`- LCM32F039xx_64K_MainFlash.FLM
```

安装或更新后必须完全退出 Ozone、J-Link GDB Server 和 J-Link Commander，再重新启动。

FLM SHA-256：

```text
CA8ED374C9106E7DE8AC381D7BE04994E5417D838DD449FCA7056AABA50A114A
```

该算法只覆盖 `0x08000000` 开始的 64 KiB Main Flash，不支持 Option Byte 写入。

## 只连接测试

以下脚本不执行 Flash 下载。默认使用通用 Cortex-M0、SWD 100 kHz：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\Debug\Test-LCA039JLinkConnection.ps1
```

指定当前探针：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\Debug\Test-LCA039JLinkConnection.ps1 `
  -SerialNumber 60978909
```

成功时日志应包含：

```text
Found SW-DP with ID 0x0BB11477
CPUID register: 0x410CC200
Found Cortex-M0 r0p0
```

日志写入 `Debug/Logs/`，该目录不会加入版本控制。

## GDB Server

CLion 或独立 GDB 调试使用：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\Debug\Run-LCA039GDBServer.ps1 `
  -Device Cortex-M0 `
  -SerialNumber 60978909
```

Attach 成功后，再把 `-Device` 改成 `LCM32F039` 测试下载路径。

## Ozone

先生成 Debug ELF 和日志目录：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

先打开 `LCA039_Attach.jdebug`。它使用 Cortex-M0、SWD 100 kHz，并覆盖空的 `TargetDownload()`，不会修改目标 Flash。

Attach 成功后再打开 `LCA039_Download.jdebug`。两个工程都会把完整 J-Link DLL 日志写到 `build/Debug/`：

- `LCA039_Attach_Ozone_JLink.log`
- `LCA039_Download_Ozone_JLink.log`

目标测试先执行 `cmake --preset TargetTest` 和 `cmake --build --preset TargetTest`，再打开 `LCA039_TargetTest.jdebug`。安全用例可以直接运行；功率级用例还需要构建期开关和运行时 arm key，具体步骤见 `docs/MotorFramework.md`。

## QFN48 评估板调试接口

官方 QFN48 评估板 P0 网络为：

| P0 | 信号 | MCU |
|---|---|---|
| 1 | SWC/SWCLK | PA14，芯片 20 脚 |
| 2 | SWD/SWDIO | PA13，芯片 21 脚 |
| 3 | VDD/VTref | VDD |
| 4 | GND | GND |

P0 没有 NRST。需要 connect-under-reset 时，应将 J-Link RESET 单独接到 PF2/NRST（芯片 8 脚），或者在连接重试期间给目标板重新上电。

## 交叉验证

1. 用当前 J-Link `60978909` 连接一块已知正常的 STM32。
2. 如果历史 J-Link `59605930` 仍可用，用它连接当前 LCA039。
3. 当前探针连 STM32 也失败：优先检查探针、固件、转接线和接口方向。
4. 当前探针只在 LCA039 失败：增加 NRST 并尝试 connect-under-reset 或上电窗口连接。

## 卸载

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\Debug\Uninstall-LCM32F039JLinkDevice.ps1
```

卸载脚本只删除上述两个已知文件，并且只在设备目录为空时删除该目录。
