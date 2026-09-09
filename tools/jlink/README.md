# J-Link TIM3 ABZ diagnostics

This folder contains read-only J-Link Commander helpers for the TIM3 ABZ
startup issue.

The helper scripts default to `STM32G474RC` for J-Link device selection. On
this setup SEGGER's `STM32G474RB` flash loader only covered the first 64KB,
leaving the ELF `.data` load image above `0x08010000` erased.

Safety first:

- Do not halt the MCU while the power stage can actively drive the motor.
- Before running a halt-based dump, keep the driver EN low or remove bus power.
- Close Ozone before running the dump, because both tools use the same J-Link.
- The script does not flash firmware and does not write RAM or registers.

Run a dry-run first:

```powershell
powershell -ExecutionPolicy Bypass -File tools\jlink\tim3_abz_dump.ps1
```

After the power stage is safe:

```powershell
powershell -ExecutionPolicy Bypass -File tools\jlink\tim3_abz_dump.ps1 -Run
```

To halt, dump, and then continue running:

```powershell
powershell -ExecutionPolicy Bypass -File tools\jlink\tim3_abz_dump.ps1 -Run -Resume
```

To flash the complete ELF without starting the target:

```powershell
powershell -ExecutionPolicy Bypass -File tools\jlink\flash_elf.ps1 -Run
```

If SEGGER reports that flash contents already match while addresses above
`0x08010000` still read as `0xFFFFFFFF`, force a full chip erase first:

```powershell
powershell -ExecutionPolicy Bypass -File tools\jlink\flash_elf.ps1 -Run -Erase
```

Only add `-Go` when the power stage is safe and you intentionally want the
firmware to run after flashing.

`flash_elf.ps1` converts the ELF to HEX and BIN, then flashes the continuous
BIN at `0x08000000`. This avoids SEGGER `loadfile` ignoring the `.data` load
image that crosses `0x08010000` on this target.

When `-Go` is used, `flash_elf.ps1` also clears Cortex-M FPB hardware
breakpoints before running. This avoids an Ozone "break at main" breakpoint
being left armed after the standalone J-Link flash flow disconnects.

The script creates timestamped files under `tools\jlink\out`:

- `*.symbols.txt`: ELF symbol addresses used for the dump.
- `*.jlink`: generated J-Link Commander script.
- `*.stdout.txt`: raw J-Link output.

Useful `BSP_ABZ_Debug_Stage` values:

```text
10   BSP_ABZ_Init entered
20   ABZ context cleared and status initialized
30   htim3 bound; about to read TIM3->CNT when counter read is enabled
40   PrepareTimerIrq entered
50   StartEncoderAB about to run
500  StartEncoderAB entered
501  TIM handle looks non-null
510  HAL_TIM_Encoder_Start(CH1) about to run
511  HAL_TIM_Encoder_Start(CH1) returned
512  HAL_TIM_Encoder_Start(CH2) about to run
513  HAL_TIM_Encoder_Start(CH2) returned
51   StartEncoderAB returned to BSP_ABZ_Init
70   A/B encoder start succeeded
90   BSP_ABZ_Init completed
520  Direct-register start path entered
530  Direct-register path about to enable CC1/CC2
540  Direct-register path about to set CEN
541  Direct-register start completed
```

First interpretation pass:

- If the core is in `HardFault_Handler`, inspect `CFSR/HFSR/BFAR/MMFAR` in
  the dump and the last `BSP_ABZ_Debug_Stage`.
- If `TIM3->CR1.CEN` is clear or `TIM3->CCER.CC1E/CC2E` are clear after
  stage 70/90, the encoder start did not leave the timer running.
- If `GPIOC.MODER` and `GPIOC.AFRL/AFRH` do not show PC6/PC7/PC8 in AF2,
  the pin mux is not what TIM3 expects.
- If `RCC_APB1ENR1.TIM3EN` is clear, TIM3 was not clocked.
- If the PC is inside `Error_Handler` before `USER_INIT_STAGE` changes from
  zero, inspect the RCC/PWR/FLASH/SysTick blocks first; TIM3 has not run yet.
