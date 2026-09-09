param(
    [string]$Elf = "build\Debug\IF-ABZ.elf",
    [string]$Device = "STM32G474RC",
    [ValidateSet("SWD", "JTAG")]
    [string]$Interface = "SWD",
    [int]$Speed = 4000,
    [string]$OutDir = "tools\jlink\out",
    [switch]$NoHalt,
    [switch]$Resume,
    [switch]$Run
)

$ErrorActionPreference = "Stop"

function Resolve-Tool {
    param(
        [string]$Name,
        [string[]]$Fallbacks
    )

    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) {
        return $cmd.Source
    }

    foreach ($candidate in $Fallbacks) {
        if (Test-Path -LiteralPath $candidate) {
            return $candidate
        }
    }

    throw "Tool not found: $Name"
}

function To-Hex32 {
    param([UInt64]$Value)
    return ("0x{0:X8}" -f $Value)
}

function Read-Symbols {
    param(
        [string]$NmPath,
        [string]$ElfPath,
        [string[]]$Names
    )

    $nmLines = & $NmPath -an $ElfPath
    if ($LASTEXITCODE -ne 0) {
        throw "arm-none-eabi-nm failed for $ElfPath"
    }

    $result = [ordered]@{}
    foreach ($name in $Names) {
        $escaped = [regex]::Escape($name)
        $match = $nmLines | Where-Object { $_ -match "^\s*([0-9A-Fa-f]+)\s+\w\s+$escaped$" } | Select-Object -First 1
        if ($match -and $match -match "^\s*([0-9A-Fa-f]+)") {
            $result[$name] = [Convert]::ToUInt64($Matches[1], 16)
        }
    }
    return $result
}

$repoRoot = (Resolve-Path -LiteralPath ".").Path
$elfPath = (Resolve-Path -LiteralPath $Elf).Path
$outPath = Join-Path $repoRoot $OutDir
New-Item -ItemType Directory -Force -Path $outPath | Out-Null

$jlink = Resolve-Tool `
    -Name "JLink.exe" `
    -Fallbacks @("C:\Program Files\SEGGER\JLink_V794\JLink.exe")

$nm = Resolve-Tool `
    -Name "arm-none-eabi-nm.exe" `
    -Fallbacks @("C:\SoftWare\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-nm.exe")

$symbolsToRead = @(
    "BSP_ABZ_Debug_InitStatus",
    "BSP_ABZ_Debug_ZStatus",
    "BSP_ABZ_Debug_ZPulseCount",
    "BSP_ABZ_Debug_ZLastCount",
    "BSP_ABZ_Debug_ZLastTick",
    "BSP_ABZ_Debug_RawCount",
    "BSP_ABZ_Debug_Stage",
    "CMD",
    "USER_INIT_STAGE",
    "USER_LOOP_COUNT",
    "USER_LAST_CMD",
    "USER_LAST_RESULT",
    "SystemCoreClock",
    "uwTickPrio",
    "uwTickFreq",
    "uwTick",
    "Fault_Debug_CFSR",
    "Fault_Debug_HFSR",
    "Fault_Debug_DFSR",
    "Fault_Debug_AFSR",
    "Fault_Debug_EXC_RETURN",
    "Fault_Debug_SP",
    "Fault_Debug_R0",
    "Fault_Debug_PC",
    "_sidata",
    "_sdata",
    "_edata",
    "htim3",
    "_ZL5g_abz",
    "_ZL17g_abz_initialized"
)

$symbols = Read-Symbols -NmPath $nm -ElfPath $elfPath -Names $symbolsToRead

$stamp = Get-Date -Format "yyyyMMdd-HHmmss"
$cmdFile = Join-Path $outPath "tim3-abz-$stamp.jlink"
$stdoutFile = Join-Path $outPath "tim3-abz-$stamp.stdout.txt"
$symbolFile = Join-Path $outPath "tim3-abz-$stamp.symbols.txt"

$symbolLines = New-Object System.Collections.Generic.List[string]
$symbolLines.Add("ELF: $elfPath")
$symbolLines.Add("Generated: $(Get-Date -Format o)")
$symbolLines.Add("")
foreach ($entry in $symbols.GetEnumerator()) {
    $symbolLines.Add(("{0,-32} {1}" -f $entry.Key, (To-Hex32 $entry.Value)))
}
[IO.File]::WriteAllLines($symbolFile, $symbolLines)

$commands = New-Object System.Collections.Generic.List[string]
$commands.Add("si $Interface")
$commands.Add("speed $Speed")
$commands.Add("device $Device")
$commands.Add("connect")

if (-not $NoHalt) {
    $commands.Add("h")
    $commands.Add("regs")
} else {
    # Keep the generated J-Link script command-only; comment support varies.
}

$commands.Add("mem32 0xE000ED04 1")
$commands.Add("mem32 0xE000ED24 1")
$commands.Add("mem32 0xE000ED28 6")
$commands.Add("mem32 0xE000EDF0 1")
$commands.Add("mem32 0xE000EDFC 1")

$commands.Add("mem32 0xE000E010 4")

$commands.Add("mem32 0xE0002000 16")

$commands.Add("mem32 0xE000E100 1")
$commands.Add("mem32 0xE000E200 1")
$commands.Add("mem32 0xE000E300 1")
$commands.Add("mem32 0xE000E400 8")

$commands.Add("mem32 0x1FFF75E0 4")

$commands.Add("mem32 0x40000400 32")

$commands.Add("mem32 0x40021000 48")

$commands.Add("mem32 0x40007000 32")

$commands.Add("mem32 0x40022000 16")

$commands.Add("mem32 0x48000800 16")

if ($symbols.Contains("BSP_ABZ_Debug_InitStatus")) {
    $commands.Add("mem32 $((To-Hex32 $symbols["BSP_ABZ_Debug_InitStatus"])) 2")
}

if ($symbols.Contains("BSP_ABZ_Debug_ZPulseCount")) {
    $commands.Add("mem32 $((To-Hex32 $symbols["BSP_ABZ_Debug_ZPulseCount"])) 5")
}

if ($symbols.Contains("SystemCoreClock")) {
    $commands.Add("mem32 $((To-Hex32 $symbols["SystemCoreClock"])) 4")
}

if ($symbols.Contains("Fault_Debug_CFSR")) {
    $commands.Add("mem32 $((To-Hex32 $symbols["Fault_Debug_CFSR"])) 18")
}

if ($symbols.Contains("_sidata")) {
    $commands.Add("mem32 $((To-Hex32 $symbols["_sidata"])) 180")
}

if ($symbols.Contains("htim3")) {
    $commands.Add("mem32 $((To-Hex32 $symbols["htim3"])) 32")
}

if ($symbols.Contains("_ZL5g_abz")) {
    $commands.Add("mem32 $((To-Hex32 $symbols["_ZL5g_abz"])) 12")
}

if ($symbols.Contains("_ZL17g_abz_initialized")) {
    $addr = $symbols["_ZL17g_abz_initialized"] -band 0xFFFFFFFC
    $commands.Add("mem8 $((To-Hex32 $addr)) 16")
}

if ($symbols.Contains("USER_LOOP_COUNT")) {
    $addr = $symbols["USER_LOOP_COUNT"] -band 0xFFFFFFFC
    $commands.Add("mem32 $((To-Hex32 $addr)) 4")
}

if ($Resume -and -not $NoHalt) {
    $commands.Add("g")
}

$commands.Add("q")
[IO.File]::WriteAllLines($cmdFile, $commands)

Write-Host "Generated:"
Write-Host "  Symbols: $symbolFile"
Write-Host "  J-Link script: $cmdFile"
Write-Host "  Output: $stdoutFile"
Write-Host ""

if (-not $Run) {
    Write-Host "Dry-run only. Re-run with -Run after the power stage is safe and Ozone is disconnected."
    Write-Host "Example:"
    Write-Host "  powershell -ExecutionPolicy Bypass -File tools\jlink\tim3_abz_dump.ps1 -Run"
    exit 0
}

Write-Host "Connecting with J-Link. This may halt the MCU unless -NoHalt is set."
if ($Resume -and -not $NoHalt) {
    Write-Host "Target will be resumed after the dump."
}
& $jlink -NoGui 1 -ExitOnError 1 -Device $Device -If $Interface -Speed $Speed -CommanderScript $cmdFile 2>&1 |
    Tee-Object -FilePath $stdoutFile

if ($LASTEXITCODE -ne 0) {
    throw "J-Link Commander failed. See $stdoutFile"
}

Write-Host ""
Write-Host "Done. Attach these files to the debug note:"
Write-Host "  $symbolFile"
Write-Host "  $stdoutFile"
