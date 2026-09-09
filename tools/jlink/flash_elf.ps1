param(
    [string]$Elf = "build\Debug\IF-ABZ.elf",
    [string]$Device = "STM32G474RC",
    [ValidateSet("SWD", "JTAG")]
    [string]$Interface = "SWD",
    [int]$Speed = 4000,
    [string]$OutDir = "tools\jlink\out",
    [switch]$Erase,
    [switch]$Go,
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

function Make-SafeFileStem {
    param([string]$Name)
    return ($Name -replace '[^A-Za-z0-9_.-]', '_')
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

$objcopy = Resolve-Tool `
    -Name "arm-none-eabi-objcopy.exe" `
    -Fallbacks @("C:\SoftWare\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-objcopy.exe")

$symbols = Read-Symbols -NmPath $nm -ElfPath $elfPath -Names @(
    "_sidata",
    "_sdata",
    "_edata",
    "SystemCoreClock",
    "uwTickPrio",
    "uwTickFreq"
)

$stamp = Get-Date -Format "yyyyMMdd-HHmmss"
$hexFile = Join-Path $outPath "$((Make-SafeFileStem ([IO.Path]::GetFileNameWithoutExtension($elfPath))))-$stamp.hex"
$binFile = Join-Path $outPath "$((Make-SafeFileStem ([IO.Path]::GetFileNameWithoutExtension($elfPath))))-$stamp.bin"
$cmdFile = Join-Path $outPath "flash-elf-$stamp.jlink"
$stdoutFile = Join-Path $outPath "flash-elf-$stamp.stdout.txt"
$symbolFile = Join-Path $outPath "flash-elf-$stamp.symbols.txt"

& $objcopy -O ihex $elfPath $hexFile
if ($LASTEXITCODE -ne 0) {
    throw "arm-none-eabi-objcopy failed while generating $hexFile"
}

& $objcopy -O binary $elfPath $binFile
if ($LASTEXITCODE -ne 0) {
    throw "arm-none-eabi-objcopy failed while generating $binFile"
}

$symbolLines = New-Object System.Collections.Generic.List[string]
$symbolLines.Add("ELF: $elfPath")
$symbolLines.Add("HEX: $hexFile")
$symbolLines.Add("BIN: $binFile")
$symbolLines.Add("Generated: $(Get-Date -Format o)")
$symbolLines.Add("")
foreach ($entry in $symbols.GetEnumerator()) {
    $symbolLines.Add(("{0,-20} {1}" -f $entry.Key, (To-Hex32 $entry.Value)))
}
[IO.File]::WriteAllLines($symbolFile, $symbolLines)

$commands = New-Object System.Collections.Generic.List[string]
$commands.Add("si $Interface")
$commands.Add("speed $Speed")
$commands.Add("device $Device")
$commands.Add("connect")
$commands.Add("w4 0xE0002008 0")
$commands.Add("w4 0xE000200C 0")
$commands.Add("w4 0xE0002010 0")
$commands.Add("w4 0xE0002014 0")
$commands.Add("w4 0xE0002018 0")
$commands.Add("w4 0xE000201C 0")
$commands.Add("w4 0xE0002000 0")
$commands.Add("r")
$commands.Add("h")
if ($Erase) {
    $commands.Add("erase")
    $commands.Add("r")
    $commands.Add("h")
}
$commands.Add("loadbin $binFile, 0x08000000")

if ($symbols.Contains("_sidata")) {
    $commands.Add("mem32 $((To-Hex32 $symbols["_sidata"])) 180")
}

if ($Go) {
    $commands.Add("w4 0xE0002008 0")
    $commands.Add("w4 0xE000200C 0")
    $commands.Add("w4 0xE0002010 0")
    $commands.Add("w4 0xE0002014 0")
    $commands.Add("w4 0xE0002018 0")
    $commands.Add("w4 0xE000201C 0")
    $commands.Add("w4 0xE0002000 0")
    $commands.Add("r")
    $commands.Add("g")
} else {
    $commands.Add("r")
    $commands.Add("h")
}

$commands.Add("q")
[IO.File]::WriteAllLines($cmdFile, $commands)

Write-Host "Generated:"
Write-Host "  Symbols: $symbolFile"
Write-Host "  J-Link script: $cmdFile"
Write-Host "  Output: $stdoutFile"
Write-Host ""

if (-not $Run) {
    Write-Host "Dry-run only. Re-run with -Run to flash the complete ELF."
    Write-Host "By default the target is reset and halted after flashing; pass -Go only when the power stage is safe."
    Write-Host "Example:"
    Write-Host "  powershell -ExecutionPolicy Bypass -File tools\jlink\flash_elf.ps1 -Run"
    exit 0
}

Write-Host "Flashing ELF with J-Link. Target will remain halted unless -Go is set."
& $jlink -NoGui 1 -ExitOnError 1 -Device $Device -If $Interface -Speed $Speed -CommanderScript $cmdFile 2>&1 |
    Tee-Object -FilePath $stdoutFile

if ($LASTEXITCODE -ne 0) {
    throw "J-Link Commander failed. See $stdoutFile"
}

Write-Host ""
Write-Host "Done. Check that the flash dump at _sidata is not erased after 0x08010000:"
Write-Host "  $stdoutFile"
