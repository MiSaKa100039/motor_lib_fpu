[CmdletBinding()]
param(
    [ValidateSet('Cortex-M0', 'LCM32F039')]
    [string]$Device = 'Cortex-M0',

    [ValidateRange(5, 50000)]
    [int]$SpeedKHz = 100,

    [string]$SerialNumber = '',

    [string]$JLinkRoot = 'C:\Program Files\SEGGER\JLink_V946'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$commander = Join-Path $JLinkRoot 'JLink.exe'
if (-not (Test-Path -LiteralPath $commander -PathType Leaf)) {
    throw "J-Link Commander not found: $commander"
}

$logDirectory = Join-Path $PSScriptRoot 'Logs'
New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null

$safeDeviceName = $Device -replace '[^A-Za-z0-9_-]', '_'
$logFile = Join-Path $logDirectory ("JLink_Connect_{0}_{1}kHz.log" -f $safeDeviceName, $SpeedKHz)
$commandFile = Join-Path ([System.IO.Path]::GetTempPath()) ("lca039-jlink-{0}.jlink" -f ([guid]::NewGuid().ToString('N')))

$commands = @(
    'connect'
    'halt'
    'mem32 0xE000ED00 1'
    'exit'
)

[System.IO.File]::WriteAllLines($commandFile, $commands, [System.Text.UTF8Encoding]::new($false))

$arguments = @(
    '-Device', $Device,
    '-If', 'SWD',
    '-Speed', $SpeedKHz.ToString(),
    '-AutoConnect', '1',
    '-ExitOnError', '1',
    '-NoGui', '1',
    '-Log', $logFile,
    '-CommandFile', $commandFile
)

if ($SerialNumber) {
    $arguments = @('-USB', $SerialNumber) + $arguments
}

try {
    & $commander @arguments
    $exitCode = $LASTEXITCODE
}
finally {
    if (Test-Path -LiteralPath $commandFile) {
        Remove-Item -LiteralPath $commandFile -Force
    }
}

Write-Output "J-Link log: $logFile"
exit $exitCode
