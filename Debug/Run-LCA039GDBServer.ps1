[CmdletBinding()]
param(
    [ValidateSet('Cortex-M0', 'LCM32F039')]
    [string]$Device = 'Cortex-M0',

    [ValidateRange(5, 50000)]
    [int]$SpeedKHz = 100,

    [ValidateRange(100, 60000)]
    [int]$TimeoutMs = 5000,

    [string]$SerialNumber = '',

    [string]$JLinkRoot = 'C:\Program Files\SEGGER\JLink_V946'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$server = Join-Path $JLinkRoot 'JLinkGDBServerCL.exe'
if (-not (Test-Path -LiteralPath $server -PathType Leaf)) {
    throw "J-Link GDB Server not found: $server"
}

$logDirectory = Join-Path $PSScriptRoot 'Logs'
New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null

$safeDeviceName = $Device -replace '[^A-Za-z0-9_-]', '_'
$logFile = Join-Path $logDirectory ("GDBServer_{0}_{1}kHz.log" -f $safeDeviceName, $SpeedKHz)

$arguments = @(
    '-device', $Device,
    '-if', 'SWD',
    '-speed', $SpeedKHz.ToString(),
    '-endian', 'little',
    '-singlerun',
    '-nogui',
    '-timeout', $TimeoutMs.ToString(),
    '-log', $logFile
)

if ($SerialNumber) {
    $arguments = @('-USB', $SerialNumber) + $arguments
}

Write-Output "J-Link log: $logFile"
& $server @arguments
exit $LASTEXITCODE
