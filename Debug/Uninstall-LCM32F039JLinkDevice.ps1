[CmdletBinding(SupportsShouldProcess)]
param(
    [string]$DestinationRoot = (Join-Path $env:APPDATA 'SEGGER\JLinkDevices')
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$deviceDirectory = Join-Path $DestinationRoot 'LCMicroelectronics\LCM32F039'
$knownFiles = @('Devices.xml', 'LCM32F039xx_64K_MainFlash.FLM')

foreach ($fileName in $knownFiles) {
    $targetFile = Join-Path $deviceDirectory $fileName
    if ((Test-Path -LiteralPath $targetFile -PathType Leaf) -and
        $PSCmdlet.ShouldProcess($targetFile, 'Remove LCM32F039 J-Link device file')) {
        Remove-Item -LiteralPath $targetFile -Force
    }
}

if (Test-Path -LiteralPath $deviceDirectory -PathType Container) {
    $remainingFiles = @(Get-ChildItem -LiteralPath $deviceDirectory -Force)
    if (($remainingFiles.Count -eq 0) -and
        $PSCmdlet.ShouldProcess($deviceDirectory, 'Remove empty LCM32F039 device directory')) {
        Remove-Item -LiteralPath $deviceDirectory
    }
}
