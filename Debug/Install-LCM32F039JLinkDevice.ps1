[CmdletBinding(SupportsShouldProcess)]
param(
    [string]$DestinationRoot = (Join-Path $env:APPDATA 'SEGGER\JLinkDevices')
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$relativeDevicePath = 'LCMicroelectronics\LCM32F039'
$sourceDirectory = Join-Path $PSScriptRoot (Join-Path 'JLinkDevices' $relativeDevicePath)
$destinationDirectory = Join-Path $DestinationRoot $relativeDevicePath
$requiredFiles = @('Devices.xml', 'LCM32F039xx_64K_MainFlash.FLM')

foreach ($fileName in $requiredFiles) {
    $sourceFile = Join-Path $sourceDirectory $fileName
    if (-not (Test-Path -LiteralPath $sourceFile -PathType Leaf)) {
        throw "Missing J-Link device file: $sourceFile"
    }
}

if ($PSCmdlet.ShouldProcess($destinationDirectory, 'Install LCM32F039 J-Link device support')) {
    New-Item -ItemType Directory -Path $destinationDirectory -Force | Out-Null
    foreach ($fileName in $requiredFiles) {
        Copy-Item -LiteralPath (Join-Path $sourceDirectory $fileName) `
                  -Destination (Join-Path $destinationDirectory $fileName) `
                  -Force
    }

    foreach ($fileName in $requiredFiles) {
        $sourceFile = Join-Path $sourceDirectory $fileName
        $destinationFile = Join-Path $destinationDirectory $fileName
        if (-not (Test-Path -LiteralPath $destinationFile -PathType Leaf)) {
            throw "Installed J-Link device file is missing: $destinationFile"
        }

        $sourceHash = (Get-FileHash -LiteralPath $sourceFile -Algorithm SHA256).Hash
        $destinationHash = (Get-FileHash -LiteralPath $destinationFile -Algorithm SHA256).Hash
        if ($sourceHash -ne $destinationHash) {
            throw "Hash verification failed for installed J-Link device file: $destinationFile"
        }

        Write-Output ("Verified {0}: SHA-256 {1}" -f $fileName, $destinationHash)
    }
}

Write-Output "LCM32F039 J-Link device files: $destinationDirectory"
Write-Output 'Restart Ozone and select device LCM32F039.'
