param(
    [string[]]$SourceSets = @("FULL", "CORE"),
    [string]$BuildType = "RelWithDebInfo"
)

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot

$configs = @(
    @{
        Name = "default"
        CMakeArgs = @()
        Flags = ""
    },
    @{
        Name = "app-all"
        CMakeArgs = @(
            "-DUSERAPP_ENABLE_MOTION_COORDINATOR=ON",
            "-DUSERAPP_ENABLE_ENERGY_COORDINATOR=ON",
            "-DUSERAPP_ENABLE_SYNC_COORDINATOR=ON"
        )
        Flags = ""
    },
    @{
        Name = "position-ff-held"
        CMakeArgs = @()
        Flags = "-DMOTOR_BUILD_ENABLE_POSITION_CONTROL -DMOTOR_BUILD_ENABLE_VELOCITY_FEEDFORWARD -DMOTOR_BUILD_ENABLE_STREAM_HOLD_WHEN_IDLE"
    },
    @{
        Name = "fifo-on"
        CMakeArgs = @()
        Flags = "-DMOTOR_BUILD_ENABLE_SETPOINT_FIFO_PLAYBACK"
    },
    @{
        Name = "impedance-torqueff"
        CMakeArgs = @()
        Flags = "-DMOTOR_BUILD_ENABLE_IMPEDANCE_CONTROL -DMOTOR_BUILD_ENABLE_TORQUE_FEEDFORWARD"
    }
)

foreach ($sourceSet in $SourceSets) {
    foreach ($cfg in $configs) {
        $buildDir = Join-Path $root ("build\matrix-{0}-{1}" -f $sourceSet.ToLowerInvariant(), $cfg.Name)
        $args = @(
            "-S", $root,
            "-B", $buildDir,
            "-G", "Ninja",
            "-DCMAKE_BUILD_TYPE=$BuildType",
            "-DMOTOR_LIB_SOURCE_SET=$sourceSet"
        )
        $args += $cfg.CMakeArgs
        if ($cfg.Flags.Length -gt 0) {
            $args += "-DCMAKE_C_FLAGS=$($cfg.Flags)"
            $args += "-DCMAKE_CXX_FLAGS=$($cfg.Flags)"
        }

        Write-Host "== Configure $sourceSet / $($cfg.Name) =="
        & cmake @args

        Write-Host "== Build $sourceSet / $($cfg.Name) =="
        & cmake --build $buildDir --target motor_lib
    }
}
