param(
    [string]$Python = "$PSScriptRoot/../.venv/Scripts/python.exe",
    [string]$Qemu = "qemu-system-arm",
    [string]$OutputDir = "build/stage5-live-windows",
    [double]$DiagnosticsTimeoutSeconds = 0,  # 0 means use the Python default (5 s).
    [switch]$ObserveHold,
    [switch]$NoHold
)
$ErrorActionPreference = "Stop"
Push-Location (Join-Path $PSScriptRoot "..")
try {
    & cmake --preset stage5-live-uart
    if ($LASTEXITCODE -ne 0) { throw "Firmware configure failed" }
    & cmake --build --preset stage5-live-uart
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed" }
    $liveArgs = @("simulation/pybullet_ur5/src/stage5_qemu_uart_live_gui.py",
        "--elf", "build/stage5-live-uart/firmware/freertos_demo/freertos_demo.elf",
        "--qemu", $Qemu, "--scenario", "six_axis", "--output-dir", $OutputDir, "--opengl2")
    if ($NoHold) { $liveArgs += "--no-hold" }
    if ($ObserveHold) { $liveArgs += "--observe-hold" }
    if ($DiagnosticsTimeoutSeconds -ne 0) {
        if ($DiagnosticsTimeoutSeconds -lt 5 -or $DiagnosticsTimeoutSeconds -gt 60) {
            throw "DiagnosticsTimeoutSeconds must be between 5 and 60 seconds."
        }
        # PowerShell binds [double] as a scalar, not as Nullable<T>. Do not
        # access .Value; serialize with invariant culture for argparse.
        $timeoutArg = $DiagnosticsTimeoutSeconds.ToString(
            [System.Globalization.CultureInfo]::InvariantCulture)
        $liveArgs += @("--diagnostics-timeout-s", $timeoutArg)
    }
    # This terminal shows UART telemetry; PyBullet opens its own native window.
    & $Python @liveArgs
    if ($LASTEXITCODE -ne 0) { throw "Live control failed; inspect $OutputDir/worker.log" }
} finally {
    Pop-Location
}
