param(
    [string]$Python = "$PSScriptRoot/../.venv/Scripts/python.exe",
    [string]$Qemu = "qemu-system-arm",
    [string]$OutputDir = "build/stage5-live-windows",
    [double]$DiagnosticsTimeoutSeconds = 0,  # 0 means use the Python default (5 s).
    [switch]$ObserveHold,
    [switch]$NoHold,
    [ValidateSet("", "clear", "blocked")][string]$Stage5RouteCase = "",
    [switch]$RawRoute,
    [ValidateSet("single-turn", "urdf")][string]$GuiJointTravel = "single-turn",
    [switch]$SafeJog
)
$ErrorActionPreference = "Stop"
if ($SafeJog -and ($Stage5RouteCase -ne "blocked" -or $NoHold -or $ObserveHold)) {
    throw "-SafeJog requires -Stage5RouteCase blocked, active HOLD and no -ObserveHold"
}
Push-Location (Join-Path $PSScriptRoot "..")
try {
    & cmake --preset stage5-live-uart
    if ($LASTEXITCODE -ne 0) { throw "Firmware configure failed" }
    & cmake --build --preset stage5-live-uart
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed" }
    $liveArgs = @("simulation/pybullet_ur5/src/stage5_qemu_uart_live_gui.py",
        "--elf", "build/stage5-live-uart/firmware/freertos_demo/freertos_demo.elf",
        "--qemu", $Qemu, "--scenario", "six_axis", "--output-dir", $OutputDir, "--opengl2",
        "--gui-joint-travel", $GuiJointTravel)
    if ($Stage5RouteCase -ne "") {
        # Opt-in only. Keep original six-axis/HOLD execution unchanged by default.
        $apfBuild = "build/stage5-avoidance-win"
        & cmake -S firmware/freertos_demo/tests/avoidance -B $apfBuild
        if ($LASTEXITCODE -ne 0) { throw "C APF/IK host bridge configure failed" }
        & cmake --build $apfBuild --config Debug
        if ($LASTEXITCODE -ne 0) { throw "C APF/IK host bridge build failed" }
        & ctest --test-dir $apfBuild -C Debug --output-on-failure
        if ($LASTEXITCODE -ne 0) { throw "C APF/IK host bridge tests failed" }
        $apfDll = Get-ChildItem -Path $apfBuild -Recurse -File |
            Where-Object { $_.Name -eq "stage5_apf_bridge.dll" -or
                           $_.Name -eq "libstage5_apf_bridge.dll" } |
            Select-Object -First 1
        if ($null -eq $apfDll) { throw "C APF/IK bridge DLL was not produced" }
        $liveArgs += @("--stage5-c-apf-library", $apfDll.FullName,
                       "--stage5-route-case", $Stage5RouteCase)
        if ($RawRoute) { $liveArgs += "--stage5-raw-route" }
        Write-Host "[Stage5] C APF route: $Stage5RouteCase; bridge: $($apfDll.FullName)"
    } elseif ($RawRoute) {
        throw "-RawRoute requires -Stage5RouteCase clear or blocked"
    }
    if ($SafeJog) { $liveArgs += "--safe-gui-jog" }
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
