# Stage5 PC-in-the-loop acceptance: real ARM FreeRTOS/QEMU UART, production
# C APF/IK Windows DLL and PyBullet DIRECT. No real motors, no GUI mouse drag.
param(
    [string]$Python = "$PSScriptRoot/../.venv/Scripts/python.exe",
    [string]$Qemu = "qemu-system-arm",
    [string]$OutputDir = "build/stage5-controlled-recovery-windows",
    [ValidateSet(1, 2)][int]$Cycles = 1
)
$ErrorActionPreference = "Stop"
Push-Location (Join-Path $PSScriptRoot "..")
try {
    if (-not (Get-Command $Python -ErrorAction SilentlyContinue)) {
        throw "Python executable not found: $Python (use -Python python if available)"
    }
    if (-not (Get-Command $Qemu -ErrorAction SilentlyContinue)) {
        throw "qemu-system-arm not found: $Qemu"
    }
    & $Python -c "import pybullet"
    if ($LASTEXITCODE -ne 0) { throw "PyBullet is missing in the selected Python" }
    if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
        throw "cmake is not on PATH"
    }
    Write-Host "[Stage5] Build actual FreeRTOS/QEMU UART firmware"
    & cmake --preset stage5-live-uart
    if ($LASTEXITCODE -ne 0) { throw "Firmware configure failed" }
    & cmake --build --preset stage5-live-uart
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed" }

    Write-Host "[Stage5] Build real production C APF/IK host bridge"
    $apfBuild = "build/stage5-avoidance-win"
    & cmake -S firmware/freertos_demo/tests/avoidance -B $apfBuild
    if ($LASTEXITCODE -ne 0) { throw "C APF bridge configure failed" }
    & cmake --build $apfBuild --config Debug
    if ($LASTEXITCODE -ne 0) { throw "C APF bridge build failed" }
    & ctest --test-dir $apfBuild -C Debug --output-on-failure
    if ($LASTEXITCODE -ne 0) { throw "C APF host tests failed" }

    $dll = Get-ChildItem -Path $apfBuild -Recurse -File |
        Where-Object { $_.Name -eq "stage5_apf_bridge.dll" -or
                        $_.Name -eq "libstage5_apf_bridge.dll" } |
        Select-Object -First 1
    if ($null -eq $dll) { throw "Production C APF/IK DLL is missing" }
    $elf = "build/stage5-live-uart/firmware/freertos_demo/freertos_demo.elf"
    if (-not (Test-Path $elf)) { throw "FreeRTOS firmware ELF not produced" }
    Write-Host "[Stage5] Starting PC/QEMU UART controlled recovery, not real motors"
    $pyArgs = @(
        "tools/tests/run_stage5_controlled_apf_hold_recovery.py",
        "--elf", $elf,
        "--library", $dll.FullName,
        "--qemu", $Qemu,
        "--output-dir", $OutputDir,
        "--cycles", "$Cycles"
    )
    & $Python @pyArgs
    $exitCode = $LASTEXITCODE
    $verdictPath = Join-Path $OutputDir "controlled_recovery_acceptance.json"
    if (Test-Path $verdictPath) {
        $verdict = Get-Content $verdictPath -Raw | ConvertFrom-Json
        Write-Host ("[Stage5] Acceptance verdict: " + $verdict.passed)
        Write-Host ("[Stage5] Evidence: " + $verdictPath)
        if (-not $verdict.passed) {
            throw "Controlled C APF return failed acceptance; inspect $OutputDir"
        }
    } else {
        throw "No controlled recovery acceptance file, inspect $OutputDir"
    }
    if ($exitCode -ne 0) { throw "Controlled recovery exited with code $exitCode" }
    Write-Host "[Stage5] PASS: controlled QEMU UART recovery with full-link safety"
} finally {
    Pop-Location
}
