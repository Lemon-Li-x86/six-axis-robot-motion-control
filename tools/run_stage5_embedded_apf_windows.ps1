# Original Stage5 PDF 5.2: real FreeRTOS Trajectory-task C APF/IK.
# Fully automated QEMU UART / PyBullet DIRECT all-link safety and physical
# settling, no operator clicking and NEVER sends to any real motor device.
param(
    [string]$Python = "python",
    [string]$Qemu = "qemu-system-arm",
    [string]$OutputDir = "build/stage5-windows-embedded-apf",
    [double]$TimeoutSeconds = 300,
    [switch]$ThreeStepOnly
)
$ErrorActionPreference = "Stop"
if ([double]::IsNaN($TimeoutSeconds) -or
    [double]::IsInfinity($TimeoutSeconds) -or
    $TimeoutSeconds -lt 30 -or $TimeoutSeconds -gt 1200) {
    throw "TimeoutSeconds must be finite and in [30, 1200] s"
}
Push-Location (Join-Path $PSScriptRoot "..")
try {
    & cmake --preset stage5-live-uart
    if ($LASTEXITCODE -ne 0) { throw "Embedded APF firmware configure failed" }
    & cmake --build --preset stage5-live-uart
    if ($LASTEXITCODE -ne 0) { throw "Embedded APF ARM firmware build failed" }

    $elf = "build/stage5-live-uart/firmware/freertos_demo/freertos_demo.elf"
    if (-not (Test-Path $elf)) { throw "ARM firmware ELF missing: $elf" }
    $seconds = $TimeoutSeconds.ToString(
        [System.Globalization.CultureInfo]::InvariantCulture)
    $argsList = @("tools/tests/run_stage5_embedded_apf_uart.py",
                  "--elf", $elf,
                  "--qemu", $Qemu, "--timeout-s", $seconds,
                  "--output-dir", $OutputDir)
    if ($ThreeStepOnly) {
        $argsList += @("--proposals", "3")
    } else {
        $argsList += "--full-route"
    }
    & $Python @argsList
    if ($LASTEXITCODE -ne 0) {
        throw "ARM embedded APF/IK route NOT PASS; preserve $OutputDir/result.json"
    }
    $resultPath = Join-Path $OutputDir "result.json"
    $result = Get-Content -Path $resultPath -Raw | ConvertFrom-Json
    if (-not $result.passed -or
        -not $result.full_link_safe -or
        -not $result.joint_motion_submitted_only_after_gate -or
        $result.uart_checksum_errors -ne 0 -or
        $result.new_apf_proposals -lt 3 -or
        $result.joint_motion_acks -lt 3) {
        throw "Embedded APF independent safety verdict NOT PASS; keep evidence"
    }
    if (-not $ThreeStepOnly -and -not $result.route_complete) {
        throw "Full static obstacle route was not reached by ARM C APF"
    }
    $archive = "$OutputDir.zip"
    Compress-Archive -Path (Join-Path $OutputDir "*") -DestinationPath $archive -Force
    Write-Host "[Stage5] ARM C APF/IK, Bullet full-link and C PID PASS. Evidence $archive"
} finally {
    Pop-Location
}
