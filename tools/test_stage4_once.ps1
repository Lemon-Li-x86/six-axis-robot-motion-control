# One-command, non-actuating Stage1-4 acceptance runner for Windows.
# Run from anywhere: powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1
# Requires CMake, CTest, ARM GNU toolchain, QEMU, GDB and Python+PyBullet.
#
# Intentional limits:
# - Stage4 command output is forcibly OFF; this never drives motors.
# - Python/PyBullet runs actual native C planner/PID/limiter closed-loop.
# - QEMU covers target driver/protocol/algorithm tests and synthetic motion;
#   only the explicitly gated Stage5 script exercises live QEMU UART feedback.
# - Stops on first failure, retaining one transcript with all tool output.
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Set-Location $root
$build = Join-Path $root 'build'
New-Item -ItemType Directory -Path $build -Force | Out-Null
$log = Join-Path $build 'stage4-final-validation.log'
Start-Transcript -Path $log -Force | Out-Null

function Assert-Exit([string]$label) {
    if ($LASTEXITCODE -ne 0) {
        throw "$label failed with exit code $LASTEXITCODE"
    }
}

function Require-Tool([string]$name) {
    if (-not (Get-Command $name -ErrorAction SilentlyContinue)) {
        throw "Tool missing from PATH: $name"
    }
}

function Run-QemuGdb {
    param(
        [string]$target,
        [string]$breakpoint,
        [string[]]$fields,
        [hashtable]$expected,
        [string]$elfBuildDir = 'build/cmake-arm',
        [string[]]$diagnosticFields = @()
    )
    $elf = Join-Path $root "$elfBuildDir/firmware/freertos_demo/$target.elf"
    if (-not (Test-Path $elf)) { throw "ELF not built: $elf" }

    $tmp = Join-Path $build "gdb_stage4_$target"
    New-Item -ItemType Directory -Path $tmp -Force | Out-Null
    $commands = Join-Path $tmp 'batch.gdb'
    $gdbOut = Join-Path $tmp 'output.txt'
    $gdbErr = Join-Path $tmp 'stderr.txt'

    $lines = @(
        'set pagination off',
        'set confirm off',
        'set remotetimeout 15',
        'target remote localhost:1234',
        "break $breakpoint",
        'continue'
    )
    foreach ($name in ($fields + $diagnosticFields)) {
        # GDB prints one unambiguous value per line for automated parsing.
        $lines += ('printf "' + $name + '=%u\n", ' + $name)
    }
    $lines += 'disconnect'
    $lines += 'quit'
    [System.IO.File]::WriteAllLines($commands, $lines,
        [System.Text.Encoding]::ASCII)

    Write-Host "[QEMU] $target" -ForegroundColor Cyan
    $qemu = $null
    $gdb = $null
    try {
        $qemuArgs = @(
            '-M', 'mps2-an386',
            '-kernel', ('"' + $elf + '"'),
            '-S',
            '-gdb', 'tcp::1234',
            '-serial', 'null',
            '-monitor', 'none',
            '-display', 'none'
        )
        $qemu = Start-Process -FilePath 'qemu-system-arm' `
            -ArgumentList $qemuArgs -PassThru -WindowStyle Hidden
        Start-Sleep -Seconds 2
        if ($qemu.HasExited) {
            throw "$target QEMU exited before GDB attached"
        }
        $gdbArgs = '-nx -batch -x "' + $commands + '" "' + $elf + '"'
        $gdb = Start-Process -FilePath 'arm-none-eabi-gdb' `
            -ArgumentList $gdbArgs -PassThru -Wait:$false `
            -RedirectStandardOutput $gdbOut `
            -RedirectStandardError $gdbErr -WindowStyle Hidden
        $gdb | Wait-Process -Timeout 90 -ErrorAction Stop
        $gdb.Refresh()
        $stdout = if(Test-Path $gdbOut) {
            [System.IO.File]::ReadAllText($gdbOut)
        } else { '' }
        $stderr = if(Test-Path $gdbErr) {
            [System.IO.File]::ReadAllText($gdbErr)
        } else { '' }
        Write-Host $stdout
        if($stderr) { Write-Host $stderr }
        if ($gdb.ExitCode -ne 0) {
            throw "$target GDB failed with exit code $($gdb.ExitCode)"
        }
        # Don't accept a clean GDB exit that failed to reach the test marker.
        $breakPattern = '(?m)^Breakpoint [0-9]+, ' +
            [regex]::Escape($breakpoint) + '\b'
        if ($stdout -notmatch $breakPattern) {
            throw "${target}: never reached $breakpoint"
        }
        foreach ($name in $fields) {
            if (-not $expected.ContainsKey($name)) {
                throw "Test script has no expectation for $name"
            }
            $match = [regex]::Match($stdout,
                '(?m)^' + [regex]::Escape($name) + '=([0-9]+)\s*$')
            if (-not $match.Success) {
                throw "${target}: missing diagnostic: $name"
            }
            $actual = [uint64]::Parse($match.Groups[1].Value)
            $wanted = [uint64]$expected[$name]
            if ($actual -ne $wanted) {
                throw "${target}: $name expected $wanted, got $actual"
            }
        }
        Write-Host "[PASS] $target" -ForegroundColor Green
    } finally {
        if ($gdb -ne $null) {
            $gdb.Refresh()
            if (-not $gdb.HasExited) { Stop-Process -Id $gdb.Id -Force }
        }
        if ($qemu -ne $null) {
            $qemu.Refresh()
            if (-not $qemu.HasExited) { Stop-Process -Id $qemu.Id -Force }
        }
        Start-Sleep -Milliseconds 300
    }
}

$passed = $false
$pythonExe = Join-Path $root '.venv\Scripts\python.exe'
if (-not (Test-Path $pythonExe)) {
    $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
    if (-not $pythonCommand) { throw 'Python interpreter is missing from PATH and .venv' }
    $pythonExe = $pythonCommand.Source
}
try {
    foreach ($tool in @(
        'cmake','ctest','qemu-system-arm','arm-none-eabi-gdb'
    )) { Require-Tool $tool }

    Write-Host '[1/9] Verify UART protocol mirror, then build native host tests' -ForegroundColor Cyan
    & $pythonExe tools/check_protocol_mirror.py
    Assert-Exit 'Python UART receiver and firmware header consistency'
    & $pythonExe -m unittest discover -s tools/tests -p test_firmware_defaults.py
    Assert-Exit 'JSON firmware policy validation tests'
    & cmake -S firmware/freertos_demo/tests/trajectory -B build/stage4-trajectory
    Assert-Exit 'Host CMake configure'
    & cmake --build build/stage4-trajectory --config Debug
    Assert-Exit 'Host build'

    Write-Host '[2/9] Run six Stage3/4 host CTest targets' -ForegroundColor Cyan
    & ctest --test-dir build/stage4-trajectory -C Debug --output-on-failure
    Assert-Exit 'Host CTest'

    Write-Host '[3/9] Build ARM firmware and QEMU targets, outputs disabled' -ForegroundColor Cyan
    & cmake --preset arm-qemu -DROBOT_ENABLE_STAGE4_TASKS=ON `
        -DROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF `
        -DROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=OFF
    Assert-Exit 'ARM configure'
    & cmake --build --preset arm-qemu --target `
        freertos_demo qemu_stage4_trajectory_tests qemu_kinematics_perf `
        qemu_ik_near_singular_trace qemu_uart_tx_tests qemu_motor_tests `
        qemu_gpio_tests
    Assert-Exit 'ARM build'

    Write-Host '[4/9] Run Stage2 Unity, Cortex-M4 protocol and driver regressions' -ForegroundColor Cyan
    & cmake -S firmware/freertos_demo/tests/host -B build/stage2-host
    Assert-Exit 'Stage2 safety/byte-stream Host configure'
    & cmake --build build/stage2-host --config Debug
    Assert-Exit 'Stage2 safety/byte-stream Host build'
    & ctest --test-dir build/stage2-host -C Debug --output-on-failure
    Assert-Exit 'Stage2 safety/byte-stream Host CTest'
    & cmake -S firmware/freertos_demo/tests/unity -B build/cmake-host-unity
    Assert-Exit 'Stage2 Unity configure'
    & cmake --build build/cmake-host-unity --config Debug
    Assert-Exit 'Stage2 Unity build'
    & ctest --test-dir build/cmake-host-unity -C Debug --output-on-failure
    Assert-Exit 'Stage2 Unity CTest'
    & cmake --build build/cmake-arm --target `
        qemu_protocol_tests qemu_uart_tx_tests qemu_motor_tests qemu_gpio_tests
    Assert-Exit 'Stage2 QEMU protocol and driver target build'
    Run-QemuGdb 'qemu_protocol_tests' 'qemu_target_test_done' `
        @('g_qemu_target_test_report.total',
          'g_qemu_target_test_report.passed',
          'g_qemu_target_test_report.failed',
          'g_qemu_target_test_report.results[7]') `
        @{
          'g_qemu_target_test_report.total'=8;
          'g_qemu_target_test_report.passed'=8;
          'g_qemu_target_test_report.failed'=0;
          'g_qemu_target_test_report.results[7]'=1
        }
    Run-QemuGdb 'qemu_uart_tx_tests' 'qemu_uart_tx_test_done' `
        @('g_uart_tx_test_report.total','g_uart_tx_test_report.passed',
          'g_uart_tx_test_report.failed') `
        @{
          'g_uart_tx_test_report.total'=7;
          'g_uart_tx_test_report.passed'=7;
          'g_uart_tx_test_report.failed'=0
        }
    Run-QemuGdb 'qemu_motor_tests' 'qemu_motor_test_done' `
        @('g_motor_test_report.total','g_motor_test_report.passed',
          'g_motor_test_report.failed') `
        @{
          'g_motor_test_report.total'=11;
          'g_motor_test_report.passed'=11;
          'g_motor_test_report.failed'=0
        }
    Run-QemuGdb 'qemu_gpio_tests' 'qemu_gpio_test_done' `
        @('g_gpio_test_report.total','g_gpio_test_report.passed',
          'g_gpio_test_report.failed') `
        @{
          'g_gpio_test_report.total'=13;
          'g_gpio_test_report.passed'=13;
          'g_gpio_test_report.failed'=0
        }

    Write-Host '[5/9] Run three independent Stage3/4 algorithm suites under QEMU' -ForegroundColor Cyan
    Run-QemuGdb 'qemu_stage4_trajectory_tests' 'qemu_stage4_test_done' `
        @('g_stage4_test_finished','g_stage4_test_report.failed',
          'g_stage4_test_report.checks','g_stage4_test_report.passed') `
        @{
          'g_stage4_test_finished'=1;
          'g_stage4_test_report.failed'=0;
          'g_stage4_test_report.checks'=20;
          'g_stage4_test_report.passed'=20
        }

    Run-QemuGdb 'qemu_kinematics_perf' 'qemu_kinematics_perf_done' `
        @('g_kin_perf_finished','g_kin_perf_report.fk_success',
          'g_kin_perf_report.ik_success',
          'g_kin_perf_report.selection_success',
          'g_kin_perf_report.roundtrip_passed',
          'g_kin_perf_report.roundtrip_failed',
          'g_kin_perf_report.roundtrip_skipped') `
        @{
          'g_kin_perf_finished'=1;
          'g_kin_perf_report.fk_success'=128;
          'g_kin_perf_report.ik_success'=128;
          'g_kin_perf_report.selection_success'=128;
          'g_kin_perf_report.roundtrip_passed'=128;
          'g_kin_perf_report.roundtrip_failed'=0;
          'g_kin_perf_report.roundtrip_skipped'=0
        }

    Run-QemuGdb 'qemu_ik_near_singular_trace' 'qemu_ik_trace_done' `
        @('g_ik_trace_done','g_ik_reference_regression_checks',
          'g_ik_reference_regression_failures') `
        @{
          'g_ik_trace_done'=1;
          'g_ik_reference_regression_checks'=2;
          'g_ik_reference_regression_failures'=0
        }

    Write-Host '[6/9] Verify FreeRTOS monitor task startup in QEMU' -ForegroundColor Cyan
    # A breakpoint hit proves the task actually ran under the scheduler;
    # task handles are POINTERS, so they must not be compared to zero.
    Run-QemuGdb 'freertos_demo' 'task_monitor' @() @{}

    Write-Host '[7/9] Isolated FreeRTOS motion+feedback+PID RTOS test' -ForegroundColor Cyan
    & cmake --preset arm-qemu -B build/cmake-arm-stage4-selftest `
        -DROBOT_ENABLE_STAGE4_TASKS=ON `
        -DROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF `
        -DROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=ON
    Assert-Exit 'QEMU-only motion smoke configure'
    & cmake --build build/cmake-arm-stage4-selftest --target freertos_demo
    Assert-Exit 'QEMU-only motion smoke build'
    Run-QemuGdb 'freertos_demo' 'stage4_rtos_selftest_done' `
        @('g_stage4_rtos_test_finished',
          'g_stage4_rtos_test_report.checks',
          'g_stage4_rtos_test_report.failed',
          'g_stage4_polynomial_rtos_report.checks',
          'g_stage4_polynomial_rtos_report.failed',
          'g_stage4_polynomial_rtos_report.rejected_before_publish',
          'g_stage4_polynomial_rtos_report.valid_execution_started') `
        @{
          'g_stage4_rtos_test_finished'=1;
          'g_stage4_rtos_test_report.checks'=29;
          'g_stage4_rtos_test_report.failed'=0;
          'g_stage4_polynomial_rtos_report.checks'=8;
          'g_stage4_polynomial_rtos_report.failed'=0;
          'g_stage4_polynomial_rtos_report.rejected_before_publish'=1;
          'g_stage4_polynomial_rtos_report.valid_execution_started'=1
        } 'build/cmake-arm-stage4-selftest' `
        -diagnosticFields @('g_stage4_polynomial_rtos_report.trajectory_stack_free_words')

    Write-Host '[8/9] Build and smoke native C firmware algorithm bridge' -ForegroundColor Cyan
    & cmake -S firmware/freertos_demo/tests/simulation -B build/stage4-simulation
    Assert-Exit 'Native simulation bridge configure'
    & cmake --build build/stage4-simulation --config Debug
    Assert-Exit 'Native simulation bridge build'
    & ctest --test-dir build/stage4-simulation -C Debug --output-on-failure
    Assert-Exit 'Native simulation bridge smoke'

    Write-Host '[9/9] PyBullet DIRECT closed-loop dynamic verification' -ForegroundColor Cyan
    $candidates = @(
        (Join-Path $root 'build/stage4-simulation/Debug/stage4_sim_bridge.dll'),
        (Join-Path $root 'build/stage4-simulation/stage4_sim_bridge.dll'),
        (Join-Path $root 'build/stage4-simulation/libstage4_sim_bridge.so'),
        (Join-Path $root 'build/stage4-simulation/libstage4_sim_bridge.dylib')
    )
    $library = $candidates | Where-Object { Test-Path $_ } |
        Select-Object -First 1
    if (-not $library) { throw 'Native Stage4 shared library not found after build' }
    & $pythonExe -c 'import pybullet'
    Assert-Exit 'PyBullet Python dependency (install with python -m pip install pybullet)'
    foreach ($trial in @('joint2','six_axis')) {
        Write-Host "[PyBullet] $trial" -ForegroundColor Cyan
        & $pythonExe simulation/pybullet_ur5/src/stage4_dynamic_closed_loop.py `
            --library $library `
            --scenario $trial `
            --output-dir (Join-Path 'build/stage4-closed-loop' $trial)
        Assert-Exit "Native C to PyBullet dynamic closed loop: $trial"
    }

    $passed=$true
} catch {
    Write-Host ('[FAIL] ' + $_.Exception.Message) -ForegroundColor Red
    throw
} finally {
    if($passed) {
        Write-Host '[PASS] Automated Stage3/4 non-actuating regression complete' `
            -ForegroundColor Green
    }
    Write-Host "Detailed transcript: $log"
    Stop-Transcript | Out-Null
}
