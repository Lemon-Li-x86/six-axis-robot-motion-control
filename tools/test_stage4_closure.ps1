<#
.SYNOPSIS
    Stage4 closure validation using native-C motion algorithms and PyBullet.
.DESCRIPTION
    Builds the existing host C tests and shared library, then runs the
    baseline PID matrix and split calibration/validation study.

    - Does not edit robot_defaults.json or the frozen Stage4 source.
    - Does not treat PyBullet DIRECT as a QEMU/FreeRTOS UART test.
    - Optional -IncludeQemuBaseline runs the existing Stage4 QEMU suite
      without adding a second build or protocol implementation.
    - All outputs stay under build/ and are not tracked by Git.
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1 -IncludeQemuBaseline -IncludeGuiReplay
#>

[CmdletBinding()]
param(
    [switch]$IncludeQemuBaseline,
    [switch]$IncludeGuiReplay
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Set-Location $projectRoot

$python = Join-Path $projectRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -Path $python -PathType Leaf)) {
    throw "Missing project Python environment: $python"
}
foreach ($executable in @('cmake', 'ctest', 'ninja')) {
    if (-not (Get-Command $executable -ErrorAction SilentlyContinue)) {
        throw "Missing required build command: $executable"
    }
}

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)][string]$Title,
        [Parameter(Mandatory = $true)][string]$Executable,
        [Parameter(Mandatory = $true)][string[]]$Arguments
    )

    Write-Host ("[Stage4] " + $Title)
    & $Executable @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$Title failed with exit code $LASTEXITCODE"
    }
}

$buildRoot = Join-Path $projectRoot 'build\stage4-closure-local'
$nativeDir = Join-Path $buildRoot 'native'
$hostDir = Join-Path $buildRoot 'host'
$evidenceDir = Join-Path $buildRoot 'evidence'

Invoke-Checked 'Configure Stage4 host tests' 'cmake' @(
    '-S', 'firmware/freertos_demo/tests/trajectory',
    '-B', $hostDir, '-G', 'Ninja'
)
Invoke-Checked 'Build Stage4 host tests' 'cmake' @('--build', $hostDir)
Invoke-Checked 'Run Stage4 host tests' 'ctest' @(
    '--test-dir', $hostDir, '--output-on-failure'
)

Invoke-Checked 'Configure C/PyBullet bridge' 'cmake' @(
    '-S', 'firmware/freertos_demo/tests/simulation',
    '-B', $nativeDir, '-G', 'Ninja'
)
Invoke-Checked 'Build C/PyBullet bridge' 'cmake' @('--build', $nativeDir)
Invoke-Checked 'Run native C smoke test' 'ctest' @(
    '--test-dir', $nativeDir, '--output-on-failure'
)

$library = Get-ChildItem -Path $nativeDir -Recurse -File |
    Where-Object { $_.Name -in @(
        'stage4_sim_bridge.dll',
        'libstage4_sim_bridge.dll',
        'libstage4_sim_bridge.so',
        'libstage4_sim_bridge.dylib'
    ) } | Select-Object -First 1

if ($null -eq $library) {
    throw "No Stage4 native C bridge library found under $nativeDir"
}

Invoke-Checked 'Validate PID sensitivity matrix definitions' $python @(
    '-m', 'unittest', 'discover', '-s', 'tools/tests',
    '-p', 'test_stage4_pid_study.py', '-v'
)
Invoke-Checked 'Validate split gain sweep definitions' $python @(
    '-m', 'unittest', 'discover', '-s', 'tools/tests',
    '-p', 'test_stage4_pid_gain_sweep.py', '-v'
)
Invoke-Checked 'Run 18-case C/PyBullet PID study' $python @(
    'tools/stage4_pid_study.py',
    '--library', $library.FullName,
    '--output-dir', (Join-Path $evidenceDir 'pid-matrix')
)
Invoke-Checked 'Run split gain calibration/validation' $python @(
    'tools/stage4_pid_gain_sweep.py',
    '--library', $library.FullName,
    '--output-dir', (Join-Path $evidenceDir 'gain-sweep')
)

if ($IncludeQemuBaseline) {
    $qemuScript = Join-Path $PSScriptRoot 'test_stage4_once.ps1'
    if (-not (Test-Path $qemuScript)) {
        throw "Existing Stage4 QEMU baseline script not found: $qemuScript"
    }
    Invoke-Checked 'Run existing QEMU/FreeRTOS baseline' 'powershell' @(
        '-ExecutionPolicy', 'Bypass', '-File', $qemuScript
    )
}

if ($IncludeGuiReplay) {
    # Stage5 integration is explicitly separated from the native Stage4
    # study. Rebuild the candidate ELF rather than reusing a stale binary.
    foreach ($executable in @('arm-none-eabi-gcc', 'qemu-system-arm')) {
        if (-not (Get-Command $executable -ErrorAction SilentlyContinue)) {
            throw "Missing ARM/QEMU executable: $executable"
        }
    }

    $qemuBuild = Join-Path $buildRoot 'qemu-uart'
    $candidateElf = Join-Path $qemuBuild 'firmware\freertos_demo\freertos_demo.elf'
    $guiEvidence = Join-Path $evidenceDir 'qemu-gui-replay'

    Invoke-Checked 'Build candidate Cortex-M4/FreeRTOS UART firmware' 'cmake' @(
        '-S', '.', '-B', $qemuBuild, '-G', 'Ninja',
        '-DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake',
        '-DCMAKE_BUILD_TYPE=Debug',
        '-DROBOT_ENABLE_STAGE4_TASKS=ON',
        '-DROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=ON',
        '-DROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION=ON',
        '-DROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=OFF'
    )
    Invoke-Checked 'Compile candidate ARM firmware' 'cmake' @(
        '--build', $qemuBuild, '--target', 'freertos_demo'
    )
    if (-not (Test-Path -Path $candidateElf -PathType Leaf)) {
        throw "Freshly built candidate ELF not found: $candidateElf"
    }

    Invoke-Checked 'Record QEMU UART/PyBullet closed loop and replay GUI' $python @(
        'simulation/pybullet_ur5/src/stage5_qemu_uart_trace_replay.py',
        '--elf', $candidateElf,
        '--scenario', 'six_axis',
        '--speed', '0.35',
        '--repeats', '2',
        '--output-dir', $guiEvidence
    )
}

Write-Host "[Stage4] Native closure study completed."
Write-Host ("[Stage4] Evidence: " + $evidenceDir)
Write-Host "[Stage4] Comparison gates are exploratory; review their measured failures."
Write-Host "[Stage4] Stage4 freeze branch and production PID config are unchanged."
