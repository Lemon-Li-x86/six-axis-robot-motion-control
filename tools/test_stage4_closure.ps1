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
    [switch]$IncludeGuiReplay,
    [switch]$HostBuildOnly,
    [string]$HostCompiler
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Set-Location $projectRoot

$python = Join-Path $projectRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -Path $python -PathType Leaf)) {
    throw "Missing project Python environment: $python"
}
foreach ($executable in @('cmake', 'ctest')) {
    if (-not (Get-Command $executable -ErrorAction SilentlyContinue)) {
        throw "Missing required build command: $executable"
    }
}

function Resolve-HostToolchain {
    param([string]$Override)

    # The native DLL must be compiled for Windows, not Cortex-M4.
    $compiler = $null
    if ($Override) {
        $found = Get-Command $Override -ErrorAction SilentlyContinue | Select-Object -First 1
        if (-not $found) { throw "Host C compiler not found: $Override" }
        if ($found.Source -match '(?i)arm-none-eabi') {
            throw 'The ARM cross compiler cannot build the Windows PyBullet DLL.'
        }
        $compiler = $found.Source
    } else {
        # VS C++ can be used through a CMake generator without cl.exe in PATH.
        $pf = [Environment]::GetFolderPath('ProgramFilesX86')
        if ($pf) {
            $vswhere = Join-Path $pf 'Microsoft Visual Studio\Installer\vswhere.exe'
            if (Test-Path -Path $vswhere -PathType Leaf) {
                $version = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationVersion
                if ($LASTEXITCODE -eq 0 -and $version) {
                    $major = [int]($version.Trim().Split('.')[0])
                    $generator = switch ($major) {
                        16 { 'Visual Studio 16 2019' }
                        17 { 'Visual Studio 17 2022' }
                        18 { 'Visual Studio 18 2026' }
                        default { '' }
                    }
                    if ($generator) {
                        $caps = & cmake -E capabilities | ConvertFrom-Json
                        if (@($caps.generators.name) -contains $generator) {
                            Write-Host "[Stage4] Host C compiler: $generator"
                            return @{ Id = "vs$major"; Configure = @('-G', $generator, '-A', 'x64') }
                        }
                    }
                }
            }
        }
        foreach ($name in @('gcc.exe', 'clang.exe')) {
            $found = Get-Command $name -ErrorAction SilentlyContinue | Select-Object -First 1
            if ($found -and $found.Source -notmatch '(?i)arm-none-eabi') {
                $compiler = $found.Source
                break
            }
        }
        if (-not $compiler) {
            foreach ($path in @(
                'C:\msys64\ucrt64\bin\gcc.exe',
                'C:\msys64\mingw64\bin\gcc.exe',
                'C:\msys64\clang64\bin\clang.exe'
            )) {
                if (Test-Path -Path $path -PathType Leaf) {
                    $compiler = $path
                    break
                }
            }
        }
    }

    if (-not $compiler) {
        throw @'
No native Windows C compiler was found.
Install Visual Studio Build Tools with "Desktop development with C++",
or install MSYS2 UCRT64 GCC.
If GCC already exists, use -HostCompiler "C:\path\to\gcc.exe".
arm-none-eabi-gcc cannot build the Python-loadable Windows DLL.
'@
    }
    if (-not (Get-Command ninja -ErrorAction SilentlyContinue)) {
        throw 'Ninja is required to build with native GCC/Clang.'
    }
    Write-Host "[Stage4] Host C compiler: $compiler"
    return @{
        Id = 'ninja-host'
        Configure = @('-G', 'Ninja', "-DCMAKE_C_COMPILER=$compiler", '-DCMAKE_BUILD_TYPE=Debug')
        Compiler = $compiler
    }
}

$hostToolchain = Resolve-HostToolchain -Override $HostCompiler

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
$nativeDir = Join-Path $buildRoot ('native-' + $hostToolchain.Id)
$hostDir = Join-Path $buildRoot ('host-' + $hostToolchain.Id)
$evidenceDir = Join-Path $buildRoot 'evidence'

$hostConfigureArgs = @('-S', 'firmware/freertos_demo/tests/trajectory', '-B', $hostDir) + $hostToolchain.Configure
Invoke-Checked 'Configure Stage4 host tests' 'cmake' $hostConfigureArgs
Invoke-Checked 'Build Stage4 host tests' 'cmake' @('--build', $hostDir, '--config', 'Debug')
Invoke-Checked 'Run Stage4 host tests' 'ctest' @(
    '--test-dir', $hostDir, '-C', 'Debug', '--output-on-failure'
)

$nativeConfigureArgs = @('-S', 'firmware/freertos_demo/tests/simulation', '-B', $nativeDir) + $hostToolchain.Configure
Invoke-Checked 'Configure C/PyBullet bridge' 'cmake' $nativeConfigureArgs
Invoke-Checked 'Build C/PyBullet bridge' 'cmake' @('--build', $nativeDir, '--config', 'Debug')
Invoke-Checked 'Run native C smoke test' 'ctest' @(
    '--test-dir', $nativeDir, '-C', 'Debug', '--output-on-failure'
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

# Diagnostic-only entry for Windows CI: verifies toolchain detection,
# native host compilation and C smoke tests, but does not claim that
# PyBullet or QEMU integration was exercised.
if ($HostBuildOnly) {
    Write-Host "[Stage4] Native host compiler and C smoke tests passed."
    Write-Host "[Stage4] PyBullet/QEMU intentionally NOT tested (-HostBuildOnly)."
    return
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
