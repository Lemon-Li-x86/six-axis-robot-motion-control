# Headless native C/PyBullet isolation of the user's first ARM APF step.
# Uses existing Python + existing packages only; no pip or virtualenv changes.
param(
    [Parameter(Mandatory = $true)][string]$Python,
    [Parameter(Mandatory = $true)][string]$ExistingSitePackages,
    [string]$OutputDir = "build/stage5-first-leg-native-bullet",
    [ValidateRange(20, 3000)][int]$Frames = 800
)

$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$launcher = Join-Path $PSScriptRoot "stage5_existing_python.py"
if (-not (Test-Path -LiteralPath $Python -PathType Leaf)) {
    throw "Existing Python executable missing: $Python"
}
$packages = (Resolve-Path -LiteralPath $ExistingSitePackages).Path
& $Python $launcher --site-packages $packages --probe
if ($LASTEXITCODE -ne 0) { throw "Existing CP311 package import failed" }

Push-Location $root
try {
    if (Test-Path -LiteralPath $OutputDir) {
        throw "Output directory already exists; preserve prior evidence: $OutputDir"
    }
    $build = "build/stage5-first-leg-c-bridge"
    & cmake -S firmware/freertos_demo/tests/simulation -B $build
    if ($LASTEXITCODE -ne 0) { throw "Native production C bridge configure failed" }
    & cmake --build $build --config Debug --target stage4_sim_bridge
    if ($LASTEXITCODE -ne 0) { throw "Native production C bridge build failed" }
    $dll = Get-ChildItem -Path $build -Recurse -File |
        Where-Object { $_.Name -eq "stage4_sim_bridge.dll" -or
                       $_.Name -eq "libstage4_sim_bridge.dll" } |
        Select-Object -First 1
    if ($null -eq $dll) { throw "Production C bridge DLL not found" }
    $argsList = @(
        $launcher, "--site-packages", $packages,
        "--script", "tools/tests/run_stage5_first_leg_native_bullet.py",
        "--library", $dll.FullName,
        "--frames", $Frames.ToString(),
        "--output-dir", $OutputDir
    )
    & $Python @argsList
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw "Production C + native PyBullet first-leg diagnostic FAIL; preserve $OutputDir/result.json"
    }
    Write-Host "[Stage5] First leg production C+Bullet settled; QEMU/RTOS/UART was excluded."
} finally {
    Pop-Location
}
