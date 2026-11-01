# Stage5 Windows interpreter selection. No pip, no venv creation, no activation.
# A successful import probe is required before launching CMake/QEMU.
function Get-Stage5PythonPathFromLauncherLine {
    param([string]$Line)
    # py -0p includes both normal -V:3.11 and uv/Astral runtimes
    # (-V:Astral/CPython3.11.15) as well as the activated venv *.
    if ($Line -match '(?i)(?<exe>[a-z]:\\[^\r\n]+\\python\.exe)\s*$') {
        return $Matches['exe']
    }
}

function Resolve-Stage5Python {
    param(
        [string]$RequestedPython = "",
        [Parameter(Mandatory = $true)][string]$RepoRoot
    )

    # Windows PowerShell 5.1 can promote native stderr to NativeCommandError
    # under Stop, even for PyBullet's harmless successful build-time banner.
    # This preference is scoped to the function, not persisted or exported.
    $ErrorActionPreference = "Continue"
    $candidates = New-Object 'System.Collections.Generic.List[string]'

    if (-not [string]::IsNullOrWhiteSpace($RequestedPython)) {
        # Explicit user selection is exact, never silently replaced.
        $candidates.Add($RequestedPython)
    } else {
        $localPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
        if (Test-Path -LiteralPath $localPython -PathType Leaf) {
            $candidates.Add($localPython)
        }
        # Query registered real interpreter paths, not only legacy -3.xx
        # launcher selectors (which can omit Astral/uv installations).
        if (Get-Command py -ErrorAction SilentlyContinue) {
            try {
                $launcherLines = & py -0p 2>&1
                if ($LASTEXITCODE -eq 0) {
                    foreach ($line in @($launcherLines)) {
                        $path = Get-Stage5PythonPathFromLauncherLine -Line ([string]$line)
                        if ($path -and (Test-Path -LiteralPath $path -PathType Leaf)) {
                            $candidates.Add($path)
                        }
                    }
                }
            } catch {
                # The already-existing PATH candidates remain usable.
            }
        }
        foreach ($command in @(Get-Command python -All -ErrorAction SilentlyContinue)) {
            if ($command.Source) { $candidates.Add($command.Source) }
        }
    }

    $checked = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
    $failures = New-Object 'System.Collections.Generic.List[string]'
    $nl = [Environment]::NewLine
    $probe = @'
import importlib
import sys
import traceback

print("PYTHON=" + sys.executable + " VERSION=" + sys.version.split()[0])
failed = False
for name in ("numpy", "pybullet"):
    try:
        module = importlib.import_module(name)
        print("OK " + name + " " + str(getattr(module, "__version__", "")))
    except Exception as error:
        failed = True
        print("FAIL " + name + " (" + type(error).__name__ + "): " + str(error))
        print("TRACE " + name + ": " + " | ".join(traceback.format_exc().strip().splitlines()[-7:]))
sys.exit(1 if failed else 0)
'@

    foreach ($candidate in $candidates) {
        if ([string]::IsNullOrWhiteSpace($candidate) -or -not $checked.Add($candidate)) {
            continue
        }
        try {
            # Check exit code rather than treating stderr output as failure.
            $details = & $candidate -c $probe 2>&1
            $exitCode = $LASTEXITCODE
            $textLines = @($details) | ForEach-Object { [string]$_ }
            if ($exitCode -eq 0) {
                $identity = $textLines | Where-Object { $_ -match '^PYTHON=' } | Select-Object -First 1
                Write-Host "[Stage5] Using existing Python: $identity"
                return $candidate
            }
            $failures.Add("$candidate : " + ($textLines -join $nl))
        } catch {
            $failures.Add("$candidate : $($_.Exception.Message)")
        }
    }

    $message = "No existing Python interpreter could import both numpy and pybullet for Stage5."
    if ($failures.Count -gt 0) { $message += $nl + "Checked:" + $nl + ($failures -join $nl) }
    $message += $nl + "No environment was changed. Specify a known working interpreter with -Python 'C:\path\to\python.exe'."
    throw $message
}
