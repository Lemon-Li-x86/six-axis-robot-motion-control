# Resolve an existing, dependency-valid interpreter for Stage5 Windows tests.
# Read-only: never creates a venv, runs pip, or changes process environment.
function Resolve-Stage5Python {
    param(
        [string]$RequestedPython = "",
        [Parameter(Mandatory = $true)][string]$RepoRoot
    )

    $candidates = New-Object 'System.Collections.Generic.List[string]'
    if (-not [string]::IsNullOrWhiteSpace($RequestedPython)) {
        # An explicit path/command is an exact choice. Do not silently replace it.
        $candidates.Add($RequestedPython)
    } else {
        $localPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
        if (Test-Path -LiteralPath $localPython -PathType Leaf) {
            $candidates.Add($localPython)
        }
        # An activated venv can belong to ANOTHER Git worktree. Ask the Windows
        # Python launcher for absolute installed interpreter paths first.
        if (Get-Command py -ErrorAction SilentlyContinue) {
            foreach ($version in @("3.11", "3.12", "3.10", "3.13", "3.14")) {
                try {
                    $resolved = & py "-$version" -c "import sys; print(sys.executable)" 2>$null
                    if ($LASTEXITCODE -eq 0 -and $resolved) {
                        $candidates.Add(([string]($resolved | Select-Object -Last 1)).Trim())
                    }
                } catch {
                    # This interpreter version is not installed; try the next.
                }
            }
        }
        foreach ($command in @(Get-Command python -All -ErrorAction SilentlyContinue)) {
            if ($command.Source) { $candidates.Add($command.Source) }
        }
    }

    $checked = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
    $failures = New-Object 'System.Collections.Generic.List[string]'
    # Check actual binary import compatibility before CMake/QEMU; CI has tested
    # multiple NumPy versions, so do not change or over-constrain installations.
    $probe = "import sys, numpy, pybullet; print(sys.executable + ' (Python ' + sys.version.split()[0] + ', NumPy ' + numpy.__version__ + ')')"
    foreach ($candidate in $candidates) {
        if ([string]::IsNullOrWhiteSpace($candidate) -or -not $checked.Add($candidate)) {
            continue
        }
        try {
            $details = & $candidate -c $probe 2>&1
            if ($LASTEXITCODE -eq 0) {
                Write-Host "[Stage5] Using existing Python: $($details | Select-Object -Last 1)"
                return $candidate
            }
            $failures.Add("$candidate : Python/NumPy/PyBullet import or version check failed")
        } catch {
            $failures.Add("$candidate : $($_.Exception.Message)")
        }
    }

    $nl = [Environment]::NewLine
    $message = "No existing Python interpreter could import both numpy and pybullet for Stage5."
    if ($failures.Count -gt 0) { $message += $nl + "Checked:" + $nl + ($failures -join $nl) }
    $message += $nl + "This command does NOT install or change any environment."
    $message += $nl + "To select a previously working interpreter explicitly, pass -Python 'C:\path\to\python.exe'."
    throw $message
}
