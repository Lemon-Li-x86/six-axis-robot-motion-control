# Resolve an existing, dependency-valid interpreter for Stage5 Windows tests.
# Read-only: never creates a venv, runs pip, or changes process environment.
# The Windows launcher lists uv/Astral runtimes using -V:Astral/..., which
# are not reliably discoverable with legacy "py -3.11" switches alone.
function Get-Stage5PythonPathFromLauncherLine {
    param([string]$Line)
    if ($Line -match '(?i)(?<exe>[a-z]:\\[^\r\n]+\\python\.exe)\s*
    param(
        [string]$RequestedPython = "",
        [Parameter(Mandatory = $true)][string]$RepoRoot
    )

    # Native modules can write a successful build banner to stderr. In
    # Windows PowerShell 5.1, Stop + stderr redirection can abort before the
    # interpreter prints the actual import failure. This preference is local
    # to the function and does not change the user's environment.
    $ErrorActionPreference = "Continue"
    $candidates = New-Object 'System.Collections.Generic.List[string]'
    if (-not [string]::IsNullOrWhiteSpace($RequestedPython)) {
        # An explicit path/command is an exact choice. Do not silently replace it.
        $candidates.Add($RequestedPython)
    } else {
        $localPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
        if (Test-Path -LiteralPath $localPython -PathType Leaf) {
            $candidates.Add($localPython)
        }
        # Read the launcher's full registry, including uv/Astral runtimes
        # such as -V:Astral/CPython3.11.15. This is read-only discovery.
        if (Get-Command py -ErrorAction SilentlyContinue) {
            try {
                $launcherLines = & py -0p 2>&1
                if ($LASTEXITCODE -eq 0) {
                    foreach ($launcherLine in @($launcherLines)) {
                        $candidatePath = Get-Stage5PythonPathFromLauncherLine -Line ([string]$launcherLine)
                        if ($candidatePath -and (Test-Path -LiteralPath $candidatePath -PathType Leaf)) {
                            $candidates.Add($candidatePath)
                        }
                    }
                }
            } catch {
                # Preserve fallback to PATH when launcher discovery fails.
            }
        }
        foreach ($command in @(Get-Command python -All -ErrorAction SilentlyContinue)) {
            if ($command.Source) { $candidates.Add($command.Source) }
        }
    }

    $checked = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
    $failures = New-Object 'System.Collections.Generic.List[string]'
    $nl = [Environment]::NewLine
    # Probe imports independently. Both candidates and the exact failure are
    # printed without pip, file writes, or any environment activation.
    $probe = @'
import importlib
import sys
import traceback
print("PYTHON=" + sys.executable + " VERSION=" + sys.version.split()[0])
failed = False
for module_name in ("numpy", "pybullet"):
    try:
        module = importlib.import_module(module_name)
        print("OK " + module_name + " " + str(getattr(module, "__version__", "")))
    except Exception as error:
        failed = True
        message = str(error).strip().splitlines()
        print("FAIL " + module_name + " (" + type(error).__name__ + "): " +
              (next((line.strip() for line in message if line.strip()), repr(error))))
        print("TRACE " + module_name + ": " +
              " | ".join(traceback.format_exc().strip().splitlines()[-7:]))
sys.exit(1 if failed else 0)
'@

    foreach ($candidate in $candidates) {
        if ([string]::IsNullOrWhiteSpace($candidate) -or -not $checked.Add($candidate)) {
            continue
        }
        try {
            # Capture stdout AND stderr with a non-terminating native-stream
            # preference, so failure details survive and exit status decides.
            $details = & $candidate -c $probe 2>&1
            if ($LASTEXITCODE -eq 0) {
                Write-Host "[Stage5] Using existing Python: $($details | Select-Object -Last 1)"
                return $candidate
            }
            $failureText = (@($details) | ForEach-Object { [string]$_ }) -join $nl
            $failures.Add("$candidate : $failureText")
        } catch {
            $failures.Add("$candidate : $($_.Exception.Message)")
        }
    }

    $message = "No existing Python interpreter could import both numpy and pybullet for Stage5."
    if ($failures.Count -gt 0) { $message += $nl + "Checked:" + $nl + ($failures -join $nl) }
    $message += $nl + "This command does NOT install or change any environment."
    $message += $nl + "To select a previously working interpreter explicitly, pass -Python 'C:\path\to\python.exe'."
    throw $message
}
) {
        return $Matches['exe']
    }
}

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
    $nl = [Environment]::NewLine
    # Probe imports independently. Both candidates and the exact failure are
    # printed without pip, file writes, or any environment activation.
    $probe = @'
import importlib
import sys
import traceback
print("PYTHON=" + sys.executable + " VERSION=" + sys.version.split()[0])
failed = False
for module_name in ("numpy", "pybullet"):
    try:
        module = importlib.import_module(module_name)
        print("OK " + module_name + " " + str(getattr(module, "__version__", "")))
    except Exception as error:
        failed = True
        message = str(error).strip().splitlines()
        print("FAIL " + module_name + " (" + type(error).__name__ + "): " +
              (next((line.strip() for line in message if line.strip()), repr(error))))
        print("TRACE " + module_name + ": " +
              " | ".join(traceback.format_exc().strip().splitlines()[-7:]))
sys.exit(1 if failed else 0)
'@

    foreach ($candidate in $candidates) {
        if ([string]::IsNullOrWhiteSpace($candidate) -or -not $checked.Add($candidate)) {
            continue
        }
        try {
            # Native PyBullet prints a build banner to stderr even on SUCCESS.
            # With ErrorActionPreference=Stop, 2>&1 can turn that benign banner
            # into a terminating PowerShell NativeCommandError (false negative).
            # The probe catches import failures and prints their details on stdout.
            $details = & $candidate -c $probe 2>$null
            if ($LASTEXITCODE -eq 0) {
                Write-Host "[Stage5] Using existing Python: $($details | Select-Object -Last 1)"
                return $candidate
            }
            $failureText = (@($details) | ForEach-Object { [string]$_ }) -join $nl
            $failures.Add("$candidate : $failureText")
        } catch {
            $failures.Add("$candidate : $($_.Exception.Message)")
        }
    }

    $message = "No existing Python interpreter could import both numpy and pybullet for Stage5."
    if ($failures.Count -gt 0) { $message += $nl + "Checked:" + $nl + ($failures -join $nl) }
    $message += $nl + "This command does NOT install or change any environment."
    $message += $nl + "To select a previously working interpreter explicitly, pass -Python 'C:\path\to\python.exe'."
    throw $message
}
