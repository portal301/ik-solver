#!/usr/bin/env pwsh
# ============================================================================
# Python Multi-Version Build Script (Python 3.10, 3.11, 3.12)
#
# This script builds ikfast_solver.pyd for multiple Python versions using uv
# ============================================================================

$ErrorActionPreference = "Stop"

Write-Host "============================================================================" -ForegroundColor Cyan
Write-Host "Building ikfast_solver.pyd for Python 3.10, 3.11, 3.12" -ForegroundColor Cyan
Write-Host "============================================================================" -ForegroundColor Cyan
Write-Host ""

# Check if uv is installed
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    Write-Host "[ERROR] uv is not installed or not in PATH" -ForegroundColor Red
    Write-Host "Please install uv: https://github.com/astral-sh/uv" -ForegroundColor Yellow
    exit 1
}

# Get project root
$ProjectRoot = $PSScriptRoot

# Set Visual Studio environment
$VSTools = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat"
if (-not (Test-Path $VSTools)) {
    Write-Host "[ERROR] Visual Studio Build Tools not found" -ForegroundColor Red
    Write-Host "Expected: $VSTools" -ForegroundColor Yellow
    exit 1
}

Write-Host "[INFO] Visual Studio Build Tools found" -ForegroundColor Green

# Python versions to build
$PythonVersions = @("3.10", "3.11", "3.12")
$BuiltFiles = @()
$SuccessCount = 0
$TotalCount = $PythonVersions.Count

# Build for each Python version
foreach ($Version in $PythonVersions) {
    Write-Host ""
    Write-Host "============================================================================" -ForegroundColor Cyan
    Write-Host "Building for Python $Version" -ForegroundColor Cyan
    Write-Host "============================================================================" -ForegroundColor Cyan

    # Create temporary venv for this version
    $VersionTag = $Version.Replace(".", "")
    $VenvDir = Join-Path $ProjectRoot ".venv_$VersionTag"

    Write-Host "[INFO] Creating venv for Python $Version..." -ForegroundColor Yellow

    # Remove existing venv if present
    if (Test-Path $VenvDir) {
        Remove-Item -Recurse -Force $VenvDir
    }

    # Create venv
    & uv venv $VenvDir --python $Version 2>&1 | Out-Null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] Failed to create venv for Python $Version" -ForegroundColor Red
        continue
    }

    # Activate venv and build
    $VenvPython = Join-Path $VenvDir "Scripts\python.exe"

    if (-not (Test-Path $VenvPython)) {
        Write-Host "[ERROR] Python executable not found in venv: $VenvPython" -ForegroundColor Red
        Remove-Item -Recurse -Force $VenvDir -ErrorAction SilentlyContinue
        continue
    }

    # Install dependencies
    Write-Host "[INFO] Installing dependencies..." -ForegroundColor Yellow
    & uv pip install --python $VenvPython numpy pybind11 setuptools 2>&1 | Out-Null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] Failed to install dependencies for Python $Version" -ForegroundColor Red
        Remove-Item -Recurse -Force $VenvDir -ErrorAction SilentlyContinue
        continue
    }

    # Set environment for VS tools and build
    Write-Host "[INFO] Building Python extension..." -ForegroundColor Yellow

    $BuildCmd = @"
call "$VSTools" x64 >nul 2>&1
cd /d "$ProjectRoot"
set PYTHON_HOME=$VenvDir
"$VenvPython" setup.py build_ext --inplace --force
"@

    $TempBat = Join-Path $ProjectRoot "temp_build_$VersionTag.bat"
    Set-Content -Path $TempBat -Value $BuildCmd -Encoding ASCII

    & cmd /c $TempBat
    $BuildResult = $LASTEXITCODE
    Remove-Item $TempBat -ErrorAction SilentlyContinue

    if ($BuildResult -ne 0) {
        Write-Host "[ERROR] Build failed for Python $Version" -ForegroundColor Red
        Remove-Item -Recurse -Force $VenvDir -ErrorAction SilentlyContinue
        continue
    }

    # Find and copy built pyd file
    $PydPattern = "ikfast_solver.cp$VersionTag-win_amd64.pyd"
    $PydFiles = Get-ChildItem -Path $ProjectRoot -Filter $PydPattern -Recurse -File -ErrorAction SilentlyContinue

    if ($PydFiles.Count -gt 0) {
        foreach ($PydFile in $PydFiles) {
            Write-Host "[SUCCESS] Built: $($PydFile.Name)" -ForegroundColor Green

            # Copy to root folder
            Copy-Item -Path $PydFile.FullName -Destination (Join-Path $ProjectRoot $PydFile.Name) -Force
            Write-Host "[INFO] Copied to root: $($PydFile.Name)" -ForegroundColor Yellow
            $BuiltFiles += $PydFile.Name
        }
        $SuccessCount++
    } else {
        Write-Host "[WARNING] No .pyd file found for Python $Version" -ForegroundColor Yellow
    }

    # Clean up temporary venv
    Write-Host "[INFO] Cleaning up temporary venv..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force $VenvDir -ErrorAction SilentlyContinue
}

Write-Host ""
Write-Host "============================================================================" -ForegroundColor Cyan
Write-Host "Build Summary" -ForegroundColor Cyan
Write-Host "============================================================================" -ForegroundColor Cyan
Write-Host ""

if ($BuiltFiles.Count -gt 0) {
    Write-Host "Built Python modules:" -ForegroundColor Green
    foreach ($File in $BuiltFiles) {
        Write-Host "  - $File" -ForegroundColor White
    }
    Write-Host ""

    # Copy Python 3.12 version as default
    $DefaultPyd = Join-Path $ProjectRoot "ikfast_solver.cp312-win_amd64.pyd"
    if (Test-Path $DefaultPyd) {
        Copy-Item -Path $DefaultPyd -Destination (Join-Path $ProjectRoot "ikfast_solver.pyd") -Force
        Write-Host "[INFO] Default version (Python 3.12): ikfast_solver.pyd" -ForegroundColor Green
    }
} else {
    Write-Host "[WARNING] No .pyd files were built successfully" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "============================================================================" -ForegroundColor Cyan
Write-Host "Build completed: $SuccessCount/$TotalCount successful" -ForegroundColor Cyan
Write-Host "============================================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Usage:" -ForegroundColor Yellow
Write-Host "  - Python 3.10 users: Copy ikfast_solver.cp310-win_amd64.pyd to your project" -ForegroundColor White
Write-Host "  - Python 3.11 users: Copy ikfast_solver.cp311-win_amd64.pyd to your project" -ForegroundColor White
Write-Host "  - Python 3.12 users: Use ikfast_solver.pyd (default) or ikfast_solver.cp312-win_amd64.pyd" -ForegroundColor White
Write-Host ""

# Exit with success if at least one build succeeded
if ($SuccessCount -gt 0) {
    exit 0
} else {
    exit 1
}
