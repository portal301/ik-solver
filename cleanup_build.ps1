# IKFast Solver - Build Artifacts Cleanup Script
# 빌드 아티팩트를 정리합니다 (최종 DLL은 보존)

Write-Host "Cleaning build artifacts..." -ForegroundColor Cyan

$cleaned = 0

# build/ directory
if (Test-Path "build") {
    Remove-Item -Path "build" -Recurse -Force
    Write-Host "✓ Removed build/" -ForegroundColor Green
    $cleaned++
}

# Root .exp, .lib files
if (Test-Path "ikfast_solver.exp") {
    Remove-Item "ikfast_solver.exp" -Force
    Write-Host "✓ Removed ikfast_solver.exp" -ForegroundColor Green
    $cleaned++
}
if (Test-Path "ikfast_solver.lib") {
    Remove-Item "ikfast_solver.lib" -Force
    Write-Host "✓ Removed ikfast_solver.lib" -ForegroundColor Green
    $cleaned++
}

# src/robots/ build artifacts
$robotArtifacts = Get-ChildItem -Path "src\robots" -Include *.obj,*.exp,*.lib -Recurse -ErrorAction SilentlyContinue
if ($robotArtifacts) {
    $robotArtifacts | Remove-Item -Force
    Write-Host "✓ Removed $($robotArtifacts.Count) robot build artifacts" -ForegroundColor Green
    $cleaned += $robotArtifacts.Count
}

# bin/ build artifacts (except final DLLs)
$binArtifacts = Get-ChildItem -Path "bin" -Include *.obj,*.exp,*.lib -Recurse -ErrorAction SilentlyContinue
if ($binArtifacts) {
    $binArtifacts | Remove-Item -Force
    Write-Host "✓ Removed $($binArtifacts.Count) bin build artifacts" -ForegroundColor Green
    $cleaned += $binArtifacts.Count
}

if ($cleaned -eq 0) {
    Write-Host "No build artifacts found (already clean)" -ForegroundColor Yellow
} else {
    Write-Host "`nCleanup completed: $cleaned item(s) removed" -ForegroundColor Green
}

Write-Host "`nFinal binaries preserved:" -ForegroundColor Cyan
Write-Host "  - ikfast_solver*.pyd (Python module)" -ForegroundColor Gray
Write-Host "  - bin/IKFastUnity_x64.dll (C#/Unity)" -ForegroundColor Gray
Write-Host "  - src/robots/*.dll (Robot plugins)" -ForegroundColor Gray
Write-Host "  - lib/*.dll (Dependencies)" -ForegroundColor Gray
