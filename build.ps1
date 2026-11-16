# Build script for peo_rover_esp32s3 (ESP-IDF v6.1, EIM environment)
# Usage: run in PowerShell from the project root:  .\build.ps1

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

# If this PowerShell was spawned from git-bash, MSYS/MSYSTEM vars are inherited,
# which makes idf.py silently exit without doing anything. Clear them first.
Remove-Item Env:MSYSTEM, Env:MSYS, Env:MINGW_PREFIX, Env:MINGW_CHOST, Env:MINGW_HOME -ErrorAction SilentlyContinue

# Activate the EIM v6.1 environment (sets PATH / IDF_PATH / IDF_TOOLS_PATH and the idf.py command)
& 'C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1'

idf.py build
if ($LASTEXITCODE -eq 0) {
    Write-Host "`n[OK] Build succeeded" -ForegroundColor Green
} else {
    Write-Host "`n[FAIL] Build failed (exit $LASTEXITCODE)" -ForegroundColor Red
    exit $LASTEXITCODE
}
