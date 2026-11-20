# ==============================================================================
# Script: install_all_dependencies.ps1
# Target: Windows (PowerShell)
# Deskripsi: Sekali eksekusi untuk menginstall seluruh dependensi Python AER Rover
# ==============================================================================

Write-Host "================================================================" -ForegroundColor Cyan
Write-Host "   AER ROVER - INSTALLASI SEMUA DEPENDENCY (WINDOWS POWERSHELL) " -ForegroundColor Cyan
Write-Host "================================================================" -ForegroundColor Cyan

# 1. Periksa Python
Write-Host "`n[1/3] Memeriksa instalasi Python..." -ForegroundColor Yellow
$py = Get-Command python -ErrorAction SilentlyContinue
if (-not $py) {
    Write-Host "Error: Python tidak ditemukan di PATH sistem!" -ForegroundColor Red
    Write-Host "Silakan download dan install Python 3.10+ dari https://www.python.org/" -ForegroundColor Yellow
    Exit 1
}

$pyVersion = python --version
Write-Host "Python terdeteksi: $pyVersion" -ForegroundColor Green

# 2. Upgrade PIP
Write-Host "`n[2/3] Meng-upgrade pip dan build tools..." -ForegroundColor Yellow
python -m pip install --upgrade pip setuptools wheel

# 3. Install Requirements
Write-Host "`n[3/3] Menginstall seluruh dependensi (Flask, OpenCV, PySerial, YOLO, PyTorch)..." -ForegroundColor Yellow
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$reqPath = Join-Path $scriptDir "requirements.txt"

if (Test-Path $reqPath) {
    python -m pip install -r $reqPath
} else {
    python -m pip install flask psutil pyserial opencv-python numpy ultralytics torch torchvision transforms3d websockets
}

Write-Host "`n================================================================" -ForegroundColor Green
Write-Host "   SEMUA DEPENDENSI BERHASIL DIINSTALL DI WINDOWS!           " -ForegroundColor Green
Write-Host "================================================================" -ForegroundColor Green
Write-Host "Untuk menjalankan web monitoring langsung:" -ForegroundColor Yellow
Write-Host "   cd 'New Monitoring System'; python main.py`n" -ForegroundColor White
