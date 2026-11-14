@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Create the Python environment and install requirements-vision.txt first.
    pause
    exit /b 1
)
".venv\Scripts\python.exe" run_backend.py --viewer --scene pickup
pause
