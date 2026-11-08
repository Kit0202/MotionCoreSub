@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Run SETUP_RL.bat first.
    pause
    exit /b 1
)
set MOTIONCORE_RL_HEADLESS=0
".venv\Scripts\python.exe" run_backend.py --viewer --walk-mode rl
pause
