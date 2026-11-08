@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    py -3.12 -m venv .venv
    if errorlevel 1 goto failed
)
".venv\Scripts\python.exe" -m pip install -r requirements.txt "PyYAML>=6,<7"
if errorlevel 1 goto failed
".venv\Scripts\python.exe" -m pip install "torch>=2.6,<3" --index-url https://download.pytorch.org/whl/cpu
if errorlevel 1 goto failed
echo.
echo Setup complete. Run RUN_RL.bat, then run the Flutter app.
pause
exit /b 0
:failed
echo.
echo Setup failed. Read the error above. Python 3.12 and internet access are required.
pause
exit /b 1
