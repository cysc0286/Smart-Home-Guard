@echo off
setlocal

cd /d "%~dp0"

python zone_controller.py --config controller_config.json

if errorlevel 1 (
  echo.
  echo [ERROR] PC controller failed to start.
  echo 1^) Check Python is installed.
  echo 2^) Check dependencies: pip install -r requirements.txt
  echo 3^) Check controller_config.json values.
)

echo.
pause
