@echo off
setlocal

REM ===== 可按需修改这三个参数 =====
set BOARD_IP=192.168.1.88
set BOARD_PORT=9000
set SNAPSHOT_URL=http://%BOARD_IP%:8081/?action=snapshot

cd /d "%~dp0"

python zone_controller.py --snapshot-url "%SNAPSHOT_URL%" --board-ip %BOARD_IP% --board-port %BOARD_PORT% --auto-send

if errorlevel 1 (
  echo.
  echo [ERROR] 上位机程序运行失败，请检查：
  echo 1) python 是否已安装
  echo 2) 依赖是否安装: pip install -r requirements.txt
  echo 3) 板端快照接口是否可访问: %SNAPSHOT_URL%
)

echo.
pause
