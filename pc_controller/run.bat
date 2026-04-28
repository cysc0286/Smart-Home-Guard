@echo off
setlocal

cd /d "%~dp0"

python zone_controller.py --config controller_config.json

if errorlevel 1 (
  echo.
  echo [ERROR] 上位机程序运行失败，请检查：
  echo 1) python 是否已安装
  echo 2) 依赖是否安装: pip install -r requirements.txt
  echo 3) controller_config.json 里的板端 IP / 快照地址 / 端口是否填写正确
)

echo.
pause
