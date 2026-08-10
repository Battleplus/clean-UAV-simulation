@echo off
if /i not "%~1"=="--conhost" (
  start "" "%SystemRoot%\System32\conhost.exe" "%ComSpec%" /d /c ""%~f0" --conhost"
  exit /b
)
title my_drone WASD control
echo Starting my_drone WASD controller in Ubuntu-24.04...
python "%~dp0windows_wasd_bridge.py"
echo.
echo Controller exited. Press any key to close this window.
pause >nul
