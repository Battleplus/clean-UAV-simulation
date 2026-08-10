@echo off
title my_drone WASD control
echo Starting my_drone WASD controller in Ubuntu-24.04...
python "%~dp0windows_wasd_bridge.py"
echo.
echo Controller exited. Press any key to close this window.
pause >nul
