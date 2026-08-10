@echo off
title my_drone SO101 Cartesian jog
echo Starting SO101 Cartesian jog controller...
wsl.exe -d Ubuntu-24.04 --cd "/home/asus/my_drone_ws" bash -lc "source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec ros2 run drone_arm_sim cartesian_arm_jog"
echo.
echo Arm jog controller exited. Press any key to close this window.
pause >nul
