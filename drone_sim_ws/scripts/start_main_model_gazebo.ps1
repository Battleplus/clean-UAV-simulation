$ErrorActionPreference = "Stop"

$wslCommand = @'
export ROS_DOMAIN_ID=42
export GZ_PARTITION=42
export IGN_PARTITION=42
source /opt/ros/jazzy/setup.bash
cd /home/asus/drone_sim_ws_codex
source install/setup.bash
exec ros2 launch drone_arm_sim wrench_hover.launch.py
'@

Write-Host "Starting the original movable-arm my_drone model in Gazebo..."
wsl.exe -d Ubuntu-24.04 -- bash -lc $wslCommand

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "Gazebo launcher exited with code $LASTEXITCODE." -ForegroundColor Red
    Read-Host "Press Enter to close"
}
