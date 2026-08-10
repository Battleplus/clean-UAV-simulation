$ErrorActionPreference = "Stop"

$windowsScriptPath = (Resolve-Path (Join-Path $PSScriptRoot "run_ros2_arm_keyboard.sh")).Path
$drive = $windowsScriptPath.Substring(0, 1).ToLowerInvariant()
$relativePath = $windowsScriptPath.Substring(3).Replace("\", "/")
$scriptPath = "/mnt/$drive/$relativePath"

Start-Process -FilePath "wsl.exe" -ArgumentList @(
    "-d", "Ubuntu-24.04", "--", "bash", $scriptPath
)

Write-Host "ROS 2 SO101 keyboard terminal started."
