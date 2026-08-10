param(
    [switch]$AttachGazeboGui
)

$ErrorActionPreference = "Stop"

$windowsScriptPath = (Resolve-Path (Join-Path $PSScriptRoot "run_ros2_dds_wasd.sh")).Path
$drive = $windowsScriptPath.Substring(0, 1).ToLowerInvariant()
$relativePath = $windowsScriptPath.Substring(3).Replace("\", "/")
$scriptPath = "/mnt/$drive/$relativePath"

if ($AttachGazeboGui) {
    # Attach a GUI only when the backend was launched with HEADLESS=true.
    Start-Process -FilePath "wsl.exe" -ArgumentList @(
        "-d", "Ubuntu-24.04", "--", "bash", "-lc",
        "source /opt/ros/jazzy/setup.bash; gz sim -g --force-version 8"
    ) -WindowStyle Hidden
}

# Keep the keyboard controller in a visible terminal so it owns stdin.
Start-Process -FilePath "wsl.exe" -ArgumentList @(
    "-d", "Ubuntu-24.04", "--", "bash", $scriptPath
)

Write-Host "ROS 2 DDS WASD terminal started."
