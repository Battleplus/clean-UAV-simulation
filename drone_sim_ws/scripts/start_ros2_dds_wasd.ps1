param(
    [switch]$AttachGazeboGui,
    [switch]$ArmCompensationTest
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

# Keep the keyboard controller in a visible terminal so it owns stdin.  The
# force feed-forward switch belongs to this process, independently of the
# backend torque feed-forward switch.
$wasdArgs = @(
    "-d", "Ubuntu-24.04", "--", "env",
    "PX4_TRUTH_HOLD_ENABLED=true"
)
if ($ArmCompensationTest) {
    # The old ARM_FEEDFORWARD_ENABLED acceleration path is removed.
    # Compensation is now handled exclusively by the pre-allocation
    # wrench reallocator overlay (activate_base1_wrench_reallocator_overlay.sh).
    Write-Host "Arm compensation: using new 6D wrench reallocator overlay (not legacy feed-forward)." -ForegroundColor Green
}
$wasdArgs += @("bash", $scriptPath)
Start-Process -FilePath "wsl.exe" -ArgumentList $wasdArgs

Write-Host "ROS 2 DDS WASD terminal started. Arm compensation test=$ArmCompensationTest"
