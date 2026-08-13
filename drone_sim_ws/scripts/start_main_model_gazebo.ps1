param(
    [switch]$ArmCompensationTest
)

$ErrorActionPreference = "Stop"

# Resolve the launcher beside this file so this entry point can never fall
# back to the retired /home/asus/drone_sim_ws_codex workspace.
$windowsScriptPath = (Resolve-Path (Join-Path $PSScriptRoot "wsl_start_ros2_dds_debug_4kg.sh")).Path
$drive = $windowsScriptPath.Substring(0, 1).ToLowerInvariant()
$relativePath = $windowsScriptPath.Substring(3).Replace("\", "/")
$scriptPath = "/mnt/$drive/$relativePath"

Write-Host "Starting current my_drone 4 kg Gazebo/PX4 backend..."
Write-Host "Source: $windowsScriptPath"

$wslArgs = @(
    "-d", "Ubuntu-24.04", "--", "env",
    "HEADLESS=false",
    "ENABLE_ARM_CONTROL=true",
    "CLEAN_STALE_RUNTIME=1"
)
if ($ArmCompensationTest) {
    # Kept for command-line compatibility.  The rejected legacy feed-forward
    # experiment is no longer enabled; the validated gravity-only Base 1
    # overlay starts automatically whenever arm control is enabled.
    Write-Host "Arm compensation is now part of the validated Base 1 startup." -ForegroundColor Green
}
$wslArgs += @("bash", $scriptPath)

# Keep this terminal attached to the backend log. Gazebo itself is launched
# through WSLg and opens in its own graphical window.
& wsl.exe @wslArgs

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "Gazebo/PX4 launcher exited with code $LASTEXITCODE." -ForegroundColor Red
    Read-Host "Press Enter to close"
}
