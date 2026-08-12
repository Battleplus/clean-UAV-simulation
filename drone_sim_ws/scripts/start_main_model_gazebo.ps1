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
    # Conservative, explicitly opt-in 4 kg experiment.  Keep the disturbance
    # observer off so this run tests only model-based force/torque terms.
    $wslArgs += @(
        "ARM_TORQUE_FEEDFORWARD_ENABLED=true",
        "ARM_REACTION_TORQUE_FEEDFORWARD_GAIN=0.25",
        "ARM_TORQUE_FEEDFORWARD_MAX_DELTA_N=0.05",
        "ARM_STATIC_COM_FEEDFORWARD_GAIN=0.25",
        "ARM_STATIC_COM_FEEDFORWARD_TIME_CONSTANT_S=2.0",
        "ARM_DISTURBANCE_OBSERVER_ENABLED=false",
        "PX4_READY_STABLE_TIMEOUT_S=180"
    )
    Write-Host "Arm compensation experiment: force FF + 25% reaction/COM torque FF" -ForegroundColor Yellow
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
