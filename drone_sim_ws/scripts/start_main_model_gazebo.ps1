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

$wslEnvironment = @(
    "HEADLESS=false",
    "ENABLE_ARM_CONTROL=true",
    "CLEAN_STALE_RUNTIME=1"
)
if ($ArmCompensationTest) {
    $candidatePath = (Resolve-Path (Join-Path $PSScriptRoot "..\px4\airframes\4028_gz_my_drone_octorotor_debug_4kg_armcomp")).Path
    $candidateDrive = $candidatePath.Substring(0, 1).ToLowerInvariant()
    $candidateRelativePath = $candidatePath.Substring(3).Replace("\", "/")
    $candidateWslPath = "/mnt/$candidateDrive/$candidateRelativePath"
    $wslEnvironment += @(
        "AIRFRAME_ID=4028",
        "PROJECT_AIRFRAME_FILE=$candidateWslPath",
        "GZ_RANDOM_SEED=4028"
    )
    Write-Host "Arm compensation candidate: PX4 airframe 4028." -ForegroundColor Green
}
$wslArgs = @("-d", "Ubuntu-24.04", "--", "env") + $wslEnvironment
$wslArgs += @("bash", $scriptPath)

# Keep this terminal attached to the backend log. Gazebo itself is launched
# through WSLg and opens in its own graphical window.
& wsl.exe @wslArgs

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "Gazebo/PX4 launcher exited with code $LASTEXITCODE." -ForegroundColor Red
    Read-Host "Press Enter to close"
}
