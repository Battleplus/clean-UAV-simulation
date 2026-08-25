$ErrorActionPreference = "Stop"

$wslExe = Join-Path $env:SystemRoot "System32\wsl.exe"
if (-not (Test-Path -LiteralPath $wslExe)) {
    throw "wsl.exe not found at $wslExe"
}

function Convert-ToWslPath([string]$WindowsPath) {
    $resolved = (Resolve-Path -LiteralPath $WindowsPath).Path
    $drive = $resolved.Substring(0, 1).ToLowerInvariant()
    $relative = $resolved.Substring(3).Replace("\", "/")
    return "/mnt/$drive/$relative"
}

$backend = Convert-ToWslPath (Join-Path $PSScriptRoot "wsl_start_ros2_dds_candidate_1p3kg.sh")
$wasd = Convert-ToWslPath (Join-Path $PSScriptRoot "run_ros2_dds_wasd.sh")
$arm = Convert-ToWslPath (Join-Path $PSScriptRoot "run_ros2_arm_keyboard.sh")
$workspace = Split-Path -Parent $PSScriptRoot
$candidateUrdf = Convert-ToWslPath (Join-Path $workspace "src\drone_arm_sim\urdf\my_drone_v3\my_drone_cad_candidate_1p3kg.urdf")
$candidateConfig = Convert-ToWslPath (Join-Path $workspace "src\drone_arm_sim\config\my_drone_v3_cad_candidate_1p3kg.json")
$motionReference = Convert-ToWslPath (Join-Path $workspace "src\drone_arm_sim\config\so101_motion_reference_4kg.json")

Write-Host "Starting 1.3 kg / 0.6 kg-arm Gazebo + PX4 candidate..."
Start-Process -FilePath $wslExe -WindowStyle Hidden -ArgumentList @(
    "-d", "Ubuntu-24.04", "--", "env",
    "HEADLESS=false", "ENABLE_ARM_CONTROL=true", "CLEAN_STALE_RUNTIME=1",
    "bash", $backend
)

Start-Sleep -Seconds 8

# These two terminals are intentionally visible because they own keyboard input.
Start-Process -FilePath $wslExe -ArgumentList @(
    "-d", "Ubuntu-24.04", "--", "env",
    "PX4_TRUTH_HOLD_ENABLED=true",
    "ARM_DIRECT_XY_OWNERSHIP=true",
    "ARM_DIRECT_XY_EXTERNAL_GUARDIAN=true",
    "bash", $wasd
)
Start-Process -FilePath $wslExe -ArgumentList @(
    "-d", "Ubuntu-24.04", "--", "env",
    "SO101_KINEMATICS_URDF=$candidateUrdf",
    "SO101_MOTION_REFERENCE=$motionReference",
    "MY_DRONE_FLIGHT_CONFIG=$candidateConfig",
    "CONFIG_FILE=$candidateConfig",
    "ARM_DIRECT_XY_OWNERSHIP=true",
    "ARM_DIRECT_XY_EXTERNAL_GUARDIAN=true",
    "bash", $arm
)

Write-Host "Gazebo backend, WASD and SO101 candidate terminals were started."
