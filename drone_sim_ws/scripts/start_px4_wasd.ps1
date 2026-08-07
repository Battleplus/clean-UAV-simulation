$ErrorActionPreference = "Stop"

$wslCommand = @'
cd '/mnt/e/清洁无人机/drone_sim_ws'
exec bash scripts/wsl_px4_wasd.sh
'@

Write-Host "Starting my_drone PX4 + Gazebo WASD control..."
Write-Host "Wait for the W/S control prompt, then click this terminal and press keys."
Write-Host "W/S forward/back, A/D left/right, R/F up/down, Q/E yaw, L land."
Write-Host ""

wsl.exe -d Ubuntu-24.04 -- bash -lc $wslCommand

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "The simulator exited with code $LASTEXITCODE." -ForegroundColor Red
    Read-Host "Press Enter to close"
}
