# Base 1 armcomp derivative backend launcher. The original 4027 airframe
# remains untouched; start_main_model_gazebo selects the separate 4028 file.

$ErrorActionPreference = "Stop"

& "$PSScriptRoot\start_main_model_gazebo.ps1" -ArmCompensationTest
