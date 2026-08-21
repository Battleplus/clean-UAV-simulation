# Base 1 armcomp derivative launcher
# This launcher sets the specific PX4 gains and WASD shaping for the armcomp profile.
# The original 4027 airframe remains untouched as the base-1 control.

param(
    [switch]$AttachGazeboGui
)

$ErrorActionPreference = "Stop"

# PX4 gains for armcomp derivative (different from base-1)
$env:PX4_ARMCOMP_AIRFRAME = "4028"

# WASD input shaping for armcomp (explicitly override shared defaults)
$env:PX4_WASD_HORIZONTAL_ACCEL_M_S2 = "0.15"
$env:PX4_WASD_HORIZONTAL_JERK_M_S3 = "0.30"
$env:PX4_WASD_YAW_ACCEL_DEG_S2 = "15.0"

# Launch the standard DDS WASD controller with armcomp settings
& "$PSScriptRoot\start_ros2_dds_wasd.ps1" @PSBoundParameters
