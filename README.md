# clean-UAV-simulation

Gazebo / ROS 2 / PX4 SITL simulation workspace for the `my_drone` octocopter with an SO101 robotic arm.

## Repository layout

- `drone_sim_ws/`: ROS 2 packages, URDF/Xacro, Gazebo models, control scripts, analysis data, and launch files.
- `零件/`: source SolidWorks assemblies and parts used as the geometric basis for the simulation model.
- `outputs/`: retained lightweight experiment outputs.
- `original_zip_model/`: reference copy of the earlier arm model.

Generated build trees, simulator logs, caches, temporary extraction directories, and the upstream PX4 source archive are intentionally excluded. CAD, STEP, and STL assets are stored with Git LFS.

## Current objective

Build a physically consistent eight-rotor vehicle with independently controlled arm joints, then validate PX4 SITL flight, keyboard WASD control, and coupled aircraft-arm dynamics.
