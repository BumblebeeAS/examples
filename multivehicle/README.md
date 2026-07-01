# Multi-vehicle examples

Missions and control demos built on
[`multivehicle_sim`](https://github.com/BumblebeeAS/multivehicle_sim).

## Launch files

- BlueROV square mission: run `bluerov_tasks`'s tree launch directly
  `ros2 launch bluerov_tasks bluerov_square_bt.launch.py` 
- `boat_control.launch.py`: BlueBoat thrust mixer + LOS controller (+ optional mission node)
- `px4_offboard.launch.py`: the `uav2_offboard` `offboard_node` action backend + the
  `mission_planner_2` UAV2 offboard demo behaviour tree (`uav2_offboard_demo_main.py`)
  driving it (takeoff → standoff → return → land). 

## Setup

```bash
cd ~/mvsim_ws
vcs import src < src/examples/multivehicle/examples.repos --recursive

cd src/multivehicle_sim
./build.bash

cd ../examples/multivehicle
./build.bash
./run.bash multivehicle_examples:humble
```

Inside the container:

```bash
cd /root/HOST/mvsim_ws
colcon build --symlink-install --packages-up-to multivehicle_examples microxrcedds_agent bb_robotx_dashboard
source install/setup.bash
tmuxp load src/examples/multivehicle/tmuxp/mvsim_debug.yaml
```
