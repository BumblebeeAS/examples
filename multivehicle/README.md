# multivehicle examples

Missions and control demos built on
[`multivehicle_sim`](https://github.com/BumblebeeAS/multivehicle_sim).

## Launch files

- `bluerov_mission.launch.py`: `multivehicle_sim` + `ground_truth_to_mavros` + `bluerov_tasks/bluerov_movement.py`
- `boat_control.launch.py`: BlueBoat thrust mixer + LOS controller (+ optional mission node)
- `px4_offboard.launch.py`: PX4 offboard demo from `uav2_offboard`

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

## Provenance

- `packages/multivehicle_examples/scripts/blueboat_mission.py` adapted from `bring-up/etc/uav2_sim`
- `packages/multivehicle_examples/scripts/blueboat_thrust_mixer.py` adapted from `bring-up/etc/uav2_sim`
- `packages/multivehicle_examples/scripts/blueboat_waypoint_controller.py` adapted from `bring-up/etc/uav2_sim`
- PX4 offboard demo is provided by `https://github.com/BumblebeeAS/uav2_offboard`
