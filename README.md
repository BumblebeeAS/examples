# BlueROV Examples

Quickstart demos for BlueROV mission trees and tmuxp sessions. The ArduSub
simulation image and sim packages come from the sibling `ardusub_sim` repo.

## Build Images

Build the minimal sim image first:

```bash
cd ~/workspaces/isaac_ros-dev/src/ardusub_sim
./build.bash
```

Build the examples image with perception and mission-tree dependencies:

```bash
cd ~/workspaces/isaac_ros-dev/src/examples
./build.bash
```

## Start The Container

Install `rocker` if needed: https://github.com/osrf/rocker

```bash
cd ~/workspaces/isaac_ros-dev/src/examples
./run.bash bluerov_ws:humble
```

## Build The Workspace

Inside the container:

```bash
cd /root/HOST/isaac_ros-dev
source /opt/ros/humble/setup.bash
vcs import --recursive src < src/examples/bluerov_ws.repos
colcon build --symlink-install
source install/setup.bash
```

`--recursive` is required: `image_matching` vendors the XFeat model code as a
git submodule (`accelerated_features`). Without it, `simple_matcher_node` dies
with `ModuleNotFoundError: feature_matcher.models.accelerated_features.modules`.
If you already imported without it, recover with:

```bash
cd src/image_matching && git submodule update --init --recursive
```

`bluerov_ws.repos` pins `foxglove-sdk` to 3.2.5 (`9323b15a`), which builds
clean with `--symlink-install`. Don't bump it to 3.4.x / `main` — those install
the bridge via `install(FILES $<TARGET_FILE:...>)`, which `--symlink-install`
cannot resolve.

Make sure the sibling `src/ardusub_sim` repo is present before building.

## Demo: Square Mission

Start QGroundControl on the host, then inside the container:

```bash
cd /root/HOST/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash
tmuxp load src/examples/bluerov_mission.yaml
```

The sim pane starts Gazebo, ArduSub, and MAVROS. The mission pane starts the
locomotion server and behavior tree that drives a square.

Useful checks:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
ros2 action list | grep controls
```

## Demo: Bin Mission

```bash
tmuxp load src/examples/bluerov_bin_mission.yaml
```

This starts separate panes for sim, controls, clustering, vision, and the bin
behavior tree.

## Demo: Torpedo Mission

```bash
tmuxp load src/examples/bluerov_torpedo_mission.yaml
```

This starts separate panes for sim, controls, clustering, torpedo vision, and
the torpedo behavior tree.

## Manual Launch

Terminal 1:

```bash
ros2 launch bluerov_sim bluerov_sim.launch.py \
  world_name:=robosub_2025_pool \
  ardusub:=true \
  mavros:=true \
  gui:=true
```

Terminal 2:

```bash
ros2 launch bluerov_tasks bluerov_square_bt.launch.py
```

## Useful Commands

```bash
ros2 topic echo /bluerov/odom --once
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
```
