# multivehicle examples

Example missions, controllers and demos that run **on top of** the
[`multivehicle_sim`](https://github.com/BumblebeeAS/multivehicle_sim) simulator.

## Contents

```
multivehicle/
  build.bash  run.bash       # multivehicle_examples:humble, FROM multivehicle_sim:humble
  examples.repos             # full workspace manifest (sim + deps + px4_msgs)
  docker/Dockerfile          # adds nav2 (BlueBoat control nav2-prep)
  tmuxp/mvsim_debug.yaml      # full stack: sim + examples + dashboard
  packages/
    multivehicle_examples/   # ament_cmake
      launch/  bluerov_mission, boat_control, px4_offboard
      scripts/ bluerov_movement, blueboat_thrust_mixer, blueboat_waypoint_controller, blueboat_mission
      src/     PX4OffboardDemo.cpp/.hpp   (px4_msgs + Eigen3)
      config/  blueboat_control.yaml
```

What each launch adds on top of the sim:

| Launch | Builds on | Adds |
|--------|-----------|------|
| `bluerov_mission.launch.py` | `multivehicle_sim bluerov.launch.py` | BlueROV2 square-dive mission |
| `boat_control.launch.py` | `multivehicle_sim boat.launch.py` | thrust mixer + LOS waypoint controller (+ optional `use_mission:=true`) |
| `px4_offboard.launch.py` | `multivehicle_sim` x500 + uXRCE agent | autonomous offboard flight (`px4_offboard_demo`) |

## Setup

```bash
# In ~/mvsim_ws (this repo cloned at src/examples):
vcs import src < src/examples/multivehicle/examples.repos --recursive

# Build the base sim image first, then the examples image:
cd src/multivehicle_sim && ./build.bash
cd ../examples/multivehicle && ./build.bash
./run.bash multivehicle_examples:humble
```

Inside the container:

```bash
cd /root/HOST/mvsim_ws
colcon build --symlink-install --packages-up-to multivehicle_examples microxrcedds_agent bb_robotx_dashboard

source install/setup.bash
tmuxp load src/examples/multivehicle/tmuxp/mvsim_debug.yaml
```

On a hybrid-GPU (NVIDIA) host, `export __NV_PRIME_RENDER_OFFLOAD=1` and
`export __GLX_VENDOR_LIBRARY_NAME=nvidia` before launching gz (or uncomment those
lines in the tmuxp's `shell_command_before`).

## License

MIT (see the repo-root `LICENSE`). The `px4_offboard_demo` is a first-party,
independent implementation of the standard PX4 offboard pattern.
