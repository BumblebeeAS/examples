# Examples

## BlueROV with ArduSub

### Quickstart

We assume the ROS workspace is `~/workspaces/bluerov_ws`. Change the paths accordingly if needed.

Clone the repositories:

```bash
cd ~/workspaces/bluerov_ws/src
vcs import --recursive < examples/bluerov_ws.repos
```

Build the minimal sim image first:

```bash
cd ~/workspaces/bluerov_ws/src/ardusub_sim
./build.bash
```

Build the examples image with perception and mission-tree dependencies:

```bash
cd ~/workspaces/bluerov_ws/src/examples
./build.bash
```

### Start the container

#### Native Ubuntu with NVIDIA

Install [Rocker](https://github.com/osrf/rocker), then run:

```bash
cd ~/workspaces/bluerov_ws/src/examples
./run.bash bluerov_ws:humble
```

#### WSL 2 with WSLg

```bash
docker run --rm -it \
  --gpus all \
  --device=/dev/dxg \
  --network=host \
  --ipc=host \
  -e DISPLAY="$DISPLAY" \
  -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  -e PULSE_SERVER="$PULSE_SERVER" \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /mnt/wslg:/mnt/wslg \
  -v /usr/lib/wsl:/usr/lib/wsl:ro \
  -v ~/workspaces/bluerov_ws:/root/HOST/bluerov_ws \
  bluerov_ws:humble
```

The WSL command exposes WSLg's X11/Wayland sockets and the `/dev/dxg` virtual
GPU device, allowing Gazebo to use hardware-accelerated rendering.

### Verify GPU rendering

Inside the container, before launching Gazebo:

```bash
glxinfo -B | grep -E "OpenGL vendor|OpenGL renderer"
```

On native Linux, the renderer should identify the NVIDIA GPU. Under WSLg, it
should mention D3D12 and the GPU. It should not report `llvmpipe`, which is
software rendering.

### Build the workspace

Inside the container:

```bash
cd /root/HOST/bluerov_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to bluerov_tasks bluerov_sim bb_worlds
source install/setup.bash
```

### Demo: Square Mission

Inside the container:

```bash
cd /root/HOST/bluerov_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
tmuxp load src/examples/bluerov_mission.yaml
```

### Demo: Bin Mission

```bash
tmuxp load src/examples/bluerov_bin_mission.yaml
```

### Demo: Torpedo Mission

```bash
tmuxp load src/examples/bluerov_torpedo_mission.yaml
```

## Useful Commands

```bash
ros2 topic echo /bluerov/odom --once
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
```
