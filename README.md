# ROS1 DRL navigation policy (Unitree A1)

ROS Melodic package **dog_lplanner** runs ONNX inference for a SAC navigation policy on a Unitree A1. This document covers the repository layout, Docker workflow, build, and runtime usage.

## Dependencies

### System (preinstalled in the Docker image)

- `python3-tk` — policy control GUI
- `ros-melodic-*` — ROS Melodic desktop stack and messages used by the package

### Python (preinstalled in the image)

- `numpy<1.20`
- `onnxruntime==1.10.0`
- `pyyaml`
- `rospkg`

### ROS packages

- **unitree_legged_msgs** — `HighState` / `HighCmd` (built from the workspace; see Docker mounts below)

## Docker workflow

Compose file: `docker/docker-compose.yml`. It expects **`dog_lplanner`** and **`unitree_legged_msgs`** as sibling directories of `docker/` (i.e. `../dog_lplanner` and `../unitree_legged_msgs` from the compose context). Adjust the volume paths in the compose file if your layout differs.

Run all `docker compose` / helper scripts from the **`docker/`** directory so paths and the compose project resolve correctly:

```bash
cd docker
```

### First-time build

```bash
./build.sh
```

### Start the container (detached) and use a shell inside

```bash
./run_container.sh
./attach_container.sh
```

### Extra terminal in the same container

```bash
./attach_container.sh
```

### Stop and remove the stack

```bash
./stop_container.sh
```

**Graphics / RViz:** the compose file forwards `DISPLAY` and `/tmp/.X11-unix`. `run_container.sh` runs `xhost +local:docker` on the host. If RViz fails with GLX errors, uncomment `LIBGL_ALWAYS_SOFTWARE=1` under `environment` in `docker/docker-compose.yml`.

**Networking:** with `network_mode: host`, set **`ROS_IP`** to an address other machines can reach (not `127.0.0.1`). If unset, `docker/ros_detect_ip.sh` (sourced from the entrypoint and `.bashrc`) tries to pick a sensible default. **`ROS_MASTER_URI`** defaults in compose to `http://192.168.123.12:11311`; override if your roscore runs elsewhere.

## Build the catkin workspace (inside the container)

```bash
cd /root/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make install
source /root/catkin_ws/install/setup.bash
```

## Usage

1. **Launch policy inference, GUI, and (by default) RViz**

   ```bash
   roslaunch dog_lplanner policy_inference.launch
   ```

2. **RViz**

   - Lidar sectors (MarkerArray): **`/policy_inference/lidar_viz`** (override with private param `~lidar_viz_topic`)
   - Policy command arrow (MarkerArray): **`/policy_inference/cmd_viz`**
   - Clicked goal marker (MarkerArray): **`/policy_inference/goal_marker`** (published by the node; RViz “Publish Point” alone does not draw a marker)
   - Tool **Publish Point** → **`/clicked_point`**. The node transforms the goal into **`map`** when TF allows; RViz **Fixed Frame** should match your TF setup (often `map`).

3. **GUI:** **START** / **STOP** sets `/policy_running`. **Safe mode** sets `/policy_safe_mode` (when ON, commands are not sent to `/high_cmd`, but RViz markers still update).

4. **Quick checks**

   ```bash
   rostopic echo /high_cmd
   rostopic echo /policy_inference/lidar_viz
   ```

## Parameters

Main defaults live in **`dog_lplanner/config/a1_ros1.yaml`** (also referenced by launch file arguments):

- `cmd_scale`, lidar limits, `num_sectors`, `goal_reached_distance`, SAC `history_length`, etc.
- Topics: `/scan`, `/high_state`, `/clicked_point`, `/high_cmd`

Launch-specific visualization topics include `~cmd_viz_topic`, `~goal_viz_topic`, `~lidar_viz_topic`.

## Visualization summary

| Topic | Content |
|--------|--------|
| `/policy_inference/lidar_viz` | Per-sector arrows and labels (index, angle) |
| `/policy_inference/cmd_viz` | Velocity arrow `(vx, vy)` in `cmd_viz_frame` (default `base_link`) |
| `/policy_inference/goal_marker` | Sphere + “Goal” label for the last clicked point |

## Troubleshooting

### RViz: policy arrow or lidar rays missing / wrong place

- Command arrow is published in **`base_link`**. For a stable arrow in a map-fixed view, set **Fixed Frame** to **`base_link`** (or ensure TF from `map` → `base_link` is published).
- Raw **LaserScan** is usually in the **`lidar`** frame; use **Fixed Frame** **`lidar`** if you need the scan to align without extra TF.

If markers still look wrong, verify transforms:

```bash
rostopic echo /tf
```

### No messages on `/policy_inference/lidar_viz`, `/policy_inference/cmd_viz`, or `/high_cmd`

- Confirm the inference node is running and subscribed topics publish data (`/scan`, `/high_state`).
- On Docker/host setups, verify **`ROS_IP`** and **`ROS_MASTER_URI`** so nodes on different hosts see the same master (see `docker/ros_detect_ip.sh`).

## Further reading

- [`dog_lplanner/POLICY_SPEC_FOR_AGENTS.md`](dog_lplanner/POLICY_SPEC_FOR_AGENTS.md) — observation vector, lidar sectorization, and pointers to the training codebase (paths assume the original training repository layout).
