# spacemouse_ros2

A minimal ROS 2 package that maps 3Dconnexion SpaceMouse input to ROS 2 topics. It publishes a continuously integrated `PoseStamped` target and optional gripper commands — nothing more. It assumes you have your own tracking controller that follows the target pose, and optionally a gripper controller that acts on the gripper topic.

## Architecture

Two nodes run together per robot:

```
SpaceMouse device
      │  (HID)
      ▼
pyspacemouse_publisher          publishes Twist at 100 Hz + gripper commands
      │  space_mouse/target_cartesian_velocity_percent
      ▼
twist_to_pose_node              integrates Twist → PoseStamped, clips, snaps
      │  /target_pose  (configurable)
      ▼
Robot controller
      │  /current_pose  (configurable)
      └──────────────────────────────► twist_to_pose_node (feedback loop)
```

**`pyspacemouse_publisher`** — reads the SpaceMouse via the `pyspacemouse` library and publishes raw 6-DOF velocity as a `geometry_msgs/Twist`. The two buttons control a gripper topic (`std_msgs/Float32`) in either absolute mode (one press = fully open/close) or relative mode (hold to gradually open/close). See [Gripper parameters](#parameter-reference) for details.

**`twist_to_pose_node`** — subscribes to the Twist and integrates it into a `PoseStamped` target. On startup it seeds the target from the first `/current_pose` message so there is no jump. Features:
- Configurable input frame (world or EE-relative) for translation and rotation independently
- Per-axis deadband with smooth onset (threshold subtracted, no step)
- Safety clipping: target cannot lead the actual pose by more than `max_distance` (m) or `max_rotation` (rad)
- Idle snap: target snaps back to actual pose as soon as inputs fall below `linear/angular_snap_threshold`
- Full quaternion orientation tracking — no gimbal lock or Euler wrapping discontinuities

---

## Prerequisites

### udev rule (one-time setup)

By default, `/dev/hidraw*` devices are root-only on Linux. Add a udev rule so the SpaceMouse is accessible without sudo:

```bash
printf 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="046d", MODE="0666"\nSUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", MODE="0666"\n' \
  | sudo tee /etc/udev/rules.d/99-spacemouse.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`046d` and `256f` are the two vendor IDs used across all 3Dconnexion devices. Replug the SpaceMouse after running this.

### Pixi

[Pixi](https://pixi.sh) is used to manage the ROS 2 Jazzy environment and Python dependencies. All dependencies are declared in `pyproject.toml`.

```bash
# Install pixi if not already installed
curl -fsSL https://pixi.sh/install.sh | bash
```

---

## Setup

### With pixi (recommended)

Pixi manages the full ROS 2 Jazzy environment and all dependencies in an isolated environment — no system ROS installation required.

```bash
git clone <repo-url> spacemouse_ros2
cd spacemouse_ros2
pixi install
pixi run build
```

### Without pixi (plain colcon)

Requires ROS 2 Jazzy already sourced in your shell.

```bash
# 1. Clone into your existing colcon workspace
cd ~/ros2_ws/src
git clone <repo-url> spacemouse_ros2

# 2. Install all system and Python deps (including libhidapi-hidraw0)
cd ~/ros2_ws
rosdep install --from-paths src/spacemouse_ros2 --ignore-src -r -y

# 3. Build
colcon build --symlink-install

# 4. Run
source install/setup.bash
ros2 launch spacemouse_publisher spacemouse_publisher.launch.py
```

All dependencies — including `libhidapi-hidraw0` (the C library needed by `pyspacemouse`) and `pyspacemouse` itself — are declared in `package.xml` and `setup.py` respectively, so `rosdep install` and `colcon build` handle everything.

---

## Running

```bash
pixi run start
```

By default this launches with `example_config.yaml`. Pass a different config:

```bash
pixi run start config_file:=my_robot_config.yaml
```

Or launch directly with ros2:

```bash
source install/setup.bash
ros2 launch spacemouse_publisher spacemouse_publisher.launch.py config_file:=my_robot_config.yaml
```

---

## Configuration

The easiest starting point is to edit `example_config.yaml` directly. If you want a separate file, copy it, rename it, and pass the filename via `config_file`. Place it in `src/spacemouse_publisher/config/`.

Each top-level key defines one SpaceMouse + integrator pair. Multiple keys launch multiple pairs (e.g. for a dual-arm setup). All parameters are optional except `namespace` and `device_path`.

### Adapting to your robot

Four things typically need to change:

| What | Parameter | Notes |
|---|---|---|
| ROS namespace | `namespace` | Must match your robot controller's namespace (e.g. `r100_0207`). Leave empty if your controller runs in the global namespace. |
| Pose topics | `current_pose_topic`, `target_pose_topic` | Set to the topics your controller publishes/subscribes. The node seeds its target from `current_pose_topic` on startup. |
| SpaceMouse device | `device_path` | Only needed with multiple connected devices. Leave empty to auto-detect. |
| Gripper | `gripper_interface`, `gripper_action` / `gripper_topic` | Set `gripper_interface: "action"` and point `gripper_action` at your gripper controller's action server. If your gripper uses a different joint range, adjust `gripper_max_position`. Set `gripper_interface: "topic"` if you have your own bridge node. |

Everything else (sensitivity, deadband, clipping, snap) has sensible defaults and can be tuned later.

### Identify connected SpaceMouse devices

Required only when multiple devices are connected. Run:

```bash
grep -H . /sys/class/hidraw/hidraw*/device/uevent | grep SpaceMouse
```

Example output: `/sys/class/hidraw/hidraw1/device/uevent:HID_NAME=3Dconnexion SpaceMouse Wireless BT`
→ set `device_path: /dev/hidraw1`

---

### Parameter reference

| Parameter | Type | Default | Description |
|---|---|---|---|
| `namespace` | string | `""` | ROS 2 namespace. Must match the robot controller's namespace. |
| `device_path` | string | `""` | HID path of the SpaceMouse (e.g. `/dev/hidraw1`). Empty = auto-detect first device. |
| `operator_position_front` | bool | `true` | `true` if the operator faces the front of the robot base. `false` inverts X/Y so "push forward" always means away from the operator. |
| `input_frame_rpy` | float[3] | `[0, 0, 0]` | Rotation `[roll, pitch, yaw]` in degrees (intrinsic ZYX) applied to all SpaceMouse inputs before EE/world-frame processing. Compensates for operator orientation (e.g. sitting 90° to the side: `[0, 0, 90]`) or non-standard tool mounting. |
| `flip_input_x` | bool | `false` | Invert the SpaceMouse X axis after `input_frame_rpy`. Any single axis can be flipped independently. |
| `flip_input_y` | bool | `false` | Invert the Y axis. |
| `flip_input_z` | bool | `false` | Invert the Z axis. |
| **Topics** | | | |
| `current_pose_topic` | string | `"/current_pose"` | Topic the robot publishes its actual EE pose on (`geometry_msgs/PoseStamped`). |
| `target_pose_topic` | string | `"/target_pose"` | Topic this node publishes the commanded target pose on (`geometry_msgs/PoseStamped`). |
| **Input frame** | | | |
| `translation_frame` | string | `"ee"` | `"world"`: push forward → world +X. `"ee"`: push forward → EE's pointing direction. |
| `rotation_frame` | string | `"ee"` | `"world"`: rotate around world axes. `"ee"`: rotate around the EE's local axes. |
| **Sensitivity** | | | |
| `linear_scale` | float | `0.1` | Translational speed at full SpaceMouse deflection (m/s). |
| `angular_scale` | float | `0.25` | Rotational speed at full SpaceMouse deflection (rad/s). |
| **Deadband** | | | |
| `linear_deadzone` | float | `0.05` | Raw input fraction (0–1) that must be exceeded before translation starts. Inputs above have the threshold subtracted so motion begins at zero with no step. |
| `angular_deadzone` | float | `0.05` | Same as `linear_deadzone` but for rotation. |
| **Safety clipping** | | | |
| `max_distance` | float | `0.15` | Maximum Euclidean distance (m) the target may lead the actual EE. The target is projected back to the boundary sphere if exceeded. |
| `max_rotation` | float | `0.5` | Maximum angular distance (rad, ≈28°) the target orientation may lead the actual EE orientation. Clipped in quaternion space. |
| **Idle snap** | | | |
| `snap_to_actual_on_idle` | bool | `true` | When `true`, the target snaps instantly back to the actual EE pose when all inputs are idle. Prevents residual offsets after releasing the device. |
| `linear_snap_threshold` | float | `0.15` | Raw input (0–1) below which a translational axis is considered idle for the snap check. Set higher than `linear_deadzone` so the snap fires while the knob is still returning to center, not after full mechanical settle. |
| `angular_snap_threshold` | float | `0.15` | Same as `linear_snap_threshold` but for rotational axes. |
| **Gripper** | | | |
| `gripper_interface` | string | `"action"` | `"topic"`: publishes `std_msgs/Float32` (0–1) to `gripper_topic`. `"action"`: sends `control_msgs/action/GripperCommand` goals directly to the gripper action server. |
| `gripper_mode` | string | `"absolute"` | `"absolute"`: one press fully opens (Button 1 → 0.0) or closes (Button 2 → 1.0). `"relative"`: hold Button 1 to open, hold Button 2 to close; release stops movement. |
| `gripper_step` | float | `0.01` | Width step per timer tick in `"relative"` mode. At 100 Hz: `0.01` → ~1 s full travel. |
| `gripper_topic` | string | `"space_mouse/target_gripper_width_percent"` | Topic name for `"topic"` mode. |
| `gripper_action` | string | `"manipulators/arm_0_gripper_controller/gripper_cmd"` | Action server name for `"action"` mode. |
| `gripper_max_position` | float | `0.8` | Fully-closed joint position in radians for `"action"` mode. `0.8` = Robotiq 2F-85. |
| `gripper_max_effort` | float | `50.0` | Max effort sent with each `GripperCommand` goal in `"action"` mode. |
| **Timing** | | | |
| `timer_period` | float | `0.01` | Integration loop period (seconds). Default 0.01 s = 100 Hz, matching the SpaceMouse publisher rate. |

---

### Topics

| Topic | Direction | Type | Description |
|---|---|---|---|
| `space_mouse/target_cartesian_velocity_percent` | published | `geometry_msgs/Twist` | Raw 6-DOF SpaceMouse input scaled to [−1, 1]. Internal topic between the two nodes. |
| `gripper_topic` *(configurable)* | published | `std_msgs/Float32` | Gripper command. Absolute mode: `0.0` = open (Button 1), `1.0` = close (Button 2). Relative mode: value accumulates while button held. |
| `current_pose_topic` *(configurable)* | subscribed | `geometry_msgs/PoseStamped` | Robot's actual EE pose. Used to seed the target on startup and for clipping/snap. |
| `target_pose_topic` *(configurable)* | published | `geometry_msgs/PoseStamped` | Integrated target EE pose for the robot controller to track. |

---

## Dual-arm example

See `example_duo_config.yaml`. Each entry gets its own namespace, device path, and topic pair:

```yaml
LEFT:
  namespace: left
  device_path: /dev/hidraw6
  current_pose_topic: "/left/current_pose"
  target_pose_topic: "/left/target_pose"
  ...

RIGHT:
  namespace: right
  device_path: /dev/hidraw7
  current_pose_topic: "/right/current_pose"
  target_pose_topic: "/right/target_pose"
  ...
```

---

## Building and testing

```bash
pixi run build       # colcon build --symlink-install
pixi run start       # source install/setup.bash && ros2 launch ...
pixi run clean       # remove build/, install/, log/
```

Run the ROS 2 linting tests:

```bash
colcon test
```

---

## Acknowledgements

This package was originally derived from the [spacemouse_ros2](https://github.com/frankaemika/spacemouse_ros2) repository by [Franka Robotics GmbH](https://www.franka.de/). It has since been substantially rewritten and extended, but the initial structure and ROS 2 packaging conventions trace back to their work. Thanks to the Franka team for the open-source foundation.
