# spacemouse_ros2

[![Build](https://github.com/niklasschlueter/spacemouse_ros2/actions/workflows/ros2.yaml/badge.svg)](https://github.com/niklasschlueter/spacemouse_ros2/actions/workflows/ros2.yaml)

<p align="center">
  <img src="media/banner.png" alt="spacemouse_ros2" width="800">
</p>

Minimal ROS 2 package that turns a 3Dconnexion SpaceMouse into a `PoseStamped` target for robot teleoperation. Plug in, launch, and your tracking controller gets a live target pose. Includes optional gripper commands via buttons.

```
SpaceMouse  ──HID──>  pyspacemouse_publisher  ──Twist──>  twist_to_pose_node  ──PoseStamped──>  Your controller
                                                                  ^
                                                    current_pose  |
                                                    (feedback)  ──┘
```

**How it works:** SpaceMouse deflection maps directly to a pose offset from the robot's actual EE position. Zero input = target at actual pose (no lag on release). The controller tracks the offset; as the robot moves, the offset origin shifts, producing continuous motion while you hold the puck.

---

## Quick start

```bash
# 1. udev rule (one-time) — makes /dev/hidraw* accessible without sudo
printf 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="046d", MODE="0666"\nSUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", MODE="0666"\n' \
  | sudo tee /etc/udev/rules.d/99-spacemouse.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 2. Install and build (pixi manages the full ROS 2 Jazzy environment)
curl -fsSL https://pixi.sh/install.sh | bash   # skip if pixi is already installed
git clone https://github.com/niklasschlueter/spacemouse_ros2.git
cd spacemouse_ros2
pixi install && pixi run build

# 3. Run
pixi run start
```

<details>
<summary><strong>Without pixi (plain colcon)</strong></summary>

Requires ROS 2 Jazzy already sourced.

```bash
cd ~/ros2_ws/src
git clone https://github.com/niklasschlueter/spacemouse_ros2.git
cd ~/ros2_ws
rosdep install --from-paths src/spacemouse_ros2 --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch spacemouse_publisher spacemouse_publisher.launch.py
```

</details>

---

## Adapting to your robot

Edit `example_config.yaml` (or copy it). Four things typically need to change:

| What | Parameter | Notes |
|---|---|---|
| Namespace | `namespace` | Must match your controller's namespace. Empty = global. |
| Pose topics | `current_pose_topic` / `target_pose_topic` | What your controller publishes and subscribes to. |
| Device | `device_path` | Only needed with multiple SpaceMice. Empty = auto-detect. |
| Gripper | `gripper_interface` | `"action"` sends `GripperCommand` goals directly. `"topic"` publishes `Float32`. |

Pass a custom config: `pixi run start config_file:=my_config.yaml`

---

## Online tuning

Most parameters can be changed at runtime without restarting:

```bash
ros2 run rqt_reconfigure rqt_reconfigure   # GUI with sliders
ros2 param set /twist_to_pose_node max_distance 0.3   # CLI
```

---

## Multi-device setups

**Dual-arm** — one SpaceMouse per arm. See `example_duo_config.yaml`:

```yaml
LEFT:
  namespace: left
  device_path: /dev/hidraw6
  current_pose_topic: "/left/current_pose"
  target_pose_topic: "/left/target_pose"
RIGHT:
  namespace: right
  device_path: /dev/hidraw7
  current_pose_topic: "/right/current_pose"
  target_pose_topic: "/right/target_pose"
```

**Dual-device** — two SpaceMice, one arm. Left hand = translation, right hand = rotation. See `example_dual_device_config.yaml`:

```yaml
robot1:
  device_path: /dev/hidraw3            # translation
  secondary_device_path: /dev/hidraw4  # rotation
```

```bash
pixi run start-dual
```

<details>
<summary><strong>Identifying connected devices</strong></summary>

```bash
grep -H . /sys/class/hidraw/hidraw*/device/uevent | grep SpaceMouse
```

Each physical device registers two hidraw entries (motion + buttons). Unplug one to see which entries belong to which device.

</details>

---

## Parameter reference

<details>
<summary><strong>All parameters</strong> (click to expand)</summary>

| Parameter | Type | Default | Description |
|---|---|---|---|
| `namespace` | string | `""` | ROS 2 namespace. Must match your controller. |
| `device_path` | string | `""` | HID path (e.g. `/dev/hidraw1`). Empty = auto-detect. |
| `secondary_device_path` | string | — | Second SpaceMouse for dual-device mode. |
| `operator_position_front` | bool | `true` | `false` inverts X/Y so "push forward" = away from operator. |
| `input_frame_rpy` | float[3] | `[0,0,0]` | Input rotation [roll, pitch, yaw] in degrees (intrinsic ZYX). |
| `flip_input_x/y/z` | bool | `false` | Per-axis sign flip after `input_frame_rpy`. |
| **Topics** | | | |
| `current_pose_topic` | string | `"/current_pose"` | Robot's actual EE pose (`PoseStamped`). |
| `target_pose_topic` | string | `"/target_pose"` | Commanded target pose (`PoseStamped`). |
| **Input frame** | | | |
| `translation_frame` | string | `"ee"` | `"world"` or `"ee"` — reference frame for translation. |
| `rotation_frame` | string | `"ee"` | `"world"` or `"ee"` — reference frame for rotation. |
| **Offset & deadband** | | | |
| `max_distance` | float | `0.15` | Max translational offset (m) at full deflection. |
| `max_rotation` | float | `0.5` | Max angular offset (rad) at full deflection. |
| `linear_deadzone` | float | `0.05` | Input threshold before translation starts. |
| `angular_deadzone` | float | `0.05` | Input threshold before rotation starts. |
| `latch_on_idle` | bool | `true` | Freeze target on release (stable setpoint) vs. track actual. |
| **Gripper** | | | |
| `gripper_interface` | string | `"action"` | `"action"` (GripperCommand) or `"topic"` (Float32). |
| `gripper_mode` | string | `"absolute"` | `"absolute"` (one press) or `"relative"` (hold to move). |
| `gripper_step` | float | `0.01` | Step per tick in relative mode. |
| `gripper_topic` | string | `"space_mouse/target_gripper_width_percent"` | Topic for `"topic"` mode. |
| `gripper_action` | string | `"manipulators/arm_0_gripper_controller/gripper_cmd"` | Action server for `"action"` mode. |
| `gripper_max_position` | float | `0.8` | Closed position in radians (`0.8` = Robotiq 2F-85). |
| `gripper_max_effort` | float | `50.0` | Max effort per goal. |
| **Timing** | | | |
| `timer_period` | float | `0.01` | Loop period (s). 100 Hz matches SpaceMouse rate. |

**Runtime tunable:** all except `*_topic`, `timer_period`, `secondary_twist_topic` (require restart).

</details>

---

## Development

```bash
pixi run build       # colcon build --symlink-install
pixi run start       # single-device launch
pixi run start-dual  # dual-device launch
pixi run clean       # remove build/, install/, log/
pixi run lint        # ruff check + format check
pixi run format      # auto-format with ruff
```

---

## Acknowledgements

Originally derived from [spacemouse_ros2](https://github.com/frankaemika/spacemouse_ros2) by [Franka Robotics GmbH](https://www.franka.de/). Substantially rewritten — thanks to the Franka team for the open-source foundation.
