import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def generate_nodes(context):
    config_file_name = LaunchConfiguration("config_file").perform(context)
    package_config_dir = FindPackageShare("spacemouse_publisher").perform(context)
    config_file = os.path.join(package_config_dir, "config", config_file_name)
    configs = load_yaml(config_file)
    nodes = []

    for item_name, config in configs.items():
        # Safely extract namespace
        namespace = str(config.get("namespace", ""))

        # 1. The primary SpaceMouse driver node
        nodes.append(
            Node(
                package="spacemouse_publisher",
                executable="pyspacemouse_publisher",
                name="spacemouse_publisher",
                namespace=namespace,
                output="screen",
                parameters=[
                    {"operator_position_front": config.get("operator_position_front", True)},
                    {"device_path": str(config.get("device_path", ""))},
                    *(
                        [{"gripper_topic": str(config["gripper_topic"])}]
                        if "gripper_topic" in config
                        else []
                    ),
                    *(
                        [{"gripper_mode": str(config["gripper_mode"])}]
                        if "gripper_mode" in config
                        else []
                    ),
                    *(
                        [{"gripper_step": float(config["gripper_step"])}]
                        if "gripper_step" in config
                        else []
                    ),
                    *(
                        [{"gripper_interface": str(config["gripper_interface"])}]
                        if "gripper_interface" in config
                        else []
                    ),
                    *(
                        [{"gripper_action": str(config["gripper_action"])}]
                        if "gripper_action" in config
                        else []
                    ),
                    *(
                        [{"gripper_max_position": float(config["gripper_max_position"])}]
                        if "gripper_max_position" in config
                        else []
                    ),
                    *(
                        [{"gripper_max_effort": float(config["gripper_max_effort"])}]
                        if "gripper_max_effort" in config
                        else []
                    ),
                ],
            )
        )

        # 2. Extract optional tuning parameters for the pose integrator
        pose_params = {}
        if "linear_scale" in config:
            pose_params["linear_scale"] = float(config["linear_scale"])
        if "angular_scale" in config:
            pose_params["angular_scale"] = float(config["angular_scale"])
        if "linear_deadzone" in config:
            pose_params["linear_deadzone"] = float(config["linear_deadzone"])
        if "angular_deadzone" in config:
            pose_params["angular_deadzone"] = float(config["angular_deadzone"])
        if "max_distance" in config:
            pose_params["max_distance"] = float(config["max_distance"])
        if "max_rotation" in config:
            pose_params["max_rotation"] = float(config["max_rotation"])
        if "snap_to_actual_on_idle" in config:
            pose_params["snap_to_actual_on_idle"] = bool(config["snap_to_actual_on_idle"])
        if "linear_snap_threshold" in config:
            pose_params["linear_snap_threshold"] = float(config["linear_snap_threshold"])
        if "angular_snap_threshold" in config:
            pose_params["angular_snap_threshold"] = float(config["angular_snap_threshold"])
        if "timer_period" in config:
            pose_params["timer_period"] = float(config["timer_period"])
        if "translation_frame" in config:
            pose_params["translation_frame"] = str(config["translation_frame"])
        if "rotation_frame" in config:
            pose_params["rotation_frame"] = str(config["rotation_frame"])
        if "input_frame_rpy" in config:
            pose_params["input_frame_rpy"] = [float(v) for v in config["input_frame_rpy"]]
        if "flip_input_x" in config:
            pose_params["flip_input_x"] = bool(config["flip_input_x"])
        if "flip_input_y" in config:
            pose_params["flip_input_y"] = bool(config["flip_input_y"])
        if "flip_input_z" in config:
            pose_params["flip_input_z"] = bool(config["flip_input_z"])
        if "current_pose_topic" in config:
            pose_params["current_pose_topic"] = str(config["current_pose_topic"])
        if "target_pose_topic" in config:
            pose_params["target_pose_topic"] = str(config["target_pose_topic"])

        # 3. The newly created Twist-to-Pose Integrator node
        nodes.append(
            Node(
                package="spacemouse_publisher",
                executable="twist_to_pose_node",
                name="twist_to_pose_node",
                namespace=namespace,
                output="screen",
                parameters=[pose_params] if pose_params else [],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="example_config.yaml",
                description="Name of the spacemouse configuration file to load",
            ),
            OpaqueFunction(function=generate_nodes),
        ]
    )
