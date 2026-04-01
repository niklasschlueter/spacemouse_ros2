"""Launch file for dual-SpaceMouse single-arm control.

Spawns two pyspacemouse_publisher nodes (one per device) and a single
twist_to_pose_node that takes translation from the primary and rotation
from the secondary device.

Requires `secondary_device_path` in the YAML config.
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, str(Path(__file__).resolve().parent))
from launch_utils import (  # noqa: E402
    load_yaml,
    pose_integrator_params,
    spacemouse_publisher_node,
    twist_to_pose_node,
)

SECONDARY_TWIST_TOPIC = "space_mouse/secondary_cartesian_velocity_percent"


def generate_nodes(context):
    config_file_name = LaunchConfiguration("config_file").perform(context)
    package_config_dir = FindPackageShare("spacemouse_publisher").perform(context)
    config_file = os.path.join(package_config_dir, "config", config_file_name)
    configs = load_yaml(config_file)
    nodes = []

    for item_name, config in configs.items():
        namespace = str(config.get("namespace", ""))
        secondary_device_path = str(config.get("secondary_device_path", ""))

        if not secondary_device_path:
            raise ValueError(
                f"Config entry '{item_name}' is missing 'secondary_device_path'. "
                "Use spacemouse_publisher.launch.py for single-device mode."
            )

        # Primary publisher (translation + gripper)
        nodes.append(spacemouse_publisher_node(namespace, config))

        # Secondary publisher (rotation only, gripper disabled)
        nodes.append(
            spacemouse_publisher_node(
                namespace,
                config,
                name="spacemouse_publisher_secondary",
                device_path=secondary_device_path,
                twist_topic=SECONDARY_TWIST_TOPIC,
                gripper_interface="topic",
            )
        )

        # Single integrator reading both devices
        pose_params = pose_integrator_params(config)
        pose_params["secondary_twist_topic"] = SECONDARY_TWIST_TOPIC
        nodes.append(twist_to_pose_node(namespace, pose_params))

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="example_dual_device_config.yaml",
                description="Name of the spacemouse configuration file to load",
            ),
            OpaqueFunction(function=generate_nodes),
        ]
    )
