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


def generate_nodes(context):
    config_file_name = LaunchConfiguration("config_file").perform(context)
    package_config_dir = FindPackageShare("spacemouse_publisher").perform(context)
    config_file = os.path.join(package_config_dir, "config", config_file_name)
    configs = load_yaml(config_file)
    nodes = []

    for item_name, config in configs.items():
        namespace = str(config.get("namespace", ""))

        nodes.append(spacemouse_publisher_node(namespace, config))
        nodes.append(twist_to_pose_node(namespace, pose_integrator_params(config)))

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
