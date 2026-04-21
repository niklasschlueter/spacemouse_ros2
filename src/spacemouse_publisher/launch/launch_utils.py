"""Shared utilities for spacemouse_publisher launch files."""

import os

import yaml
from launch_ros.actions import Node


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def gripper_params(config):
    """Extract optional gripper parameters for a pyspacemouse_publisher node."""
    params = []
    for key, cast in [
        ("gripper_topic", str),
        ("gripper_mode", str),
        ("gripper_step", float),
        ("gripper_interface", str),
        ("gripper_action", str),
        ("gripper_max_position", float),
        ("gripper_max_effort", float),
    ]:
        if key in config:
            params.append({key: cast(config[key])})
    return params


def pose_integrator_params(config):
    """Extract optional tuning parameters for the twist_to_pose_node."""
    params = {}
    for key, cast in [
        ("linear_deadzone", float),
        ("angular_deadzone", float),
        ("max_distance", float),
        ("max_rotation", float),
        ("latch_on_idle", bool),
        ("timer_period", float),
        ("translation_frame", str),
        ("rotation_frame", str),
        ("flip_input_x", bool),
        ("flip_input_y", bool),
        ("flip_input_z", bool),
        ("current_pose_topic", str),
        ("target_pose_topic", str),
        ("controller_aware", bool),
        ("controller_name", str),
        ("controller_manager_node", str),
    ]:
        if key in config:
            params[key] = cast(config[key])
    if "input_frame_rpy" in config:
        params["input_frame_rpy"] = [float(v) for v in config["input_frame_rpy"]]
    return params


def spacemouse_publisher_node(namespace, config, name="spacemouse_publisher", **param_overrides):
    """Create a pyspacemouse_publisher Node with standard parameters."""
    params = [
        {"operator_position_front": config.get("operator_position_front", True)},
        {"device_path": str(config.get("device_path", ""))},
        *gripper_params(config),
    ]
    for key, value in param_overrides.items():
        params.append({key: value})
    return Node(
        package="spacemouse_publisher",
        executable="pyspacemouse_publisher",
        name=name,
        namespace=namespace,
        output="screen",
        parameters=params,
    )


def twist_to_pose_node(namespace, pose_params):
    """Create a twist_to_pose_node Node."""
    return Node(
        package="spacemouse_publisher",
        executable="twist_to_pose_node",
        name="twist_to_pose_node",
        namespace=namespace,
        output="screen",
        parameters=[pose_params] if pose_params else [],
    )
