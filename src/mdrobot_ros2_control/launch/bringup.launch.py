# Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
"""Bring up a MDROBOT controller through ros2_control.

  ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=single
  ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=dual
  ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=twin

Connection settings (serial port, per-motor Modbus id, reverse, counts_per_rev,
...) live in config/<device_type>_controllers.yaml under the `mdrobot_hardware`
section — edit them there. This launch reads that section and injects it into the
URDF (the ros2_control hardware params); controller_manager ignores it. `port`
and `counts_per_rev` can also be overridden on the command line.

Starts robot_state_publisher, the controller_manager (ros2_control_node), and
spawns joint_state_broadcaster (+ diff_cont for dual/twin, velocity_cont for single).

twin = two single-channel controllers on one bus at distinct slave ids (skid-steer);
set motor_id_L / motor_id_R (must differ) in twin_controllers.yaml and re-ID one
unit (PID_ID) so they actually differ on the bus before bringup.
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _load_hardware_params(controllers_path):
    """Read the `mdrobot_hardware` connection section from a controllers yaml."""
    try:
        with open(controllers_path) as f:
            data = yaml.safe_load(f) or {}
    except OSError:
        return {}
    section = data.get("mdrobot_hardware", {}) or {}
    return section.get("ros__parameters", {}) or {}


def launch_setup(context, *args, **kwargs):
    device_type = LaunchConfiguration("device_type").perform(context)
    if device_type not in ("single", "dual", "twin"):
        raise RuntimeError(
            f"device_type must be 'single', 'dual', or 'twin', got {device_type!r}")

    pkg_share = FindPackageShare("mdrobot_ros2_control").perform(context)
    urdf = os.path.join(pkg_share, "urdf", f"mdrobot_{device_type}.urdf.xacro")
    controllers = os.path.join(pkg_share, "config", f"{device_type}_controllers.yaml")

    # Connection config from the yaml, with optional command-line overrides.
    hw = _load_hardware_params(controllers)
    for name in ("port", "counts_per_rev"):
        value = LaunchConfiguration(name).perform(context)
        if value != "":
            hw[name] = value

    # Build the xacro command. Every hardware key becomes a `key:=value` xacro
    # arg; xacro ignores ones a given urdf does not declare, so the single/dual/
    # twin yamls can carry slightly different keys safely. Booleans -> lowercase
    # so the urdf/plugin see "true"/"false".
    xacro_cmd = [FindExecutable(name="xacro"), " ", urdf]
    for key, value in hw.items():
        sval = str(value).lower() if isinstance(value, bool) else str(value)
        xacro_cmd += [f" {key}:=", sval]
    robot_description = {
        "robot_description": ParameterValue(Command(xacro_cmd), value_type=str)
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers],
        output="screen",
    )
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    extra = "diff_cont" if device_type in ("dual", "twin") else "velocity_cont"
    return [
        control_node,
        rsp_node,
        jsb_spawner,
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[extra, "--controller-manager", "/controller_manager"],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_type", default_value="single",
                description="single (MD400), dual (PNT50/MD400T), or twin (two single controllers on one bus)",
            ),
            DeclareLaunchArgument(
                "port", default_value="",
                description="serial port; empty keeps the value from <device_type>_controllers.yaml",
            ),
            DeclareLaunchArgument(
                "counts_per_rev", default_value="",
                description="single/dual: counts per rev override; empty keeps the yaml value "
                            "(twin uses counts_per_rev_L/R in the yaml)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
