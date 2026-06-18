"""
tb3_bridge.launch.py

Launches:
  1. tb3_bridge_node   — safety bridge between student and robot topics
  2. rosbridge_websocket — websocket server so students can connect
                           with roslibpy (no ROS install needed)

Usage:
  ros2 launch tb3_bridge tb3_bridge.launch.py
  ros2 launch tb3_bridge tb3_bridge.launch.py config_file:=/path/to/custom.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    pkg_share = FindPackageShare("tb3_bridge")

    default_config = PathJoinSubstitution(
        [pkg_share, "config", "bridge_params.yaml"]
    )

    return LaunchDescription([

        # ── Launch Arguments ──────────────────────────────────────────────────
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Absolute path to the bridge YAML parameter file.",
        ),

        DeclareLaunchArgument(
            "rosbridge_port",
            default_value="9090",
            description="Websocket port for rosbridge (students connect here).",
        ),

        # ── Bridge Node ───────────────────────────────────────────────────────
        Node(
            package    = "tb3_bridge",
            executable = "tb3_bridge_node",
            name       = "tb3_bridge_node",
            output     = "screen",
            emulate_tty = True,
            parameters = [LaunchConfiguration("config_file")],
        ),

        # ── Rosbridge Websocket ───────────────────────────────────────────────
        Node(
            package    = "rosbridge_server",
            executable = "rosbridge_websocket",
            name       = "rosbridge_websocket",
            output     = "screen",
            parameters = [{
                "port": LaunchConfiguration("rosbridge_port"),
                "address": "",
                "unregister_timeout": 10.0,
            }],
        ),
    ])
