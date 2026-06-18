"""
gazebo_only.launch.py

Pure-sim launch: brings up Gazebo (TurtleBot3 Burger empty world) and
rosbridge_websocket. NO C++ bridge node - in simulation we don't need the
Twist->TwistStamped conversion, the watchdog, or the namespace isolation.

Student Python code (with TB3_MODE=sim, the default) talks straight to
Gazebo's /cmd_vel, /scan, /odom topics over the websocket.

Usage:
  export TURTLEBOT3_MODEL=burger
  ros2 launch tb3_bridge gazebo_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    tb3_gazebo_share = FindPackageShare("turtlebot3_gazebo")
    tb3_world_launch = PathJoinSubstitution(
        [tb3_gazebo_share, "launch", "empty_world.launch.py"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rosbridge_port",
            default_value="9090",
            description="Websocket port for rosbridge.",
        ),

        # Gazebo + TurtleBot3 Burger world (Humble TB3 Gazebo Classic).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_world_launch),
        ),

        # rosbridge websocket only - no tb3_bridge_node in sim.
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
