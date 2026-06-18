"""
DEPRECATED. Replaced by gazebo_only.launch.py.

The earlier sim launch ran the C++ bridge in front of Gazebo, which was
unnecessary (the bridge exists to solve real-robot problems - TwistStamped
conversion, watchdog, namespace isolation - that don't apply in sim).

For Gazebo testing use:
    ros2 launch tb3_bridge gazebo_only.launch.py

For real-robot deployment (on the Pi) use:
    ros2 launch tb3_bridge tb3_bridge.launch.py
"""

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    raise RuntimeError(
        "tb3_bridge_sim.launch.py is deprecated. Use gazebo_only.launch.py "
        "(sim) or tb3_bridge.launch.py (real robot).")
