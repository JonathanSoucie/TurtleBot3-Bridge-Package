#!/usr/bin/env python3

"""
student_template.py - Path Planner Template
============================================
Place roslib_wrapper.py in the same folder.

"""

import roslib_wrapper as rclpy
from roslib_wrapper import Node, Twist, LaserScan, Odometry


class MyPlanner(Node):
    def __init__(self):
        super().__init__('my_planner')

        # Publisher - send velocity commands to the robot
        self.cmd_pub = self.create_publisher(Twist, '/student/cmd_vel', 10)

        # Subscribers - receive sensor data from the robot
        self.scan_sub = self.create_subscription(
            LaserScan, '/student/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/student/odom', self.odom_cb, 10)

        # Control loop at 10 Hz
        self.create_timer(0.1, self.control_loop)

        self.latest_scan = None
        self.latest_odom = None
        self.get_logger().info("Planner started - waiting for sensor data")

    def scan_cb(self, msg):
        # msg.ranges is a list of floats - one per LiDAR angle
        # msg.ranges[0]   = directly ahead
        # msg.ranges[90]  = left
        # msg.ranges[180] = behind
        # msg.ranges[270] = right
        self.latest_scan = msg

    def odom_cb(self, msg):
        # msg.pose.pose.position.x  = x position (meters)
        # msg.pose.pose.position.y  = y position (meters)
        # msg.twist.twist.linear.x  = forward velocity (m/s)
        # msg.twist.twist.angular.z = turning velocity (rad/s)
        self.latest_odom = msg

    def control_loop(self):
        if self.latest_scan is None:
            return

        # === YOUR ALGORITHM GOES HERE ===============================

        cmd = Twist()
        cmd.linear.x = 0.1    # forward speed (m/s), max 0.22
        cmd.angular.z = 0.0   # turning speed (rad/s), max 2.84

        # ============================================================

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = MyPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before exiting
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
