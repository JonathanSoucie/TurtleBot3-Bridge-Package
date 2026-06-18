#!/usr/bin/env python3
"""
bridge_test_terminal.py — TB3 Bridge Test
==========================================
Place roslib_wrapper.py in the same folder.

Usage:
  TB3_MODE=sim TB3_BRIDGE_HOST=localhost python3 bridge_test_terminal.py
  TB3_MODE=real TB3_BRIDGE_HOST=tb3bridge-desktop.local python3 bridge_test_terminal.py
"""

import math
import time
import os

import roslib_wrapper as rclpy
from roslib_wrapper import Node, Twist, LaserScan, Odometry

LINEAR_SPEED  = 0.10
ANGULAR_SPEED = 0.4
DIST_TOL      = 0.05
ANGLE_TOL     = 0.05

PLAN = [
    ("Forward 2m",   2.0,  None),
    ("Turn right",   None, -math.pi / 2),
    ("Forward 1m",   1.0,  None),
    ("Turn left",    None,  math.pi),
    ("Forward 1m",   1.0,  None),
    ("Turn right",   None,  math.pi / 2),
    ("Backward 1m",  1.0,  None),
]


def clear():
    os.system('cls' if os.name == 'nt' else 'clear')


class BridgeTest(Node):
    def __init__(self):
        super().__init__('bridge_test')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.scan_ranges = []
        self.scan_count = 0
        self.odom_count = 0
        self.start_time = time.time()

        self.cmd_pub = self.create_publisher(Twist, '/student/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/student/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/student/odom', self.odom_cb, 10)

    def scan_cb(self, msg):
        self.scan_ranges = msg.ranges
        self.scan_count += 1

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        siny = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy = 1.0 - 2.0 * (ori.y ** 2 + ori.z ** 2)
        self.theta = math.atan2(siny, cosy)
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z
        self.odom_count += 1

    def send_cmd(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.send_cmd(0.0, 0.0)

    def is_connected(self):
        """Check if still connected to rosbridge."""
        return rclpy._ros is not None and rclpy._ros.is_connected

    def sector_min(self, start_deg, end_deg):
        if not self.scan_ranges:
            return 3.5
        n = len(self.scan_ranges)
        vals = []
        for i in range(start_deg, end_deg + 1):
            r = self.scan_ranges[i % n]
            if isinstance(r, (int, float)) and math.isfinite(r) and 0.01 < r < 3.5:
                vals.append(r)
        return min(vals) if vals else 3.5

    def lidar_bar(self, dist, max_dist=3.5, width=20):
        filled = int((min(dist, max_dist) / max_dist) * width)
        return "█" * filled + "░" * (width - filled)

    def display(self, step_name, detail=""):
        clear()
        elapsed = time.time() - self.start_time
        front = self.sector_min(-15, 15)
        right = self.sector_min(255, 285)
        left  = self.sector_min(75, 105)
        back  = self.sector_min(170, 190)

        arrows = ["→", "↗", "↑", "↖", "←", "↙", "↓", "↘"]
        idx = int(((self.theta + math.pi) / (2 * math.pi)) * 8 + 0.5) % 8

        print("══════════════════════════════════════════════════")
        print("  TB3 BRIDGE TEST")
        print("══════════════════════════════════════════════════")
        print(f"  Step:  {step_name}")
        if detail:
            print(f"         {detail}")
        print(f"  Time:  {elapsed:.0f}s")
        print()
        print("── ODOMETRY ──────────────────────────────────────")
        print(f"  Position:  x = {self.x:+.3f} m   y = {self.y:+.3f} m")
        print(f"  Heading:   {math.degrees(self.theta):+.1f}°  {arrows[idx]}")
        print(f"  Velocity:  linear = {self.linear_vel:+.3f} m/s")
        print(f"             angular = {self.angular_vel:+.3f} rad/s")
        print()
        print("── LIDAR ─────────────────────────────────────────")
        print(f"  Front: {front:.2f}m  {self.lidar_bar(front)}")
        print(f"  Right: {right:.2f}m  {self.lidar_bar(right)}")
        print(f"  Back:  {back:.2f}m  {self.lidar_bar(back)}")
        print(f"  Left:  {left:.2f}m  {self.lidar_bar(left)}")
        print()
        print("── DATA FLOW ─────────────────────────────────────")
        print(f"  /student/scan  ↓  {self.scan_count:>5d} msgs  {'OK' if self.scan_count > 0 else '--'}")
        print(f"  /student/odom  ↓  {self.odom_count:>5d} msgs  {'OK' if self.odom_count > 0 else '--'}")
        print(f"  /student/cmd   ↑  publishing       {'OK' if self.linear_vel != 0 or self.angular_vel != 0 else 'IDLE'}")
        print("══════════════════════════════════════════════════")

    def run(self):
        self.get_logger().info("Waiting for sensor data ...")
        timeout = time.time() + 10
        while time.time() < timeout:
            if self.scan_count > 0 and self.odom_count > 0:
                break
            time.sleep(0.2)

        if self.scan_count == 0 or self.odom_count == 0:
            self.get_logger().error("No sensor data received")
            if self.scan_count == 0:
                print("  /student/scan — NO DATA")
            if self.odom_count == 0:
                print("  /student/odom — NO DATA")
            return

        self.get_logger().info(f"Sensors OK — scan: {self.scan_count}, odom: {self.odom_count}")

        for i, (name, dist, angle) in enumerate(PLAN):
            label = f"[{i+1}/{len(PLAN)}] {name}"

            if dist is not None:
                start_x, start_y = self.x, self.y
                while self.is_connected():
                    traveled = math.hypot(self.x - start_x, self.y - start_y)
                    remaining = dist - traveled
                    if remaining < DIST_TOL:
                        break
                    self.send_cmd(LINEAR_SPEED, 0.0)
                    self.display(label, f"{remaining:.2f}m remaining")
                    time.sleep(0.1)
                self.stop()

            elif angle is not None:
                turned = 0.0
                last_theta = self.theta
                direction = 1.0 if angle > 0 else -1.0
                while self.is_connected():
                    dt = self.theta - last_theta
                    if dt > math.pi:    dt -= 2 * math.pi
                    if dt < -math.pi:   dt += 2 * math.pi
                    turned += dt
                    last_theta = self.theta
                    remaining = abs(angle) - abs(turned)
                    if remaining < ANGLE_TOL:
                        break
                    self.send_cmd(0.0, direction * ANGULAR_SPEED)
                    self.display(label, f"{math.degrees(remaining):.0f}° remaining")
                    time.sleep(0.1)
                self.stop()

            for _ in range(5):
                self.display(label, "Done!")
                time.sleep(0.1)

        self.stop()
        self.display("COMPLETE", "All steps finished")
        print(f"\n  Final: ({self.x:+.3f}, {self.y:+.3f})")
        print(f"  Scan: {self.scan_count}  Odom: {self.odom_count}\n")


def main():
    rclpy.init()
    node = BridgeTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
