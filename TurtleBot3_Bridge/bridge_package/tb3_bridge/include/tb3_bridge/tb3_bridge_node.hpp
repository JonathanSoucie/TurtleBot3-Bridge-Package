#ifndef TB3_BRIDGE__TB3_BRIDGE_NODE_HPP_
#define TB3_BRIDGE__TB3_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <atomic>
#include <mutex>
#include <string>

namespace tb3_bridge
{

class Tb3BridgeNode : public rclcpp::Node
{
public:
  explicit Tb3BridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      student_cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    robot_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        robot_odom_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   robot_cmd_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr       student_scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           student_odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  e_stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  e_stop_reset_service_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  void studentCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void robotScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void robotOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void watchdogTimerCallback();
  void diagnosticsTimerCallback();

  void enableServiceCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);
  void eStopServiceCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void eStopResetServiceCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void declareAndGetParams();
  void publishZeroTwist();

  std::mutex     state_mutex_;
  bool           forwarding_enabled_{true};
  bool           e_stop_active_{false};
  bool           watchdog_triggered_{false};
  bool           received_any_cmd_vel_{false};
  rclcpp::Time   last_cmd_vel_time_;

  uint64_t cmd_vel_count_{0};
  uint64_t scan_count_{0};
  uint64_t odom_count_{0};

  std::string student_cmd_vel_topic_;
  std::string robot_cmd_vel_topic_;
  std::string robot_scan_topic_;
  std::string student_scan_topic_;
  std::string robot_odom_topic_;
  std::string student_odom_topic_;
  double watchdog_timeout_s_;
  double watchdog_check_rate_hz_;
  double diagnostics_rate_hz_;
};

}  // namespace tb3_bridge

#endif  // TB3_BRIDGE__TB3_BRIDGE_NODE_HPP_
