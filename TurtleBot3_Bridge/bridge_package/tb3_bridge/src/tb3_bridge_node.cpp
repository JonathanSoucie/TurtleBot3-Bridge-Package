#include "tb3_bridge/tb3_bridge_node.hpp"
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace tb3_bridge
{

Tb3BridgeNode::Tb3BridgeNode(const rclcpp::NodeOptions & options)
: Node("tb3_bridge_node", options)
{
  declareAndGetParams();

  // Publish TwistStamped to /cmd_vel (TB3 Jazzy requires TwistStamped)
  robot_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    robot_cmd_vel_topic_, 10);
  student_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    student_scan_topic_, rclcpp::SensorDataQoS());
  student_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    student_odom_topic_, rclcpp::SensorDataQoS());
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/bridge/diagnostics", 10);

  // Subscribe to plain Twist from students (simpler for them)
  student_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    student_cmd_vel_topic_, 10,
    std::bind(&Tb3BridgeNode::studentCmdVelCallback, this, std::placeholders::_1));
  robot_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    robot_scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&Tb3BridgeNode::robotScanCallback, this, std::placeholders::_1));
  robot_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    robot_odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&Tb3BridgeNode::robotOdomCallback, this, std::placeholders::_1));

  enable_service_ = create_service<std_srvs::srv::SetBool>(
    "/bridge/enable",
    std::bind(&Tb3BridgeNode::enableServiceCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  e_stop_service_ = create_service<std_srvs::srv::Trigger>(
    "/bridge/e_stop",
    std::bind(&Tb3BridgeNode::eStopServiceCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  e_stop_reset_service_ = create_service<std_srvs::srv::Trigger>(
    "/bridge/e_stop_reset",
    std::bind(&Tb3BridgeNode::eStopResetServiceCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  {
    auto period = std::chrono::duration<double>(1.0 / watchdog_check_rate_hz_);
    watchdog_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Tb3BridgeNode::watchdogTimerCallback, this));
  }
  {
    auto period = std::chrono::duration<double>(1.0 / diagnostics_rate_hz_);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Tb3BridgeNode::diagnosticsTimerCallback, this));
  }

  last_cmd_vel_time_ = now();

  RCLCPP_INFO(get_logger(), "=========================================");
  RCLCPP_INFO(get_logger(), "  TB3 Bridge Node — ROS 2 Jazzy");
  RCLCPP_INFO(get_logger(), "=========================================");
  RCLCPP_INFO(get_logger(), "  [STUDENT → ROBOT]");
  RCLCPP_INFO(get_logger(), "    %-30s → %s (Twist→TwistStamped)",
    student_cmd_vel_topic_.c_str(), robot_cmd_vel_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  [ROBOT → STUDENT]");
  RCLCPP_INFO(get_logger(), "    %-30s → %s",
    robot_scan_topic_.c_str(), student_scan_topic_.c_str());
  RCLCPP_INFO(get_logger(), "    %-30s → %s",
    robot_odom_topic_.c_str(), student_odom_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  Watchdog timeout : %.2f s", watchdog_timeout_s_);
  RCLCPP_INFO(get_logger(), "  Forwarding       : %s",
    forwarding_enabled_ ? "ENABLED" : "DISABLED");
  RCLCPP_INFO(get_logger(), "=========================================");
}

void Tb3BridgeNode::declareAndGetParams()
{
  declare_parameter("student_cmd_vel_topic",  "/student/cmd_vel");
  declare_parameter("robot_cmd_vel_topic",    "/cmd_vel");
  declare_parameter("robot_scan_topic",       "/scan");
  declare_parameter("student_scan_topic",     "/student/scan");
  declare_parameter("robot_odom_topic",       "/odom");
  declare_parameter("student_odom_topic",     "/student/odom");
  declare_parameter("watchdog_timeout_s",     0.5);
  declare_parameter("watchdog_check_rate_hz", 20.0);
  declare_parameter("diagnostics_rate_hz",    1.0);
  declare_parameter("forwarding_enabled",     true);

  student_cmd_vel_topic_  = get_parameter("student_cmd_vel_topic").as_string();
  robot_cmd_vel_topic_    = get_parameter("robot_cmd_vel_topic").as_string();
  robot_scan_topic_       = get_parameter("robot_scan_topic").as_string();
  student_scan_topic_     = get_parameter("student_scan_topic").as_string();
  robot_odom_topic_       = get_parameter("robot_odom_topic").as_string();
  student_odom_topic_     = get_parameter("student_odom_topic").as_string();
  watchdog_timeout_s_     = get_parameter("watchdog_timeout_s").as_double();
  watchdog_check_rate_hz_ = get_parameter("watchdog_check_rate_hz").as_double();
  diagnostics_rate_hz_    = get_parameter("diagnostics_rate_hz").as_double();
  forwarding_enabled_     = get_parameter("forwarding_enabled").as_bool();
}

void Tb3BridgeNode::studentCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_cmd_vel_time_    = now();
  received_any_cmd_vel_ = true;
  ++cmd_vel_count_;

  if (!forwarding_enabled_) return;
  if (e_stop_active_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "E-STOP active — cmd_vel dropped.");
    return;
  }
  if (watchdog_triggered_) {
    watchdog_triggered_ = false;
    RCLCPP_INFO(get_logger(), "Watchdog cleared — student publishing again.");
  }

  // Convert Twist -> TwistStamped for TB3 Jazzy
  auto stamped = geometry_msgs::msg::TwistStamped();
  stamped.header.stamp = now();
  stamped.header.frame_id = "base_footprint";
  stamped.twist = *msg;
  robot_cmd_vel_pub_->publish(stamped);
}

void Tb3BridgeNode::robotScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{ ++scan_count_; student_scan_pub_->publish(*msg); }

void Tb3BridgeNode::robotOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{ ++odom_count_; student_odom_pub_->publish(*msg); }

void Tb3BridgeNode::watchdogTimerCallback()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!forwarding_enabled_ || e_stop_active_ || !received_any_cmd_vel_) return;
  const double elapsed = (now() - last_cmd_vel_time_).seconds();
  if (elapsed > watchdog_timeout_s_) {
    if (!watchdog_triggered_) {
      watchdog_triggered_ = true;
      RCLCPP_WARN(get_logger(), "WATCHDOG: No cmd_vel for %.2f s. Stopping robot.", elapsed);
    }
    publishZeroTwist();
  }
}

void Tb3BridgeNode::diagnosticsTimerCallback()
{
  using DS = diagnostic_msgs::msg::DiagnosticStatus;
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "tb3_bridge"; status.hardware_id = "raspberry_pi_5";
  if (e_stop_active_) { status.level = DS::ERROR; status.message = "E-STOP ACTIVE"; }
  else if (watchdog_triggered_) { status.level = DS::WARN; status.message = "Watchdog triggered"; }
  else if (!forwarding_enabled_) { status.level = DS::WARN; status.message = "Forwarding disabled"; }
  else { status.level = DS::OK; status.message = "Nominal"; }
  auto kv = [](const std::string & key, const std::string & val) {
    diagnostic_msgs::msg::KeyValue m; m.key = key; m.value = val; return m; };
  { std::lock_guard<std::mutex> lock(state_mutex_);
    status.values.push_back(kv("forwarding_enabled", forwarding_enabled_ ? "true" : "false"));
    status.values.push_back(kv("e_stop_active", e_stop_active_ ? "true" : "false"));
    status.values.push_back(kv("watchdog_triggered", watchdog_triggered_ ? "true" : "false"));
    status.values.push_back(kv("cmd_vel_msgs_received", std::to_string(cmd_vel_count_)));
    status.values.push_back(kv("scan_msgs_forwarded", std::to_string(scan_count_)));
    status.values.push_back(kv("odom_msgs_forwarded", std::to_string(odom_count_)));
    const double secs = received_any_cmd_vel_ ? (now() - last_cmd_vel_time_).seconds() : -1.0;
    status.values.push_back(kv("secs_since_last_cmd_vel", std::to_string(secs)));
  }
  diag_array.status.push_back(status);
  diagnostics_pub_->publish(diag_array);
}

void Tb3BridgeNode::enableServiceCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  forwarding_enabled_ = req->data;
  if (!forwarding_enabled_) { publishZeroTwist(); RCLCPP_WARN(get_logger(), "Forwarding DISABLED."); res->message = "Disabled."; }
  else { RCLCPP_INFO(get_logger(), "Forwarding ENABLED."); res->message = "Enabled."; }
  res->success = true;
}

void Tb3BridgeNode::eStopServiceCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  e_stop_active_ = true; publishZeroTwist();
  RCLCPP_ERROR(get_logger(), "EMERGENCY STOP activated.");
  res->success = true; res->message = "E-STOP activated.";
}

void Tb3BridgeNode::eStopResetServiceCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!e_stop_active_) { res->success = false; res->message = "Not active."; return; }
  e_stop_active_ = false; watchdog_triggered_ = false; last_cmd_vel_time_ = now();
  RCLCPP_INFO(get_logger(), "E-STOP cleared.");
  res->success = true; res->message = "E-STOP cleared.";
}

void Tb3BridgeNode::publishZeroTwist()
{
  auto stamped = geometry_msgs::msg::TwistStamped();
  stamped.header.stamp = now();
  stamped.header.frame_id = "base_footprint";
  robot_cmd_vel_pub_->publish(stamped);
}

}  // namespace tb3_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tb3_bridge::Tb3BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
