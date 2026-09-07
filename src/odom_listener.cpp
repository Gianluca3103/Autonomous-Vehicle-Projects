#include "turtlebot3_waypoint_controller/odom_listener.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "turtlebot3_waypoint_controller/controller_math.hpp"

using namespace std::chrono_literals;

namespace turtlebot3_waypoint_controller
{

OdomListener::OdomListener()
: Node("odom_listener"), last_time_(now()), start_time_(now())
{
  load_parameters();
  create_ros_interfaces();

  RCLCPP_INFO(get_logger(), "Subscribed to /odom");
  RCLCPP_INFO(
    get_logger(), "Subscribed to waypoints on '%s' (PoseArray)", waypoints_topic_.c_str());
}

OdomListener::~OdomListener()
{
  if (telemetry_ && !telemetry_->saved() && !telemetry_->empty()) {
    save_telemetry();
  }
}

void OdomListener::load_parameters()
{
  distance_tolerance_ = declare_parameter("distance_tolerance", 0.05);
  yaw_tolerance_ = declare_parameter("yaw_tolerance", 0.02);
  v_max_ = declare_parameter("v_max", 0.2);
  w_max_ = declare_parameter("w_max", 0.4);
  slow_distance_ = declare_parameter("slow_distance", 1.5);
  lookahead_distance_ = declare_parameter("lookahead_distance", 0.2);
  v_cruise_ = declare_parameter("v_cruise", 0.2);
  k_yaw_align_ = declare_parameter("k_yaw_align", 2.0);
  turn_in_place_alpha_ = declare_parameter("turn_in_place_alpha", 1.2);
  k_turn_ = declare_parameter("k_turn", 1.5);
  away_eps_ = declare_parameter("away_eps", 0.01);
  away_cycles_ = declare_parameter("away_cycles", 20);
  waypoints_topic_ = declare_parameter<std::string>("waypoints_topic", "/waypoints");
  const auto telemetry_path =
    declare_parameter<std::string>("telemetry_path", "project2_telemetry.csv");
  telemetry_ = std::make_unique<TelemetryLogger>(telemetry_path);
}

void OdomListener::create_ros_interfaces()
{
  const auto qos = rclcpp::QoS(10);
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", qos,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {odom_callback(*message);});
  waypoint_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
    waypoints_topic_, qos,
    [this](geometry_msgs::msg::PoseArray::ConstSharedPtr message) {
      waypoints_callback(*message);
    });
  command_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  timer_ = create_wall_timer(50ms, [this]() {control_step();});
}

void OdomListener::waypoints_callback(const geometry_msgs::msg::PoseArray & message)
{
  if (message.poses.empty()) {
    RCLCPP_WARN(get_logger(), "Ignoring an empty waypoint array.");
    return;
  }

  std::vector<Waypoint> received;
  received.reserve(message.poses.size());
  for (const auto & pose : message.poses) {
    const auto & orientation = pose.orientation;
    received.push_back({
      pose.position.x,
      pose.position.y,
      quaternion_to_yaw(
        orientation.x, orientation.y, orientation.z, orientation.w)});
  }

  waypoints_ = std::move(received);
  waypoint_index_ = 0;
  final_align_ = false;
  previous_waypoint_distance_ = std::numeric_limits<double>::quiet_NaN();
  away_count_ = 0;

  const auto & first = waypoints_.front();
  const auto & last = waypoints_.back();
  RCLCPP_INFO(get_logger(), "Loaded %zu waypoints.", waypoints_.size());
  RCLCPP_INFO(get_logger(), "First wp: (%.3f, %.3f, %.3f)", first.x, first.y, first.yaw);
  RCLCPP_INFO(get_logger(), "Last wp: (%.3f, %.3f, %.3f)", last.x, last.y, last.yaw);
}

void OdomListener::odom_callback(const nav_msgs::msg::Odometry & message)
{
  const auto & position = message.pose.pose.position;
  const auto & orientation = message.pose.pose.orientation;
  current_pose_ = {
    position.x,
    position.y,
    quaternion_to_yaw(
      orientation.x, orientation.y, orientation.z, orientation.w),
    true};
  actual_v_ = message.twist.twist.linear.x;
  actual_w_ = message.twist.twist.angular.z;
}

void OdomListener::publish_command(double linear_velocity, double angular_velocity)
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now();
  command.header.frame_id = "base_link";
  command.twist.linear.x = linear_velocity;
  command.twist.angular.z = angular_velocity;
  command_publisher_->publish(command);
}

void OdomListener::record_sample(
  double heading_error,
  double goal_distance,
  double linear_velocity,
  double angular_velocity,
  double dt)
{
  telemetry_->record({
    (now() - start_time_).seconds(),
    current_pose_.x,
    current_pose_.y,
    current_pose_.yaw,
    cross_track_error(current_pose_, waypoints_),
    heading_error,
    goal_distance,
    linear_velocity,
    angular_velocity,
    actual_v_,
    actual_w_,
    dt});
}

void OdomListener::save_telemetry()
{
  if (!telemetry_->save()) {
    RCLCPP_ERROR(
      get_logger(), "Could not write telemetry to '%s'.", telemetry_->output_path().c_str());
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Saved %zu telemetry samples to '%s'.",
    telemetry_->size(), telemetry_->output_path().c_str());
}

void OdomListener::control_step()
{
  if (!current_pose_.valid) {
    return;
  }
  if (waypoints_.empty()) {
    publish_command(0.0, 0.0);
    return;
  }

  const auto current_time = now();
  const double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;
  if (dt <= 0.0) {
    return;
  }

  while (waypoint_index_ + 1 < waypoints_.size()) {
    const auto & waypoint = waypoints_[waypoint_index_];
    if (std::hypot(
        waypoint.x - current_pose_.x,
        waypoint.y - current_pose_.y) >= distance_tolerance_)
    {
      break;
    }
    ++waypoint_index_;
    previous_waypoint_distance_ = std::numeric_limits<double>::quiet_NaN();
    away_count_ = 0;
  }

  const auto & goal = waypoints_.back();
  const double goal_distance =
    std::hypot(goal.x - current_pose_.x, goal.y - current_pose_.y);
  if (waypoint_index_ + 1 >= waypoints_.size() && goal_distance < distance_tolerance_) {
    final_align_ = true;
  }

  if (final_align_) {
    const double yaw_error = wrap_to_pi(goal.yaw - current_pose_.yaw);
    const double angular_velocity = clamp(k_yaw_align_ * yaw_error, -w_max_, w_max_);
    record_sample(yaw_error, goal_distance, 0.0, angular_velocity, dt);
    publish_command(0.0, angular_velocity);

    if (std::abs(yaw_error) < yaw_tolerance_) {
      publish_command(0.0, 0.0);
      RCLCPP_INFO(
        get_logger(), "Total completion time: %.2f s", telemetry_->latest_time());
      RCLCPP_INFO(get_logger(), "Final waypoint reached and yaw aligned.");
      save_telemetry();
      timer_->cancel();
      rclcpp::shutdown();
    }
    return;
  }

  const auto & waypoint = waypoints_[waypoint_index_];
  const double waypoint_distance =
    std::hypot(waypoint.x - current_pose_.x, waypoint.y - current_pose_.y);
  if (std::isnan(previous_waypoint_distance_)) {
    previous_waypoint_distance_ = waypoint_distance;
  }
  if (waypoint_distance > previous_waypoint_distance_ + away_eps_) {
    ++away_count_;
  } else {
    away_count_ = 0;
  }
  previous_waypoint_distance_ = waypoint_distance;

  const double target_heading =
    std::atan2(waypoint.y - current_pose_.y, waypoint.x - current_pose_.x);
  const double alpha = wrap_to_pi(target_heading - current_pose_.yaw);
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;

  if (away_count_ > away_cycles_ || std::abs(alpha) > turn_in_place_alpha_) {
    angular_velocity = clamp(k_turn_ * alpha, -w_max_, w_max_);
  } else {
    const double lookahead = std::max(0.05, lookahead_distance_);
    const double curvature = 2.0 * std::sin(alpha) / lookahead;
    linear_velocity = v_cruise_;
    if (slow_distance_ > 1e-6) {
      linear_velocity *= clamp(goal_distance / slow_distance_, 0.2, 1.0);
    }
    linear_velocity *= clamp(waypoint_distance / 0.7, 0.15, 1.0);
    angular_velocity = linear_velocity * curvature;
    linear_velocity = clamp(linear_velocity, -v_max_, v_max_);
    angular_velocity = clamp(angular_velocity, -w_max_, w_max_);
  }

  if (++debug_counter_ % 20 == 0) {
    RCLCPP_INFO(
      get_logger(),
      "[dbg] wp_idx=%zu/%zu d_wp=%.3f rho_final=%.3f alpha=%.3f v=%.3f w=%.3f",
      waypoint_index_, waypoints_.size() - 1, waypoint_distance, goal_distance, alpha,
      linear_velocity, angular_velocity);
  }

  record_sample(alpha, goal_distance, linear_velocity, angular_velocity, dt);
  publish_command(linear_velocity, angular_velocity);
}

}  // namespace turtlebot3_waypoint_controller
