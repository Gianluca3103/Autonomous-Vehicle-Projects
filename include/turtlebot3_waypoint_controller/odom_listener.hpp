#ifndef TURTLEBOT3_WAYPOINT_CONTROLLER__ODOM_LISTENER_HPP_
#define TURTLEBOT3_WAYPOINT_CONTROLLER__ODOM_LISTENER_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlebot3_waypoint_controller/telemetry_logger.hpp"
#include "turtlebot3_waypoint_controller/types.hpp"

namespace turtlebot3_waypoint_controller
{

class OdomListener final : public rclcpp::Node
{
public:
  OdomListener();
  ~OdomListener() override;

private:
  void load_parameters();
  void create_ros_interfaces();

  void waypoints_callback(const geometry_msgs::msg::PoseArray & message);
  void odom_callback(const nav_msgs::msg::Odometry & message);

  void publish_command(double linear_velocity, double angular_velocity);
  void record_sample(
    double heading_error,
    double goal_distance,
    double linear_velocity,
    double angular_velocity,
    double dt);
  void save_telemetry();
  void control_step();

  double distance_tolerance_{};
  double yaw_tolerance_{};
  double v_max_{};
  double w_max_{};
  double slow_distance_{};
  double lookahead_distance_{};
  double v_cruise_{};
  double k_yaw_align_{};
  double turn_in_place_alpha_{};
  double k_turn_{};
  double away_eps_{};
  std::int64_t away_cycles_{};
  std::string waypoints_topic_;

  RobotPose current_pose_;
  std::vector<Waypoint> waypoints_;
  std::size_t waypoint_index_{0};
  bool final_align_{false};
  double previous_waypoint_distance_{std::numeric_limits<double>::quiet_NaN()};
  std::int64_t away_count_{0};
  int debug_counter_{0};
  double actual_v_{0.0};
  double actual_w_{0.0};
  rclcpp::Time last_time_;
  rclcpp::Time start_time_;
  std::unique_ptr<TelemetryLogger> telemetry_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace turtlebot3_waypoint_controller

#endif  // TURTLEBOT3_WAYPOINT_CONTROLLER__ODOM_LISTENER_HPP_
