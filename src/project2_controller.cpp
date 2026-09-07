#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;

double quaternion_to_yaw(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double wrap_to_pi(double angle)
{
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

double clamp(double value, double low, double high)
{
  return std::clamp(value, low, high);
}
}  // namespace

class OdomListener final : public rclcpp::Node
{
public:
  OdomListener()
  : Node("odom_listener"), last_time_(now()), start_time_(now())
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
    const auto waypoints_topic = declare_parameter<std::string>("waypoints_topic", "/waypoints");
    telemetry_path_ = declare_parameter<std::string>("telemetry_path", "project2_telemetry.csv");

    const auto qos = rclcpp::QoS(10);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_callback(*msg); });
    waypoint_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      waypoints_topic, qos,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) { waypoints_callback(*msg); });
    cmd_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    timer_ = create_wall_timer(50ms, [this]() { control_step(); });

    RCLCPP_INFO(get_logger(), "Subscribed to /odom");
    RCLCPP_INFO(get_logger(), "Subscribed to waypoints on '%s' (PoseArray)", waypoints_topic.c_str());
  }

  ~OdomListener() override
  {
    if (!telemetry_saved_ && !samples_.empty()) {
      save_telemetry();
    }
  }

private:
  struct Waypoint
  {
    double x;
    double y;
    double yaw;
  };

  struct Pose
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    bool valid{false};
  };

  struct Sample
  {
    double time;
    double x;
    double y;
    double yaw;
    double cross_track_error;
    double heading_error;
    double goal_distance;
    double commanded_v;
    double commanded_w;
    double actual_v;
    double actual_w;
    double dt;
  };

  void waypoints_callback(const geometry_msgs::msg::PoseArray & msg)
  {
    if (msg.poses.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring an empty waypoint array.");
      return;
    }

    std::vector<Waypoint> received;
    received.reserve(msg.poses.size());
    for (const auto & pose : msg.poses) {
      const auto & q = pose.orientation;
      received.push_back({
        pose.position.x,
        pose.position.y,
        quaternion_to_yaw(q.x, q.y, q.z, q.w)});
    }

    waypoints_ = std::move(received);
    wp_idx_ = 0;
    final_align_ = false;
    previous_waypoint_distance_ = std::numeric_limits<double>::quiet_NaN();
    away_count_ = 0;

    const auto & first = waypoints_.front();
    const auto & last = waypoints_.back();
    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints.", waypoints_.size());
    RCLCPP_INFO(get_logger(), "First wp: (%.3f, %.3f, %.3f)", first.x, first.y, first.yaw);
    RCLCPP_INFO(get_logger(), "Last wp: (%.3f, %.3f, %.3f)", last.x, last.y, last.yaw);
  }

  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
    const auto & position = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    current_pose_ = {
      position.x, position.y, quaternion_to_yaw(q.x, q.y, q.z, q.w), true};
    actual_v_ = msg.twist.twist.linear.x;
    actual_w_ = msg.twist.twist.angular.z;
  }

  void publish_command(double v, double w)
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = "base_link";
    command.twist.linear.x = v;
    command.twist.angular.z = w;
    cmd_publisher_->publish(command);
  }

  bool is_forward(double wx, double wy) const
  {
    const double heading_x = std::cos(current_pose_.yaw);
    const double heading_y = std::sin(current_pose_.yaw);
    return heading_x * (wx - current_pose_.x) + heading_y * (wy - current_pose_.y) > 0.0;
  }

  [[maybe_unused]] std::size_t find_lookahead_index(std::size_t start_idx, double distance) const
  {
    const std::size_t last = waypoints_.size() - 1;
    const std::size_t baseline = std::min(start_idx + 1, last);
    for (std::size_t i = start_idx; i < waypoints_.size(); ++i) {
      const auto & wp = waypoints_[i];
      if (is_forward(wp.x, wp.y) &&
        std::hypot(wp.x - current_pose_.x, wp.y - current_pose_.y) >= distance)
      {
        return i;
      }
    }
    return baseline;
  }

  double cross_track_error() const
  {
    if (waypoints_.size() < 2) {
      return 0.0;
    }

    double minimum = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < waypoints_.size(); ++i) {
      const auto & first = waypoints_[i];
      const auto & second = waypoints_[i + 1];
      const double dx = second.x - first.x;
      const double dy = second.y - first.y;
      const double denominator = dx * dx + dy * dy;
      if (denominator < 1e-12) {
        minimum = std::min(
          minimum, std::hypot(current_pose_.x - first.x, current_pose_.y - first.y));
        continue;
      }
      const double projection = clamp(
        ((current_pose_.x - first.x) * dx + (current_pose_.y - first.y) * dy) /
        denominator, 0.0, 1.0);
      minimum = std::min(
        minimum,
        std::hypot(
          current_pose_.x - (first.x + projection * dx),
          current_pose_.y - (first.y + projection * dy)));
    }
    return minimum;
  }

  void record_sample(
    double heading_error, double goal_distance, double v, double w, double dt)
  {
    samples_.push_back({
      (now() - start_time_).seconds(), current_pose_.x, current_pose_.y, current_pose_.yaw,
      cross_track_error(), heading_error, goal_distance, v, w, actual_v_, actual_w_, dt});
  }

  void save_telemetry()
  {
    std::ofstream output(telemetry_path_);
    if (!output) {
      RCLCPP_ERROR(get_logger(), "Could not write telemetry to '%s'.", telemetry_path_.c_str());
      return;
    }
    output << "time,x,y,yaw,cross_track_error,heading_error,goal_distance,"
              "commanded_v,commanded_w,actual_v,actual_w,dt\n";
    for (const auto & s : samples_) {
      output << s.time << ',' << s.x << ',' << s.y << ',' << s.yaw << ','
             << s.cross_track_error << ',' << s.heading_error << ',' << s.goal_distance << ','
             << s.commanded_v << ',' << s.commanded_w << ',' << s.actual_v << ','
             << s.actual_w << ',' << s.dt << '\n';
    }
    telemetry_saved_ = true;
    RCLCPP_INFO(get_logger(), "Saved %zu telemetry samples to '%s'.", samples_.size(),
      telemetry_path_.c_str());
  }

  void control_step()
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

    while (wp_idx_ + 1 < waypoints_.size()) {
      const auto & wp = waypoints_[wp_idx_];
      if (std::hypot(wp.x - current_pose_.x, wp.y - current_pose_.y) >= distance_tolerance_) {
        break;
      }
      ++wp_idx_;
      previous_waypoint_distance_ = std::numeric_limits<double>::quiet_NaN();
      away_count_ = 0;
    }

    const auto & goal = waypoints_.back();
    const double goal_distance = std::hypot(
      goal.x - current_pose_.x, goal.y - current_pose_.y);
    if (wp_idx_ + 1 >= waypoints_.size() && goal_distance < distance_tolerance_) {
      final_align_ = true;
    }

    if (final_align_) {
      const double yaw_error = wrap_to_pi(goal.yaw - current_pose_.yaw);
      const double w = clamp(k_yaw_align_ * yaw_error, -w_max_, w_max_);
      record_sample(yaw_error, goal_distance, 0.0, w, dt);
      publish_command(0.0, w);
      if (std::abs(yaw_error) < yaw_tolerance_) {
        publish_command(0.0, 0.0);
        RCLCPP_INFO(get_logger(), "Total completion time: %.2f s", samples_.back().time);
        RCLCPP_INFO(get_logger(), "Final waypoint reached and yaw aligned.");
        save_telemetry();
        timer_->cancel();
        rclcpp::shutdown();
      }
      return;
    }

    const auto & wp = waypoints_[wp_idx_];
    const double waypoint_distance = std::hypot(
      wp.x - current_pose_.x, wp.y - current_pose_.y);
    if (std::isnan(previous_waypoint_distance_)) {
      previous_waypoint_distance_ = waypoint_distance;
    }
    if (waypoint_distance > previous_waypoint_distance_ + away_eps_) {
      ++away_count_;
    } else {
      away_count_ = 0;
    }
    previous_waypoint_distance_ = waypoint_distance;

    const double target_heading = std::atan2(
      wp.y - current_pose_.y, wp.x - current_pose_.x);
    const double alpha = wrap_to_pi(target_heading - current_pose_.yaw);
    double v = 0.0;
    double w = 0.0;

    if (away_count_ > away_cycles_ || std::abs(alpha) > turn_in_place_alpha_) {
      w = clamp(k_turn_ * alpha, -w_max_, w_max_);
    } else {
      const double lookahead = std::max(0.05, lookahead_distance_);
      const double curvature = 2.0 * std::sin(alpha) / lookahead;
      v = v_cruise_;
      if (slow_distance_ > 1e-6) {
        v *= clamp(goal_distance / slow_distance_, 0.2, 1.0);
      }
      v *= clamp(waypoint_distance / 0.7, 0.15, 1.0);
      w = v * curvature;
      v = clamp(v, -v_max_, v_max_);
      w = clamp(w, -w_max_, w_max_);
    }

    if (++debug_counter_ % 20 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "[dbg] wp_idx=%zu/%zu d_wp=%.3f rho_final=%.3f alpha=%.3f v=%.3f w=%.3f",
        wp_idx_, waypoints_.size() - 1, waypoint_distance, goal_distance, alpha, v, w);
    }
    record_sample(alpha, goal_distance, v, w, dt);
    publish_command(v, w);
  }

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
  int64_t away_cycles_{};
  std::string telemetry_path_;

  Pose current_pose_;
  std::vector<Waypoint> waypoints_;
  std::size_t wp_idx_{0};
  bool final_align_{false};
  double previous_waypoint_distance_{std::numeric_limits<double>::quiet_NaN()};
  int64_t away_count_{0};
  int debug_counter_{0};
  double actual_v_{0.0};
  double actual_w_{0.0};
  bool telemetry_saved_{false};
  rclcpp::Time last_time_;
  rclcpp::Time start_time_;
  std::vector<Sample> samples_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomListener>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
