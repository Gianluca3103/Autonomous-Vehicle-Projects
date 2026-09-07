#include "turtlebot3_waypoint_controller/controller_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace turtlebot3_waypoint_controller
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

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

bool is_forward(const RobotPose & pose, const Waypoint & waypoint)
{
  const double heading_x = std::cos(pose.yaw);
  const double heading_y = std::sin(pose.yaw);
  return heading_x * (waypoint.x - pose.x) + heading_y * (waypoint.y - pose.y) > 0.0;
}

std::size_t find_lookahead_index(
  const RobotPose & pose,
  const std::vector<Waypoint> & waypoints,
  std::size_t start_index,
  double lookahead_distance)
{
  if (waypoints.empty()) {
    return 0;
  }

  const std::size_t last = waypoints.size() - 1;
  const std::size_t baseline = std::min(start_index + 1, last);
  for (std::size_t index = start_index; index < waypoints.size(); ++index) {
    const auto & waypoint = waypoints[index];
    if (is_forward(pose, waypoint) &&
      std::hypot(waypoint.x - pose.x, waypoint.y - pose.y) >= lookahead_distance)
    {
      return index;
    }
  }
  return baseline;
}

double cross_track_error(
  const RobotPose & pose,
  const std::vector<Waypoint> & waypoints)
{
  if (waypoints.size() < 2) {
    return 0.0;
  }

  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index + 1 < waypoints.size(); ++index) {
    const auto & first = waypoints[index];
    const auto & second = waypoints[index + 1];
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double denominator = dx * dx + dy * dy;

    if (denominator < 1e-12) {
      minimum = std::min(minimum, std::hypot(pose.x - first.x, pose.y - first.y));
      continue;
    }

    const double projection = clamp(
      ((pose.x - first.x) * dx + (pose.y - first.y) * dy) / denominator,
      0.0,
      1.0);
    minimum = std::min(
      minimum,
      std::hypot(
        pose.x - (first.x + projection * dx),
        pose.y - (first.y + projection * dy)));
  }
  return minimum;
}

}  // namespace turtlebot3_waypoint_controller
