#ifndef TURTLEBOT3_WAYPOINT_CONTROLLER__CONTROLLER_MATH_HPP_
#define TURTLEBOT3_WAYPOINT_CONTROLLER__CONTROLLER_MATH_HPP_

#include <cstddef>
#include <vector>

#include "turtlebot3_waypoint_controller/types.hpp"

namespace turtlebot3_waypoint_controller
{

double quaternion_to_yaw(double x, double y, double z, double w);
double wrap_to_pi(double angle);
double clamp(double value, double low, double high);

bool is_forward(const RobotPose & pose, const Waypoint & waypoint);

std::size_t find_lookahead_index(
  const RobotPose & pose,
  const std::vector<Waypoint> & waypoints,
  std::size_t start_index,
  double lookahead_distance);

double cross_track_error(
  const RobotPose & pose,
  const std::vector<Waypoint> & waypoints);

}  // namespace turtlebot3_waypoint_controller

#endif  // TURTLEBOT3_WAYPOINT_CONTROLLER__CONTROLLER_MATH_HPP_
