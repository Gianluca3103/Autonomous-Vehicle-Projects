#ifndef TURTLEBOT3_WAYPOINT_CONTROLLER__TYPES_HPP_
#define TURTLEBOT3_WAYPOINT_CONTROLLER__TYPES_HPP_

namespace turtlebot3_waypoint_controller
{

struct Waypoint
{
  double x;
  double y;
  double yaw;
};

struct RobotPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool valid{false};
};

struct TelemetrySample
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

}  // namespace turtlebot3_waypoint_controller

#endif  // TURTLEBOT3_WAYPOINT_CONTROLLER__TYPES_HPP_
