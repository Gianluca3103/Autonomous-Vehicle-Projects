# TurtleBot3 Waypoint Controller (C++)

A ROS 2 C++ conversion of `Project2.py`. The node follows a `PoseArray` of waypoints
using a pure-pursuit-style controller, performs recovery turns if it moves away from
the active waypoint, and aligns to the final waypoint's yaw.

## Requirements

- ROS 2 (tested package structure for Humble and newer)
- `geometry_msgs`, `nav_msgs`, and `rclcpp`
- TurtleBot3 Gazebo packages for the included simulation launch file

## Build

Clone this repository into the `src` directory of a ROS 2 workspace, then run:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select turtlebot3_waypoint_controller
source install/setup.bash
```

## Run

Start Gazebo and the controller together:

```bash
ros2 launch turtlebot3_waypoint_controller project2.launch.py
```

Or run only the controller:

```bash
ros2 run turtlebot3_waypoint_controller project2_controller
```

Publish waypoints as a `geometry_msgs/msg/PoseArray` on `/waypoints`. For example:

```bash
ros2 topic pub --once /waypoints geometry_msgs/msg/PoseArray \
  "{header: {frame_id: 'odom'}, poses: [
    {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}},
    {position: {x: 1.0, y: 1.0}, orientation: {z: 0.7071068, w: 0.7071068}}
  ]}"
```

The controller writes `project2_telemetry.csv` when the route completes or the node
shuts down. Override the path with `--ros-args -p telemetry_path:=/path/to/run.csv`.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `distance_tolerance` | `0.05` | Waypoint position tolerance (m) |
| `yaw_tolerance` | `0.02` | Final yaw tolerance (rad) |
| `v_max` | `0.2` | Maximum linear speed (m/s) |
| `w_max` | `0.4` | Maximum angular speed (rad/s) |
| `slow_distance` | `1.5` | Final-goal slowdown radius (m) |
| `lookahead_distance` | `0.2` | Pure-pursuit lookahead (m) |
| `v_cruise` | `0.2` | Nominal linear speed (m/s) |
| `k_yaw_align` | `2.0` | Final heading proportional gain |
| `turn_in_place_alpha` | `1.2` | Heading error that triggers an in-place turn (rad) |
| `k_turn` | `1.5` | In-place turn proportional gain |
| `away_eps` | `0.01` | Distance increase counted as moving away (m) |
| `away_cycles` | `20` | Consecutive away samples before recovery |
| `waypoints_topic` | `/waypoints` | Waypoint `PoseArray` topic |
| `telemetry_path` | `project2_telemetry.csv` | Output CSV path |

## License

MIT
