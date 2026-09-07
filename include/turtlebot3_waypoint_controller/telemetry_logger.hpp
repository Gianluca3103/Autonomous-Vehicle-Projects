#ifndef TURTLEBOT3_WAYPOINT_CONTROLLER__TELEMETRY_LOGGER_HPP_
#define TURTLEBOT3_WAYPOINT_CONTROLLER__TELEMETRY_LOGGER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "turtlebot3_waypoint_controller/types.hpp"

namespace turtlebot3_waypoint_controller
{

class TelemetryLogger
{
public:
  explicit TelemetryLogger(std::string output_path);

  void record(const TelemetrySample & sample);
  bool save();

  bool empty() const;
  bool saved() const;
  std::size_t size() const;
  double latest_time() const;
  const std::string & output_path() const;

private:
  std::string output_path_;
  std::vector<TelemetrySample> samples_;
  bool saved_{false};
};

}  // namespace turtlebot3_waypoint_controller

#endif  // TURTLEBOT3_WAYPOINT_CONTROLLER__TELEMETRY_LOGGER_HPP_
