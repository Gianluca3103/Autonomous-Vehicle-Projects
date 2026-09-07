#include "turtlebot3_waypoint_controller/telemetry_logger.hpp"

#include <fstream>
#include <utility>

namespace turtlebot3_waypoint_controller
{

TelemetryLogger::TelemetryLogger(std::string output_path)
: output_path_(std::move(output_path))
{
}

void TelemetryLogger::record(const TelemetrySample & sample)
{
  samples_.push_back(sample);
  saved_ = false;
}

bool TelemetryLogger::save()
{
  std::ofstream output(output_path_);
  if (!output) {
    return false;
  }

  output << "time,x,y,yaw,cross_track_error,heading_error,goal_distance,"
            "commanded_v,commanded_w,actual_v,actual_w,dt\n";
  for (const auto & sample : samples_) {
    output << sample.time << ',' << sample.x << ',' << sample.y << ',' << sample.yaw << ','
           << sample.cross_track_error << ',' << sample.heading_error << ','
           << sample.goal_distance << ',' << sample.commanded_v << ',' << sample.commanded_w << ','
           << sample.actual_v << ',' << sample.actual_w << ',' << sample.dt << '\n';
  }

  saved_ = true;
  return true;
}

bool TelemetryLogger::empty() const
{
  return samples_.empty();
}

bool TelemetryLogger::saved() const
{
  return saved_;
}

std::size_t TelemetryLogger::size() const
{
  return samples_.size();
}

double TelemetryLogger::latest_time() const
{
  return samples_.empty() ? 0.0 : samples_.back().time;
}

const std::string & TelemetryLogger::output_path() const
{
  return output_path_;
}

}  // namespace turtlebot3_waypoint_controller
