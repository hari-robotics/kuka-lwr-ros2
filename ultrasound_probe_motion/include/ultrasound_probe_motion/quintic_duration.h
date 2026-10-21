#ifndef ULTRASOUND_PROBE_MOTION_QUINTIC_DURATION_H
#define ULTRASOUND_PROBE_MOTION_QUINTIC_DURATION_H

#include <string>

namespace ultrasound_probe_motion
{

struct QuinticDurationLimits
{
  double max_linear_velocity_mps = 0.06;
  double max_linear_acceleration_mps2 = 0.54;
  double max_angular_velocity_radps = 0.245436926062;       // 14.0625 deg/s
  double max_angular_acceleration_radps2 = 2.26729185243;   // 129.904 deg/s^2
  double minimum_duration_sec = 0.25;

  bool validate(std::string& error) const;
};

// Duration for s(r)=10r^3-15r^4+6r^5. Its normalized peak velocity is
// 1.875 and peak acceleration is 10/sqrt(3) ~= 5.7735.
double quinticDurationSec(double linear_distance_m, double angular_distance_rad,
                          const QuinticDurationLimits& limits);

}  // namespace ultrasound_probe_motion

#endif
