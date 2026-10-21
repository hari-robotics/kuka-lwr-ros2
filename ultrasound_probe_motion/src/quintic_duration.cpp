#include "ultrasound_probe_motion/quintic_duration.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ultrasound_probe_motion
{

bool QuinticDurationLimits::validate(std::string& error) const
{
  if (!(std::isfinite(max_linear_velocity_mps) && max_linear_velocity_mps > 0.0 &&
        std::isfinite(max_linear_acceleration_mps2) && max_linear_acceleration_mps2 > 0.0 &&
        std::isfinite(max_angular_velocity_radps) && max_angular_velocity_radps > 0.0 &&
        std::isfinite(max_angular_acceleration_radps2) && max_angular_acceleration_radps2 > 0.0 &&
        std::isfinite(minimum_duration_sec) && minimum_duration_sec > 0.0))
  { error = "all quintic duration limits must be finite and positive"; return false; }
  return true;
}

double quinticDurationSec(double linear_distance_m, double angular_distance_rad,
                          const QuinticDurationLimits& limits)
{
  std::string error;
  if (!limits.validate(error) || !std::isfinite(linear_distance_m) ||
      !std::isfinite(angular_distance_rad)) throw std::invalid_argument("invalid quintic duration input");
  const double linear = std::abs(linear_distance_m);
  const double angular = std::abs(angular_distance_rad);
  const double peak_velocity = 1.875;
  const double peak_acceleration = 10.0 / std::sqrt(3.0);
  return std::max({limits.minimum_duration_sec,
      peak_velocity * linear / limits.max_linear_velocity_mps,
      std::sqrt(peak_acceleration * linear / limits.max_linear_acceleration_mps2),
      peak_velocity * angular / limits.max_angular_velocity_radps,
      std::sqrt(peak_acceleration * angular / limits.max_angular_acceleration_radps2)});
}

}  // namespace ultrasound_probe_motion
