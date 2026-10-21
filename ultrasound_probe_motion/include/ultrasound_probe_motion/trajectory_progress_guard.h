#pragma once

#include <cmath>

namespace ultrasound_probe_motion
{

class TrajectoryProgressGuard
{
public:
  void reset()
  {
    elapsed_sec_ = 0.0;
    paused_sec_ = 0.0;
  }

  bool update(double dt_sec, double position_error_m, double orientation_error_rad,
              double max_position_error_m, double max_orientation_error_rad)
  {
    if (!(std::isfinite(dt_sec) && dt_sec >= 0.0 &&
          std::isfinite(position_error_m) && std::isfinite(orientation_error_rad)))
      return false;
    const bool within = position_error_m <= max_position_error_m &&
                        orientation_error_rad <= max_orientation_error_rad;
    if (within) elapsed_sec_ += dt_sec;
    else paused_sec_ += dt_sec;
    return within;
  }

  double elapsedSec() const { return elapsed_sec_; }
  double pausedSec() const { return paused_sec_; }

private:
  double elapsed_sec_ = 0.0;
  double paused_sec_ = 0.0;
};

}  // namespace ultrasound_probe_motion
