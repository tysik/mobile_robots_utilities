#pragma once

#include <cmath>

namespace mru
{

class DiffDriveScaling
{
public:
  DiffDriveScaling(double base_distance, double wheel_radius, double wheel_max_speed)  :
    base_distance_(base_distance),
    left_wheel_radius_(wheel_radius),
    right_wheel_radius_(wheel_radius),
    left_wheel_max_speed_(wheel_max_speed),
    right_wheel_max_speed_(wheel_max_speed)
  {}

  DiffDriveScaling(double base_distance, double left_wheel_radius, double right_wheel_radius,
                   double left_wheel_max_speed, double right_wheel_max_speed)  :
    base_distance_(base_distance),
    left_wheel_radius_(left_wheel_radius),
    right_wheel_radius_(right_wheel_radius),
    left_wheel_max_speed_(left_wheel_max_speed),
    right_wheel_max_speed_(right_wheel_max_speed)
  {}

  void scaleControls(double &u, double &w)
  {
    double w_l, w_r;
    double scale;

    // Compute wheels angular velocities from u and w
    w_r = (u + w * base_distance_ / 2.0) / right_wheel_radius_;
    w_l = (u - w * base_distance_ / 2.0) / left_wheel_radius_;

    // Check if they are below threshold and scale if so
    scale = fmax(fabs(w_r / right_wheel_max_speed_), fabs(w_l / left_wheel_max_speed_));
    if (scale > 1.0)
    {
      w_r /= scale;
      w_l /= scale;
    }
    else
      return;

    // Recompute platform velocities
    u = (w_r * right_wheel_radius_ + w_l * left_wheel_radius_) / 2.0;
    w = (w_r * right_wheel_radius_ - w_l * left_wheel_radius_) / base_distance_;
  }

  void operator() (double &u, double &w) { return scaleControls(u, w); }

protected:
  double left_wheel_max_speed_, right_wheel_max_speed_; // [rad / s]
  double left_wheel_radius_, right_wheel_radius_;       // [m]
  double base_distance_;                // Distance between wheels [m]
};

} // end namespace mru
