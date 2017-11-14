#pragma once

#include <cmath>

namespace mru
{

double makeAngleRestricted(double angle)
{
  if (angle < -M_PI || angle > M_PI)
    angle = atan2(sin(angle), cos(angle));

  return angle;
}

double makeAngleContinuous(double angle, double prev_angle)
{
  double new_angle_aux = makeAngleRestricted(angle);
  double prev_angle_aux = makeAngleRestricted(prev_angle);

  double angle_diff = new_angle_aux - prev_angle_aux;

  if (angle_diff < -M_PI)
    angle = prev_angle + angle_diff + 2.0 * M_PI;
  else if (angle_diff > M_PI)
    angle = prev_angle + angle_diff - 2.0 * M_PI;
  else
    angle = prev_angle + angle_diff;

  return angle;
}

} // end namespace mru
