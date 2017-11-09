#pragma once

#include <cmath>

namespace mru
{

struct Pose
{
  double x;
  double y;
  double theta;
};

struct Twist
{
  double u;
  double v;
  double w;
};

class Trajectory
{
public:
  Trajectory() : x_0_(0.0), y_0_(0.0), theta_0_(0.0) {}
  Trajectory(double x, double y, double theta) : x_0_(x), y_0_(y), theta_0_(theta) {}

  virtual Pose calculatePose(double t) = 0;
  virtual Twist calculateVelocity(double t) = 0;

protected:
  Pose pose_;
  Twist twist_;

  double x_0_;
  double y_0_;
  double theta_0_;
};


class PointTrajectory : public Trajectory
{
public:
  PointTrajectory(double x, double y, double theta) : Trajectory(x, y, theta) {}

  virtual Pose calculatePose(double t)
  {
    pose_.x = x_0_;
    pose_.y = y_0_;
    pose_.theta = theta_0_;

    return pose_;
  }

  virtual Twist calculateVelocity(double t)
  {
    twist_.u = 0.0;
    twist_.v = 0.0;
    twist_.w = 0.0;

    return twist_;
  }
};


class LinearTrajectory : public Trajectory
{
public:
  LinearTrajectory() : v_(0.0) {}
  LinearTrajectory(double theta, double v) : Trajectory(0.0, 0.0, theta), v_(v) {}
  LinearTrajectory(double x, double y, double theta, double v) : Trajectory(x, y, theta), v_(v) {}

  virtual Pose calculatePose(double t)
  {
    pose_.x = x_0_ + v_ * cos(theta_0_) * t;
    pose_.y = y_0_ + v_ * sin(theta_0_) * t;
    pose_.theta = theta_0_;

    return pose_;
  }

  virtual Twist calculateVelocity(double t)
  {
    twist_.u = v_;
    twist_.v = 0.0;
    twist_.w = 0.0;

    return twist_;
  }

protected:
  double v_;    // Speed [m/s]
};


class HarmonicTrajectory : public Trajectory
{
public:
  HarmonicTrajectory() :
    w_(0.0), r_x_(0.0), r_y_(0.0), n_x_(0.0), n_y_(0.0)
  {}

  HarmonicTrajectory(double T, double r_x, double r_y, int n_x, int n_y) :
    r_x_(r_x), r_y_(r_y), n_x_(n_x), n_y_(n_y)
  {
    (T > 0.0) ? w_ = 2.0 * M_PI / T : w_ = 0.0;
  }

  HarmonicTrajectory(double x, double y, double T, double r_x, double r_y, int n_x, int n_y) :
    Trajectory(x, y, 0.0), r_x_(r_x), r_y_(r_y), n_x_(n_x), n_y_(n_y)
  {
    (T > 0.0) ? w_ = 2.0 * M_PI / T : w_ = 0.0;
  }

  virtual Pose calculatePose(double t)
  {
    pose_.x = x_0_ + r_x_ * cos(n_x_ * w_ * t);
    pose_.y = y_0_ + r_y_ * sin(n_y_ * w_ * t);
    pose_.theta = atan2(r_y_ * n_y_ * w_ * cos(n_y_ * w_ * t), -r_x_ * n_x_ * w_ * sin(n_x_ * w_ * t));

    return pose_;
  }

  virtual Twist calculateVelocity(double t)
  {
    // TODO: Make local frame
    twist_.u = -r_x_ * n_x_ * w_ * sin(n_x_ * w_ * t);
    twist_.v =  r_y_ * n_y_ * w_ * cos(n_y_ * w_ * t);

    if (pow(twist_.u, 2.0) + pow(twist_.v, 2.0) > 0.0)
      twist_.w = (-r_y_ * pow(n_y_, 2.0) * pow(w_, 2.0) * sin(n_y_ * w_ * t) * twist_.u -
                  -r_x_ * pow(n_x_, 2.0) * pow(w_, 2.0) * cos(n_x_ * w_ * t) * twist_.v) /
                 (pow(twist_.u, 2.0) + pow(twist_.v, 2.0));
    else
      twist_.w = 0.0;

    return twist_;
  }

protected:
  double w_;           // Frequency [rad/s]
  double r_x_, r_y_;   // Radii [m]
  int n_x_, n_y_;      // Frequency multipliers [-]
};

} // end namespace mru
