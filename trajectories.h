#pragma once

#include <cmath>
#include <list>

#include "ramps.h"

namespace mru
{

/**
 * Position in meters and orientation in radians denoted
 * with respect to global coordinate frame
 */
struct Pose
{
  Pose(double x = 0.0, double y = 0.0, double phi = 0.0) :
    x(x), y(y), phi(phi)
  {}

  double x;
  double y;
  double phi;
};

/**
 * Linear velocity in meters per second and angular velocity
 * in radians per second denoted with respect to global
 * coordinate frame
 */
struct Twist
{
  Twist(double x = 0.0, double y = 0.0, double w = 0.0) :
    x(x), y(y), w(w)
  {}

  double x;
  double y;
  double w;
};

/**
 * Position, orientation, linear and angular velocities
 * denoted with respect to global coordinate frame
 */
struct Configuration
{
  Configuration(const Pose &pose = Pose(), const Twist &twist = Twist()) :
    pose(pose), twist(twist)
  {}

  Pose pose;
  Twist twist;
};


/**
 * Base class for trajectories
 */
class Trajectory2D
{
public:
  double getDuration() const { return duration_; }
  const Pose& getPose() const { return current_.pose; }
  const Twist& getTwist() const { return current_.twist; }
  const Configuration& getConfiguration() const { return current_; }

  const Configuration& operator() (double time) { return recalculate(time); }

protected:
  Trajectory2D(const Configuration &start = Configuration(),
               const Configuration &end = Configuration(),
               double duration = 0.0) :
    duration_(duration),
    start_(start),
    current_(start),
    end_(end)
  {}

  virtual const Configuration& recalculate(double t) = 0;

  double duration_;
  Configuration start_, current_, end_;
};


class PointTrajectory : public Trajectory2D
{
public:
  PointTrajectory(const Pose &pose) :
    Trajectory2D(Configuration(pose))
  {}

  virtual const Configuration& recalculate(double t) { return current_; }
};


class LinearTrajectory : public Trajectory2D
{
public:
  LinearTrajectory(const Pose &start, const Pose &end, double duration)
  {
    duration_ = duration;

    double phi = atan2(end.y - start.y, end.x - start.x);

    start_ = Configuration(Pose(start.x, start.y, phi),
                           Twist((end.x - start.x) / duration, (end.y - start.y) / duration));
    current_ = start_;

    end_ = Configuration(Pose(end.x, end.y, phi),
                        Twist((end.x - start.x) / duration, (end.y - start.y) / duration));
  }

  LinearTrajectory(const Configuration &start, const Configuration &end, double duration) :
    Trajectory2D(start, end, duration)
  {

  }

protected:
  virtual const Configuration& recalculate(double time)
  {
    if (time <= 0.0)
    {
      current_.pose = start_.pose;
      current_.twist = Twist();
    }
    else if (time > 0.0 && time < duration_)
    {
      current_.pose.x = start_.pose.x + (end_.pose.x - start_.pose.x) * time / duration_;
      current_.pose.y = start_.pose.y + (end_.pose.y - start_.pose.y) * time / duration_;
    }
    else
    {
      current_.pose = end_.pose;
      current_.twist = Twist();
    }

    return current_;
  }
};


//class CircularTrajectory : public Trajectory2D
//{
//public:
//  CircularTrajectory(const Pose &start, double radius, double duration) :
//    duration(duration),
//    radius(radius),
//    omega((duration > 0.0) ? 2.0 * M_PI / duration : 0.0),
//    origin(start.x - radius * sin(start.phi), start.y + radius * cos(start.phi)),
//    initial_pose(start),
//    initial_twist(radius * omega * cos(start.phi), radius * omega * sin(start.phi), omega),
//    configuration(initial_pose, initial_twist)
//  {}

//  CircularTrajectory(const Pose &origin, double phase, double radius, double duration) :
//    duration(duration),
//    radius(radius),
//    omega((duration > 0.0) ? 2.0 * M_PI / duration : 0.0),
//    origin(origin),
//    initial_pose(origin.x + radius * cos(phase), origin.y + radius * sin(phase)),
//    initial_twist(-radius * omega * sin(phase), radius * omega * cos(phase), omega),
//    configuration(initial_pose, initial_twist)
//  {}

//  virtual void recalculate(double t)
//  {
//    if (t <= 0.0)
//    {
//      configuration.pose = initial_pose;
//      configuration.twist = Twist();
//    }
//    else if (t > 0.0 && t < duration)
//    {
//      configuration.pose.x = origin.x + radius * cos(omega * t);
//      configuration.pose.y = origin.y + radius * sin(omega * t);

//      configuration.twist.x = -radius * omega * sin(omega * t);
//      configuration.twist.y =  radius * omega * cos(omega * t);

//      configuration.pose.phi = atan2(configuration.twist.y, configuration.twist.x);
//      configuration.twist.w = (-radius * pow(omega, 2.0) * sin(omega * t) * configuration.twist.x -
//                               -radius * pow(omega, 2.0) * cos(omega * t) * configuration.twist.y) /
//                              (pow(configuration.twist.x, 2.0) + pow(configuration.twist.y, 2.0));

//    }
//    else
//    {
//      configuration.pose = initial_pose;
//      configuration.twist = Twist();
//    }
//  }

//protected:
//  Pose origin;
//  double omega;
//  double radius;
//};


//class PolynomialTrajectory : public Trajectory2D
//{
//  PolynomialTrajectory(const Configuration &start, const Configuration &end, double duration) :
//    duration(duration),
//    initial_pose(start.pose),
//    initial_twist(start.twist),
//    configuration(initial_pose, initial_twist)
//  {
//    a_x = (-2.0 * (end.pose.x - start.pose.x) + 2.0 * start.twist.x * duration + (end.twist.x - start.twist.x) * duration) / pow(duration, 3.0);
//    b_x = ( 3.0 * (end.pose.x - start.pose.x) - 3.0 * start.twist.x * duration - (end.twist.x - start.twist.x) * duration) / pow(duration, 2.0);
//    c_x = start.twist.x;
//    d_x = start.pose.x;

//    a_y = (-2.0 * (end.pose.y - start.pose.y) + 2.0 * start.twist.y * duration + (end.twist.y - start.twist.y) * duration) / pow(duration, 3.0);
//    b_y = ( 3.0 * (end.pose.y - start.pose.y) - 3.0 * start.twist.y * duration - (end.twist.y - start.twist.y) * duration) / pow(duration, 2.0);
//    c_y = start.twist.y;
//    d_y = start.pose.y;
//  }

//  virtual void recalculate(double t)
//  {
//    if (t <= 0.0)
//    {
//      configuration.pose = initial_pose;
//      configuration.twist = Twist();
//    }
//    else if (t > 0.0 && t < duration)
//    {
//      configuration.pose.x = a_x * pow(t, 3.0) + b_x * pow(t, 2.0) + c_x * t + d_x;
//      configuration.pose.y = a_y * pow(t, 3.0) + b_y * pow(t, 2.0) + c_y * t + d_y;

//      configuration.twist.x = 3.0 * a_x * pow(t, 2.0) + 2.0 * b_x * t + c_x;
//      configuration.twist.y = 3.0 * a_y * pow(t, 2.0) + 2.0 * b_y * t + c_y;

//      configuration.pose.phi = atan2(configuration.twist.y, configuration.twist.x);

//      if (pow(configuration.twist.x, 2.0) + pow(configuration.twist.y, 2.0) > 0.0)
//      {
//        double acc_x = 6.0 * a_x * t + 2.0 * b_x;
//        double acc_y = 6.0 * a_y * t + 2.0 * b_y;
//        configuration.twist.w = (acc_y * configuration.twist.x - acc_x * configuration.twist.y) / (pow(configuration.twist.x, 2.0) + pow(configuration.twist.y, 2.0));
//      }
//      else
//        configuration.twist.w = 0.0;
//    }
//    else
//    {
////      configuration.pose.
//      configuration.twist = Twist();
//    }
//  }

//private:
//  double a_x, b_x, c_x, d_x;
//  double a_y, b_y, c_y, d_y;
//};

//class CombinedTrajectory : public Trajectory2D
//{
//private:
//  std::list<Configuration> configurations_;
//};


} // end namespace mru
