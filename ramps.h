#pragma once

#include <cmath>
#include <functional>

namespace mru
{

class Ramp
{
public:
  bool isInverse() const { return inverse; }
  double getDuration() const { return duration; }

  double operator() (double time) { return ramp_function(time); }

protected:
  Ramp(double duration = 0.0, bool inverse = false) :
    inverse(inverse),
    duration(duration)
  {
    if (inverse)
      ramp_function = std::function<double(double)>([this](double time){ return getInverseRamp(time); });
    else
      ramp_function = std::function<double(double)>([this](double time){ return getRamp(time); });
  }

  virtual double getRamp(double time) = 0;

  double getInverseRamp(double time)
  {
    return getRamp(duration - time);
  }

  bool inverse;
  double duration;
  std::function<double(double time)> ramp_function;
};


class NoRamp : public Ramp
{
public:
  NoRamp(bool inverse = false) : Ramp(0.0, inverse) {}

  double getRamp(double time)
  {
    return (time <= 0.0) ? 0.0 : 1.0;
  }
};


class LaggedRamp : public Ramp
{
public:
  LaggedRamp(double duration, bool inverse = false) : Ramp(duration, inverse) {}

  double getRamp(double time)
  {
    return (time <= duration) ? 0.0 : 1.0;
  }
};


class LinearRamp : public Ramp
{
public:
  LinearRamp(double duration, bool inverse = false) : Ramp(duration, inverse) {}

  double getRamp(double time)
  {
    if (time <= 0.0)
      return 0.0;
    else if (time > 0.0 && time < duration)
      return time / duration;
    else
      return 1.0;
  }
};


class PolynomialRamp : public Ramp
{
public:
  PolynomialRamp(double duration, bool inverse = false) : Ramp(duration, inverse) {}

  double getRamp(double time)
  {
    if (time <= 0.0)
      return 0.0;
    else if (time > 0.0 && time < duration)
      return -2.0 * pow(time, 3.0) / pow(duration, 3.0) +
              3.0 * pow(time, 2.0) / pow(duration, 2.0);
    else
      return 1.0;
  }
};


class CosineRamp : public Ramp
{
public:
  CosineRamp(double duration, bool inverse = false) : Ramp(duration, inverse) {}

  double getRamp(double time)
  {
    if (time <= 0.0)
      return 0.0;
    else if (time >= 0.0 && time < duration)
      return 0.5 * (1.0 - cos(M_PI / duration * time));
    else
      return 1.0;
  }
};

} // end namespace mru
