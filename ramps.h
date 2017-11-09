#pragma once

#include <cmath>

namespace mru
{

class VelocityRamp
{
public:
  VelocityRamp(double v0, double vd) : v0_(v0), vd_(vd), T_(0.0) {}
  VelocityRamp(double v0, double vd, double T) : v0_(v0), vd_(vd), T_(T) {}

  virtual double calculateVelocity(double t) = 0;
  double operator() (double t) { return calculateVelocity(t); }

protected:
  double v0_, vd_, T_;
};

class NoRamp : public VelocityRamp
{
  NoRamp(double v0, double vd) : VelocityRamp(v0, vd) {}

  double calculateVelocity(double t)
  {
    double v = 0.0;

    if (t < 0.0)
      v = v0_;
    else
      v = vd_;

    return v;
  }
};

class LinearRamp : public VelocityRamp
{
public:
  LinearRamp(double v0, double vd, double T) : VelocityRamp(v0, vd, T) {}

  double calculateVelocity(double t)
  {
    double v = 0.0;

    if (t < 0.0)
      v = v0_;
    else if (t >= 0.0 && t < T_)
      v = v0_ + (t / T_) * vd_;
    else
      v = vd_;

    return v;
  }
};

class PolynomialRamp : public VelocityRamp
{
public:
  PolynomialRamp(double v0, double vd, double T) : VelocityRamp(v0, vd, T)
  {
    a_ = 2.0 * (v0_ - vd_) / pow(T_, 3.0);
    b_ = 3.0 * (vd_ - v0_) / pow(T_, 2.0);
    c_ = 0.0;
    d_ = v0_;
  }

  double calculateVelocity(double t)
  {
    double v = 0.0;

    if (t < 0.0)
      v = v0_;
    else if (t >= 0.0 && t < T_)
      v = a_ * pow(t, 3.0) + b_ * pow(t, 2.0) + c_ * t + d_;
    else
      v = vd_;

    return v;
  }

private:
  double a_, b_, c_, d_;
};

class CosineRamp : public VelocityRamp
{
public:
  CosineRamp(double v0, double vd, double T) : VelocityRamp(v0, vd, T)
  {
    A_ = (vd_ - v0_) / 2.0; // Amplitude is twice smaller because the cosine is shifted up
    omega_ = 2.0 * M_PI / (2.0 * T_); // Period is twice bigger because we take only half
  }

  double calculateVelocity(double t)
  {
    double v = 0.0;

    if (t < 0.0)
      v = v0_;
    else if (t >= 0.0 && t < T_)
      v = v0_ + A_ * (1.0 - cos(omega_ * t));
    else
      v = vd_;
  }

private:
  double omega_;
  double A_;
};


} // end namespace mru
