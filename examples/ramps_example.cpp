#include <iostream>

#include "matplotlibcpp.h"
#include "../ramps.h"

using namespace std;
using namespace ramps;

namespace plt = matplotlibcpp;

int main()
{
  double v0 = 0.0;
  double vd = 1.0;
  double T = 0.2;

  LinearRamp linear_ramp(v0, vd, T);
  PolynomialRamp polynomial_ramp(v0, vd, T);
  CosineRamp cosine_ramp(v0, vd, T);

  cout << "Ramp from velocity: " << v0 << " to velocity: " << vd << " in time: " << T << endl;
  cout << "Linear ramp at half time: " << linear_ramp(T / 2.0) << endl;
  cout << "Polynomial ramp at half time: " << polynomial_ramp(T / 2.0) << endl;
  cout << "Cosine ramp at half time: " << cosine_ramp(T / 2.0) << endl;

  const size_t N = 1000;
  vector<double> time(N);
  vector<double> linear(N);
  vector<double> polynomial(N);
  vector<double> cosine(N);

  double t = 0.0;
  for (size_t i = 0; i < N; ++i)
  {
    t = T * ((double)i / (double)N);
    time[i] = t;
    linear[i] = linear_ramp.calculateVelocity(t);
    polynomial[i] = polynomial_ramp.calculateVelocity(t);
    cosine[i] = cosine_ramp.calculateVelocity(t);
  }

  // Plot
  plt::title("Ramps");
  plt::xlabel("Time [s]");
  plt::ylabel("Velocity [m/s]");
  plt::named_plot("Linear", time, linear, "--");
  plt::named_plot("Polynomial", time, polynomial, "-");
  plt::named_plot("Cosine", time, cosine, "-");
  plt::legend();
  plt::grid(true);
  plt::show();
}
