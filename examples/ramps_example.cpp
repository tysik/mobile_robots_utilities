#include <iostream>

#include "matplotlibcpp.h"
#include "../ramps.h"

using namespace std;
using namespace mru;

namespace plt = matplotlibcpp;

int main()
{
  const double T = 5.0;
  const double dt = 0.01;
  const size_t N = static_cast<size_t>(T / dt);

  vector<double> time(N);
  vector<double> lagged(N);
  vector<double> linear(N);
  vector<double> polynomial(N);
  vector<double> cosine(N);

  double ramp_duration = 1.0;

  LinearRamp linear_ramp(ramp_duration);
  PolynomialRamp polynomial_ramp(ramp_duration);
  CosineRamp cosine_ramp(ramp_duration);

  LinearRamp inv_linear_ramp(ramp_duration, true);
  PolynomialRamp inv_polynomial_ramp(ramp_duration, true);
  CosineRamp inv_cosine_ramp(ramp_duration, true);

  cout << "Ramps for given duration: " << ramp_duration << " seconds." << endl;
  cout << "Linear ramp at half time: " << linear_ramp(ramp_duration / 2.0) << endl;
  cout << "Polynomial ramp at half time: " << polynomial_ramp(ramp_duration / 2.0) << endl;
  cout << "Cosine ramp at half time: " << cosine_ramp(ramp_duration / 2.0) << endl;

  double t = 0.0;
  double start_time = 1.0;
  double end_time = 3.0;

  for (size_t i = 0; i < N; ++i)
  {
    t = i * dt;

    time[i] = t;

    if (t < T / 2.0)
    {
      linear[i] = linear_ramp(t - start_time);
      polynomial[i] = polynomial_ramp(t - start_time);
      cosine[i] = cosine_ramp(t - start_time);
    }
    else
    {
      linear[i] = inv_linear_ramp(t - end_time);
      polynomial[i] = inv_polynomial_ramp(t - end_time);
      cosine[i] = inv_cosine_ramp(t - end_time);
    }
  }

  // Plot
  plt::title("Ramps");
  plt::xlabel("Time [s]");
  plt::ylabel("Ramp value [-]");
  plt::named_plot("Linear", time, linear, "--");
  plt::named_plot("Polynomial", time, polynomial, "--");
  plt::named_plot("Cosine", time, cosine, "--");
  plt::ylim(-0.1, 1.1);
  plt::legend();
  plt::grid(true);
  plt::show();
}
