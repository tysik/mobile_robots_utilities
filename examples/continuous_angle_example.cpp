#include <iostream>

#include "matplotlibcpp.h"
#include "../continuous_angle.h"

using namespace std;
using namespace mru;

namespace plt = matplotlibcpp;

int main()
{
  double tp = 0.01;

  const size_t N = 1000;
  vector<double> time(N);
  vector<double> angle(N);
  vector<double> continuous_angle(N);

  angle[0] = 0.0;
  continuous_angle[0] = makeAngleContinuous(angle[0], 0.0);

  for (size_t i = 1; i < N; ++i)
  {
    time[i] = i * tp;

    double theta = 10.0 * sin(2.0 * M_PI * time[i] / 5.0);
    angle[i] = atan2(sin(theta), cos(theta));
    continuous_angle[i] = makeAngleContinuous(angle[i], continuous_angle[i-1]);
  }

  // Plot
  plt::title("Angle continuer");
  plt::xlabel("Time [s]");
  plt::ylabel("Angle [rad]");
  plt::named_plot("Original angle", time, angle, "--");
  plt::named_plot("Continuous angle", time, continuous_angle, "-");
  plt::legend();
  plt::grid(true);
  plt::show();
}
