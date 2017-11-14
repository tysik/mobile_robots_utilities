#include <iostream>

#include "matplotlibcpp.h"
#include "../scaling.h"

using namespace std;
using namespace mru;

namespace plt = matplotlibcpp;

int main()
{
  double tp = 0.01;
  double wheel_radius = 0.1;
  double base_distance = 0.5;
  double max_wheel_speed = 2.0 * M_PI * 2.0; // [rad / s]

  DiffDriveScaling diff_drive_scale(base_distance, wheel_radius, max_wheel_speed);

  const size_t N = 1000;
  vector<double> time(N);
  vector<double> u(N);
  vector<double> w(N);
  vector<double> u_scaled(N);
  vector<double> w_scaled(N);

  double t = 0.0;
  double u_c = 0.0;
  double w_c = 0.0;
  for (size_t i = 0; i < N; ++i)
  {
    t = i * tp;
    time[i] = t;

    u_c = 1.0 * (1.0 + sin(2.0 * M_PI * t / 3.0));
    w_c = 2.0 * M_PI / 2.0;

    u[i] = u_c;
    w[i] = w_c;

    diff_drive_scale(u_c, w_c);

    u_scaled[i] = u_c;
    w_scaled[i] = w_c;
  }

  // Plot
  plt::title("Scaling");
  plt::xlabel("Time [s]");
  plt::ylabel("Velocity");
  plt::named_plot("Linear [m/s]", time, u, "-");
  plt::named_plot("Angular [rad/s]", time, w, "-");
  plt::named_plot("Linear scaled [m/s]", time, u_scaled, "--");
  plt::named_plot("Angular scaled [rad/s]", time, w_scaled, "--");
  plt::ylim(-0.1, 3.25);
  plt::legend();
  plt::grid(true);
  plt::show();
}
