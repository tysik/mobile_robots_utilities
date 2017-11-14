#include <iostream>

#include "matplotlibcpp.h"
#include "../trajectories.h"

using namespace std;
using namespace mru;

namespace plt = matplotlibcpp;

int main()
{
  const double T = 5.0;
  const double dt = 0.01;

  const size_t N = static_cast<size_t>(T / dt);

  vector<double> time(N);
  vector<double> x(N);
  vector<double> y(N);
  vector<double> phi(N);

  LinearTrajectory linear_trajectory(Pose(0.5, 0.5), Pose(-1.0, -0.5), 3.0);

  double t = 0.0;
  double time_shift = 1.0;
  for (size_t i = 0; i < N; ++i)
  {
    t = i * dt;
    time[i] = t;

    Configuration lin_traj = linear_trajectory(t - time_shift);

    x[i] = lin_traj.pose.x;
    y[i] = lin_traj.pose.y;
    phi[i] = lin_traj.pose.phi;
  }

  // Plot
  plt::title("Time plot");
  plt::xlabel("Time [s]");
  plt::ylabel("X and Y [m]");
  plt::named_plot("X [m/s]", time, x, "--");
  plt::named_plot("Y [rad/s]", time, y, "-");
  plt::legend();
  plt::grid(true);
  plt::show();

  // Plot
  plt::title("XY plot");
  plt::xlabel("X [m]");
  plt::ylabel("Y [m]");
  plt::plot(x, y, "-");
  plt::grid(true);
  plt::show();
}
