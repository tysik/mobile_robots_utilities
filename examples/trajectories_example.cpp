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
  const double trajectory_duration = 3.0;

  const size_t N = static_cast<size_t>(T / dt);

  vector<double> time(N);

  vector<double> x(N);
  vector<double> y(N);
  vector<double> phi(N);

  vector<double> x_p(N);
  vector<double> y_p(N);
  vector<double> phi_p(N);

  Configuration point_start;
  Configuration point_end(Pose(0.0, 0.0, M_PI / 2.0));
  PointTrajectory point_trajectory(point_start, point_end, trajectory_duration);

  Configuration linear_start;
  Configuration linear_end(Pose(1.0, 1.0), Twist(0.5, 0.5));
  LinearTrajectory linear_trajectory(linear_start, linear_end, trajectory_duration);

  double t = 0.0;
  double time_shift = 1.0;
  for (size_t i = 0; i < N; ++i)
  {
    t = i * dt;
    time[i] = t;

    Configuration current = point_trajectory(t - time_shift);

    x[i] = current.pose.x;
    y[i] = current.pose.y;
    phi[i] = current.pose.phi;

    x_p[i] = current.twist.x;
    y_p[i] = current.twist.y;
    phi_p[i] = current.twist.w;
  }

  // Plots
  plt::figure();
  plt::title("Positions in time");
  plt::xlabel("Time [s]");
  plt::ylabel("X, Y and Phi");
  plt::named_plot("X [m]", time, x, "-");
  plt::named_plot("Y [m]", time, y, "-");
  plt::named_plot("Phi [rad]", time, phi, "-");
  plt::legend();
  plt::grid(true);
  plt::show();

  plt::figure();
  plt::title("Velocities in time");
  plt::xlabel("Time [s]");
  plt::ylabel("X', Y' and Phi'");
  plt::named_plot("X' [m/s]", time, x_p, "-");
  plt::named_plot("Y' [m/s]", time, y_p, "-");
  plt::named_plot("Phi' [rad/s]", time, phi_p, "-");
  plt::legend();
  plt::grid(true);
  plt::show();

  plt::figure();
  plt::title("Path");
  plt::xlabel("X [m]");
  plt::ylabel("Y [m]");
  plt::named_plot("Path", x, y, "-");
  plt::named_plot("Start", {x[0]}, {y[0]}, "o");
  plt::named_plot("End", {x[N-1]}, {y[N-1]}, "x");
  plt::legend();
  plt::grid(true);
  plt::show();
}
