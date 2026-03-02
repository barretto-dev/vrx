#pragma once
#include <algorithm>

inline double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

struct PID
{
  double kp{0.0}, ki{0.0}, kd{0.0};
  double i_term{0.0};
  double prev_e{0.0};
  bool have_prev{false};

  double i_min{-1e9}, i_max{1e9};
  double out_min{-1e9}, out_max{1e9};

  void reset()
  {
    i_term = 0.0;
    prev_e = 0.0;
    have_prev = false;
  }

  double step(double e, double dt)
  {
    if (dt <= 0.0) return 0.0;

    i_term += e * dt;
    i_term = clamp(i_term, i_min, i_max);

    double de = 0.0;
    if (have_prev) de = (e - prev_e) / dt;
    prev_e = e;
    have_prev = true;

    const double u = kp * e + ki * i_term + kd * de;
    return clamp(u, out_min, out_max);
  }
};