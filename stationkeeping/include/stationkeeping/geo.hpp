#pragma once
#include <cmath>

//Garante que o angulo recebido sempre fique no intervalor [−π,π]
inline double wrap_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

//Recebe os valores dos quarterions e retornar o yam em radianos
inline double yaw_from_quat(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// Aproximação local: erro (east,north) em metros usando a latitude atual.
// Bom para stationkeeping.
inline void latlon_error_m(double lat_deg, double lon_deg,
                           double goal_lat_deg, double goal_lon_deg,
                           double &east_m, double &north_m)
{
  constexpr double R = 6378137.0;
  const double lat  = lat_deg * M_PI / 180.0;
  const double lon  = lon_deg * M_PI / 180.0;
  const double glat = goal_lat_deg * M_PI / 180.0;
  const double glon = goal_lon_deg * M_PI / 180.0;

  const double dlat = (glat - lat);
  const double dlon = (glon - lon);

  east_m  = R * dlon * std::cos(lat);
  north_m = R * dlat;
}