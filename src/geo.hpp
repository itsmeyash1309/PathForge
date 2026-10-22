#pragma once

#include <algorithm>
#include <cmath>
using namespace std;

constexpr double EARTH_RADIUS_M = 6371000.0;

inline double to_radians(double deg) { return deg * (M_PI / 180.0); }

// great-circle distance between two (lat, lon) coordinates in metres using
// Haversine formula
inline double haversine_m(double lat1, double lon1, double lat2, double lon2) {
  double phi1 = to_radians(lat1);
  double phi2 = to_radians(lat2);
  double delta_phi = to_radians(lat2 - lat1);
  double delta_lambda = to_radians(lon2 - lon1);

  double sin_dphi = sin(delta_phi / 2.0);
  double sin_dlambda = sin(delta_lambda / 2.0);
  double a =
      sin_dphi * sin_dphi + cos(phi1) * cos(phi2) * sin_dlambda * sin_dlambda;

  // required angel
  double c = 2.0 * atan2(sqrt(a), sqrt(max(0.0, 1.0 - a)));

  return EARTH_RADIUS_M * c;
}
