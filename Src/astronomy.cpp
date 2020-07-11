#include <cmath>
#include "astronomy.h"

double julian_date(int32_t year, int32_t month, int32_t day, int32_t hour,
 int32_t min, int32_t sec)
{
  int32_t a = (14 - month) / 12;
  int32_t y = year + 4800 + a;
  int32_t m = month + 12 * a - 3;

  int32_t jdn = day + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100
    + y / 400 - 32045;
  double jd = double(jdn) + double(hour - 12) / 24 + double(min) / 1440
    + double(sec) / 86400;
  return jd;
}

static inline float RADIANS(float deg)
{
  return 0.0174532924f * deg;
}

static inline float DEGREES(float rad)
{
  return 57.2957802f * rad;
}

struct day_fraction_result compute_day_fraction(float lat, float lon, double jd)
{
  int32_t epoch = 2451545;  // January 1, 2000 12:00 UT

  // Reference: Meeus, Jean. Astronomical Algorithms, 2nd ed.
  // Richmond, Virginia: Willmann-Bell, 1998.

  // Julian century
  float T = (jd - epoch) / 36525;
  // Solar mean anomoly (deg)
  float M = fmodf(357.52911f + T * (35999.05029f - .0001537f * T), 360);
  // Solar equation of the center
  float C = (1.914602f - T * (.004817f - .000014f * T)) * sinf(RADIANS(M))
    + (.019993f - .000101f * T) * sinf(RADIANS(2 * M))
    + .000289f * sinf(RADIANS(3 * M));
  // Mean and true solar longitude (deg)
  float L0 = fmodf(280.46646f + T * (36000.76983f + .0003032f * T), 360);
  float L = L0 + C;
  // Longitude of the ascending node of the moon's mean orbit
  // on the ecliptic (deg)
  float Omega = 125.04452f - 1934.136261f * T;
  // Mean and true obliquity of the ecliptic (deg)
  float eps0 = 23.439291f
    - (T * (46.8150f - T * (.00059f + .001813f * T)))/3600;
  float eps = eps0 + 9.2f / 3600 * cosf(RADIANS(Omega));
  // Apparent longitude of the sun, adjusted for nutation (deg)
  float lambda = fmodf(L - .00569f - .00478f * sinf(RADIANS(Omega)), 360);
  // Solar declination (rad)
  float delta = asinf(sinf(RADIANS(eps)) * sinf(RADIANS(lambda)));
  // Eccentricity of Earth's orbit
  float e = .016708634f - T * (.000042037f - .0000001267f * T);
  // Approximate equation of time (deg)
  float y = tanf(RADIANS(eps / 2));
  y *= y;
  float E = DEGREES(y * sinf(RADIANS(2 * L0))
    - 2 * e * sinf(RADIANS(M))
    + 4 * e * y * sinf(RADIANS(M)) * cosf(RADIANS(2 * L0))
    - y * y / 2 * sinf(RADIANS(4 * L0))
    - 1.25f * e * e * sinf(RADIANS(2 * M)));
  // Hour angle (deg)
  float H0 = DEGREES(acosf(-0.0145438975f / (cosf(RADIANS(lat)) * cosf(delta))
    - tanf(RADIANS(lat)) * tanf(delta)));

  float solar_noon = -(lon + E)/360;
  float sunrise = solar_noon - H0/360;
  float day_fraction = float(jd - floor(jd)) - sunrise;
  float daylength_fraction = H0/180;

  if (day_fraction < 0) {
    day_fraction += 1;
  } else if (day_fraction > 1) {
    day_fraction -= 1;
  }

  return {
    .day_fraction = day_fraction,
    .daylength_fraction = daylength_fraction,
  };
}
