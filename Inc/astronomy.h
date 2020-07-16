#ifndef __ASTRONOMY_H
#define __ASTRONOMY_H

#include <cstdint>

struct JD {
  int jdn;
  float jdf;
};

struct day_fraction_result {
  float day_fraction;
  float daylength_fraction;
  float solar_azimuth;
  float solar_elevation;
};

JD julian_date(int year, int month, int day, int hour,
  int min, int sec);
day_fraction_result compute_day_fraction(float lat, float lon, const JD &jd);

#endif // __ASTRONOMY_H
