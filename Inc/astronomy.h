#ifndef __ASTRONOMY_H
#define __ASTRONOMY_H

struct JD {
  int jdn;
  float jdf;
};

struct SolarDayData {
  float day_fraction;
  float daylength_fraction;
  float solar_azimuth;
  float solar_elevation;
};

JD JulianDay(const int year, const int month, const int day, const int hour,
             const int min, const int sec);
SolarDayData DayFraction(const float lat, const float lon, const JD &jd);

#endif // __ASTRONOMY_H
