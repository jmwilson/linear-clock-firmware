#ifndef __ASTRONOMY_H
#define __ASTRONOMY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct day_fraction_result {
  float day_fraction;
  float daylength_fraction;
};

double julian_date(int32_t year, int32_t month, int32_t day, int32_t hour,
  int32_t min, int32_t sec);
struct day_fraction_result compute_day_fraction(float lat, float lon, double jd);

#ifdef __cplusplus
}
#endif

#endif // __ASTRONOMY_H
