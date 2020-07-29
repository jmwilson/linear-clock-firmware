/**
 * Copyright 2020 James Wilson <jmw@jmw.name>. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>
#include "astronomy.h"

JD JulianDay(const int year, const int month, const int day, const int hour,
             const int min, const int sec)
{
  const int a = (14 - month) / 12;
  const int y = year + 4800 + a;
  const int m = month + 12 * a - 3;

  int jdn = day + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100
    + y / 400 - 32045;
  float jdf = static_cast<float>(hour - 12) / 24
    + static_cast<float>(min) / 1440 + static_cast<float>(sec) / 86400;
  if (jdf < 0) {
    jdn -= 1;
    jdf += 1;
  }
  return { jdn, jdf };
}

static constexpr float RADIANS(const float deg)
{
  return 0.0174532924f * deg;
}

static constexpr float DEGREES(const float rad)
{
  return 57.2957802f * rad;
}

SolarDayData DayFraction(const float lat, const float lon, const JD &jd)
{
  constexpr int epoch = 2451545;  // January 1, 2000 12:00 UT

  // Reference: Meeus, Jean. Astronomical Algorithms, 2nd ed.
  // Richmond, Virginia: Willmann-Bell, 1998.

  // Julian century
  const float T = static_cast<float>(jd.jdn - epoch)/36525 + jd.jdf/36525;
  // Solar mean anomoly (deg)
  const float M = fmodf(357.52911f + T * (35999.05029f - .0001537f * T), 360);
  // Solar equation of the center
  const float C = (1.914602f - T * (.004817f - .000014f * T)) * sinf(RADIANS(M))
    + (.019993f - .000101f * T) * sinf(RADIANS(2 * M))
    + .000289f * sinf(RADIANS(3 * M));
  // Mean and true solar longitude (deg)
  const float L0 = fmodf(280.46646f + T * (36000.76983f + .0003032f * T), 360);
  const float L = L0 + C;
  // Longitude of the ascending node of the moon's mean orbit
  // on the ecliptic (deg)
  const float Omega = 125.04452f - 1934.136261f * T;
  // Mean and true obliquity of the ecliptic (deg)
  const float eps0 = 23.439291f
    - (T * (46.8150f - T * (.00059f + .001813f * T)))/3600;
  const float eps = eps0 + 9.2f / 3600 * cosf(RADIANS(Omega));
  // Apparent longitude of the sun, adjusted for nutation (deg)
  const float lambda = fmodf(L - .00569f - .00478f * sinf(RADIANS(Omega)), 360);
  // Solar declination (rad)
  const float delta = asinf(sinf(RADIANS(eps)) * sinf(RADIANS(lambda)));
  // Eccentricity of Earth's orbit
  const float e = .016708634f - T * (.000042037f - .0000001267f * T);
  // Approximate equation of time (deg)
  const float sqrt_y = tanf(RADIANS(eps / 2));
  const float y = sqrt_y * sqrt_y;
  const float E = DEGREES(y * sinf(RADIANS(2 * L0))
    - 2 * e * sinf(RADIANS(M))
    + 4 * e * y * sinf(RADIANS(M)) * cosf(RADIANS(2 * L0))
    - y * y / 2 * sinf(RADIANS(4 * L0))
    - 1.25f * e * e * sinf(RADIANS(2 * M)));
  // Solar hour angle (deg)
  const float H = 360 * jd.jdf + lon + E;
  // Solar elevation above (+) or below (-) the horizon (deg)
  const float h = DEGREES(asinf(sinf(RADIANS(lat)) * sinf(delta)
    + cosf(RADIANS(lat)) * cosf(delta) * cosf(RADIANS(H))));
  // Solar azimuth (deg east from north)
  const float A = 180 + DEGREES(atan2f(
    sinf(RADIANS(H)),
    cosf(RADIANS(H)) * sinf(RADIANS(lat)) - tanf(delta) * cosf(RADIANS(lat))
  ));
  // Hour angle at sunrise/sunset (deg)
  const float H0 = DEGREES(acosf(-0.0145438975f / (cosf(RADIANS(lat)) * cosf(delta))
    - tanf(RADIANS(lat)) * tanf(delta)));

  const float solar_noon = -(lon + E)/360;
  const float solar_midnight = -(lon + E - 180)/360;
  float day_fraction, daylength_fraction;

  if (isnanf(H0)) {
    // For continuity of the solution, define the start of the day at solar
    // midnight when the sun remains above the horizon, or at solar noon when
    // it remains below the horizon.
    if (h > 0) {
      day_fraction = jd.jdf - solar_midnight;
    } else {
      day_fraction = jd.jdf - solar_noon;
    }
    daylength_fraction = nanf("");
  } else {
    const float sunrise = solar_noon - H0/360;

    day_fraction = jd.jdf - sunrise;
    daylength_fraction = H0/180;
  }

  if (day_fraction < 0) {
    day_fraction += 1;
  } else if (day_fraction > 1) {
    day_fraction -= 1;
  }

  return { day_fraction, daylength_fraction, A, h };
}
