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
