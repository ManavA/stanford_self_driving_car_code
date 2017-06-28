/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/


#ifndef PERCEPTION_UTILS_H_
#define PERCEPTION_UTILS_H_

#include <string>
#include <grid.h>

#include <perception_types.h>

namespace perception {

int inside_car(float x, float y, dgc::dgc_pose_t robot);
uint16_t counter_diff(uint16_t last, uint16_t now);

double sample(double b);

void display_time(const std::string label, double time);

inline double linear(double x, double slope, double intercept) {
  return (slope * x + intercept);
}

inline double angle_diff(double t1, double t2) {
  double dt = t1 - t2;
  while (dt > M_PI)
    dt -= 2 * M_PI;
  while (dt < -M_PI)
    dt += 2 * M_PI;
  return dt;
}

inline double theta(double x, double y) {
  return atan2(y, x);
}

inline double range(double x, double y, double z) {
  return sqrt(x * x + y * y + z * z);
}

inline double range(double x, double y) {
  return hypot(x, y);
}

inline double logistic(double x, double max_x, double max, double min) {
  return (max - (max - min) / (1.0 + exp(max_x / 2.0 - x)));
}

} // namespace perception

#endif // UTILS_H_
