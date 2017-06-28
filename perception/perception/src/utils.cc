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


#include <utils.h>
#include <perception.h>
#include <passat_constants.h>
#include <perception/StaticObstaclePoint.h>
#include <velodyne/ScanPoint.h>

using namespace dgc;
using namespace perception;

namespace drc = driving_common;

namespace perception {

#define AVG_OVER_N_VALUES     5

void dgc_transform_integrate_pose(dgc_transform_t t, dgc_pose_t pose) {
  dgc_transform_rotate_x(t, pose.roll);
  dgc_transform_rotate_y(t, pose.pitch);
  dgc_transform_rotate_z(t, pose.yaw);
  dgc_transform_translate(t, pose.x, pose.y, pose.z);
}

int inside_car(float x, float y, dgc_pose_t robot) {
  if (hypotf(robot.x - x, robot.y - y) < 3.0) {
    return (TRUE);
  }
  else {
    return (FALSE);
  }
}

//unsigned char cell_eval(PerceptionCell* cell) {
//  float h, seen;
//  unsigned char t;
//  if (cell->seen == 0) {
//    seen = 1;
//  }
//  else {
//    seen = cell->seen;
//  }
//  if ((cell->hits > settings_.map_cell_min_hits) || (cell->hits / seen > settings_.map_cell_threshold)) {
//    h = cell_height(cell);
//    if (h < 0.4) {
//      t = StaticObstaclePoint::LOW;
//    }
//    else if (h < 1.8) {
//      t = StaticObstaclePoint::HIGH;
//    }
//    else {
//      t = StaticObstaclePoint::UNKNOWN;
//    }
//  }
//  else {
//    t = StaticObstaclePoint::FREE;
//  }
//  return t;
//}
//
//int change_cell(PerceptionCell* cell, uint16_t counter) {
//  if (cell->last_mod == counter) return 0;
//  if (cell->obstacle != cell_eval(cell)) {
//    cell->last_mod = counter;
//    return 1;
//  }
//  return 0;
//}

uint16_t counter_diff(uint16_t last, uint16_t now) {
  if (now < last) {
    return (USHRT_MAX - (last - now));
  }
  return (now - last);
}

double beam_dist(velodyne::ScanPoint* pt1, velodyne::ScanPoint* pt2) {
  int dx = pt1->x - pt2->x;
  int dy = pt1->y - pt2->y;
  int dz = pt1->z - pt2->z;
  return (sqrt(dx * dx + dy * dy + dz * dz));
}

double pose_dist(applanix::ApplanixPose* p1, applanix::ApplanixPose* p2) {
  return (hypotf(p2->smooth_x - p1->smooth_x, p2->smooth_y - p1->smooth_y));
}

double sample(double b) {
  double sum = 0;
  int i;
  for (i = 0; i < 12; i++)
    sum += ((double) (rand()) / RAND_MAX) * 2 * b - b;
  return .5 * sum;
}

void display_time(const std::string label, double time) {
  return; // TODO: disabled for debugging...
  int ms = (int) ((drc::Time::current() - time) * 1000);
  if (label.length() < 9) {printf("#TIME: %s\t\t%02d", label.c_str(), ms);}
  else {printf("#TIME: %s\t%02d", label.c_str(), ms);}

  if (ms > 20) {printf(" * ");}

  printf("\n");
}

} // namespace vlr
