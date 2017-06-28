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


#include <box.h>
#include <global.h>
#include <transform.h>

#include <assert.h>

using namespace std;
using namespace dgc;

namespace perception {

void bounding_box(vector<point3d_t>* points, float angle,
                  double* x, double* y, double *yaw, double* width, double* length,
                  double min_car_width, double min_car_length)
{
  vector<point3d_t>::iterator it;
  double rot_min_x = DBL_MAX;
  double rot_min_y = DBL_MAX;
  double rot_max_x = -DBL_MAX;
  double rot_max_y = -DBL_MAX;

  assert(points->size());

  *yaw = vlr::normalizeAngle(angle);

  // expand the bounds to include all points
  dgc_transform_t t;
  dgc_transform_identity(t);
  dgc_transform_rotate_z(t, -angle);

  double rx, ry;
  double rz = 0.0;
  for (it = points->begin(); it != points->end(); it++) {
    point3d_t& point = *it;
    rx = point.x;
    ry = point.y;
    rz = 0.0;
    dgc_transform_point(&rx, &ry, &rz, t);
    if (rx < rot_min_x) rot_min_x = rx;
    if (rx > rot_max_x) rot_max_x = rx;
    if (ry < rot_min_y) rot_min_y = ry;
    if (ry > rot_max_y) rot_max_y = ry;
  }

  // fit car-sized bounding box
  if (rot_max_y < 0) {
    rot_min_y = min(rot_min_y, min_car_width);
  } else if (rot_min_y > 0) {
    rot_max_y = max(rot_max_y, rot_min_y + min_car_width);
  }

  if (rot_max_x < 0) {
    rot_min_x = min(rot_min_x, rot_max_x - min_car_length);
  } else if (rot_min_x > 0) {
    rot_max_x = max(rot_max_x, rot_min_x + min_car_length);
  }

  // find center of bounding box
  double rot_x = (rot_min_x + rot_max_x ) / 2;
  double rot_y = (rot_min_y + rot_max_y ) / 2;
  double rot_z = 0.0;

  // extent
  *width = max(rot_max_y - rot_min_y, min_car_width);
  *length = max(rot_max_x - rot_min_x, min_car_length);

  // un-rotate
  dgc_transform_identity(t);
  dgc_transform_rotate_z(t, angle);

  *x = rot_x;
  *y = rot_y;
  dgc_transform_point(x, y, &rot_z, t);
}

// fit a car-shaped bounding box to an obstacle, given the angle with which to align it
void bounding_box(Obstacle* obstacle, double angle, double min_car_width, double min_car_length)
{
  bounding_box(&obstacle->getPoints(), angle, &obstacle->pose.x, &obstacle->pose.y, &obstacle->pose.yaw, &obstacle->width, &obstacle->length, min_car_width, min_car_length);
//  printf("estimated %fx%f\n", obstacle->length, obstacle->width);
}

// will find the principal axis of a point cloud
// works best for rectilinear objects
float align_points(vector<point3d_t>& points) {
  static const int num_bins = 30;
  static const int max_samples = 200;
  static unsigned int count[num_bins];
  static double angle_total[num_bins];

  for (int i=0; i < num_bins; i++) {
    count[i] = 0;
    angle_total[i] = 0;
  }

  int num_points = points.size();
  int num_samples = 0;
  unsigned int max_count = 0;
  int max_index = 0;
  while (num_samples < max_samples) {
    point3d_t p1 = points[rand() % num_points];
    point3d_t p2 = points[rand() % num_points];
    double dy = (p1.y - p2.y);
    double dx = (p1.x - p2.x);
    if ((fabs(dy) < 0.01) && ( fabs(dx) < 0.01))
      continue;
    double y = atan2(dy, dx);

    // wrap into range
    if (y < 0) y += M_PI;
    if (y >= M_PI_2) y -= M_PI_2;

    int idx = (num_bins * y / M_PI_2);
    if (idx >= num_bins) {
      idx = 0;
      y = 0.0;
    }
    angle_total[idx] += y;
    count[idx]++;
    if (count[idx] > max_count) {
      max_count = count[idx];
      max_index = idx;
    }

    num_samples++;
  }

  return angle_total[max_index] / max_count;
 }

float align_obstacle(Obstacle* obstacle) {
  return align_points(obstacle->getPoints());
}

// determine the alignment (fit a car-shaped bounding box to an obstacle with PCA alignment
void align_bounding_box(Obstacle* obstacle, double min_car_width, double min_car_length)
{
  vector<point3d_t>& points = obstacle->getPoints();
  double yaw;

  vector<point3d_t>::iterator it;
  double mean_x = 0, mean_y = 0, mean_z = 0;
  float xc, yc;
  float a = 0 , b = 0 , d = 0;

  int count = points.size();
  assert(count);

  for (it = points.begin(); it != points.end(); it++)
  {
    mean_x += it->x;
    mean_y += it->y;
    mean_z += it->z;
  }
  mean_x /= count;
  mean_y /= count;
  mean_z /= count;

  for (it = points.begin(); it != points.end(); it++)
  {
    xc = it->x - mean_x;
    yc = it->y - mean_y;

    a += xc*xc;
    b += xc*yc;
    d += yc*yc;
  }

  a /= count-1;
  b /= count-1;
  d /= count-1;

  float D=a*d-b*b;
  float T=a+d;

  float tmp=sqrt(T*T/4-D);

  float L1=T/2.+ tmp;
//  float L2=T/2.- tmp;

  yaw=-atan2(L1-d,b);

  bounding_box(obstacle, yaw, min_car_width, min_car_length);
}

} // namespace perception
