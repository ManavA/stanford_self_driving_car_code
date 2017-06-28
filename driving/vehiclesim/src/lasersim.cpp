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


#include <vector>
#include <algorithm>
#include <global.h>
#include <passat_constants.h>
#include <vehicle.h>
#include <lasersim.h>

using namespace dgc;
using std::vector;

namespace vlr {
typedef struct {
  char spanning;
  float min_range, angle1, angle2;
  double x1, y1, x2, y2;
} segment_t, *segment_p;

typedef vector <segment_t> seg_list;

seg_list segment_list, car_segment_list;

inline int segment_segment_intersection(double x1, double y1,
					double x2, double y2, 
					double x3, double y3, 
					double x4, double y4,
					double *xc, double *yc)
{
  double denom, ua, ub;

  denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
  if(denom == 0)
    return 0;
  ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
  ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;
  if(ua >= -0.0 && ua <= 1.0 && ub >= -0.0 && ub <= 1.0) {
    *xc = x1 + ua * (x2 - x1);
    *yc = y1 + ua * (y2 - y1);
    return 1;
  }
  else
    return 0;
}

int compare_range(const void *a, const void *b)
{
  segment_p sega = (segment_p)a, segb = (segment_p)b;

  if(sega->min_range < segb->min_range)
    return -1;
  else if(sega->min_range > segb->min_range)
    return 1;
  else
    return 0;
}

void compute_angles(double laser_x, double laser_y)
{
  vector <segment_t>::iterator iter;
  double temp;

  /* compute angles */
  for(iter = segment_list.begin(); iter != segment_list.end(); iter++) {
    iter->angle1 = atan2(iter->y1 - laser_y, iter->x1 - laser_x);
    iter->angle2 = atan2(iter->y2 - laser_y, iter->x2 - laser_x);
    iter->min_range = 
      std::min(hypot(iter->y1 - laser_y, iter->x1 - laser_x), hypot(iter->y2 - laser_y, iter->x2 - laser_x));
    if(iter->angle1 > iter->angle2) {
      temp = iter->angle1;
      iter->angle1 = iter->angle2;
      iter->angle2 = temp;
    }
    iter->spanning = 0;
    if(iter->angle2 - iter->angle1 > M_PI)
      iter->spanning = 1;
  }
  qsort(&segment_list[0], (signed)segment_list.size(), sizeof(segment_t),
	compare_range);

  /* compute angles */
  for(iter = car_segment_list.begin(); iter != car_segment_list.end(); 
      iter++) {
    iter->angle1 = atan2(iter->y1 - laser_y, iter->x1 - laser_x);
    iter->angle2 = atan2(iter->y2 - laser_y, iter->x2 - laser_x);
    iter->min_range = 
      std::min(hypot(iter->y1 - laser_y, iter->x1 - laser_x), hypot(iter->y2 - laser_y, iter->x2 - laser_x));
    if(iter->angle1 > iter->angle2) {
      temp = iter->angle1;
      iter->angle1 = iter->angle2;
      iter->angle2 = temp;
    }
    iter->spanning = 0;
    if(iter->angle2 - iter->angle1 > M_PI)
      iter->spanning = 1;
  }
  qsort(&car_segment_list[0], (signed)car_segment_list.size(), 
	sizeof(segment_t), compare_range);
}

inline void find_range(double beam_x1, double beam_y1, 
		       double beam_x2, double beam_y2,
		       double theta, double cmirror, double smirror, 
		       seg_list *slist, double *min_r, double *min_x, 
		       double *min_y)
{
  seg_list::iterator iter;
  double xc, yc, r;

  for(iter = slist->begin(); iter != slist->end(); iter++) {
    if(iter->min_range > *min_r) 
      break;
    if(iter->spanning || (theta >= iter->angle1 && theta <= iter->angle2)) {
      if(segment_segment_intersection(beam_x1, beam_y1, beam_x2, beam_y2,
				      iter->x1, iter->y1, iter->x2, iter->y2, 
				      &xc, &yc)) {
	r = hypot(xc - beam_x1, yc - beam_y1);
	if(r < *min_r) {
	  *min_r = r;
	  *min_x = r * cmirror;
	  *min_y = r * smirror;
	}
      }
    }
  }
}

//void generate_ldlrs_laser_scan(LdlrsLaser *ldlrs,
//			       double laser_x, double laser_y,
//			       double laser_yaw, double start_angle,
//			       double laser_max_range)
//{
//  double beam_x1, beam_y1, beam_x2, beam_y2;
//  double min_r, min_x, min_y, theta;
//  int i;
//
//  compute_angles(laser_x, laser_y);
//
//  beam_x1 = laser_x;
//  beam_y1 = laser_y;
//  ldlrs->num_range = 270 * 4;
//  ldlrs->num_intensity = 0;
//  ldlrs->sector_start_ts = 0;
//  ldlrs->sector_end_ts = 0;
//  ldlrs->angular_resolution = dgc_d2r(0.25);
//
//  ldlrs->start_angle = start_angle;
//  ldlrs->end_angle = start_angle + dgc_d2r(270.0);
//
//  for(i = 0; i < 270 * 4; i++) {
//    theta = vlr::normalizeAngle(laser_yaw - ldlrs->start_angle -
//				i * ldlrs->angular_resolution);
//    beam_x2 = laser_x + laser_max_range * cos(theta);
//    beam_y2 = laser_y + laser_max_range * sin(theta);
//
//    min_r = laser_max_range;
//    find_range(beam_x1, beam_y1, beam_x2, beam_y2, theta, 0,
//	       0, &segment_list, &min_r, &min_x, &min_y);
//    find_range(beam_x1, beam_y1, beam_x2, beam_y2, theta, 0,
//	       0, &car_segment_list, &min_r, &min_x, &min_y);
//    ldlrs->range[i] = min_r;
//  }
//}

inline void add_interp_segs(double x1, double y1, double x2, double y2)
{
  double l, u, du;
  segment_t seg;
  int i, n;

  l = hypot(x2 - x1, y2 - y1);
  n = (int)ceil(l / 3.0) + 1;
  du = 1.0 / (double)n;
  for(i = 0; i < n; i++) {
    u = i * du;
    seg.x1 = x1 + u * (x2 - x1);
    seg.y1 = y1 + u * (y2 - y1);
    seg.x2 = x1 + (u + du) * (x2 - x1);
    seg.y2 = y1 + (u + du) * (y2 - y1);
    segment_list.push_back(seg);
  }
}

void load_obstacle_map(const std::string& filename)
{
  int i, n, n2;
  FILE *fp;
  double x1, y1, x2, y2, w, yaw;

  fp = fopen(filename.c_str(), "r");
  if(fp == NULL) {
    throw VLRException("Could not open file " + filename + " for reading.");
    return;
  }

  if(fscanf(fp, "%d\n", &n) != 1) {
    throw VLRException("Format error in " + filename);
    fclose(fp);
    return;
  }

  segment_list.clear();
  for(i = 0; i < n; i++) {
    if(fscanf(fp, "%lf %lf %lf %lf %lf\n", &x1, &y1, &x2, &y2, &w) != 5) {
      throw VLRException("Format error in " + filename);
      fclose(fp);
      return;
    }
    yaw = atan2(y2 - y1, x2 - x1);

    add_interp_segs(x1 + w / 2.0 * cos(yaw + M_PI / 2.0),
		    y1 + w / 2.0 * sin(yaw + M_PI / 2.0),
		    x1 - w / 2.0 * cos(yaw + M_PI / 2.0),
		    y1 - w / 2.0 * sin(yaw + M_PI / 2.0));

    add_interp_segs(x2 + w / 2.0 * cos(yaw + M_PI / 2.0),
		    y2 + w / 2.0 * sin(yaw + M_PI / 2.0),
		    x2 - w / 2.0 * cos(yaw + M_PI / 2.0),
		    y2 - w / 2.0 * sin(yaw + M_PI / 2.0));

    add_interp_segs(x1 + w / 2.0 * cos(yaw + M_PI / 2.0),
		    y1 + w / 2.0 * sin(yaw + M_PI / 2.0),
		    x2 + w / 2.0 * cos(yaw + M_PI / 2.0),
		    y2 + w / 2.0 * sin(yaw + M_PI / 2.0));

    add_interp_segs(x1 - w / 2.0 * cos(yaw + M_PI / 2.0),
		    y1 - w / 2.0 * sin(yaw + M_PI / 2.0),
		    x2 - w / 2.0 * cos(yaw + M_PI / 2.0),
		    y2 - w / 2.0 * sin(yaw + M_PI / 2.0));
  }

  if(fscanf(fp, "%d\n", &n2) != 1) {
    throw VLRException("Format error in " + filename);
    fclose(fp);
    return;
  }

  for(i = 0; i < n2; i++) {
    if(fscanf(fp, "%lf %lf %lf %lf\n", &x1, &y1, &x2, &y2) != 4) {
      throw VLRException("Format error in " + filename);
      fclose(fp);
      return;
    }

    add_interp_segs(x1, y1, x2, y2);
  }
  fclose(fp);

  fprintf(stderr, "Read %d segments\n", (int)segment_list.size());
}

void fill_car_segments(int num_vehicles, vehicle_state *vehicle)
{
  int i;
  double dx, dy, x1, y1, x2, y2;
  segment_t seg;

  car_segment_list.clear();

  for(i = 0; i < num_vehicles; i++) {
    dx = DGC_PASSAT_LENGTH / 2.0 * cos(vehicle[i].yaw);
    dy = DGC_PASSAT_LENGTH / 2.0 * sin(vehicle[i].yaw);

    x1 = vehicle[i].x + vehicle[i].origin_x - dx;
    y1 = vehicle[i].y + vehicle[i].origin_y - dy;
    x2 = vehicle[i].x + vehicle[i].origin_x + dx;
    y2 = vehicle[i].y + vehicle[i].origin_y + dy;
    
    dx = DGC_PASSAT_WIDTH / 2.0 * cos(vehicle[i].yaw + M_PI / 2.0);
    dy = DGC_PASSAT_WIDTH / 2.0 * sin(vehicle[i].yaw + M_PI / 2.0);

    seg.x1 = x1 + dx;
    seg.y1 = y1 + dy;
    seg.x2 = x1 - dx;
    seg.y2 = y1 - dy;
    car_segment_list.push_back(seg);

    seg.x1 = x2 + dx;
    seg.y1 = y2 + dy;
    seg.x2 = x2 - dx;
    seg.y2 = y2 - dy;
    car_segment_list.push_back(seg);

    seg.x1 = x1 + dx;
    seg.y1 = y1 + dy;
    seg.x2 = x2 + dx;
    seg.y2 = y2 + dy;
    car_segment_list.push_back(seg);

    seg.x1 = x1 - dx;
    seg.y1 = y1 - dy;
    seg.x2 = x2 - dx;
    seg.y2 = y2 - dy;
    car_segment_list.push_back(seg);
  }
}

} // namespace vlr
