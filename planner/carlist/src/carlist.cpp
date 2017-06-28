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


#include <global.h>
#include <carlist.h>

using std::vector;

namespace vlr {

void car_list_t::save_labels(char *filename)
{
  FILE *fp;
  int i, j;

  fp = fopen(filename, "w");
  if(fp == NULL)
    dgc::dgc_die("Error: could not open file %s for writing.\n", filename);

  fprintf(fp, "%d\n", num_cars());
  for(i = 0; i < num_cars(); i++) {
    fprintf(fp, "%f %f %d %d %d %d ", car[i].w, car[i].l, car[i].fixed,
        car[i].start_cap, car[i].end_cap, car[i].num_poses());
    for(j = 0; j < car[i].num_poses(); j++)
      fprintf(fp, "1 %f %f %f %d %d %lf ", car[i].pose[j].x, car[i].pose[j].y,
          car[i].pose[j].theta, (int)car[i].pose[j].signal.signal, car[i].pose[j].t, car[i].pose[j].timestamp);
    fprintf(fp, "\n");
  }
  fclose(fp);
}

void car_list_t::save_labels_simple(char *filename, int num_spins)
{
  FILE *fp;
  int i, j, count = 0;
  double x, y, theta;
  bool extrapolated;

  fp = fopen(filename, "w");
  if(fp == NULL)
    dgc::dgc_die("Error: could not open file %s for writing.\n", filename);

  for(i = 0; i < num_spins; i++) {
    fprintf(fp, "%d ", i);
    count = 0;
    for(j = 0; j < num_cars(); j++)
      if(car[j].estimate_pose(i, &x, &y, &theta, &extrapolated))
        count++;
    fprintf(fp, "%d ", count);
    for(j = 0; j < num_cars(); j++)
      if(car[j].estimate_pose(i, &x, &y, &theta, &extrapolated)) {
        fprintf(fp, "%d %d %f %f %f %f %f ", j, 1, x, y, theta,
            car[j].w, car[j].l);
      }
    fprintf(fp, "\n");
  }

  fclose(fp);
}

void car_list_t::load_labels(char *filename)
{
  FILE *fp;
  int i, j, num;

  fp = fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Warning: could not open file %s for reading.\n", filename);
    return;
  }
  fscanf(fp, "%d\n", &num);
  car.resize(num);
  for(i = 0; i < num_cars(); i++) {
    fscanf(fp, "%lf %lf %d %d %d %d ", &car[i].w, &car[i].l, &car[i].fixed,
        &car[i].start_cap, &car[i].end_cap, &num);
    car[i].pose.resize(num);
    for(j = 0; j < car[i].num_poses(); j++) {
      fscanf(fp, "%*d %lf %lf %lf %d %d %lf ", &car[i].pose[j].x, &car[i].pose[j].y,
          &car[i].pose[j].theta, (int*)&car[i].pose[j].signal.signal, &car[i].pose[j].t, &car[i].pose[j].timestamp);
    }
  }
  fclose(fp);
}

void car_list_t::add_car(double x, double y, double theta, double w, double l,
    int t, double time)
{
  car_pose_t p;
  int n;

  car.resize(num_cars() + 1);
  n = num_cars() - 1;
  car[n].w = w;
  car[n].l = l;
  car[n].start_cap = 1;
  car[n].end_cap = 0;
  car[n].fixed = 0;
  p.x = x;
  p.y = y;
  p.theta = theta;
  p.t = t;
  p.signal.signal = driving_common::TurnSignal::UNKNOWN;
  p.timestamp = time;
  car[n].pose.push_back(p);
}

car_pose_t *car_t::get_pose(int t)
{
  car_pose_t p;
  int i, n;

  p.t = t;

  if(num_poses() == 0) {
    pose.push_back(p);
    return &(pose[num_poses() - 1]);
  }
  else if(fixed) {
    return &(pose[0]);
  }
  else if(t > pose[num_poses() - 1].t) {
    pose.push_back(p);
    n = num_poses() - 1;
    pose[n].x = pose[n - 1].x;
    pose[n].y = pose[n - 1].y;
    pose[n].theta = pose[n - 1].theta;
    pose[n].signal = pose[n - 1].signal;
    return &(pose[num_poses() - 1]);
  }
  else if(t < pose[0].t) {
    pose.insert(pose.begin(), p);
    pose[0].x = pose[1].x;
    pose[0].y = pose[1].y;
    pose[0].theta = pose[1].theta;
    pose[0].signal = pose[1].signal;
    return &(pose[0]);
  }
  for(i = 0; i < num_poses(); i++)
    if(t == pose[i].t)
      return &(pose[i]);
    else if(i < num_poses() - 1 && t > pose[i].t && t < pose[i + 1].t) {
      pose.insert(pose.begin() + i + 1, p);
      pose[i + 1].x = pose[i + 2].x;
      pose[i + 1].y = pose[i + 2].y;
      pose[i + 1].theta = pose[i + 2].theta;
      pose[i + 1].signal = pose[i + 2].signal;
      return &(pose[i + 1]);
    }
  return NULL;
}

//car_pose_t *car_t::get_pose(double time)
//{
//  car_pose_t p;
//  int i, n;
//
//  p.timestamp = time;
//
//  if(num_poses() == 0) {
//    pose.push_back(p);
//    return &(pose[num_poses() - 1]);
//  }
//  else if(fixed) {
//    return &(pose[0]);
//  }
//  else if(time > pose[num_poses() - 1].timestamp) {
//    pose.push_back(p);
//    n = num_poses() - 1;
//    pose[n].x = pose[n - 1].x;
//    pose[n].y = pose[n - 1].y;
//    pose[n].theta = pose[n - 1].theta;
//    return &(pose[num_poses() - 1]);
//  }
//  else if(time < pose[0].timestamp) {
//    pose.insert(pose.begin(), p);
//    pose[0].x = pose[1].x;
//    pose[0].y = pose[1].y;
//    pose[0].theta = pose[1].theta;
//    return &(pose[0]);
//  }
//  for(i = 0; i < num_poses(); i++)
//    if(time == pose[i].timestamp)
//      return &(pose[i]);
//    else if(i < num_poses() - 1 && time > pose[i].timestamp && time < pose[i + 1].timestamp) {
//      pose.insert(pose.begin() + i + 1, p);
//      pose[i + 1].x = pose[i + 2].x;
//      pose[i + 1].y = pose[i + 2].y;
//      pose[i + 1].theta = pose[i + 2].theta;
//      return &(pose[i + 1]);
//    }
//  return NULL;
//}

bool car_t::estimate_pose(double t, double *x, double *y, double *theta,
    bool *extrapolated)
{
  double u, dtheta;
  int i;
  if(num_poses() == 0) {
    return false;
}

  if(t < pose[0].timestamp) {
    if (start_cap) {
      return false;
    } else {
      *x = pose[0].x;
      *y = pose[0].y;
      *theta = pose[0].theta;
      *extrapolated = true;
      return true;
    }
  }

  if(t > pose[num_poses() - 1].timestamp) {
    if(end_cap) {
      return false;
    } else {
      *x = pose[num_poses() - 1].x;
      *y = pose[num_poses() - 1].y;
      *theta = pose[num_poses() - 1].theta;
      *extrapolated = true;
      return true;
    }
  }

  for(i = 0; i < num_poses(); i++) {
    if(t == pose[i].timestamp) {
      *x = pose[i].x;
      *y = pose[i].y;
      *theta = pose[i].theta;
      *extrapolated = false;
      return true;
    }
    else if(i < num_poses() - 1 &&
        t > pose[i].timestamp && t < pose[i + 1].timestamp) {
      u = (t - pose[i].timestamp) / (pose[i + 1].timestamp - pose[i].timestamp);
      *x = pose[i].x + u * (pose[i + 1].x - pose[i].x);
      *y = pose[i].y + u * (pose[i + 1].y - pose[i].y);
      dtheta = normalizeAngle(pose[i + 1].theta - pose[i].theta);
      *theta = pose[i].theta + u * dtheta;
      *extrapolated = true;
      return true;
    }
  }
  return false;
}

bool car_t::estimate_pose(int t, double *x, double *y, double *theta,
    bool *extrapolated)
{
  double u, dtheta;
  int i;

  if(num_poses() == 0) {
    return false;
}
  if(t < pose[0].t && !start_cap) {
    *x = pose[0].x;
    *y = pose[0].y;
    *theta = pose[0].theta;
    *extrapolated = true;
    return true;
  }
  if(t > pose[num_poses() - 1].t && !end_cap) {
    *x = pose[num_poses() - 1].x;
    *y = pose[num_poses() - 1].y;
    *theta = pose[num_poses() - 1].theta;
    *extrapolated = true;
    return true;
  }
  for(i = 0; i < num_poses(); i++) {
    if(t == pose[i].t) {
      *x = pose[i].x;
      *y = pose[i].y;
      *theta = pose[i].theta;
      *extrapolated = false;
      return true;
    }
    else if(i < num_poses() - 1 &&
        t > pose[i].t && t < pose[i + 1].t) {
      u = (t - pose[i].t) / (double)(pose[i + 1].t - pose[i].t);
      *x = pose[i].x + u * (pose[i + 1].x - pose[i].x);
      *y = pose[i].y + u * (pose[i + 1].y - pose[i].y);
      dtheta = normalizeAngle(pose[i + 1].theta - pose[i].theta);
      *theta = pose[i].theta + u * dtheta;
      *extrapolated = true;
      return true;
    }
  }
  return false;
}

bool car_t::estimate_signal(double t, driving_common::TurnSignal* signal)
{
  int i;

  if(num_poses() == 0)
    return false;

  if(t < pose[0].timestamp) {
    if (start_cap) {
      return false;
    } else {
      *signal = pose[0].signal;
      return true;
    }
  }

  if(t > pose[num_poses() - 1].timestamp) {
    if(end_cap) {
      return false;
    } else {
      *signal = pose[num_poses() - 1].signal;
      return true;
    }
  }

  for(i = 0; i < num_poses(); i++) {
    if(t == pose[i].timestamp) {
      *signal = pose[i].signal;
      return true;
    }
    else if(i < num_poses() - 1 &&
        t > pose[i].timestamp && t < pose[i + 1].timestamp) {
      *signal = pose[i].signal;
      return true;
    }
  }
  return false;
}

bool car_t::estimate_signal(int t, driving_common::TurnSignal* signal)
{
  int i;

  if(num_poses() == 0)
    return false;
  if(t < pose[0].t && !start_cap) {
    *signal = pose[0].signal;
    return true;
  }
  if(t > pose[num_poses() - 1].t && !end_cap) {
    *signal = pose[num_poses() - 1].signal;
    return true;
  }
  for(i = 0; i < num_poses(); i++) {
    if(t == pose[i].t) {
      *signal = pose[i].signal;
      return true;
    }
    else if(i < num_poses() - 1 &&
        t > pose[i].t && t < pose[i + 1].t) {
      *signal = pose[i].signal;
      return true;
    }
  }
  return false;
}

bool car_t::estimate_velocity(int t, double *vel, double dt)
{
  double x1, x2, y1, y2, theta;
  bool extrapolated;

  if (estimate_pose(t-1, &x1, &y1, &theta, &extrapolated) &&
      estimate_pose(t+1, &x2, &y2, &theta, &extrapolated)) {
    double dx = (x1 - x2);
    double dy = (y1 - y2);
    *vel = sqrt(dx*dx + dy*dy) / dt;
    return true;
  }
  return false;
}

bool car_t::estimate_velocity(double t, double *vel)
{
  double x1, x2, y1, y2, theta;
  bool extrapolated;

  if (estimate_pose(t-0.1, &x1, &y1, &theta, &extrapolated) &&
      estimate_pose(t+0.1, &x2, &y2, &theta, &extrapolated)) {
    double dx = (x1 - x2);
    double dy = (y1 - y2);
    *vel = sqrt(dx*dx + dy*dy) / 0.2;
    return true;
  }
  return false;
}

void car_t::delete_pose(int t)
{
  int i;

  for(i = 0; i < num_poses(); i++)
    if(pose[i].t == t) {
      pose.erase(pose.begin() + i);
      break;
    }
}

bool car_t::center_selected(double scene_x, double scene_y, int t)
{
  double x, y, theta;
  bool extrapolated;

  if(!estimate_pose(t, &x, &y, &theta, &extrapolated))
    return false;
  return (hypot(scene_x - x, scene_y - y) < 0.2);
}

bool car_t::side_selected(double scene_x, double scene_y, int t)
{
  double x, y, theta;
  double x2, y2;
  bool extrapolated;

  if(!estimate_pose(t, &x, &y, &theta, &extrapolated))
    return false;
  x2 = x + w / 2 * sin(theta);
  y2 = y - w / 2 * cos(theta);
  return (hypot(scene_x - x2, scene_y - y2) < 0.2);
}

bool car_t::front_selected(double scene_x, double scene_y, int t)
{
  double x, y, theta;
  bool extrapolated;
  double x2, y2;

  if(!estimate_pose(t, &x, &y, &theta, &extrapolated))
    return false;
  x2 = x + l / 2 * cos(theta);
  y2 = y + l / 2 * sin(theta);
  return (hypot(scene_x - x2, scene_y - y2) < 0.2);
}

bool car_t::corner_selected(double scene_x, double scene_y, int t)
{
  double x, y, theta;
  bool extrapolated;
  double x2, y2;

  if(!estimate_pose(t, &x, &y, &theta, &extrapolated))
    return false;
  x2 = x + l / 2 * cos(theta) + w / 2 * sin(theta);
  y2 = y + l / 2 * sin(theta) - w / 2 * cos(theta);
  return (hypot(scene_x - x2, scene_y - y2) < 0.2);
}

bool car_t::car_selected(double scene_x, double scene_y, int t)
{
  double x, y, theta;
  bool extrapolated;
  double ctheta, stheta;
  double x1, y1, x2, y2;
  double perp_dist, par_dist, x_match, y_match;

  if(!estimate_pose(t, &x, &y, &theta, &extrapolated))
    return false;

  /* compute center axis vector */
  ctheta = cos(theta);
  stheta = sin(theta);
  x1  = x - (w / 2.0) * stheta;
  y1  = y + (w / 2.0) * ctheta;
  x2  = x - (-w / 2.0) * stheta;
  y2  = y + (-w / 2.0) * ctheta;
  dgc::dgc_point_to_line_distance(scene_x, scene_y, x1, y1, x2, y2, &par_dist,
      &perp_dist, &x_match, &y_match);
  if(par_dist >= 0.0 && par_dist <= 1.0 && perp_dist <= l / 2.0)
    return true;
  return false;
}

} // namespace vlr
