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


#ifndef CAR_LIST_H_
#define CAR_LIST_H_

#include <vector>
#include <driving_common/TurnSignal.h>
#include <obstacle_types.h>

namespace vlr {

struct car_pose_t {
  double x, y, theta;
  double timestamp;
  int t;
  driving_common::TurnSignal signal;
};

class car_t {
public:
  std::vector <car_pose_t> pose;
  double w, l;
  int fixed, start_cap, end_cap;

  car_pose_t *get_pose(int t);
  bool estimate_pose(int t, double *x, double *y, double *theta,
      bool *extrapolated);
  bool estimate_pose(double t, double *x, double *y, double *theta,
      bool *extrapolated);
  bool estimate_signal(int t, driving_common::TurnSignal* signal);
  bool estimate_signal(double t, driving_common::TurnSignal* signal);
  bool estimate_velocity(int t, double *vel, double dt);
  bool estimate_velocity(double t, double *vel);
  void delete_pose(int t);
  bool center_selected(double x, double y, int t);
  bool front_selected(double x, double y, int t);
  bool side_selected(double x, double y, int t);
  bool car_selected(double x, double y, int t);
  bool corner_selected(double x, double y, int t);
  int num_poses(void) { return (int)pose.size(); }
private:
};

class car_list_t {
public:
  std::vector <car_t> car;
  int num_cars(void) { return (int)car.size(); }
  void add_car(double x, double y, double theta, double w, double l, int t, double time);
  void rotate_car(float dtheta);
  void save_labels(char *filename);
  void save_labels_simple(char *filename, int num_spins);
  void load_labels(char *filename);
};

} // namespace vlr

#endif // CAR_LIST_H_
