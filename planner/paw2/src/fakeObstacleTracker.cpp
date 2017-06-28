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


#include <cmath>
#include <vector>
#include <iostream>

#include <obstacle_types.h>
#include <obstaclePrediction.h>
#include <fakeObstacleTracker.h>

namespace vlr {

FakeObstacleTracker::FakeObstacleTracker(std::vector<CircleDemoCarState>& init_states,
		double t_horizon, double deltaT_sampling) :
		car_states_(init_states), t_horizon_(t_horizon), deltaT_sampling_(deltaT_sampling) {
}

FakeObstacleTracker::~FakeObstacleTracker() {
}


void FakeObstacleTracker::predictObstacles(double t, double offset_x, double offset_y, std::vector<ObstaclePrediction>& obstacle_predictions,
                                           perception::PerceptionObstacles& obstacle_msg) {
  obstacle_predictions.clear();
  std::vector<CircleDemoCarState>::iterator it;
  uint32_t id;
  double degree90 = 0.5*M_PI;
  static bool init = true;
  static double last_t=0;
  double delta_t=0;
  if(init) {
    init=false;
    delta_t=0;
  }
  else {
    delta_t = t - last_t;
  }

  last_t = t;

  for (it = car_states_.begin(), id=0; it != car_states_.end(); it++, id++ ) {
    ObstaclePrediction obst_pred;
    double& v     = it->v;
    double& r     = it->r;
    double x_r   = it->x - offset_x;
    double y_r   = it->y - offset_y;

    double phi_dot = v / r;

    it->phi += phi_dot*delta_t;

    double& phi_0 = it->phi;

    for (double t_temp = 0; t_temp <= t_horizon_; t_temp += deltaT_sampling_) {
      MovingBox traj_point;
      traj_point.t  = t+t_temp;
      double phi = phi_0 + phi_dot * (t_temp);
      traj_point.x  = r * cos(phi) + x_r;
      traj_point.y  = r * sin(phi) + y_r;
      traj_point.psi  = phi +  degree90;
      traj_point.length = it->length;
      traj_point.width = it->width;
      traj_point.ref_offset = it->ref_offset;

      obst_pred.predicted_traj_.push_back(traj_point);
    }
    obstacle_predictions.push_back(obst_pred);
    obstacle_msg.dynamic_obstacle[id].id = id+1;  // renumber from 1
    obstacle_msg.dynamic_obstacle[id].type = OBSTACLE_CAR;
    obstacle_msg.dynamic_obstacle[id].confidence = 255;
    obstacle_msg.dynamic_obstacle[id].x = obst_pred.predicted_traj_[0].x;
    obstacle_msg.dynamic_obstacle[id].y = obst_pred.predicted_traj_[0].y;
    obstacle_msg.dynamic_obstacle[id].direction = obst_pred.predicted_traj_[0].psi;

    obstacle_msg.dynamic_obstacle[id].width = obst_pred.predicted_traj_[0].width;
    obstacle_msg.dynamic_obstacle[id].length = obst_pred.predicted_traj_[0].length;
    obstacle_msg.dynamic_obstacle[id].velocity = v;
    obstacle_msg.dynamic_obstacle[id].x_var = 0;
    obstacle_msg.dynamic_obstacle[id].y_var = 0;
    obstacle_msg.dynamic_obstacle[id].xy_cov = 0;
    obstacle_msg.timestamp = t;
  }
}

} // namespace vlr
