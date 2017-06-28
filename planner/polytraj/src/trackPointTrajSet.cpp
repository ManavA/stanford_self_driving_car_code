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


#include <stdio.h>
#include <float.h>
#include <cmath>
#include <set>

#include <vlrException.h>
#include <polyTraj.h>
#include <rootTraj.h>
#include <trackPointTrajSet.h>

namespace vlr {

TrackPointTrajSet::TrackPointTrajSet(tracking_params par) {
    par_ = par;
}

int TrackPointTrajSet::generate(double t_start, movement_state start_state, double desired_position,
    double desired_velocity, double desired_acceleration, const std::vector<CurvePoint>& center_line,
    GenerationMode /*mode*/) { // TODO: remove mode?!?

  // Generate vectors to vary time..
  int number_of_alt_times = (int) floor(par_.time_max / par_.time_res + .5);
  std::vector<double> t_alt(number_of_alt_times);
  t_alt[0] = ceil(t_start / par_.time_res) * par_.time_res; // next discrete point in time
  for (int i = 1; i < number_of_alt_times; i++) { // generate alternative points in time
    t_alt[i] = t_alt[i - 1] + par_.time_res;
  }

  // .. and offsets around s_d
  int number_of_alt_s_offsets = (int) ceil((par_.s_offset_max - par_.s_offset_min) / par_.s_res);
  std::vector<double> s_alt(number_of_alt_s_offsets);

  s_alt[0] = (int) ceil(par_.s_offset_min / par_.s_res) * par_.s_res + desired_position;

  for (int i = 1; i < number_of_alt_s_offsets; i++) {
    s_alt[i] = s_alt[i - 1] + par_.s_res;
  }

  // Generate set for all alternatives
  for (int i = 0; i < number_of_alt_s_offsets; i++) {
    for (int j = 0; j < number_of_alt_times; j++) {

      //printf("t_alt[j],s_alt[i] = %f, %f\n", t_alt[j], s_alt[i]);
      // Generate 5th order polynomial with different s_d and t_end and s_d_dot and s_d_ddot at the end
      RootTraj root_traj(center_line);
      double delta_t_alt = (t_alt[j] - t_start);

      // Predict for t_alt[j]
      double s_ddot_track_alt = desired_acceleration;
      double s_dot_track_alt = desired_velocity + desired_acceleration * delta_t_alt;
      double s_track_alt = s_alt[i] + desired_velocity * delta_t_alt + 0.5 * desired_acceleration * delta_t_alt
          * delta_t_alt;

//      printf("root_traj.generate_5th_order-input: start_state.x=%f start_state.x_der=%f start_state.x_dder=%f
//          s_track_alt=%f s_dot_track_alt=%f s_ddot_track_alt=%f t_start=%f delta_t_alt=%f par_.t_horizon=%f\n",
//          start_state.x, start_state.x_der, start_state.x_dder, s_track_alt,
//          s_dot_track_alt, s_ddot_track_alt, t_start, delta_t_alt, par_.t_horizon);
      int too_close = root_traj.generate_5th_order(start_state.x, start_state.x_der, start_state.x_dder, s_track_alt,
          s_dot_track_alt, s_ddot_track_alt, t_start, delta_t_alt, par_.t_horizon /*todo*/);
      if (!too_close) {
        double cost;
 //       printf("-1a- %s: root_traj.get_dder_min(): %f (>%f?), root_traj.get_dder_max(): %f (<%f?)\n", __FUNCTION__, root_traj.get_dder_min(), par_.a_min, root_traj.get_dder_max(), par_.a_max);
        // printf("root_traj.get_a_min() = %f, root_traj.get_a_max()=%f\n",root_traj.get_a_min(), root_traj.get_a_max());
        if ((root_traj.get_dder_min() > par_.a_min) && (root_traj.get_dder_max() < par_.a_max)) { // extrema a ok?

          root_traj.discrete_traj_points_.resize(1);
          root_traj.sampleArgumentEquidistant(t_start, par_.time_sample_res); // time equidistant

          // Calculate costs
          double arg_t = (t_alt[j] - t_start);
          double J_integral = root_traj.get_jerk_integral();
          double deviation_s = (s_alt[i] - desired_position);

          double cost_time = par_.k_t * arg_t;
          double cost_jerk = par_.k_j * J_integral;
          double cost_deviation = par_.k_s * (deviation_s * deviation_s);

          cost = cost_time + cost_jerk + cost_deviation;

          root_traj.set_debug_info(arg_t, J_integral, deviation_s, cost_time, cost_jerk, cost_deviation);

          root_traj.set_total_cost(cost);

          // Generate corresponding center line curvepoints
          try {
            root_traj.calculateRootTrajectory();
          }
          catch (vlr::Ex<>& e) {
            throw VLRException("Track point trajectory set -> " + e.what());
          }
          set_data_.insert(root_traj); // insert new trajectory
        }
      }
      else {
        printf("TOO CLOSE!\n");
      }
    }
  }
  if (set_data_.size() == 0) {
    throw VLRException("Longitudinal point tracking set empty!");
  }
  return 0;
}


void TrackPointTrajSet::clear() {
    set_data_.clear();
}

} // namespace vlr
