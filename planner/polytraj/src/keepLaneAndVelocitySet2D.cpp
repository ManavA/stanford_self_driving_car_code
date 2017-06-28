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


#include <float.h>
#include <stdio.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include <vlrException.h>
#include <keepLaneAndVelocitySet2D.h>

namespace vlr {

KeepLaneAndVelocitySet2D::KeepLaneAndVelocitySet2D(const lanekeeping_params &par_kl, const velocity_params &par_vel,
    const PolyTraj2D_params &par_2D) :
  keep_lane_traj_set_(par_kl), keep_velocity_traj_set_(par_vel), par_(par_2D) {
}

int KeepLaneAndVelocitySet2D::generate(const std::vector<CurvePoint>& center_line, double t,
    driving_common::TrajectoryPoint2D start_trajectory_point, GenerationMode generation_mode, double desired_velocity) {
  // get rid of last cicles's data
  keep_lane_traj_set_.clear();
  keep_velocity_traj_set_.clear();
  set_data_.clear();
  movement_state start_state_long, start_state_lat;

  // We need to convert the start_trajectory point from global coordinates to local (Frenet Frame)
  // as the center line might have changed in the meantime
  try {
    PolyTraj2D::globalCordinates2LatLong(start_trajectory_point, center_line, generation_mode, start_state_lat,
        start_state_long);
  }
  catch (vlr::Ex<>& e) {
    throw VLRException("Velocity keeping -> " + e.what());
  }

  // Generate new elementary sets
  // first we need the longitudinal set...
  try {
    keep_velocity_traj_set_.generate(t, start_state_long, desired_velocity, center_line, generation_mode);
  }
  catch (vlr::Ex<>& e) {
    throw VLRException("Velocity keeping -> " + e.what());
  }

  // ...in order to combine it with the lateral set
  // therefore, we take the sample pattern of the longitudinal set to make sure that we can combine
  // the discrete points later
  // this depends on generation mode
  std::vector<std::vector<double> > sample_ref;
  if (generation_mode == time_based) {
    // here we generate only one set of sampled trajectory points
    sample_ref.resize(1);
    std::vector<trajectory_point_1D>::const_iterator it_point;
    for (it_point = keep_velocity_traj_set_.set_data_.begin()->discrete_traj_points_.begin()->begin(); it_point
        != keep_velocity_traj_set_.set_data_.begin()->discrete_traj_points_.begin()->end(); it_point++) {
      sample_ref[0].push_back(it_point->arg); // these are the discrete points in time
    }
  }
  else { // arclength-based
    // here we generate as many sets of sampled trajectory points as there are in the longitudinal trajectory set
    // this is needed when lat and long trajectories are combined in the arclength-based generation mode
    sample_ref.resize(keep_velocity_traj_set_.set_data_.size());
    std::vector<std::vector<double> >::iterator it_sample_ref;
    std::multiset<RootTraj, PolyTraj::CompPolyTraj>::const_iterator it_traj;
    for (it_traj = keep_velocity_traj_set_.set_data_.begin(), it_sample_ref = sample_ref.begin(); it_traj
        != keep_velocity_traj_set_.set_data_.end(); it_traj++, it_sample_ref++) {
      std::vector<trajectory_point_1D>::const_iterator it_point;
      for (it_point = it_traj->discrete_traj_points_[0].begin(); // keep in mind that longitudinal trajectory: discrete_traj_points_.size = 1
      it_point != it_traj->discrete_traj_points_[0].end(); it_point++) {
        it_sample_ref->push_back(it_point->x); // this is s need for d = d(s)
      }
    }
  }

  // generate the lateral set
  try {
    keep_lane_traj_set_.generate(t, start_state_long.x, start_state_lat, generation_mode,
        keep_velocity_traj_set_.par_.t_horizon, sample_ref);
  }
  catch (vlr::Ex<>& e) {
    throw VLRException("Velocity keeping -> " + e.what());
  }

  // Now combining the discrete longitudinal and lateral trajectory points
  std::multiset<PolyTraj, PolyTraj::CompPolyTraj>::iterator it_lat;
  std::multiset<RootTraj, PolyTraj::CompPolyTraj>::iterator it_root;

  int index_lon = 0; // debug info

  int traj_index;
  for (it_root = keep_velocity_traj_set_.set_data_.begin(), traj_index = 0; it_root
      != keep_velocity_traj_set_.set_data_.end(); it_root++, traj_index++, index_lon++) {

    int index_lat = 0; // debug info
    for (it_lat = keep_lane_traj_set_.set_data.begin(); it_lat != keep_lane_traj_set_.set_data.end(); it_lat++, index_lat++) {
      std::vector<std::vector<trajectory_point_1D> >::const_iterator it_lat_sampled_trajectory;
      if (generation_mode == time_based) {
        it_lat_sampled_trajectory = it_lat->discrete_traj_points_.begin(); // always take the first (and only) sampled point set
      }
      else { // generation_mode == arclength_based
        it_lat_sampled_trajectory = it_lat->discrete_traj_points_.begin();
        it_lat_sampled_trajectory = it_lat_sampled_trajectory + traj_index;
      }

      // generate 2D trajectory
      PolyTraj2D combined_2D_trajectory(center_line, *it_lat, *it_root, generation_mode, it_lat_sampled_trajectory,
          index_lat, index_lon); // make sure that no copy is fed in for correct pointers in the function!

      combined_2D_trajectory.calc_extreme_values(par_.time_delay);
      double max_curvature = combined_2D_trajectory.get_max_curvature();
      double min_velocity = combined_2D_trajectory.get_min_velocity();
      double max_offset = combined_2D_trajectory.get_max_dist_to_centerline();
      double max_angle = combined_2D_trajectory.get_max_angle_to_centerline();
      double max_lat_acc = combined_2D_trajectory.get_max_lat_acceleration();
      double max_lon_acc = combined_2D_trajectory.get_max_lon_acceleration();
      double min_lon_acc = combined_2D_trajectory.get_min_lon_acceleration();

      bool curvature_ok = (max_curvature < par_.max_curvature);
      bool velocity_ok = (min_velocity > -0.01);
      bool offset_ok = (max_offset < par_.max_center_line_offset);
      bool angle_ok = (max_angle < par_.max_center_line_angular_offset);
      bool lat_acc_ok = (max_lat_acc < par_.max_lat_acceleration);
      bool lon_acc_max_ok = (max_lon_acc < par_.max_lon_acceleration);
      bool lon_acc_min_ok = (min_lon_acc > par_.min_lon_acceleration);
      if (curvature_ok && velocity_ok && offset_ok && angle_ok && lat_acc_ok && lon_acc_min_ok && lon_acc_max_ok) {
        // calculate combined cost
        double cost = it_lat->get_total_cost() + it_root->get_total_cost(); // TODO
        combined_2D_trajectory.set_total_cost(cost);
        set_data_.insert(combined_2D_trajectory); // insert new 2D-trajectory
      }
    }
  }
  if (set_data_.size() == 0) {
    throw VLRException(
        "Velocity keeping: no trajectories in set as they all exceed curvature limits or would drive backwards!! Either we are too far off or unrealistic max_curvature value.");
  }
  return 0;
}

} // namespace vlr
