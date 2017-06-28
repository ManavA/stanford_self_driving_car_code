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
#include <iostream>
#include <cmath>

#include <vlrException.h>
#include <keepLaneAndTrackPointSet2D.h>

namespace vlr {

//#define TRACE(x)  std::cout << " [KeepLaneAndTrackPointSet2D] " << x << std::endl;
#define TRACE(x)

KeepLaneAndTrackPointSet2D::KeepLaneAndTrackPointSet2D(const lanekeeping_params &par_kl,
		const tracking_params &par_track, const PolyTraj2D_params &par_2D):
		keep_lane_traj_set_(par_kl), track_point_traj_set_(par_track),
		par_(par_2D) {
}

int KeepLaneAndTrackPointSet2D::generate(const std::vector<CurvePoint>& center_line, double t,
		driving_common::TrajectoryPoint2D start_trajectory_point, GenerationMode generation_mode, double desired_position, double desired_velocity, double desired_accleration, bool no_check) {
    // get rid of last cicles's data
    keep_lane_traj_set_.clear();
    track_point_traj_set_.clear();
    set_data_.clear();
    movement_state start_state_long, start_state_lat;

    // We need to convert the start_trajectory point from global coordinates to local (Frenet Frame)
    // as the center line might have changed in the meantime
    try {
    	PolyTraj2D::globalCordinates2LatLong(start_trajectory_point, center_line, generation_mode,
    			start_state_lat, start_state_long);
    }
	catch(vlr::Ex<>& e) {
		throw VLRException( "Point tracking ->" + e.what());
	}

    // Generate new elementary sets
    // first we need the longitudinal set...
    try {
    	track_point_traj_set_.generate(t, start_state_long, desired_position, desired_velocity, desired_accleration, center_line, generation_mode);
    }
    catch(vlr::Ex<>& e) {
    	throw VLRException( "Point tracking -> " + e.what());
    	//std::cout << "Error: " << e.what() << std::endl;
    }

    // ...in order to combine it with the lateral set
    // therefore, we take the sample pattern of the longitudinal set to make sure that we can combine
    // the discrete points later
    // this depends on generation mode
    std::vector<std::vector<double> > sample_ref;
    if ( generation_mode == time_based ) {
    	// here we generate only one set of sampled trajectory points
    	sample_ref.resize(1);
    	std::vector<trajectory_point_1D>::const_iterator it_point;
    	for ( 	it_point  = track_point_traj_set_.set_data_.begin()->discrete_traj_points_.begin()->begin();
    			it_point != track_point_traj_set_.set_data_.begin()->discrete_traj_points_.begin()->end();
    			it_point++ ) {
    		sample_ref[0].push_back(it_point->arg); // these are the discrete points in time
    	}
    }
   	else { // arclength-based
   		// here we generate as many sets of sampled trajectory points as there are in the longitudinal trajectory set
   		// this is needed when lat and long trajectories are combined in the arclength-based generation mode
		sample_ref.resize(track_point_traj_set_.set_data_.size());
		std::vector<std::vector<double> >::iterator it_sample_ref;
		std::multiset<RootTraj,PolyTraj::CompPolyTraj>::const_iterator it_traj;
		for ( 	it_traj  = track_point_traj_set_.set_data_.begin(), it_sample_ref = sample_ref.begin();
				it_traj != track_point_traj_set_.set_data_.end();
				it_traj++, it_sample_ref++ ) {
			std::vector<trajectory_point_1D>::const_iterator it_point;
			for ( 	it_point  = it_traj->discrete_traj_points_[0].begin(); // keep in mind that longitudinal trajectory: discrete_traj_points_.size = 1
					it_point != it_traj->discrete_traj_points_[0].end();
					it_point++ ) {
				it_sample_ref->push_back(it_point->x); // this is s need for d = d(s)
			}
		}
	}

    // generate the lateral set
    try {
    keep_lane_traj_set_.generate(t, start_state_long.x, start_state_lat, generation_mode, track_point_traj_set_.par_.t_horizon, sample_ref);
    }
    catch(vlr::Ex<>& e) {
    	throw VLRException( "Point tracking -> " + e.what());
    }

	// Now combining the discrete longitudinal and lateral trajectory points
    std::multiset<PolyTraj,PolyTraj::CompPolyTraj>::iterator it_lat;
	std::multiset<RootTraj,PolyTraj::CompPolyTraj>::iterator it_root;

	int index_lon = 0; // debug info

	int traj_index;
    for (it_root = track_point_traj_set_.set_data_.begin(), traj_index = 0;
            it_root != track_point_traj_set_.set_data_.end();
            it_root++, traj_index++, index_lon++) {

    	int index_lat = 0; // debug info
        for (it_lat = keep_lane_traj_set_.set_data.begin();
                it_lat != keep_lane_traj_set_.set_data.end();
                it_lat++, index_lat++) {
        	std::vector<std::vector<trajectory_point_1D> >::const_iterator it_lat_sampled_trajectory;
        	if ( generation_mode == time_based ) {
				it_lat_sampled_trajectory = it_lat->discrete_traj_points_.begin(); // always take the first (and only) sampled point set
			}
			else { // generation_mode == arclength_based
				it_lat_sampled_trajectory = it_lat->discrete_traj_points_.begin();
				it_lat_sampled_trajectory = it_lat_sampled_trajectory + traj_index;
			}

                // generate 2D trajectory
            PolyTraj2D combined_2D_trajectory( center_line, *it_lat, *it_root, generation_mode, it_lat_sampled_trajectory, index_lat, index_lon); // make sure that no copy is fed in for correct pointers in the function!

            combined_2D_trajectory.calc_extreme_values(par_.time_delay);
            double max_curvature = combined_2D_trajectory.get_max_curvature();
            double min_velocity = combined_2D_trajectory.get_min_velocity();
            double max_offset = combined_2D_trajectory.get_max_dist_to_centerline();
            double max_angle = combined_2D_trajectory.get_max_angle_to_centerline();
            double max_lat_acc = combined_2D_trajectory.get_max_lat_acceleration();
            double max_lon_acc = combined_2D_trajectory.get_max_lon_acceleration();
            double min_lon_acc = combined_2D_trajectory.get_min_lon_acceleration();

            bool curvature_ok = ( max_curvature < par_.max_curvature );
            bool velocity_ok = ( min_velocity > -0.01 );
            bool offset_ok = ( max_offset < par_.max_center_line_offset);
            bool angle_ok = ( max_angle < par_.max_center_line_angular_offset );
            bool lat_acc_ok = ( max_lat_acc < par_.max_lat_acceleration );
            bool lon_acc_max_ok = ( max_lon_acc < par_.max_lon_acceleration );
            bool lon_acc_min_ok = ( min_lon_acc > par_.min_lon_acceleration );
            if ( no_check || (curvature_ok && velocity_ok && offset_ok && angle_ok &&  lat_acc_ok && lon_acc_min_ok && lon_acc_max_ok)) {
                // calculate combined cost

                double cost =  it_lat->get_total_cost() + it_root->get_total_cost(); // TODO
                combined_2D_trajectory.set_total_cost(cost);
                set_data_.insert(combined_2D_trajectory);    // insert new 2D-trajectory
            }
            //else {
                //printf(" maximal curvature exceeded! \n");
            //}
        }
    }
    if(no_check) {
      PolyTraj2D* trp=const_cast<PolyTraj2D*>(&(*set_data_.begin()));
      PolyTraj2D& tr=*trp;
      tr.calc_extreme_values(par_.time_delay);
      double max_curvature = tr.get_max_curvature();
      double min_velocity = tr.get_min_velocity();
      double max_offset = tr.get_max_dist_to_centerline();
      double max_angle = tr.get_max_angle_to_centerline();
      double max_lat_acc = tr.get_max_lat_acceleration();
      double min_lon_acc = tr.get_min_lon_acceleration();
      double max_lon_acc = tr.get_max_lon_acceleration();

      bool curvature_ok = ( max_curvature < par_.max_curvature );
      bool velocity_ok = ( min_velocity > -0.01 );
      bool offset_ok = ( max_offset < par_.max_center_line_offset);
      bool angle_ok = ( max_angle < par_.max_center_line_angular_offset );
      bool lat_acc_ok = ( max_lat_acc < par_.max_lat_acceleration );
      bool lon_acc_min_ok = ( min_lon_acc > par_.min_lon_acceleration );
      bool lon_acc_max_ok = ( max_lon_acc < par_.max_lon_acceleration );
      printf("Best broken trajectory:\n");
      printf("max_curvature (%i): %f\n", curvature_ok, max_curvature);
      printf("min_velocity (%i): %f\n", velocity_ok, min_velocity);
      printf("max_offset (%i): %f\n", offset_ok, max_offset);
      printf("max_angle (%i): %f\n", angle_ok, max_angle);
      printf("max_lat_acc (%i): %f\n", lat_acc_ok, max_lat_acc);
      printf("min_lon_acc (%i): %f\n", lon_acc_min_ok, min_lon_acc);
      printf("max_lon_acc (%i): %f\n", lon_acc_max_ok, max_lon_acc);

      std::vector<driving_common::TrajectoryPoint2D>::const_iterator trit=tr.trajectory2D_.begin();
      for(;trit!=tr.trajectory2D_.end(); trit++) {
        printf("v: %f, a: %f, theta: %f, kappa: %f\n", (*trit).v, (*trit).a, (*trit).theta, (*trit).kappa);
      }
    }

    if ( set_data_.size() == 0 ) {
    	throw VLRException ( "Point tracking: no trajectories in set as they all exceed curvature limits or would drive backwards! Either we are too far off or unrealistic max_curvature value." );
    }
	return 0;
}

void LeadingVehicle2Target::calc(const double& s_lv, const double& s_dot_lv, const double& s_ddot_lv,
		double& s_target, double& s_dot_target, double& s_ddot_target) {
	// converts the movement of the leading vehicle to the target point movement according to the constant time gap law
	// under the assumption that leading vehicle will keep initial acceleration, s. paper
	// _lv: leading vehicle
	// _target: moving target to track

	// Determine desired position with safety distance
	double d_ctg   = d0_ + tau_ * s_dot_lv;    // constant time gap law
	s_target       = s_lv - d_ctg;
	s_dot_target   = s_dot_lv - tau_ * s_ddot_lv;
	s_ddot_target  = s_ddot_lv;

	TRACE("s_target = "<< s_target << ", s_dot_target = " << s_dot_target << ", s_ddot_target = " << s_ddot_target);

	  // prohibit backing up that possibly could have been induced by sensor noise
	if ( s_dot_target < vel_thr_ ) {                  // if leading vehicle drives very slowly
		if ( s_target < last_stop_s_ + delta_s_thr_ ) { // if the target point has moved only little
			s_target = last_stop_s_; 	                    // keep old value
			s_dot_target 	= 0.;		                        // and assume it is stationary
			s_ddot_target = 0.;
		}
	}
	last_stop_s_ = s_target;
}

} // namespace vlr
