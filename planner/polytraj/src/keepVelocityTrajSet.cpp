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
#include <vlrException.h>
#include <keepVelocityTrajSet.h>

#define LIN_VELOCITY_SAMPLING

namespace vlr {

KeepVelocityTrajSet::KeepVelocityTrajSet(velocity_params par) {
    par_ = par;
}

inline double KeepVelocityTrajSet::logLowerVelocityStep(double vel_ref, uint32_t step, double step_size) {
  if(vel_ref == 0.0) {return 0.0;}  // TODO: define Epsilon
  double rel_vel = (vel_ref - step * step_size) / vel_ref;
  return vel_ref / M_LN2 * log(1 + pow(rel_vel, .7));
}

int KeepVelocityTrajSet::generate(double t_start, movement_state start_state,
        double desired_velocity, const std::vector<CurvePoint> &center_line,
        GenerationMode /*mode*/) {  // TODO: remove mode?!?

    // check start state
    if(start_state.x_dder < par_.a_min || start_state.x_dder > par_.a_max) {
      throw VLRException("KeepVelocityTrajSet::generate: Insane input acceleration.");
    }

    int number_of_alt_times = (int)floor( par_.time_max / par_.time_res +.5);
 	  std::vector<double> t_alt (number_of_alt_times);
    t_alt[0] = ceil(t_start/par_.time_res)*par_.time_res;   // next discrete point in time
    for (int i=1; i<number_of_alt_times; i++) {     // generate alternative points in time
        t_alt[i] = t_alt[i-1] + par_.time_res;
    }

#ifdef LIN_VELOCITY_SAMPLING
    int number_of_alt_velocities = (int)floor( (par_.v_offset_max - par_.v_offset_min) / par_.v_res + .5);
    std::vector<double> v_alt (number_of_alt_velocities+1);

    int number_of_lower_alt_velocities = int(-par_.v_offset_min/(-par_.v_offset_min+par_.v_offset_max)*number_of_alt_velocities+.5);
    v_alt[number_of_lower_alt_velocities] = std::max(0.0, desired_velocity);
    for (int i=number_of_lower_alt_velocities-1; i>=0; i--) {
        v_alt[i] = std::max(0.0, v_alt[i+1] - par_.v_res);
    }
#else
    int number_of_alt_velocities = 10;
    std::vector<double> v_alt (number_of_alt_velocities+1);

    int number_of_lower_alt_velocities = 7;
    double lin_vel_step = desired_velocity/(number_of_lower_alt_velocities);

    for(int i=number_of_lower_alt_velocities; i>=0; i--) {  // TODO: set feasible value for minimum velocity
      v_alt[i]=logLowerVelocityStep(desired_velocity, i, lin_vel_step);
    }
#endif

    for (int i=number_of_lower_alt_velocities+1; i<number_of_alt_velocities; i++) {
        v_alt[i] = v_alt[i-1] + par_.v_res;
    }
    for (int i=0; i<number_of_alt_velocities; i++) {
        for (int j=0; j<number_of_alt_times; j++) {

                // Generate 4th order polynomial with different v_d and t_end and no acceleration at the end
            RootTraj root_traj(center_line);

            int too_close = root_traj.generate_4th_order( start_state.x, start_state.x_der, start_state.x_dder,
                    v_alt[i], 0.0, t_start, ( t_alt[j]-t_start ), par_.t_horizon /*todo*/);
            if ( !too_close ) { 
                double cost;
               // printf("root_traj.get_a_min() = %f, root_traj.get_a_max()=%f\n",root_traj.get_a_min(), root_traj.get_a_max());

            	if ( ( root_traj.get_dder_min() > par_.a_min ) && ( root_traj.get_dder_max() < par_.a_max ) ) {   // extrema a ok?

                	root_traj.discrete_traj_points_.resize(1);
                    root_traj.sampleArgumentEquidistant(t_start, par_.time_sample_res); // time euqidistant

                    // Calculate costs
                    double arg_t = ( t_alt[j] -t_start );
                    double J_integral = root_traj.get_jerk_integral();
                    double deviation_v = (v_alt[i]-desired_velocity);

                    double cost_time 		= par_.k_t 	* arg_t;
                    double cost_jerk 		= par_.k_j 	* J_integral;
                    double cost_deviation 	= par_.k_v 	* ( deviation_v*deviation_v );

                    cost = cost_time + cost_jerk + cost_deviation;

                    root_traj.set_debug_info(arg_t, J_integral, deviation_v, cost_time, cost_jerk, cost_deviation);

                    root_traj.set_total_cost(cost);

                    // Generate corresponding center line curvepoints
                    try {
                    	root_traj.calculateRootTrajectory();
                    }
                    catch(vlr::Ex<>& e) {
                        	throw VLRException( "Keep velocity trajectory set -> " + e.what());
                    }

                     set_data_.insert(root_traj);    // insert new trajectory
                }
            }
        }
    }
    if ( set_data_.size() == 0 ) {
        throw VLRException("Keep velocity set empty!");
    }
    return 0;
}

void KeepVelocityTrajSet::clear() {
    set_data_.clear();
}

} // namespace vlr
