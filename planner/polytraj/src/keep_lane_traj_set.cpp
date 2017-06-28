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
#include <float.h>

#include <vlrException.h>
#include <keep_lane_traj_set.h>

namespace vlr {

keep_lane_traj_set::keep_lane_traj_set(lanekeeping_params params) {
    par = params;
}

int keep_lane_traj_set::generate(double t_start, double s_start, movement_state start_state,
        GenerationMode mode, double sampling_horizon/*todo: get rid*/,
        const std::vector<std::vector<double> >& sample_ref) {

    double arg_max;
    double arg_res;
    double dder_min;
    double dder_max;
    double weight_arg;
    double weight_j;
    double weight_d;
    
    std::vector<double> arg_alt;
    int number_of_alt_args;


    if ( mode == time_based ) {
        arg_max        = par.holon.time_max;
        arg_res        = par.holon.time_res;
        dder_min       = par.holon.d_ddot_min;
        dder_max       = par.holon.d_ddot_max;
        weight_arg     = par.holon.k_t;
        weight_j       = par.holon.k_j;
        weight_d       = par.holon.k_d;

        number_of_alt_args = (int)floor( arg_max / arg_res +.5 );
    	arg_alt.resize(number_of_alt_args);

    	double t0 = ceil(t_start/arg_res)*arg_res;
    	arg_alt[0] =  t0;   // next discrete point in time
        for (int i=1; i<number_of_alt_args; i++) {     // generate alternative points in time
            arg_alt[i] = arg_alt[i-1] + arg_res;
        }
    }
    else {// ( mode == arclength_based ) 
        arg_max        = par.nonhol.s_max;
        arg_res        = par.nonhol.s_res;
        dder_min       = par.nonhol.d_pprime_min;
        dder_max       = par.nonhol.d_pprime_max;
        weight_arg     = par.nonhol.k_s;
        weight_j       = par.nonhol.k_j;
        weight_d       = par.nonhol.k_d;
    
        number_of_alt_args = (int)floor( arg_max / arg_res +.5 );
    	arg_alt.resize(number_of_alt_args);
    	double s0 = ceil(s_start/arg_res)*arg_res;
    	arg_alt[0] = s0;   // next discrete point in s (arclength)
    for (int i=1; i<number_of_alt_args; i++) {     // generate alternative points in time
        arg_alt[i] = arg_alt[i-1] + arg_res;
    }
    }
    
//    for (int i=0; i<number_of_alt_args; i++) {
//        mexPrintf("arg_alt[%d] = %f\n", i, arg_alt[i]);
//    }

    int number_of_alt_d_offsets = (int)ceil( (par.d_offset_max - par.d_offset_min) / par.d_res);
	std::vector<double> d_alt(number_of_alt_d_offsets);

    d_alt[0] = (int) ceil( par.d_offset_min / par.d_res) * par.d_res;

    for (int i=1; i<number_of_alt_d_offsets; i++) {
        d_alt[i] = d_alt[i-1] + par.d_res;
    }
    //printf("start_state.x_dot = %f\n", start_state.x_dot);
//for (int i = 0; i<number_of_alt_d_offsets; i++)  printf("d_alt[%d] = %f\n", i, d_alt[i]);

    //v_i_2 = v_next:-par.v_res:(max( (v_next+par.v_offset_min) ,0));    // but not smaller than v = 0

    for (int i=0; i<number_of_alt_d_offsets; i++) {
        for (int j=0; j<number_of_alt_args; j++) {
            // Generate 5th order polynomial with d and t_end and no acceleration and velocity at the end
            PolyTraj traj;
                
     //printf("time_sample_res= %f\n",par.time_sample_res);        
            double arg_start = mode==time_based? t_start : s_start;
            int too_close = traj.generate_5th_order( start_state.x, start_state.x_der, start_state.x_dder,
                    d_alt[i], 0.0, 0.0, arg_start, ( arg_alt[j]-arg_start ), sampling_horizon);
            if ( !too_close  ) {
                double cost;
                if ( ( traj.get_dder_min() > dder_min ) && ( traj.get_dder_max() < dder_max ) ) {   // extrema a ok?

                	// Sample for collision check
                	traj.sampleArgumentSynchronized(sample_ref);

                    // Calculate costs
                    double arg = mode==time_based? ( arg_alt[j] -t_start ) : ( arg_alt[j] -s_start );
                    double J_integral = traj.get_jerk_integral();
                    double deviation = d_alt[i];
                    //movement_state end_state = traj.get_end_state();

                    double cost_arg 		= weight_arg 	* arg;
                    double cost_jerk 		= weight_j 		* J_integral;
                    double cost_deviation 	= weight_d 		* ( deviation*deviation );

                    cost = cost_arg + cost_jerk + cost_deviation;

//                      if (d_alt[i]*d_alt[i] < 0.01 ) {
//                    	printf("--------------------------------------------------J_integral = %f\n", J_integral);
//                    }
                    
                    traj.set_debug_info(arg, J_integral, deviation, cost_arg, cost_jerk, cost_deviation);
                    traj.set_total_cost(cost);
                    set_data.insert(traj);    // insert new trajectory
                }
            }
        }
    }
    if ( set_data.size() == 0 ) {
        throw VLRException("Lane keeping set empty!");
    }
    return 0;
}

void keep_lane_traj_set::clear() {
    set_data.clear();
}

} // namespace vlr

