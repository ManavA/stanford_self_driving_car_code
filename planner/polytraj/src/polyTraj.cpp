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

#include <polyTraj.h>
#include <horner.h>
//---------------- Calculate coefficients of 5th order polynomial ---------

namespace vlr {

int PolyTraj::generate_5th_order( double x0, double x0_dot, double x0_ddot, double x1,
        double x1_dot, double x1_ddot, double /*t_current*/, double deltaT,       // TODO: remove t_current?!?
        double horizon) {

  if ( deltaT<0.0001 ) {return -1;}

  double& t = deltaT; // calculation in relative time
  double t2 = t*t;
  double t3 = t*t2;
  double t4 = t2*t2;
  double t5 = t*t4;
  coeff_[0] = x0;
  coeff_[1] = x0_dot;
  coeff_[2] = x0_ddot / 2.0;
  coeff_[3] = (x1_ddot*t2 - 3.0*x0_ddot*t2 - 8.0*x1_dot*t - 12.0*x0_dot*t + 20.0*x1 - 20.0*x0)/(2.0*t3);
  coeff_[4] = (-2.0*x1_ddot*t*t + 3.0*x0_ddot*t2 + 14.0*x1_dot*t + 16.0*x0_dot*t - 30.0*x1 + 30.0*x0)/(2.0*t4);
  coeff_[5] = (x1_ddot*t2 - x0_ddot*t2 - 6.0*x1_dot*t - 6.0*x0_dot*t + 12.0*x1 - 12.0*x0)/(2.0*t5);
  te = t;
  calc_min_max_acceleration_of_1D_traj();
  calc_jerk_integral();
  calc_end_state();
//      printf("te = %f",te);
  horizon_ = horizon;

  x0_   = x0;
  return 0;
}

int PolyTraj::generate_4th_order( double x0, double x0_dot, double x0_ddot,
        double x1_dot, double x1_ddot, double /*t_current*/, double deltaT,   // TODO: remove t_current?!?
        double horizon) {

  if ( deltaT<0.0001 ) {return -1;}

  //printf("x0, x0_dot, x0_ddot, x1_dot, x1_ddot, t, t_hor = %f, %f, %f, %f, %f, %f, %f\n", x0, x0_dot, x0_ddot, x1_dot, x1_ddot, t, t_hor);
  double& t = deltaT; // calculation in relative time
  double t2 = t*t;
  double t3 = t*t2;
  coeff_[0] = x0;
  coeff_[1] = x0_dot;
  coeff_[2] = x0_ddot / 2.0;
  coeff_[3] = -(coeff_[1] - x1_dot + 2.0*coeff_[2]*t)/t2 - (x1_ddot - 2.0*coeff_[2])/(3.0*t);
  coeff_[4] = (coeff_[1] - x1_dot + 2.0*coeff_[2]*t)/(2.0*t3) + (x1_ddot - 2.0*coeff_[2])/(4.0*t2);
  coeff_[5] = 0.0;

  //for(int i=0; i<6; i++)  printf("coeff[%d] = %f \n",i,coeff[i]);

  // calculate other stuff
  te = t;
  calc_min_max_acceleration_of_1D_traj();
  calc_jerk_integral();
  calc_end_state();
  horizon_ = horizon;
  x0_ = x0;
  return 0;
}

int PolyTraj::calc_end_state() {
// Memorize end state for faster extrapolation
//    printf("te = %f",te);
    std::vector<double> eval(6);

    horner(coeff_, te, eval);
    
    end_state.x         = eval[0];
    end_state.x_der     = eval[1];
    end_state.x_dder    = eval[2];
    return 0;
}

int PolyTraj::calc_min_max_acceleration_of_1D_traj() {
    double a0, ae, ag;
    double tg, t_min, t_max;
    std::vector<double> eval(6);

        // calculate limit values

    //a0 = eval_poly(2, 0.0);
    //ae = eval_poly(2, te);

    horner(coeff_, 0.0, eval);
    a0 = eval[2];
    horner(coeff_, te, eval);
    ae = eval[2];
    

    if ( a0 < ae ) {  // compare limit values
        a_min = a0;
        a_max = ae;
        t_min = 0;
        t_max = te;
    }
    else {
        a_min = ae;
        a_max = a0;
        t_min = te;
        t_max = 0;
    }

    // Check for extrema between limits
    if ( coeff_[5] == 0 ) {
        if ( coeff_[4] == 0 ) {  // polynomial 3rd order -> no global extremum
            ;// -> do nothing, use limit values
        }
        else {   // polynomial 4th order -> only one global extremum
            tg = -0.25 * coeff_[3] / coeff_[4];
            horner(coeff_, tg, eval);
            ag = eval[2];
            //ag = eval_poly(2, tg);
            
            if ( ( tg > 0 ) && (tg < te )       // if global extr between limits
                 &&  ( ag > a_max ) ) {         // and bigger than limit value
                a_max = ag;                     // override limit values
                t_max = tg;
            }
            else if ( ( tg > 0 ) && (tg < te )   // if global extr between limits
                    && ( ag < a_min ) ) {       // and smaller than limit value
                a_min = ag;                     // override limit values
                t_min = tg;
            }
        }
    }
    else { // polynomial 5th order
        double discr;
        discr = coeff_[4]*coeff_[4] - 2.5 * coeff_[3] * coeff_[5];
        if ( discr < 0 ) {
            ;// -> do nothing, use limits
        }
        else {
            double tg1,tg2;
            tg1 = ( -coeff_[4] + sqrt(discr) ) / ( 5 * coeff_[5] );
            tg2 = ( -coeff_[4] - sqrt(discr) ) / ( 5 * coeff_[5] );

            //double ag1 = eval_poly(2, tg1);
            horner(coeff_, tg1, eval);
            double ag1 = eval[2];
            //double ag2 = eval_poly(2, tg2);
            horner(coeff_, tg2, eval);
            double ag2 = eval[2];
            

            double ag_max, ag_min;
            double tg_max, tg_min;
            if ( ag1 > ag2 ) {    // determin global max and min
                ag_max = ag1;
                tg_max = tg1;
                ag_min = ag2;
                tg_min = tg2;
            }
            else {
                ag_max = ag2;
                tg_max = tg2;
                ag_min = ag1;
                tg_min = tg1;
            }
            if ( ( tg_max > 0 ) && (tg_max < te )       // if global tmax between limits
                            && ( ag_max > a_max ) ) {   // and bigger than limit max
                a_max = ag_max;     // override limit values
                t_max = tg_max;
            }
            if ( ( tg_min > 0 ) && (tg_min < te )       // if global tmin between limits
                            &&  ( ag_min < a_min ) ) {  // and smaller than limit min
                a_min = ag_min;     // override limit values
                t_min = tg_min;
            }
        }
    }
return 0;
}

//---------------- Evaluate nth order of trajectory at t ------------------
int PolyTraj::eval_trajectory (const double& t, std::vector<double>& eval) const {
    //printf("t = %f, te = %f\n", t, te);
    if ( t < te ) {
        //return eval_poly(derivative, t, eval);
        return horner(coeff_, t, eval);
    }
    else {
        //t -= te;    
        // extrapolate with constant acceleration
    std::vector<double> third_order_coeff(6);
    
    third_order_coeff[0] = end_state.x;
    third_order_coeff[1] = end_state.x_der;
    third_order_coeff[2] = end_state.x_dder/2.0;
    third_order_coeff[3] = 0.0;
    third_order_coeff[4] = 0.0;
	third_order_coeff[5] = 0.0;
     
    return horner(third_order_coeff, (t-te), eval);       
    }
}

void PolyTraj::calc_jerk_integral() {
    double te2 = te*te;
    double te3 = te*te2;
    double te4 = te2*te2;
    double te5 = te*te4;
    jerk_integral = (   36.0*(coeff_[3]*coeff_[3])*te
            + te3*(192.0*(coeff_[4]*coeff_[4]) + 240.0*coeff_[3]*coeff_[5])
            + 720.0*coeff_[5]*coeff_[5]*te5
            + 144.0*coeff_[3]*coeff_[4]*te2
            + 720.0*coeff_[4]*coeff_[5]*te4);
}

void PolyTraj::sampleArgumentEquidistant(const double& t_current, double resolution){
    int number_of_points = (int)floor(horizon_ / resolution +.5);
    //mexPrintf("\n number_of_points = %d\n", number_of_points);
    trajectory_point_1D traj_point;
    double ti=0;
    std::vector<double> eval(6);
    for (int i = 0; i<number_of_points; i++) {
        eval_trajectory(ti, eval);
        traj_point.arg      = ti + t_current; // here we are back in absolute time
        traj_point.x        = eval[0];
        traj_point.x_der    = eval[1];
        traj_point.x_dder   = eval[2];
        traj_point.x_ddder  = eval[3];
        discrete_traj_points_[0].push_back(traj_point);
        ti += resolution;
        // mexPrintf("ti = %f\n", ti);
    }
}

void PolyTraj::sampleArgumentSynchronized(const std::vector<std::vector<double> >& sample_ref ){
	// Samples the trajectory at the same discrete arguments as the reference vectors

	const double t0 = sample_ref[0][0];
	discrete_traj_points_.resize(sample_ref.size());
	std::vector<std::vector<double> >::const_iterator it;
	std::vector<std::vector<trajectory_point_1D> >::iterator it_traj;
	std::vector<double> eval;
	eval.resize(6);
	for ( 	it  = sample_ref.begin(), it_traj = discrete_traj_points_.begin();
			it != sample_ref.end();
			it++, it_traj++ ) {

		std::vector<double>::const_iterator it_vec;
		for ( 	it_vec  = it->begin();
				it_vec != it->end();
				it_vec++ ) {
			trajectory_point_1D traj_point;
			eval_trajectory( (*it_vec - t0), eval); // relative time
			traj_point.arg      = *it_vec; // here we are back in absolute time
			traj_point.x        = eval[0];
			traj_point.x_der    = eval[1];
			traj_point.x_dder   = eval[2];
			traj_point.x_ddder  = eval[3];
			it_traj->push_back(traj_point);
		}
	}
}

void PolyTraj::set_debug_info( const double& arg, const double& jerk, const double& deviation, const double& cost_arg, const double& cost_jerk, const double& cost_deviation ) {
	arg_ 	= arg;
	jerk_	= jerk;
	deviation_ = deviation;
	cost_arg_ = cost_arg;
	cost_jerk_ = cost_jerk;
	cost_deviation_ = cost_deviation;
}

void PolyTraj::get_debug_info( double& arg, double& jerk, double& deviation, double& cost_arg, double& cost_jerk, double& cost_deviation ) const {
	arg 	= arg_;
	jerk	= jerk_;
	deviation = deviation_;
	cost_arg = cost_arg_;
	cost_jerk = cost_jerk_;
	cost_deviation = cost_deviation_;
}

} // namespace vlr
