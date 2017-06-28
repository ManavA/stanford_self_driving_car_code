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
#include <cmath>
#include <iostream>
#include <global.h>
#include <vlrException.h>

#include <polyTraj2D.h>
#include <rootTraj.h>

namespace vlr {

//#define TRACE(x)  std::cout << " [PolyTraj2D] " << x << std::endl;
#define TRACE(x)

// TODO: remove center_line?!?
PolyTraj2D::PolyTraj2D( const std::vector<CurvePoint>& /*center_line*/, const PolyTraj& traj_lat,
        const RootTraj& root_traj, const GenerationMode mode, std::vector<std::vector<trajectory_point_1D> >::const_iterator it_lat_sampled_trajectory,
        int& index_lat, int& index_lon) {

    generation_mode_ = mode; // either arclength-based for slow speeds or time base for higher speeds
    index_lat_ = index_lat;
    index_lon_ = index_lon;
    // memorize original lat/long trajectories for later use
    pt_traj_lat_  = & traj_lat;
    pt_traj_root_ = & root_traj;
    
			// lateral sampled traj varies
    int number_of_points_long = root_traj.discrete_traj_points_[0].size();	// long does not

    trajectory2D_.reserve(number_of_points_long);
    std::vector<trajectory_point_1D>::const_iterator it_root_traj_points, it_root_, it_lat_trajectory_points;
    std::vector<CurvePoint>::const_iterator it_foot_curve_points;
    for ( 	it_root_traj_points  = root_traj.discrete_traj_points_[0].begin(),
				it_foot_curve_points = root_traj.foot_curve_points_.begin(),
				it_lat_trajectory_points = (*it_lat_sampled_trajectory).begin();
    		it_root_traj_points != root_traj.discrete_traj_points_[0].end();
    		it_root_traj_points++, it_foot_curve_points++, it_lat_trajectory_points++ ) {
        driving_common::TrajectoryPoint2D transformed_point;
    	transformed_point = latLongTrajectories2GlobalCordinates( *it_lat_trajectory_points,
    			*it_root_traj_points, *it_foot_curve_points);
            // Add new 2D point to vector
        trajectory2D_.push_back(transformed_point);
    }
}
double PolyTraj2D::getInitialJerk(void) const {
	return pt_traj_root_->discrete_traj_points_.begin()->begin()->x_ddder;
}

void PolyTraj2D::calc_extreme_values(const double t_delay) {
    double kappa_max = - DBL_MAX;
    double kappa_min =   DBL_MAX;
    
    double a_lon_max = - DBL_MAX;
    double a_lon_min =   DBL_MAX;

    double a_lat_max = - DBL_MAX;
    double a_lat_min =   DBL_MAX;

    double dist_max =  - DBL_MAX;
    double dist_min =    DBL_MAX;

	double angle_max = - DBL_MAX;
	double angle_min =   DBL_MAX;

    double v_min	 =   DBL_MAX;

    // Move forward to t_start to avoid numerical effects with start value
	double t_start = trajectory2D_.begin()->t;
    t_start += t_delay;
    std::vector<driving_common::TrajectoryPoint2D>::iterator it_tp;
    for (  it_tp = trajectory2D_.begin();
    		it_tp !=  trajectory2D_.end() ;
    		            it_tp++ ) {
    	if ( it_tp->t > t_start) {
    		break;
    	}
    }
    for ( 	;
            it_tp !=  trajectory2D_.end() ;
            it_tp++ ) {

    	// curvature
        if ( it_tp->kappa > kappa_max ) {
            kappa_max = it_tp->kappa;
        }
        if ( it_tp->kappa < kappa_min ) {
            kappa_min = it_tp->kappa;
        }

    	// lateral acceleration
        if ( it_tp->a_lat > a_lat_max ) {
        	a_lat_max = it_tp->a_lat;
        }
        if ( it_tp->a_lat < a_lat_min ) {
        	a_lat_min = it_tp->a_lat;
        }

    	// longitudinal acceleration
        if ( it_tp->a > a_lon_max ) {
        	a_lon_max = it_tp->a;
        }
        if ( it_tp->a < a_lon_min ) {
        	a_lon_min = it_tp->a;
        }

        // distance to center line
		if ( it_tp->d  > dist_max ) {
			dist_max = it_tp->d;
		}
		if ( it_tp->d < dist_min ) {
			dist_min = it_tp->d;
		}

		// angular offset to center line
		if ( it_tp->delta_theta  > angle_max ) {
			angle_max = it_tp->delta_theta;
		}
		if ( it_tp->delta_theta < angle_min ) {
			angle_min = it_tp->delta_theta;
		}

		// velocity
        if ( it_tp->v < v_min ) {
        	v_min = it_tp->v;
    }
    }

    // Convert to absolute values
    if ( ( kappa_max * kappa_max ) > ( kappa_min * kappa_min ) ) {
        max_curvature_ = kappa_max;
    }
    else {
        max_curvature_ = -kappa_min;
    }

	if ( ( dist_max * dist_max ) > ( dist_min * dist_min ) ) {
		max_distance_ = dist_max;
	}
	else {
		max_distance_ = -dist_min;
}

	if ( ( angle_max * angle_max ) > ( angle_min * angle_min ) ) {
		max_angle_ = angle_max;
	}
	else {
		max_angle_ = -angle_min;
	}

	if ( ( a_lat_max * a_lat_max ) > ( a_lat_min * a_lat_min ) ) {
		max_a_lat_ = a_lat_max;
        }
	else {
		max_a_lat_ = -a_lat_max;
    }

	max_a_lon_ = a_lon_max;

	min_a_lon_ = a_lon_min;

	min_velocity_ = v_min;
}

driving_common::TrajectoryPoint2D PolyTraj2D::latLongTrajectories2GlobalCordinates(const trajectory_point_1D& lat_traj_point,
        const trajectory_point_1D& long_traj_point, const CurvePoint& foot_curve_point ) const {
    
    driving_common::TrajectoryPoint2D traj_point_res;
   
            // keep formulas short
    double& t_x         = traj_point_res.t;
    double& x1          = traj_point_res.x;
    double& x2          = traj_point_res.y;
    double& theta_x     = traj_point_res.theta;
    double& kappa_x     = traj_point_res.kappa;
    double& kappa_dot_x = traj_point_res.kappa_dot;
    double& v_x         = traj_point_res.v;
    double& a_x         = traj_point_res.a;
    double& jerk_x      = traj_point_res.jerk;
    
    double& d_x      	= traj_point_res.d;
    double& delta_theta_x = traj_point_res.delta_theta;
    double& a_lat_x     = traj_point_res.a_lat;

    // const double& s       = long_traj_point.x;
    const double& s_dot      = long_traj_point.x_der;
    const double& s_ddot     = long_traj_point.x_dder;
    //const double& s_dddot    = long_traj_point.x_ddder;
    
    const double& d       = lat_traj_point.x;
    
    d_x = d; // memorize for filtering the trajectories that are too far off
    //printf("traj_point_res.d = %f\n", traj_point_res.d);

    const double& r1              = foot_curve_point.x;
    const double& r2              = foot_curve_point.y;
    const double& theta           = foot_curve_point.theta;
    const double& kappa           = foot_curve_point.kappa;
    const double& kappa_prime     = foot_curve_point.kappa_prime;
 
    const double sin_theta = sin( foot_curve_point.theta ); // for fast computation
    const double cos_theta = cos( foot_curve_point.theta );
    
      // normal vector
    const double  n1 = - sin_theta;
    const double& n2 =   cos_theta;
    // tangential vector
    
    const double one_minus_kappa_d = ( 1 - kappa * d );
    
    if (  ( - DBL_EPSILON < one_minus_kappa_d ) && (  one_minus_kappa_d < DBL_EPSILON ) ) {
    	throw VLRException( "The calculated trajectory is too far off from center line for its curvature!");
    }
    
   const double s_dot_square = s_dot * s_dot;
             
    // calculate resulting trajectory point
    t_x = long_traj_point.arg;   // use this time
    x1 = r1 + d * n1;
    x2 = r2 + d * n2;
    
    double d_prime, d_pprime;
    
    if ( generation_mode_ == time_based ) {
    
        const double& d_dot   = lat_traj_point.x_der;
        const double& d_ddot  = lat_traj_point.x_dder;
        //const double& d_dddot = lat_traj_point.x_ddder;

    	// calc local variable s_dot
        // Explanation for the following lines:
        // We restrict the trajectories with high speed generation to be aligned to the center line at the end
        const double MIN_VEL = 0.0001;
        const double MIN_DPRIME_SQUARE = 0.000000001;
        if ( s_dot > MIN_VEL ) {
        	d_prime 	= d_dot / s_dot; // s. paper
        }
        else if ( d_dot > MIN_VEL )/* || */ {
        	kappa_x = DBL_MAX; // so that won't use it anyway
        	theta_x = theta;
        	v_x     = 0.0;
        	a_x     = 0.0;
        	kappa_dot_x = 0;
        	jerk_x      = 0;
        	return traj_point_res;
        	}
        else {
        	d_prime = 0.0;	// we assume that car is already aligned to centerline
        }
    
        double d_ddot_minus_d_prime_s_ddot = ( d_ddot - d_prime * s_ddot );
        double expr_square = d_ddot_minus_d_prime_s_ddot*d_ddot_minus_d_prime_s_ddot;
        if ( s_dot > MIN_VEL ) {
        	d_pprime	= d_ddot_minus_d_prime_s_ddot / s_dot_square;
        }
        else if ( expr_square > MIN_DPRIME_SQUARE ) {
        	d_pprime = DBL_MAX; // set curvature to infinity so that we won't use trajectory
        	}
        else {
        	d_pprime = 0.0;	// we assume that car drives parallel to centerline
        }
    }
    else { // generation mode == arclength_based   -> take d_prime and d_pprime directly from generation
        d_prime   = lat_traj_point.x_der;
        d_pprime  = lat_traj_point.x_dder;
        //d_ppprime = lat_traj_point.x_ddder;
    }

    // Now we can use the formulas in the paper for both generation modes
	const double delta_theta = atan( d_prime / one_minus_kappa_d );
	delta_theta_x = delta_theta; // memorize for filtering the trajectories that are too far off
	if ( ( delta_theta <= - M_PI_2 ) || (  M_PI_2 <= delta_theta ) ) {
		throw VLRException( "The calculated heading is too far off from the center line!");
	}
	const double cos_delta_theta = cos(delta_theta);
	//const double sin_delta_theta = sin(delta_theta);
	const double tan_delta_theta = tan(delta_theta);
	const double one_over_cos_delta_theta = 1.0 / cos_delta_theta;
	const double one_minus_kappa_d_div_cos_delta_theta = one_minus_kappa_d * one_over_cos_delta_theta;
	const double cos_delta_theta_div_one_minus_kappa_d = 1.0 / one_minus_kappa_d_div_cos_delta_theta;
	const double kappa_times_d__prime = kappa_prime * d + kappa * d_prime;

	theta_x     = theta + delta_theta;
	kappa_x     = ( d_pprime + kappa_times_d__prime * tan_delta_theta )
			* cos_delta_theta * cos_delta_theta_div_one_minus_kappa_d * cos_delta_theta_div_one_minus_kappa_d
			+ cos_delta_theta_div_one_minus_kappa_d * kappa ;
	kappa_dot_x = 0;
	v_x         = s_dot  * one_minus_kappa_d_div_cos_delta_theta;

	a_lat_x 	= v_x*v_x * kappa_x;  // lateral vehicle acceleration

	const double delta_theta_prime = one_minus_kappa_d_div_cos_delta_theta * kappa_x - kappa;

	a_x         = s_ddot * one_minus_kappa_d_div_cos_delta_theta
					+ s_dot_square * one_over_cos_delta_theta * ( one_minus_kappa_d * tan_delta_theta * delta_theta_prime
										-  kappa_times_d__prime );
	kappa_dot_x = 0; // TODO
	jerk_x      = 0;
    
    return traj_point_res;
}
    
/*static*/ void PolyTraj2D::globalCordinates2LatLong(const driving_common::TrajectoryPoint2D& tp_2d,
        const std::vector<CurvePoint>& center_line, const GenerationMode& generation_mode,
        movement_state& lat_state, movement_state& lon_state) {
    
    const double& x1      = tp_2d.x;
    const double& x2      = tp_2d.y;
    const double& theta_x = tp_2d.theta;
    const double& kappa_x = tp_2d.kappa;
    const double& v_x     = tp_2d.v;
    const double& a_x     = tp_2d.a;
    
                       // Find closest index      
    if ( center_line.size() <= 1 ) {
		throw VLRException( "No entries in center line!");
    }
    std::vector<CurvePoint>::const_iterator it_closest = center_line.begin();
    double min_dist_sq = DBL_MAX;
    double dist_sq;
    
    // Find closest point as start solution
    std::vector<CurvePoint>::const_iterator it_cl_point;
    for ( it_cl_point = center_line.begin();
            it_cl_point != center_line.end(); 
            it_cl_point++ ) {

        const double& r1 = it_cl_point->x;
        const double& r2 = it_cl_point->y;
        
        double delta_x = r1 - x1;
        double delta_y = r2 - x2;
        dist_sq = delta_x*delta_x + delta_y*delta_y;
        if ( dist_sq < min_dist_sq ) {
            min_dist_sq = dist_sq;
            it_closest = it_cl_point;
        }
    }
    
    // Solve projection problem by minimizing projectionCondition() 
    // by means of the SECANT METHOD
    const double accuracy = 0.0000001; 
    int count = 100;
    
    // Use the following start values
    double s0 = it_closest->s;
    double s1;
    
    if ( it_closest != center_line.begin() ) {
        it_closest--;
        s1 = it_closest->s;
    }
    else {
        it_closest++;
        s1 = it_closest->s;
    }
 
    double f0, f1, f2, s2;

    try {
		projectionCondition( center_line, s0, tp_2d.x, tp_2d.y, f0 ); //f0 = f(s, x, y)
		projectionCondition( center_line, s1, tp_2d.x, tp_2d.y, f1 );
    }
    catch(vlr::Ex<>& e) {
 	throw VLRException( "Secant method -> " + e.what());
    }

    double delta_s;
    do {
        double delta_f = ( f1 - f0 );
        if ( ( - DBL_EPSILON < delta_f ) && ( delta_f < DBL_EPSILON ) ) {
         	throw VLRException( "Secant method: too many iterations?" );
        }
        s2 = s1 - ( s1 - s0 ) / ( f1 - f0 ) * f1;
        
        try {
        	projectionCondition( center_line, s2, tp_2d.x, tp_2d.y, f2 ); // f2 = f(s, x, y)
        }
        catch(vlr::Ex<>& e) {
      	throw VLRException( "Secant method: f2 = f(s, x, y) -> " + e.what());
        }
        // prepare for next step
        s0 = s1; f0 = f1;
        s1 = s2; f1 = f2;
        // printf("s0, s1 =%f, %f\n", s0, s1);
        
        delta_s=( s1 - s0 );
        // printf("delta_s=%f\n",delta_s);
        count--;
        if (count == 0) {
          	throw VLRException( "Secant method has not converged fast enough!");
        }
    } while (  !(( - accuracy < delta_s ) && ( delta_s < accuracy ))  );
    
    double& s       = lon_state.x;
    double& s_dot   = lon_state.x_der;
    double& s_ddot  = lon_state.x_dder;
    
    s 		= s2; // use result from secant method

    // Get position, orientation, etc. from centerline at s
    CurvePoint foot_point;
    //printf("s = %f\n", s);
    RootTraj::evalCenterlineAtS_static(center_line, s, foot_point);

    const double& r1                = foot_point.x;
    const double& r2                = foot_point.y;
    const double& theta_r           = foot_point.theta;
    const double& kappa_r           = foot_point.kappa;
    const double& kappa_r_prime     = foot_point.kappa_prime;
    
    const double sin_theta = sin( theta_r );
    const double cos_theta = cos( theta_r );
    
          // normal vector
    const double  n1 = - sin_theta;
    const double& n2 =   cos_theta;
    
    double delta_theta = ( theta_x - theta_r );
    delta_theta = vlr::normalizeAngle( delta_theta );

    if (  (delta_theta >= M_PI_2) || ( delta_theta  <= -M_PI_2 )  ) {
      printf("theta_r: %f, theta_x: %f, delta_theta: %f\n", theta_r, theta_x, delta_theta);
      throw VLRException( "Projection failed! Reference heading is too far off!");
    }
    
    double& d       = lat_state.x;

    d       = ( x1 - r1 ) * n1 + ( x2 - r2 ) * n2;              // s. paper

    const double one_minus_kappa_d = ( 1 - kappa_r * d );
    const double cos_delta_theta = cos(delta_theta);
    const double sin_delta_theta = sin(delta_theta);
    const double tan_delta_theta = tan(delta_theta);
    const double one_div_cos_delta_theta = 1.0 / cos_delta_theta;
    const double one_minus_kappa_d_div_cos_delta_theta = one_minus_kappa_d / cos_delta_theta;
    const double cos_delta_theta_div_one_minus_kappa_d = 1.0 / one_minus_kappa_d_div_cos_delta_theta;
    const double delta_theta_prime = one_minus_kappa_d_div_cos_delta_theta * kappa_x - kappa_r;

    s_dot   = v_x * cos_delta_theta_div_one_minus_kappa_d;      // s. paper
    const double s_dot_square = s_dot * s_dot;
    /*const*/ double d_prime_ = one_minus_kappa_d * tan_delta_theta; 				// needed in either case
    const double kappa_times_d__prime = kappa_r_prime * d + kappa_r * d_prime_; // needed in either case

    if ( generation_mode == time_based ) {
        double& d_dot   = lat_state.x_der;
        double& d_ddot  = lat_state.x_dder;

        d_dot   = v_x * sin_delta_theta;                           	// s. paper
        d_ddot  = a_x * sin_delta_theta + v_x * cos_delta_theta
        		* ( kappa_x * v_x - kappa_r * s_dot );          	// s. paper
        //printf("Projection: d, d_dot, d_ddot = %f, %f, %f\n", d, d_dot, d_ddot );
    }
    else { // generation_mode_ == arclength_based
        double& d_prime   = lat_state.x_der;
        double& d_pprime  = lat_state.x_dder;

    	d_prime 	= d_prime_;
    	d_pprime	= kappa_times_d__prime * tan_delta_theta
    	    			+ one_minus_kappa_d_div_cos_delta_theta * one_div_cos_delta_theta
    	    			* ( kappa_x * one_minus_kappa_d_div_cos_delta_theta - kappa_r);
        //printf("Projection: d, d_prime, d_pprime = %f, %f, %f\n", d, d_prime, d_pprime );
    }
    
    s_ddot = ( a_x - s_dot_square * one_div_cos_delta_theta * ( one_minus_kappa_d * tan_delta_theta * delta_theta_prime
                                            -  kappa_times_d__prime ) ) * cos_delta_theta_div_one_minus_kappa_d;
                                                                // s. paper
    TRACE("Projection: s, s_dot, s_ddot = " << s << ", " << s_dot << ", " << s_ddot);
}

/*static*/ void PolyTraj2D::projectionCondition(const std::vector<CurvePoint>& center_line, const double& s_foot,
        const double& x1, const double& x2, double& f) {
    CurvePoint foot_point;
    
        // Interpolate at s
    try {
    	RootTraj::evalCenterlineAtS_static(center_line, s_foot, foot_point);
    }
   catch(vlr::Ex<>& e) {
	throw VLRException( "Polynomial trajectory -> " + e.what());
   }
        // Function which should be zero
    f = ( x1 - foot_point.x ) * cos(foot_point.theta) + ( x2 - foot_point.y ) * sin(foot_point.theta);
    return;
}

void PolyTraj2D::calculateNextInitialStates(const double& t_next_deadline,
        movement_state& next_lat_state, 
        movement_state& next_long_state) const {
    
    trajectory_point_1D lat_tp;
    trajectory_point_1D long_tp;
      double t_last_deadline = pt_traj_root_->discrete_traj_points_[0][0].arg; // TODO: introduce new member var t_start
    if ( t_last_deadline > t_next_deadline ) {
      	throw VLRException( "t_last_deadline > t_next_deadline!" );
    }
    evalLatLongTrajectoriesTimeBased ( t_last_deadline, ( t_next_deadline - t_last_deadline ),
            lat_tp, long_tp );
    
    next_lat_state.x        = lat_tp.x;
    next_lat_state.x_der    = lat_tp.x_der;
    next_lat_state.x_dder   = lat_tp.x_dder;
    next_long_state.x       = long_tp.x;
    next_long_state.x_der   = long_tp.x_der;
    next_long_state.x_dder  = long_tp.x_dder;
}

driving_common::TrajectoryPoint2D PolyTraj2D::evaluateTrajectoryAtT(const double& t) const {
    if(trajectory2D_.empty()) {
      throw VLRException("trajectory is empty");
    }

    const double& t_planning = trajectory2D_[0].t;  // // TODO: introduce new member var t_start
    trajectory_point_1D lat_tp, long_tp;
    
    // get data from lateral and longitudinal trajectory
    if ( generation_mode_ == time_based ) {
        evalLatLongTrajectoriesTimeBased( t_planning, ( t - t_planning ), lat_tp, long_tp);
    }
    else { // generation_mode_ == arclength_based
        const double& s_start = pt_traj_root_->discrete_traj_points_[0][0].x; // TODO: introduce new member var s_start
        evalLatLongTrajectoriesArclengthBased( t_planning, ( t - t_planning ), 
                s_start, lat_tp, long_tp);
    }

    // generate data for trajectory point
    const double& s_interpol = long_tp.x;
    CurvePoint cp_interpol_foot;

    try {
    	pt_traj_root_->evalCenterlineAtS(s_interpol, cp_interpol_foot);
    }
    catch(vlr::Ex<>& e) {
      throw VLRException("Polynomial trajectory -> " + e.what());
    }
    return latLongTrajectories2GlobalCordinates(lat_tp, long_tp, cp_interpol_foot );
}

driving_common::TrajectoryPoint2D PolyTraj2D::calculateNextStartTrajectoryPoint(const double& t_next) const {
    return evaluateTrajectoryAtT(t_next);
}

void PolyTraj2D::evalLatLongTrajectoriesTimeBased(const double& t, const double& delta_t,
        trajectory_point_1D& lat_tp, 
        trajectory_point_1D& long_tp) const {
    /* t = relative time */
    
    std::vector<double> derivatives(6);
    
    pt_traj_root_->eval_trajectory(delta_t, derivatives);
    long_tp.arg       = t + delta_t;
    long_tp.x       = derivatives[0];
    long_tp.x_der   = derivatives[1];
    long_tp.x_dder  = derivatives[2];
    long_tp.x_ddder = derivatives[3];
    
    pt_traj_lat_->eval_trajectory(delta_t, derivatives);
    lat_tp.arg        = t + delta_t; // back in absolute time
    lat_tp.x        = derivatives[0];
    lat_tp.x_der    = derivatives[1];
    lat_tp.x_dder   = derivatives[2];
    lat_tp.x_ddder  = derivatives[3];
}

void PolyTraj2D::evalLatLongTrajectoriesArclengthBased(const double& t, 
        const double& delta_t, const double& s,
        trajectory_point_1D& lat_tp, 
        trajectory_point_1D& long_tp) const {
    /* t = relative time */
    
    std::vector<double> derivatives(6);

    pt_traj_root_->eval_trajectory(delta_t, derivatives);
    long_tp.arg     = t + delta_t; // back in absolute time
    long_tp.x       = derivatives[0];
    long_tp.x_der   = derivatives[1];
    long_tp.x_dder  = derivatives[2];
    long_tp.x_ddder = derivatives[3];

    const double& delta_s = long_tp.x - s; // total_s - start_s
    pt_traj_lat_->eval_trajectory(delta_s, derivatives); // d(delta_s)!
    lat_tp.arg      = s;    // not zero based
    lat_tp.x        = derivatives[0];
    lat_tp.x_der    = derivatives[1];
    lat_tp.x_dder   = derivatives[2];
    lat_tp.x_ddder  = derivatives[3];
}

void PolyTraj2D::generateNewControlTrajectory(const double& t_current, const double& t_control_horizon,
                                              const double& control_t_res, std::vector<driving_common::TrajectoryPoint2D>& trajectory) const {
  std::vector<driving_common::TrajectoryPoint2D> new_traj;
  std::vector<driving_common::TrajectoryPoint2D>::const_iterator it;
  if (trajectory2D_.size() == 0) {
    printf("Error: no new points as trajectory2D_.size() == 0 ");
    return;
  }

  const double& t_planning = trajectory2D_[0].t; // todo
  // keep some points from last trajectory
  if (trajectory.size() > 0) {
    for (it = trajectory.begin(); ((it->t < t_planning) && (it != trajectory.end())); it++) {

      if (((it + 1)->t > t_current) // keep last point to be able to interpolate
          || ((t_current < it->t) && (it->t < t_planning))) {// also keep points between now and next planning step
        new_traj.push_back(*it);
      }
    }
  }
  // sample new trajectory points and add them
  driving_common::TrajectoryPoint2D tp_2d;

  double t_last_in_new_traj;
  if (new_traj.size() == 0) {
    t_last_in_new_traj = t_planning - control_t_res; // thats the best you can do when you start
    //printf("first step\n");                       // minus control_t_res because of for loop
  }
  else {
    t_last_in_new_traj = new_traj.back().t;
  }
  for (double t = t_last_in_new_traj + control_t_res; t <= (t_planning + t_control_horizon); t += control_t_res) {
    tp_2d = evaluateTrajectoryAtT(t);
    new_traj.push_back(tp_2d);
  }

  trajectory.swap(new_traj);
}

std::ostream &operator<< (std::ostream &ostr, const PolyTraj2D &traj2d)
{
	double arg_lat, jerk_lat, deviation_lat, cost_arg_lat, cost_jerk_lat, cost_deviation_lat;
	double arg_lon, jerk_lon, deviation_lon, cost_arg_lon, cost_jerk_lon, cost_deviation_lon;
	traj2d.pt_traj_lat_->get_debug_info( arg_lat, jerk_lat, deviation_lat, cost_arg_lat, cost_jerk_lat, cost_deviation_lat );
	traj2d.pt_traj_root_->get_debug_info( arg_lon, jerk_lon, deviation_lon, cost_arg_lon, cost_jerk_lon, cost_deviation_lon );
	ostr << "---------------------2D Trajectory Info ----------------------------------------------" << std::endl;
	ostr << "total_costs=" << traj2d.total_cost_;
	ostr << "  mode_lat=" << ( traj2d.generation_mode_ == time_based ? "time_based" : "arclength_based" ) << "  max_curvature=" << traj2d.get_max_curvature() << "  min_velocity=" << traj2d.get_min_velocity() << std::endl;
	ostr << "lat: " << "index="<< traj2d.index_lat_ << " arg=" << arg_lat << " jerk=" << jerk_lat << " deviation=" << deviation_lat
		 << "  total_costs=" << traj2d.pt_traj_lat_->get_total_cost()
		 << "  cost_arg=" << cost_arg_lat << "  cost_jerk=" << cost_jerk_lat << "  cost_dev="  << cost_deviation_lat  << std::endl;
	ostr << "lon: " << "index="<< traj2d.index_lon_ << " arg=" << arg_lon << " jerk=" << jerk_lon << " deviation=" << deviation_lon
		 << "  total_costs=" << traj2d.pt_traj_root_->get_total_cost()
		 << "  cost_arg=" << cost_arg_lon << "  cost_jerk=" << cost_jerk_lon << "  cost_dev="  << cost_deviation_lon << std::endl;
	ostr << "--------------------------------------------------------------------------------------" << std::endl;
	return ostr;
}

} // namespace vlr
