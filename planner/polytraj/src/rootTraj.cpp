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
#include <iostream>
#include <algorithm>

#include <global.h>
#include <vlrException.h>

#include <rootTraj.h>

namespace vlr {

//RootTraj::RootTraj( ) {
//}

RootTraj::RootTraj( const std::vector<vlr::CurvePoint>& center_line ) {
    center_line_ = center_line;
}

void RootTraj::calculateRootTrajectory() {

    if ( discrete_traj_points_.size() == 0 ) {
        throw VLRException("Cannot calc root trajectory as there are no discrete_traj_points_ sets!");
    }
    if ( discrete_traj_points_.size() == 0 ) {
        throw VLRException("Cannot calc root trajectory as there are no points in first set of the discrete_traj_points_!");
    }
    std::vector<trajectory_point_1D>::const_iterator it_traj_point_1D;
    for ( it_traj_point_1D = discrete_traj_points_[0].begin();
            it_traj_point_1D != discrete_traj_points_[0].end();
            it_traj_point_1D++ ) {
        CurvePoint cp_temp;
 
        try {
        	evalCenterlineAtS(it_traj_point_1D->x, cp_temp);
        }
        catch(vlr::Ex<>& e) {
        	throw VLRException( "Calculate root trajectory -> " + e.what());
        }
        foot_curve_points_.push_back(cp_temp);
    }
    return;
}

void RootTraj::evalCenterlineAtS(const double& s_interpol,
        CurvePoint& cp_interpol) const {
    evalCenterlineAtS_static(center_line_, s_interpol, cp_interpol);
    return;
}

void RootTraj::evalCenterlineAtS_static(const std::vector<CurvePoint>& center_line,
            const double& s_interpol, CurvePoint& cp_interpol) {
        
                      // Find closest index      
    if ( center_line.size() == 0 ) {
      throw VLRException("Center line doesn't have entries!");
    }
    if ( s_interpol < center_line.begin()->s ) {
      throw VLRException("Center line too short at the beginning!");
    }
    if ( s_interpol > center_line.rbegin()->s ) {
      throw VLRException("Center line too short at the end!");
    }

    CurvePoint cp_tmp;
    cp_tmp.s = s_interpol;
    std::vector<CurvePoint>::const_iterator it_upper = std::lower_bound(center_line.begin(), center_line.end(), cp_tmp, cp_comp_);
    if(it_upper == center_line.end()) {
    	throw VLRException("Current s not contained in center_line");
    }

    const CurvePoint& cp2 = *it_upper;
    double s2 = cp2.s;
    it_upper--;  // it_lower now
    const CurvePoint& cp1 = *it_upper;
    double s1 = cp1.s;

    double lambda;
    if ( (( s2 - s1 ) > DBL_EPSILON ) || (( s2 - s1 ) < -DBL_EPSILON ) ) {
        lambda = ( s_interpol - s1 ) / ( s2 - s1 );
    }
    else {
		throw VLRException("Center line points too close for interpolation!");
    }

    // Linear interpolation between points
    double x_interpol = cp1.x + lambda *( cp2.x - cp1.x );
    double y_interpol = cp1.y + lambda *( cp2.y - cp1.y );

    double delta_theta = cp2.theta - cp1.theta;

    delta_theta = vlr::normalizeAngle(delta_theta);
    double theta_interpol = cp1.theta + lambda * delta_theta;
    theta_interpol = vlr::normalizeAngle(theta_interpol);
    double kappa_interpol = cp1.kappa + lambda *( cp2.kappa  - cp1.kappa );
    double kappa_prime_interpol = cp1.kappa_prime + lambda *( cp2.kappa_prime - cp1.kappa_prime );

    cp_interpol.x           = x_interpol;
    cp_interpol.y           = y_interpol;
    cp_interpol.theta       = theta_interpol;
    cp_interpol.kappa       = kappa_interpol;
    cp_interpol.kappa_prime = kappa_prime_interpol;
}

} // namespace vlr
