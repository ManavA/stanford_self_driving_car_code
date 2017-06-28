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


#ifndef KEEPLANEANDVELOCITYSET2D_H_
#define KEEPLANEANDVELOCITYSET2D_H_

#include <set>

#include <poly_traj_structs.h>
#include <polyTraj2D.h>
#include <keep_lane_traj_set.h>
#include <keepVelocityTrajSet.h>
#include <driving_common/Trajectory2D.h>

namespace vlr {

class KeepLaneAndVelocitySet2D {
public:
    keep_lane_traj_set keep_lane_traj_set_;                      // lane keeping set
    KeepVelocityTrajSet  keep_velocity_traj_set_;                // velocity set
    std::multiset<PolyTraj2D,PolyTraj2D::CompPolyTraj2D> set_data_; // cost sorted overlay of both

    KeepLaneAndVelocitySet2D(const lanekeeping_params &par_kl,
            const velocity_params &par_vel, const PolyTraj2D_params &par_2D);   // sets only the lat/long params of new object
    int generate(const std::vector<CurvePoint>& center_line, double t,
    		driving_common::TrajectoryPoint2D start_trajectory_point, GenerationMode generation_mode, double desired_velocity); // generates in each step the new lat, long and combined trajectories

    inline const PolyTraj2D_params& params() const {return par_;}
    inline void params(const PolyTraj2D_params& params) {par_=params;}
    inline void params(const lanekeeping_params& keep_lane_params, const velocity_params& keep_velocity_params,
                       const PolyTraj2D_params& params) {
      keep_lane_traj_set_.params(keep_lane_params);
      keep_velocity_traj_set_.params(keep_velocity_params);
      par_=params;
    }

private:
    PolyTraj2D_params par_;
};

} // namespace vlr

#endif // KEEPLANEANDVELOCITYSET2D_H_
