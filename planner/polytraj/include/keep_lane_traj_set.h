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


#ifndef KEEP_LANE_TRAJ_SET_H_
#define KEEP_LANE_TRAJ_SET_H_

#include <set>
#include <vector>

#include <poly_traj_structs.h>
#include <polyTraj.h>

namespace vlr {
  //------------------- Lateral trajectory set for lane keeping --------

class keep_lane_traj_set {
    lanekeeping_params par;
public:
    std::multiset<PolyTraj,PolyTraj::CompPolyTraj> set_data; // contains the 1D trajectories

    keep_lane_traj_set(lanekeeping_params params);
    int generate(double t, double s, movement_state start_state, GenerationMode mode,
            double sampling_horizon, const std::vector<std::vector<double> >& sample_ref);
		// t 			.. time in the future we start generation from
		// start_state 	.. state in the future we start generation from
		// mode			.. normally time based, arc length based only for low speeds
		// sampling_hor .. temporal horizon we sample in for collision check
		// sample_ref	.. in order to combine the set with longitudinal traj sets, we sample according to their sample_ref

    void clear();

    inline const lanekeeping_params& params() const {return par;}
    inline void params(const lanekeeping_params& params) {par=params;}

    // get rid of data of previous cycle
};

} // namespace vlr

#endif // KEEP_LANE_TRAJ_SET_H_
