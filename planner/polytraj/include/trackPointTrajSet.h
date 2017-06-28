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


#ifndef TRACKPOINTTRAJSET_H_
#define TRACKPOINTTRAJSET_H_

#include <set>
#include <poly_traj_structs.h>

#include <rootTraj.h>

namespace vlr {

class TrackPointTrajSet {

  // tracking_params par_; TODO
public:
	tracking_params par_; //TODO
  std::multiset<RootTraj,PolyTraj::CompPolyTraj> set_data_;
    //bool slowSpeed_;

    TrackPointTrajSet(tracking_params params);
    int generate(double t, movement_state start_state,
    		double desired_position, double desired_velocity, double desire_acceleration,
            const std::vector<CurvePoint>& center_line, GenerationMode mode);
    void clear();

    inline const tracking_params& params() const {return par_;}
    inline void params(const tracking_params& params) {par_=params;}
};

} // namespace vlr

#endif // TRACKPOINTTRAJSET_H_
