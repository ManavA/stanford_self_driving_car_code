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


#ifndef ROOTTRAJ_H_
#define ROOTTRAJ_H_

#include <vector>
#include <map>
#include <poly_traj_structs.h>

#include <polyTraj.h>

namespace vlr {

class RootTraj: public PolyTraj {
    
public:
    std::vector<CurvePoint> foot_curve_points_; // sampled at the required positions
    std::vector<CurvePoint> center_line_; // discrete representation of the center curve
    
    RootTraj( const std::vector<CurvePoint>& center_line );
    void calculateRootTrajectory();
    void evalCenterlineAtS(const double& s_interpol, CurvePoint& cp_interpol) const;
    static void evalCenterlineAtS_static(const std::vector<CurvePoint>& center_line,
            const double& s_interpol, CurvePoint& cp_interpol);
private:
    struct CompareCurvePoints {
      bool operator() (const CurvePoint& cp1, const CurvePoint& cp2) {
        return cp1.s < cp2.s;
      }
    };
    static CompareCurvePoints cp_comp_;

};

} // namespace vlr

#endif // ROOTTRAJ_H_
