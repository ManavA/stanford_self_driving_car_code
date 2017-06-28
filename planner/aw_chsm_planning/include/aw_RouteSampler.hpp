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


#ifndef AW_ROUTESAMPLER_H
#define AW_ROUTESAMPLER_H

#include <aw_Topology.hpp>
#include <driving_common/Trajectory2D.h>

namespace vlr {

//--------------------------------------------------------
//             RouteSampler
//--------------------------------------------------------

class RouteSampler
{
public:
	RouteSampler(Topology* top);
	~RouteSampler();

//	// samples the route beginnning at the current edge from the topology route
//	bool samplePoints(CurvePoints* curvepoints, double back_sampl_dist = -10., double length = 30., int point_anz = 15, double start_lateral_offset = 0., bool make_step = false, double dist_to_step = -1., double gap_length = 0., double end_lateral_offset = 0. );
//
//	static bool samplePoints(Route::RouteEdgeList::iterator edge_it, double dist_from_start, Route::RouteEdgeList* edges, CurvePoints* curvepoints, double back_sampl_dist = -10., double length = 30., int point_anz = 15, double start_lateral_offset = 0., bool make_step = false, double dist_to_step = -1., double gap_length = 0., double end_lateral_offset = 0. );

  bool sampleMission(std::vector<CurvePoint>& waypoints, std::vector<bool>& points_to_ignore);
  void vertexToCurvePoint(RndfVertex* v);


private:
	Topology* top_;
	RndfGraph* graph_;
	Route* route_;
	Route::RouteEdgeList* edges_;
	std::map<RndfEdge*, double> lastOffsets_;
//	CurvePoints oldCurvePoints_; // keep curve points from last step
  CurvePoint cp_;
};

} // namespace vlr

#endif // AW_ROUTESAMPLER_H
