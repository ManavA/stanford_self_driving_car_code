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


#include <aw_CGAL.h>
#include <aw_RouteSampler.hpp>

using namespace std;

namespace vlr {

using namespace RoutePlanner;

#undef TRACE
#define TRACE(str) cout << "[RouteSampler][" << __FUNCTION__ << "] " << str << endl;

RouteSampler::RouteSampler(Topology* top) : top_(top), graph_(NULL), route_(NULL), edges_(NULL),
                                            lastOffsets_() {
  assert(top);
  graph_ = top_->complete_graph;
  assert(graph_);
  route_ = &top_->route;
  edges_ = &route_->route;

    // temporary curve point will be initialized later
  cp_.s = 0;
  cp_.theta = 0;
  cp_.kappa = 0;
  cp_.kappa_prime = 0;
}

RouteSampler::~RouteSampler() {
}

void RouteSampler::vertexToCurvePoint(RndfVertex* v) {
  if(!v) {return;}
  cp_.x = v->x();
  cp_.y = v->y();
}

bool RouteSampler::sampleMission(std::vector<CurvePoint>& waypoints, std::vector<bool>& points_to_ignore) {
  if (!edges_) {
    TRACE("Got zero pointer to edge list.");
    return false;
  }

  if (edges_->begin() == edges_->end()) {
    TRACE("empty mission");
    return false;
  }

  waypoints.clear();
  Route::RouteEdgeList::const_iterator eit = edges_->begin();
  for(; eit != edges_->end(); eit++) {
       vertexToCurvePoint((*eit)->edge()->fromVertex());
        waypoints.push_back(cp_);
        points_to_ignore.push_back((*eit)->edge()->isVirtualEdge());
     }
  vertexToCurvePoint( (*(--edges_->end()))->edge()->toVertex());
  waypoints.push_back(cp_);
//  points_to_ignore.push_back((*eit)->edge()->fromVertex()->isVirtualVertex());
  return true;
}
} // namespace vlr
