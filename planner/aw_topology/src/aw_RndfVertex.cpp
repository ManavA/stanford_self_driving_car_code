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

#include <aw_RndfVertex.h>
#include <aw_RndfEdge.h>

using namespace std;

namespace vlr {

namespace RoutePlanner {

RndfVertex::RndfVertex(double lat, double lon, double x, double y, string name, int id) :
  Vertex<RndfEdge>(id), latitude_(lat), longitude_(lon), x_(x), y_(y), name_(name), isStop(false), isExit(false),
      isCheckpoint(false), isPerimeterPoint(false), isParkingSpot(false), isVirtual(false), isBlocked(false), exit_count_(0),
      checkpoint_id_(-1)/*, m_intersection(NULL)*/ {
}

RndfVertex::~RndfVertex() {
};

bool RndfVertex::hasIntersectionOutEdge(const RndfIntersection* isec) const
{
	for (TEdgeSet::const_iterator i=m_outEdges.begin(); i!=m_outEdges.end(); ++i) {
		if ((*i)->intersection() == isec) {
			return true;
		}
	}
	return false;
}
bool RndfVertex::hasIntersectionOutEdge() const
{
	for (TEdgeSet::const_iterator i=m_outEdges.begin(); i!=m_outEdges.end(); ++i) {
		if ((*i)->intersection() != 0) {
			return true;
		}
	}
	return false;
}

} // namespace RoutePlanner

} // namespace vlr
