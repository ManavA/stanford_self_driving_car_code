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


#ifndef AW_LANECHANGEMANAGER_H
#define AW_LANECHANGEMANAGER_H

#include <aw_RndfGraph.h>
#include <aw_Topology.hpp>
#include <aw_VehicleManager.hpp>

namespace vlr {

class LaneChangeManager {

public:
	static void annotateLaneChanges(Topology* top); // requires planned mission. propagate planned lane changes as far as possible, so that vehicle has a bigger region to perform lane change
	// when executing the mission, this allows for an easy check to detect a required lane change as early as possible by checking for   currentAnnotatedRouteEdge.laneChange != AnnotatedRouteEdge::LC_NONE (not defined yet)

	LaneChangeManager(Topology* top, VehicleManager* vman);
	~LaneChangeManager();


protected:
	Topology* top;
	RndfGraph* graph;
	VehicleManager* vman;
};

} // namespace vlr

#endif // AW_LANECHANGEMANAGER_H
