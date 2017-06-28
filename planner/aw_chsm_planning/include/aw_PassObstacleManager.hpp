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


#ifndef AW_PASSOBSTACLEMANAGER_HPP
#define AW_PASSOBSTACLEMANAGER_HPP

#include <GlobalPose.h>
#include <aw_Vehicle.h>
#include <aw_RndfGraph.h>
#include <aw_MergeFeasabilityCheck.hpp>
#include <aw_Route.h>

namespace vlr {

class Topology;
class VehicleManager;

class PassObstacleManager
{
public:
	PassObstacleManager(Topology* top);
	virtual ~PassObstacleManager();

	bool isInited() const { return inited; };

	// init manager with obstacle to pass
	void setObstacle(Vehicle* veh);

	// get called every cycle, updates internals
	void update();

	// true if there is a neigbor lane
	bool isPassPossible();

	// true if passing is allowed
	bool mayPass(driving_common::GlobalPose pose, double speed, double desired_speed);

	// generates trajectory for passing an obstacle
	void generateTrajectory(driving_common::GlobalPose pose, std::vector<CurvePoint>& center_line, double front_sample_length, double back_sample_length);

	double getFollowingSpeed();

private:
	enum IterationDirection {
		INVALID,
		FORWARD,
		BACKWARD
	};
	bool inited;
	Topology* top;
	Vehicle obstacle; // local copy
	VehicleManager* vman;
	RoutePlanner::RndfEdge* passEdge;
	IterationDirection edgeIterationDirection;
	RoutePlanner::RndfGraph::EdgeMap edges_map;
	RoutePlanner::Route::RouteEdgeList edges_list;
	MergeFeasabilityCheck* mfc;

	void cleanup();
};

} // namespace vlr

#endif // AW_PASSOBSTACLEMANAGER_HPP
