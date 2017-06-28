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


#include <aw_StPause.hpp>
#include <aw_StDrive.hpp>
#include <aw_StReplan.hpp>
#include <aw_StIntersectionTrafficLightWait.hpp>
#include <aw_StIntersectionTrafficLightStop.hpp>

namespace vlr {

/*---------------------------------------------------------------------------
 * StIntersectionTrafficLightStop
 *---------------------------------------------------------------------------*/
StIntersectionTrafficLightStop::StIntersectionTrafficLightStop(my_context ctx) : my_base(ctx), StBase<StIntersectionTrafficLightStop>(std::string("StIntersectionTrafficLightStop"))
{
	context<ChsmPlanner>().bIntersection = true;
}

StIntersectionTrafficLightStop::~StIntersectionTrafficLightStop()
{
}

sc::result StIntersectionTrafficLightStop::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	IntersectionManager* isec_man = context<StIntersection>().isec_man;
	Topology* topology = context<ChsmPlanner>().topology_;
	assert(isec_man);


  // Transition: Recovery Mode
	if (isec_man->hasPrioMovement()) {
		context<StIntersection>().clearRecoveryIndicator();
	}
  if (planner.params().enable_recovery && (context<StIntersection>().checkRecovery() || isExpired(context<StIntersection>().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover>();
  }

  // Transition: Replanning (because ego vehicle is off track)
	if (topology->isOffTrack())
	  return transit<StReplan>();

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();

    // Transition: If traffic light switched to green while slowing down cross intersection
	if(!context<StIntersection>().isec_man->hasToStop()) {
    return transit<StIntersectionDriveInside>();
  }

	// calculate distances
	// TODO: make sure that stop line belongs to intersection
	double traffic_light_dist = topology->distToTrafficLight(isec_man->intersection(), NULL, &planner.stop_point_);
	double intersec_dist = topology->distToIntersection(isec_man->intersection());
	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

	// set turn signal
	planner.turn_signal_.signal = context<StIntersection>().turnDirection;

//	printf("STOP IN %f m;\t CURRENT SPEED %f\n", traffic_light_dist, currentPose().v());

  // Transition: Wait at stop line (because ego_vehicle stopped at stopline)
	if ( traffic_light_dist < TRAFFIC_LIGHT_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD ) {
    return transit<StIntersectionTrafficLightWait>();
	}
	else if (traffic_light_dist == std::numeric_limits<double>::infinity()) {
	  std::cout << "WE RAN OVER A (NON GREEN) TRAFFIC LIGHT!!\n";
    return transit<StIntersectionDriveInside>();
	}

	// Transition: Queueing (if vehicle backed up)
	if ( traffic_light_dist < TRIGGER_DIST_TRAFFIC_LIGHT && (mv_veh_dist < traffic_light_dist || sv_veh_dist < traffic_light_dist) )
		return transit<StIntersectionQueue>();
	// Transition: To get states right if something goes wrong: leave intersection mode if we behind intersection
	if ( intersec_dist <= -0.1 ) {
		return transit<StDrive>();
	}

	// generate curvepoints
	context<StIntersection>().generateCurvepoints(traffic_light_dist, planner.params().max_speed_traffic_light_approach);
	return forward_event();
}

} // namespace vlr

