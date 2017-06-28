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
#include <aw_StLaneChange.hpp>
//#include <aw_StIntersectionTrafficLightWait.hpp>
#include <aw_StIntersectionTrafficLightStop.hpp>
#include <aw_StIntersectionTrafficLightQueue.hpp>

namespace drc = driving_common;

namespace vlr {

/*---------------------------------------------------------------------------
 * StIntersectionTrafficLightQueue
 *---------------------------------------------------------------------------*/
StIntersectionTrafficLightQueue::StIntersectionTrafficLightQueue(my_context ctx) : my_base(ctx), StBase<StIntersectionTrafficLightQueue>(std::string("StIntersectionTrafficLightQueue"))
{
//	context<ChsmPlanner>().bIntersection = true; // ?!?
  congestionTimeout = drc::Time::current();
  congestionTimeout += RECOVERY_TIMEOUT;
  congestion_last_pose_ = context<ChsmPlanner>().currentPose();
}

StIntersectionTrafficLightQueue::~StIntersectionTrafficLightQueue()
{
}

sc::result StIntersectionTrafficLightQueue::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology_;
  IntersectionManager* isec_man = context<StIntersection>().isec_man;

  // calculate distances
  double traffic_light_dist = topology->distToTrafficLight(isec_man->intersection(), NULL, &planner.stop_point_);
  double intersec_dist = topology->distToIntersection(isec_man->intersection());
  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  // Transition: Recovery Mode
	if (isec_man->hasPrioMovement()) {
		context<StIntersection>().clearRecoveryIndicator();
	}

  // set intersection flag
  if ( intersec_dist < 40) {
    context<ChsmPlanner>().bIntersection = true;
  }

  // TODO: remove
  if (planner.params().enable_recovery && (/*intersec_dist < TRIGGER_DIST_INTERSECTION_RECOVER && */context<StIntersection>().checkRecovery() || isExpired(context<StIntersection>().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover>();
  }

  // Transition: Replanning (because ego vehicle is off track)
	if (topology->isOffTrack())
	  return transit<StReplan>();

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked() )
		return transit<StReplan>();


  bool onPrio = isec_man->isOnPrio();
  bool hasToStop = isec_man->hasToStop();

  // set turn signal
  if ( intersec_dist < TRIGGER_DIST_BLINK )
    planner.turn_signal_.signal = context<StIntersection>().turnDirection;

  // Transition: Intersection Approach (because fwd vehicles disappeared)
  if ( intersec_dist < std::min(sv_veh_dist, mv_veh_dist) ) {
    return transit<StIntersectionApproach>();
  }

  // Transition: Lanechange
  double dist_to_lanechange = topology->distToNextLaneChange();
  if ( dist_to_lanechange >= 0.0 && dist_to_lanechange < TRIGGER_DIST_LANECHANGE)
    return transit<StLaneChange>();


  // transition: stop at traffic light
  if ( !onPrio && hasToStop && traffic_light_dist <= TRIGGER_DIST_TRAFFIC_LIGHT && traffic_light_dist >= TRAFFIC_LIGHT_DIST_THRESHOLD &&
      traffic_light_dist <= sv_veh_dist && traffic_light_dist <= mv_veh_dist)
    return transit<StIntersectionTrafficLightStop>();

  // Transition: stop at intersection (because Intersection is blocked)
  if ( onPrio && hasToStop && intersec_dist < TRIGGER_DIST_TRAFFIC_LIGHT && intersec_dist > TRAFFIC_LIGHT_DIST_THRESHOLD &&
      intersec_dist <= sv_veh_dist && intersec_dist <= mv_veh_dist) {
    return transit<StIntersectionPrioStop>();
  }

  // Transition: Intersection Drive
  if ( fabs(intersec_dist) <= 0.01 ) {
    if (onPrio) {
      return transit<StIntersectionPrioDriveInside>();
    } else {
      return transit<StIntersectionDriveInside>();
    }
  }

  // Transition: To get states right if something goes wrong: leave intersection mode if we behind intersection
  if ( intersec_dist <= -0.1 )
    return transit<StDrive>();

  if ( onPrio ) {
    std::pair< bool, double > prioMerge = isec_man->isPrioOppositeMergeAllowed(planner.params().max_speed_intersection);
    if (!prioMerge.first) {
      traffic_light_dist = prioMerge.second;
    }
  }

  // generate curvepoints
  context<StIntersection>().generateCurvepoints(traffic_light_dist, planner.params().max_speed_traffic_light_approach);
  return forward_event();
}

} // namespace vlr

