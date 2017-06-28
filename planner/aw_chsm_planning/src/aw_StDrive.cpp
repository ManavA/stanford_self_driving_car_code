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
#include <aw_StIntersection.hpp>
#include <aw_StStop.hpp>
#include <aw_StDrive.hpp>
#include <aw_StZone.hpp>
#include <aw_StTrafficLight.hpp>
#include <aw_StCrosswalk.hpp>
#include <aw_StReplan.hpp>
#include <aw_StLaneChange.hpp>

namespace drc = driving_common;

namespace vlr {

//#define TRACE(x) 	std::cout << context<ChsmPlanner>().name() << " [StDrive] " << x << std::endl;
#define TRACE(x)

/*---------------------------------------------------------------------------
 * StDrive
 *---------------------------------------------------------------------------*/
StDrive::StDrive(my_context ctx) : my_base(ctx), StBase<StDrive>(std::string("StDrive"))
{
}

StDrive::~StDrive() {
}

sc::result StDrive::react(const EvStop&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  return transit<StStop>();
}

sc::result StDrive::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = planner.topology_;

  double dist_to_missionend = topology->distToMissionEnd();
  CurvePoint cp; //dummy point: not forwarded to planner since in this stage it's unclear which the next stop point will be
  double intersection_dist = std::min(topology->distToNextIntersection(), topology->distToNextStopLine());
//  printf("distances:\n zone:\t\t%f\nstop:\t\t%f\nisec:\t\t%f\nkturn:\t\t%f\nlach:\t\t%f\ntrali:\t\t%f\ncrow:\t\t%f\n\n",
//      topology->distToNextZone(), topology->distToNextSoleStopLine(cp),
//      topology->distToNextIntersection(), topology->distToNextKTurn(),
//      topology->distToNextLaneChange(), topology->distToNextTrafficLight(),
//      topology->distToNextCrosswalk());
  // select minimum trigger dist
  // ATTN: order does matter!
  // ATTN: also add transition in StDriveStart
  std::map<double, TransitionEnum> transit_map;
  transit_map[topology->distToNextZone()]         = TRANSIT_TO_ZONE;
  transit_map[topology->distToNextSoleStopLine(cp)]= TRANSIT_TO_STOPLINE;
  transit_map[intersection_dist] = TRANSIT_TO_INTERSECTION;
  transit_map[topology->distToNextKTurn()]        = TRANSIT_TO_KTURN;
//  transit_map[topology->distToNextLaneChange()]   = TRANSIT_TO_LANECHANGE;
  transit_map[topology->distToNextTrafficLight()]   = TRANSIT_TO_TRAFFIC_LIGHT;
  transit_map[topology->distToNextCrosswalk()]   = TRANSIT_TO_CROSSWALK;
  transit_map[dist_to_missionend]                    = TRANSIT_TO_GOAL;

  // failsafe, would be really bad to go in a wrong state at mission end
  if (dist_to_missionend < 0.0) {
  	return transit<StStop>();
  }

	double min_dist = transit_map.begin()->first;
	switch (transit_map.begin()->second) {
	case TRANSIT_TO_GOAL: if (min_dist < TRIGGER_DIST_GOAL) return transit<StStop>(); break;
	case TRANSIT_TO_ZONE: if (min_dist > 0.0 && min_dist < TRIGGER_DIST_ENTERING_ZONE) return transit<StZone>(); break;
	case TRANSIT_TO_KTURN: if (/*min_dist >= 0.0 && */min_dist < TRIGGER_DIST_KTURN) return transit<StDriveKTurn>(); break;
	case TRANSIT_TO_INTERSECTION: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_APPROACH_INTERSECTION) return transit<StIntersection>(); break;
	case TRANSIT_TO_LANECHANGE: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_LANECHANGE) return transit<StLaneChange>(); break;
  case TRANSIT_TO_STOPLINE: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_STOPLINE) return transit<StDriveStop>(); break;
  case TRANSIT_TO_TRAFFIC_LIGHT: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_APPROACH_TRAFFIC_LIGHT) return transit<StTrafficLight>(); break;
  case TRANSIT_TO_CROSSWALK: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_APPROACH_CROSSWALK) return transit<StCrosswalk>(); break;
	case TRANSIT_DEFAULT: default: break;
	}

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StDriveStart
 *---------------------------------------------------------------------------*/
StDriveStart::StDriveStart(my_context ctx) : my_base(ctx), StBase<StDriveStart>(std::string("StDriveStart"))
{
}

StDriveStart::~StDriveStart() {
}

sc::result StDriveStart::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) {return transit<StGlobalRecover>();}

  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = planner.topology_;

  double dist_to_missionend = topology->distToMissionEnd();
  double intersection_dist = std::min(topology->distToNextIntersection(), topology->distToNextStopLine());
  CurvePoint cp; //dummy point: not forwarded to planner since in this stage it's unclear which the next stop point will be

  // select minimum trigger dist
  std::map<double, TransitionEnum> transit_map;
  transit_map[topology->distToNextZone()]         = TRANSIT_TO_ZONE;
  transit_map[topology->distToNextSoleStopLine(cp)]= TRANSIT_TO_STOPLINE;
  transit_map[intersection_dist] = TRANSIT_TO_INTERSECTION;
  transit_map[topology->distToNextKTurn()]        = TRANSIT_TO_KTURN;
//  transit_map[topology->distToNextLaneChange()]   = TRANSIT_TO_LANECHANGE;
  transit_map[topology->distToNextTrafficLight()]   = TRANSIT_TO_TRAFFIC_LIGHT;
  transit_map[topology->distToNextCrosswalk()]   = TRANSIT_TO_CROSSWALK;
  transit_map[dist_to_missionend]                    = TRANSIT_TO_GOAL;

//  printf("distance to:\nzone: %f\nsole stop line: %f\nintersection: %f\nkturn: %f\ntraffic light: %f\n crosswalk: %f\n EOM: %f\n\n",
//          topology->distToNextZone(), topology->distToNextSoleStopLine(cp), topology->distToNextIntersection(),
//          topology->distToNextKTurn(), topology->distToNextTrafficLight(), topology->distToNextCrosswalk(),dist_to_missionend);
  	// TODO: Transition *from* lane change

  // failsafe, would be really bad to go in a wrong state at mission end
  if (dist_to_missionend < 0.0) {
  	return transit<StStop>();
  }

  bool inZone = (*topology->current_edge_it)->edge()->isZoneEdge() && !(*topology->current_edge_it)->hasAnnotation(UC_MANEUVER_ZONE_EXIT);

	double min_dist = transit_map.begin()->first;
	switch (transit_map.begin()->second) {
	case TRANSIT_TO_GOAL: if (min_dist < TRIGGER_DIST_GOAL) return transit<StStop>(); break;
	case TRANSIT_TO_ZONE: if ( (min_dist > 0.0 && min_dist < TRIGGER_DIST_ENTERING_ZONE) || inZone) return transit<StZone>(); break;
	case TRANSIT_TO_KTURN: if (min_dist < TRIGGER_DIST_KTURN) return transit<StDriveKTurn>(); break;
    case TRANSIT_TO_INTERSECTION: if (min_dist < TRIGGER_DIST_APPROACH_INTERSECTION) return transit<StIntersection>(); break;
	case TRANSIT_TO_LANECHANGE: if (min_dist >= 0.0 && min_dist < TRIGGER_DIST_LANECHANGE) return transit<StLaneChange>(); break;
	case TRANSIT_TO_STOPLINE:
		if (dist_to_missionend < TRIGGER_DIST_GOAL) return transit<StStop>(); break; // special case: mission end is a stoppoint
		if (min_dist < TRIGGER_DIST_STOPLINE && min_dist >= STOP_DIST_THRESHOLD) return transit<StDriveStop>();
		break;
	case TRANSIT_TO_TRAFFIC_LIGHT: if (min_dist < TRIGGER_DIST_APPROACH_TRAFFIC_LIGHT) return transit<StTrafficLight>(); break;
	case TRANSIT_TO_CROSSWALK: if (min_dist < TRIGGER_DIST_APPROACH_CROSSWALK) return transit<StCrosswalk>(); break;
	case TRANSIT_DEFAULT: default: return transit<StDriveOnLane>(); break;
	}

	return transit<StDriveOnLane>(); // this case DOES happen !? - so don't delete this line
}

/*---------------------------------------------------------------------------
 * StDriveOnLane
 *---------------------------------------------------------------------------*/
StDriveOnLane::StDriveOnLane(my_context ctx) : my_base(ctx), StBase<StDriveOnLane>(std::string("StDriveOnLane"))
{
	setRecoveryTime(RECOVERY_TIMEOUT_DRIVEONLANE);
	clearRecoveryIndicator();
}

StDriveOnLane::~StDriveOnLane() {
}

sc::result StDriveOnLane::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* top = planner.topology_;

  // Transition: Replanning (because ego vehicle is off track)
  if ( top->isOffTrack() )
	  return transit<StReplan>();

  // Transition: Replanning (because route is blocked)
  if ( top->isRouteBlocked() )
	  return transit<StReplan>();

  // Transition: Recover if we do not make progress
  if ( checkRecovery() ) {
	  if ( top->isRouteBlocked( FROG_MODE_LEAP_DIST, false ) )
		  return transit<StReplan>();
	  else
		  return transit<StDriveRecover>();
  }

  // Transition: lane change
  // TODO: replace with trajectory test
	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

//	if (sv_veh_dist < mv_veh_dist && sv_veh_dist < TRIGGER_DIST_OBSTACLE && planner.currentPose().v() < STOP_SPEED_THRESHOLD) {
//		return transit<StLaneChange>();
//	}

//	printf("mv_veh_dist: %f, sv_veh_dist: %f\n", mv_veh_dist, sv_veh_dist);
	double veh_dist = std::min(mv_veh_dist, sv_veh_dist);
//	TRACE("veh_dist = " << veh_dist << " ~ " << top->distToNextStandingVehicle());
	// generate curvepoints
  planner.generateCurvePoints(std::numeric_limits<double>::infinity(), veh_dist, planner.params().max_speed_drive);

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StDriveRecover
 *---------------------------------------------------------------------------*/
StDriveRecover::StDriveRecover(my_context ctx) : my_base(ctx), StBase<StDriveRecover>(std::string("StDriveRecover"))
{
	map_timer = drc::Time::current();
	map_timer += MIN_WAIT_FOR_OBSTACLEMAP;
}

StDriveRecover::~StDriveRecover() {
  //   TODO: reactivate gpp / emergency planner
}

sc::result StDriveRecover::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	return discard_event(); // intersection states don't get this
}

/*---------------------------------------------------------------------------
 * StDriveRecoverPrepare
 *---------------------------------------------------------------------------*/
StDriveRecoverPrepare::StDriveRecoverPrepare(my_context ctx) : my_base(ctx), StBase<StDriveRecoverPrepare>(std::string("StDriveRecoverPrepare"))
{
}

StDriveRecoverPrepare::~StDriveRecoverPrepare() {
}

sc::result StDriveRecoverPrepare::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  context<ChsmPlanner>().generateStopTrajectory();
  return forward_event();
}

/*---------------------------------------------------------------------------
 * StDriveStop
 *---------------------------------------------------------------------------*/
StDriveStop::StDriveStop(my_context ctx) : my_base(ctx), StBase<StDriveStop>(std::string("StDriveStop"))
{
}

StDriveStop::~StDriveStop() {
}

sc::result StDriveStop::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = planner.topology_;

  // Transition: Replanning (because ego vehicle is off track)
  if (topology->isOffTrack())
	  return transit<StReplan>();

  // Transition: Replanning (because route is blocked)
  if ( topology->isRouteBlocked() )
	  return transit<StReplan>();

  double stopline_dist = topology->distToNextSoleStopLine(planner.stop_point_);  // TODO: make a nicer access to planner...
	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
	double veh_dist = std::min(mv_veh_dist, sv_veh_dist);

	if (stopline_dist < STOP_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD) {
		return transit<StDriveStopped>();
	} else {

	  // Transition: Recover if we do not make progress
	  if (checkRecovery()) {
	  	return transit<StDriveRecover>();
	  }

		planner.generateCurvePoints(stopline_dist, veh_dist-1.0, planner.params().max_speed_intersection_approach); // TODO: following distance?
		//return forward_event();
		return discard_event();
	}

}

/*---------------------------------------------------------------------------
 * StDriveStopped
 *---------------------------------------------------------------------------*/
StDriveStopped::StDriveStopped(my_context ctx) : my_base(ctx), StBase<StDriveStopped>(std::string("StDriveStopped"))
{
	stop_timer = drc::Time::current();
	stop_timer += MIN_WAIT_STOPLINE;
}

StDriveStopped::~StDriveStopped() {
}

sc::result StDriveStopped::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  ChsmPlanner& planner = context<ChsmPlanner>();

	if (isExpired(stop_timer)) {
		return transit<StDrive>();
	} else {
		planner.generateStopTrajectory();
		return discard_event();
	}

}

} // namespace vlr
