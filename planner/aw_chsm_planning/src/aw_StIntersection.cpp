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
#include <aw_StStop.hpp>
#include <aw_StDrive.hpp>
#include <aw_StReplan.hpp>
#include <aw_StIntersection.hpp>
#include <aw_StIntersectionTrafficLightStop.hpp>
#include <aw_StLaneChange.hpp>
#include <aw_StError.hpp>
#include <aw_RndfVertex.h>
#include <aw_RndfEdge.h>

namespace drc = driving_common;

namespace vlr {

/*---------------------------------------------------------------------------
 * StIntersection
 *---------------------------------------------------------------------------*/
StIntersection::StIntersection(my_context ctx) :
  my_base(ctx), StBase<StIntersection>(std::string("StIntersection")), recover_recover_edge(0), inIntersection(false), recover_mode(RECOVER_TO_ENTRY) {
  // entry
  isec_man = new IntersectionManager(*context<ChsmPlanner> ().topology_, *context<ChsmPlanner> ().vehicle_manager,
      context<ChsmPlanner> ().params().max_speed_merge_intersection, context<ChsmPlanner> ().traffic_light_states_,
      context<ChsmPlanner> ().intersection_predictor_mutex_);
  turnDirection = context<ChsmPlanner> ().topology_->nextTurnDirection();
  context<ChsmPlanner> ().topology_->intersection_manager = isec_man; // for visualisation

  max_wait_at_intersection = drc::Time::current();
  max_wait_at_intersection += INTERSECTION_MAX_WAIT_TIME;

  setRecoveryTime(INTERSECTION_RECOVERY_TIMEOUT);
  clearRecoveryIndicator();
}

StIntersection::~StIntersection() {
  context<ChsmPlanner> ().topology_->intersection_manager = NULL;
  delete isec_man;
  context<ChsmPlanner> ().turn_signal_.signal = driving_common::TurnSignal::NONE;
  context<ChsmPlanner> ().bIntersection = false;
}

sc::result StIntersection::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // get the last edge we were on BEFORE the intersection begins. used to recover to this edge
  if (!context<ChsmPlanner> ().topology_->route_is_finished()) {
    RndfEdge* currentEdge = (*context<ChsmPlanner> ().topology_->current_edge_it)->edge();
    if (!inIntersection && currentEdge->intersection() == 0) {
      recover_recover_edge = currentEdge;
    }
    if (currentEdge->intersection()) {
      inIntersection = true;
    }
  }
  //context<ChsmPlanner>().vehiclecmd.turnsignal = context<StIntersection>().turnDirection;
  // forward event
  return forward_event();
}

void StIntersection::generateCurvepoints(const double stop_distance, const double max_speed) {
  ChsmPlanner& planner = context<ChsmPlanner> ();
  double sv_veh_dist, mv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
  //std::pair<double, Vehicle*> nextVehicle = isec_man->distToVehicleOnWayThrouIntersection();
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  //double obstacle_dist = min(nextVehicle.first, std_dist);
  planner.generateCurvePoints(stop_distance, std_dist, max_speed);
}

/*---------------------------------------------------------------------------
 * StIntersectionApproach
 *---------------------------------------------------------------------------*/
StIntersectionApproach::StIntersectionApproach(my_context ctx) :
  my_base(ctx), StBase<StIntersectionApproach>(std::string("StIntersectionApproach")) {

}

StIntersectionApproach::~StIntersectionApproach() {

}

sc::result StIntersectionApproach::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;

  // calculate distances
  // TODO: Make sure stop line is part of this intersection...
  double intersec_dist = topology->distToIntersection(isec_man->intersection());
  CurvePoint stop_point, tl_point;
  double stopline_dist = topology->distToStopLine(isec_man->intersection(), &stop_point); // TODO: make a nicer access to planner...
  double traffic_light_dist = topology->distToTrafficLight(isec_man->intersection(), NULL, &tl_point); // TODO: make a nicer access to planner...

  if (traffic_light_dist < stopline_dist) {
    planner.stop_point_ = tl_point;
  }
  else {
    planner.stop_point_ = stop_point;
  }

  double event_dist = std::min(traffic_light_dist, stopline_dist);

  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  bool onPrio = isec_man->isOnPrio();
  bool hasToStop = isec_man->hasToStop();

  // set intersection flag
  if (intersec_dist < 40) {
    context<ChsmPlanner> ().bIntersection = true;
  }

  // activate turn signal
  if (intersec_dist < TRIGGER_DIST_BLINK) planner.turn_signal_.signal = context<StIntersection> ().turnDirection;

  // Transition: Recovery Mode
  if (isec_man->hasPrioMovement()) {
    context<StIntersection> ().clearRecoveryIndicator();
  }
  // TODO: remove
  if (planner.params().enable_recovery && (context<StIntersection> ().checkRecovery() || isExpired(context<StIntersection> ().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover> ();
  }

  // Transition: Replanning (vehicle is off track)
  if (topology->isOffTrack()) return transit<StReplan> ();

  // Transition: Replanning (route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  // TODO: TRAFFIC LIGHT!!
  // Transition: Queueing
  if (mv_veh_dist < intersec_dist || sv_veh_dist < intersec_dist || (stopline_dist < TRIGGER_DIST_STOPLINE && (mv_veh_dist < stopline_dist || sv_veh_dist
      < stopline_dist))) return transit<StIntersectionQueue> ();

  // Transition: Lanechange
  double dist_to_lanechange = topology->distToNextLaneChange();
  if (dist_to_lanechange >= 0.0 && dist_to_lanechange < TRIGGER_DIST_LANECHANGE) return transit<StLaneChange> ();

  if (traffic_light_dist < stopline_dist) {
    // Transition: Stop at traffic light
    if (hasToStop && traffic_light_dist < TRIGGER_DIST_TRAFFIC_LIGHT && traffic_light_dist > TRAFFIC_LIGHT_DIST_THRESHOLD) {
      return transit<StIntersectionTrafficLightStop> ();
    }
  }
  else {
    // Transition: Stop at stop sign
    if (!onPrio && hasToStop && stopline_dist < TRIGGER_DIST_STOPLINE && stopline_dist > STOP_DIST_THRESHOLD) {
      return transit<StIntersectionStop> ();
    }
    // Transition: Stop at Intersection (because Intersection is blocked)
    if (onPrio && hasToStop && intersec_dist < TRIGGER_DIST_STOPLINE && intersec_dist > STOP_DIST_THRESHOLD) {
      return transit<StIntersectionPrioStop> ();
    }
  }

  // Transition: To get states right if something goes wrong: leave intersection mode if we are behind intersection
  if (intersec_dist <= -0.1) return transit<StDrive> ();

  // Transition: Intersection Drive
  if (onPrio && fabs(intersec_dist) <= 0.01) {
    return transit<StIntersectionPrioDriveInside> ();
  }
  if (!onPrio && fabs(event_dist) <= 0.01) {
    return transit<StIntersectionDriveInside> ();
  }

  double max_speed = planner.params().max_speed_intersection_approach;

  if (onPrio) {
    std::pair<bool, double> prioMerge = isec_man->isPrioOppositeMergeAllowed(planner.params().max_speed_intersection);
    if (!prioMerge.first) {
      event_dist = prioMerge.second;
      max_speed = planner.params().max_speed_intersection;
    }
  }

  if (event_dist < TRIGGER_DIST_BLINK) {
    max_speed = planner.params().max_speed_intersection; // not planner.params().max_speed_intersection_approach ?!?
  }

  // generate curvepoints
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  planner.generateCurvePoints(event_dist, std_dist, max_speed);
  return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionQueue
 *---------------------------------------------------------------------------*/
StIntersectionQueue::StIntersectionQueue(my_context ctx) :
  my_base(ctx), StBase<StIntersectionQueue>(std::string("StIntersectionQueue")) {
  congestionTimeout = drc::Time::current();
  congestionTimeout += RECOVERY_TIMEOUT;
  congestion_last_pose_ = context<ChsmPlanner> ().currentPose();
}

StIntersectionQueue::~StIntersectionQueue() {
}

sc::result StIntersectionQueue::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;

  // calculate distances
  // TODO: Make sure stop line is part of this intersection...
  double intersec_dist = topology->distToIntersection(isec_man->intersection());
  double stopline_dist = topology->distToStopLine(isec_man->intersection(), &planner.stop_point_);
  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  // Transition: Recovery Mode
  if (isec_man->hasPrioMovement()) {
    context<StIntersection> ().clearRecoveryIndicator();
  }

  // set intersection flag
  if (intersec_dist < 40) {
    context<ChsmPlanner> ().bIntersection = true;
  }

  // TODO: remove
  if (planner.params().enable_recovery && (/*intersec_dist < TRIGGER_DIST_INTERSECTION_RECOVER && */context<StIntersection> ().checkRecovery() || isExpired(
      context<StIntersection> ().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover> ();
  }

  // Transition: Replanning (because ego vehicle is off track)
  if (topology->isOffTrack()) return transit<StReplan> ();

  // Transition: Replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  bool onPrio = isec_man->isOnPrio();
  bool hasToStop = isec_man->hasToStop();

  if (intersec_dist < TRIGGER_DIST_BLINK) planner.turn_signal_.signal = context<StIntersection> ().turnDirection;

  // Transition: Intersection Approach (because fwd vehicles disapeared)
  if (intersec_dist < std::min(sv_veh_dist, mv_veh_dist)) return transit<StIntersectionApproach> ();

  // Transition: Lanechange
  double dist_to_lanechange = topology->distToNextLaneChange();
  if (dist_to_lanechange >= 0.0 && dist_to_lanechange < TRIGGER_DIST_LANECHANGE) return transit<StLaneChange> ();

  // Transition: Stop at Stopline
  if (!onPrio && hasToStop && stopline_dist <= TRIGGER_DIST_STOPLINE && stopline_dist >= STOP_DIST_THRESHOLD && stopline_dist <= sv_veh_dist && stopline_dist
      <= mv_veh_dist) return transit<StIntersectionStop> ();

  // Transition: Stop at Intersection (because Intersection is blocked)
  if (onPrio && hasToStop && intersec_dist < TRIGGER_DIST_STOPLINE && intersec_dist > STOP_DIST_THRESHOLD && intersec_dist <= sv_veh_dist && intersec_dist
      <= mv_veh_dist) {
    return transit<StIntersectionPrioStop> ();
  }

  // Transition: Intersection Drive
  if (fabs(intersec_dist) <= 0.01) {
    if (onPrio) {
      return transit<StIntersectionPrioDriveInside> ();
    }
    else {
      return transit<StIntersectionDriveInside> ();
    }
  }

  // Transition: To get states right if something is wrong: leave intersection mode if we are behind intersection
  if (intersec_dist <= -0.1) return transit<StDrive> ();

  if (onPrio) {
    std::pair<bool, double> prioMerge = isec_man->isPrioOppositeMergeAllowed(planner.params().max_speed_intersection);
    if (!prioMerge.first) {
      stopline_dist = prioMerge.second;
    }
  }

  // generate curvepoints
  context<StIntersection> ().generateCurvepoints(stopline_dist, planner.params().max_speed_intersection);

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionStop
 *---------------------------------------------------------------------------*/
StIntersectionStop::StIntersectionStop(my_context ctx) :
  my_base(ctx), StBase<StIntersectionStop>(std::string("StIntersectionStop")) {
  context<ChsmPlanner> ().bIntersection = true;
}

StIntersectionStop::~StIntersectionStop() {
}

sc::result StIntersectionStop::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;
  Topology* topology = context<ChsmPlanner> ().topology_;

  // Transition: Recovery Mode
  if (isec_man->hasPrioMovement()) {
    context<StIntersection> ().clearRecoveryIndicator();
  }
  if (planner.params().enable_recovery && (context<StIntersection> ().checkRecovery() || isExpired(context<StIntersection> ().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover> ();
  }

  // Transition: Replanning (because ego vehicle is off track)
  if (topology->isOffTrack()) return transit<StReplan> ();

  // Transition: Replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  context<StIntersection> ().isec_man->hasToStop(); // for debug visualisation

  // calculate distances
  // TODO: Make sure stop line is part of this intersection...
  double stopline_dist = topology->distToStopLine(isec_man->intersection(), &planner.stop_point_);
  double intersec_dist = topology->distToIntersection(isec_man->intersection());
  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  // set turn signal
  planner.turn_signal_.signal = context<StIntersection> ().turnDirection;

  //	printf("STOP IN %f m;\t CURRENT SPEED %f\n", stopline_dist, planner.currentSpeed());

  // Transition: Wait at Stopline (because ego_vehicle stopped at stopline)
  if (stopline_dist < STOP_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD) {
    return transit<StIntersectionWait> ();
  }
  else if (stopline_dist == std::numeric_limits<double>::infinity()) {
    printf("WE RAN OVER STOP SIGN!!\n");
    return transit<StIntersectionDriveInside> ();
  }

  // Transition: Queueing (in case leading vehicle backed up)
  if (stopline_dist < TRIGGER_DIST_STOPLINE && (mv_veh_dist < stopline_dist || sv_veh_dist < stopline_dist)) return transit<StIntersectionQueue> ();
  // Transition: To get states right if something goes wrong: leave intersection mode if we behind intersection
  if (intersec_dist <= -0.1) {
    return transit<StDrive> ();
  }

  // generate curvepoints
  context<StIntersection> ().generateCurvepoints(stopline_dist, planner.params().max_speed_intersection);
  return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionStop
 *---------------------------------------------------------------------------*/
StIntersectionWait::StIntersectionWait(my_context ctx) :
  my_base(ctx), StBase<StIntersectionWait>(std::string("StIntersectionWait")) {
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;
  isec_man->stoppedOnStopline();
  stop_time = drc::Time::current();
  stop_time += MIN_WAIT_STOPLINE;
  context<StIntersection> ().recover_mode = StIntersection::RECOVER_TO_EXIT;
}

StIntersectionWait::~StIntersectionWait() {
}

sc::result StIntersectionWait::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;

  // Transition: Recovery Mode
  if (isec_man->hasPrioMovement()) {
    context<StIntersection> ().clearRecoveryIndicator();
  }
  if (planner.params().enable_recovery && (context<StIntersection> ().checkRecovery() || isExpired(context<StIntersection> ().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover> ();
  }

  // Transition: Replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  // stop
  planner.generateStopTrajectory();

  planner.turn_signal_.signal = context<StIntersection> ().turnDirection;

  context<StIntersection> ().isec_man->hasToStop(); // for debug visualisation

  //	// Transition: drive on intersection
  //  // We will go if it's our turn and rely on vehicle prediction
  //  // to not run into anybody
  //  //
  //  //  if (isec_man->hasRightOfWay() && isExpired(stop_time)) {
  if (isec_man->hasRightOfWay() && isExpired(stop_time) && !isec_man->isVehicleOnIntersectionInFront()) {
    return transit<StIntersectionDriveInside> ();
  }

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionPrioStop
 *---------------------------------------------------------------------------*/
StIntersectionPrioStop::StIntersectionPrioStop(my_context ctx) :
  my_base(ctx), StBase<StIntersectionPrioStop>(std::string("StIntersectionPrioStop")) {
  context<ChsmPlanner> ().bIntersection = true;
}

StIntersectionPrioStop::~StIntersectionPrioStop() {
}

sc::result StIntersectionPrioStop::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;
  Topology* topology = context<ChsmPlanner> ().topology_;

  bool hasToStop = context<StIntersection> ().isec_man->hasToStop();

  // Transition: Recovery Mode
  if (isec_man->hasPrioMovement()) {
    context<StIntersection> ().clearRecoveryIndicator();
  }
  if (planner.params().enable_recovery && (context<StIntersection> ().checkRecovery() || isExpired(context<StIntersection> ().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover> ();
  }

  // Transition: Replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  // Transition: Replanning (because ego vehicle is off track)
  if (topology->isOffTrack()) return transit<StReplan> ();

  // calculate distances
  double intersection_dist = topology->distToIntersection(isec_man->intersection()); // Für die Reifenstellung
  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  // reduce distance a little in favor of a nice wheel orientation
  if (intersection_dist > 0.) {
    intersection_dist = std::max(intersection_dist - 1.0, 0.);
  }

  planner.turn_signal_.signal = context<StIntersection> ().turnDirection;

  // Transition: To get states right if something goes wrong: leave intersection mode if we are behind intersection
  if (intersection_dist <= -0.1) return transit<StDrive> ();

  if (hasToStop) {

    // Transition: Wait for Prio-Lane (because ego_vehicle stopped at intersection)
    if (intersection_dist < STOP_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD) {
      return transit<StIntersectionPrioWait> ();
    }

    // Transition: Queueing (in case car backed up)
    if (intersection_dist < TRIGGER_DIST_STOPLINE && (mv_veh_dist < intersection_dist || sv_veh_dist < intersection_dist)) return transit<StIntersectionQueue> ();

    // generate curvepoints
    context<StIntersection> ().generateCurvepoints(intersection_dist, planner.params().max_speed_intersection);
  }
  else {
    // Transition: pass intersection on priority lane, no vehicle has priority
    if (intersection_dist < STOP_DIST_THRESHOLD) {
      return transit<StIntersectionPrioDriveInside> ();
    }

    // generate curvepoints without stopline
    context<StIntersection> ().generateCurvepoints(std::numeric_limits<double>::infinity(), planner.params().max_speed_intersection);
  }

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionPrioWait
 *---------------------------------------------------------------------------*/
StIntersectionPrioWait::StIntersectionPrioWait(my_context ctx) :
  my_base(ctx), StBase<StIntersectionPrioWait>(std::string("StIntersectionPrioWait")) {
  context<StIntersection> ().recover_mode = StIntersection::RECOVER_TO_EXIT;
}

StIntersectionPrioWait::~StIntersectionPrioWait() {
}

sc::result StIntersectionPrioWait::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;

  // Transition: Recovery Mode
  if (isec_man->hasPrioMovement()) {
    context<StIntersection> ().clearRecoveryIndicator();
  }
  if (planner.params().enable_recovery && (context<StIntersection> ().checkRecovery() || isExpired(context<StIntersection> ().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover> ();
  }

  // Transition: Replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  // stoppen
  planner.generateStopTrajectory();

  // set turn signal
  planner.turn_signal_.signal = context<StIntersection> ().turnDirection;

  context<StIntersection> ().isec_man->hasToStop(); // for debug visualisation

  // Transition: drive on intersection
  if (isec_man->hasRightOfWay()) return transit<StIntersectionPrioDriveInside> ();

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionDriveInside
 *---------------------------------------------------------------------------*/
StIntersectionDriveInside::StIntersectionDriveInside(my_context ctx) :
  my_base(ctx), StBase<StIntersectionDriveInside>(std::string("StIntersectionDriveInside")) {
  intersectionEntered = false;
  context<StIntersection> ().recover_mode = StIntersection::RECOVER_TO_EXIT;
}

StIntersectionDriveInside::~StIntersectionDriveInside() {
}

sc::result StIntersectionDriveInside::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // Daten holen
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = planner.topology_;
  Vehicle* ego_veh = &topology->ego_vehicle;
  assert(ego_veh->edge());
  RndfEdge* edge = ego_veh->edge();
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;

  //double stopline_dist = topology->distToStopLine(isec_man->intersection(), &stop_point_);
  GraphTools::PlaceOnGraph place(topology->current_edge_it, topology->ego_vehicle.distFromStart(), topology->route.route);
  double stopline_fwd_dist;
  place.go_fwd_to_stopline(stopline_fwd_dist, TRIGGER_DIST_MAX_LOOKAHEAD, isec_man->intersection());
  stopline_fwd_dist -= FRONT_BUMPER_DELTA;
  bool isNearStopline = fabs(stopline_fwd_dist) < 0.3;

  isec_man->hasToStop(); // for debug visualisation

  // Transition: Recovery Mode
  if (!isNearStopline && isec_man->hasPrioMovement()) {
    context<StIntersection> ().clearRecoveryIndicator();
  }
  if (planner.params().enable_recovery && (context<StIntersection> ().checkRecovery() || isExpired(context<StIntersection> ().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover> ();
  }

  // Transition: Replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  //bool onPrio = context<StIntersection>().isec_man->isOnPrio();
  bool isMergeAllowed = !isNearStopline || isec_man->hasRightOfWay();

    // set turn signal
  planner.turn_signal_.signal = context<StIntersection> ().turnDirection;

    // make sure we really entered the intersection (rear axle matching)
  if (!intersectionEntered && edge->intersection()) intersectionEntered = true;

    // Transition: normal driving (we left intersection)
  if (intersectionEntered && !edge->intersection()) return transit<StDrive> ();

  //     We will go if it's our turn and rely on vehicle prediction
  //     to not run into anybody
  //  bool isVehOnIntersection = isec_man->isVehicleOnIntersectionInFront();
  //  if (isVehOnIntersection || !isMergeAllowed) {
  if (!isMergeAllowed) {
    planner.generateStopTrajectory();
  }
  else {
    // generate curvepoints without stopline
    context<StIntersection>().generateCurvepoints(std::numeric_limits<double>::infinity(), planner.params().max_speed_intersection);
  }

  // special case: mission end is the intersection exit vertex
  double dist_to_end = topology->distToMissionEnd();
  if (dist_to_end < TRIGGER_DIST_NASTY_GOAL) {
    return transit<StStop>();
  }

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionPrioDriveInside
 *---------------------------------------------------------------------------*/
StIntersectionPrioDriveInside::StIntersectionPrioDriveInside(my_context ctx) :
  my_base(ctx), StBase<StIntersectionPrioDriveInside>(std::string("StIntersectionPrioDriveInside")) {
  intersectionEntered = false;
  context<StIntersection> ().recover_mode = StIntersection::RECOVER_TO_EXIT;
}

StIntersectionPrioDriveInside::~StIntersectionPrioDriveInside() {
}

sc::result StIntersectionPrioDriveInside::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = planner.topology_;
  Vehicle* ego_veh = &topology->ego_vehicle;
  assert(ego_veh->edge());
  RndfEdge* edge = ego_veh->edge();
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;

  isec_man->hasToStop(); // for debug visualisation

  // Transition: Recovery Mode
  if (isec_man->hasPrioMovement()) {
    context<StIntersection> ().clearRecoveryIndicator();
  }
  if (planner.params().enable_recovery && (context<StIntersection> ().checkRecovery() || isExpired(context<StIntersection> ().max_wait_at_intersection))) { // measure progress in the parent state
    return transit<StIntersectionRecover> ();
  }

  // Transition: Replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  //bool onPrio = context<StIntersection>().isec_man->isOnPrio();
  //bool isVehOnIntersection = context<StIntersection>().isec_man->isVehicleOnIntersectionInFront();

  bool hasToStop = isec_man->isInfrontMergePoint() && isec_man->hasToStop();

  // set turn signal
  planner.turn_signal_.signal = context<StIntersection> ().turnDirection;

  if (hasToStop) {
    planner.generateStopTrajectory();
    return forward_event();
  }

    // make sure we really entered the intersection (rear axle matching)
  if (!intersectionEntered && edge->intersection()) intersectionEntered = true;

  // Transition: normal driving (we left intersection)
  if (intersectionEntered && !edge->intersection()) return transit<StDrive> ();

  double stopline_dist = std::numeric_limits<double>::infinity();

  std::pair<bool, double> prioMerge = isec_man->isPrioOppositeMergeAllowed(planner.params().max_speed_intersection);
  if (!prioMerge.first) {
    stopline_dist = prioMerge.second;
  }

  // generate curvepoints
  context<StIntersection> ().generateCurvepoints(stopline_dist, planner.params().max_speed_intersection);

  // special case: mission end is the intersection exit vertex
  double dist_to_end = topology->distToMissionEnd();
  if (dist_to_end < TRIGGER_DIST_NASTY_GOAL) {
    return transit<StStop> ();
  }

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionRecover
 *---------------------------------------------------------------------------*/
StIntersectionRecover::StIntersectionRecover(my_context ctx) :
  my_base(ctx), StBase<StIntersectionRecover>(std::string("StIntersectionRecover")) {
  map_timer = drc::Time::current();
  map_timer += MIN_WAIT_FOR_OBSTACLEMAP;
}

StIntersectionRecover::~StIntersectionRecover() {
  //   TODO: reactivate gpp / emergency planner
}

sc::result StIntersectionRecover::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  return discard_event(); // intersection states don't get this
}

/*---------------------------------------------------------------------------
 * StIntersectionRecoverPrepare
 *---------------------------------------------------------------------------*/
StIntersectionRecoverPrepare::StIntersectionRecoverPrepare(my_context ctx) :
  my_base(ctx), StBase<StIntersectionRecoverPrepare>(std::string("StIntersectionRecoverPrepare")) {
}

StIntersectionRecoverPrepare::~StIntersectionRecoverPrepare() {
}

sc::result StIntersectionRecoverPrepare::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  if (isExpired(context<StIntersectionRecover> ().map_timer)) {
    return transit<StIntersectionRecoverToExit> ();
  }
  else {
    context<ChsmPlanner> ().generateStopTrajectory();
    return forward_event();
  }
}

/*---------------------------------------------------------------------------
 * StIntersectionRecoverToExit
 *---------------------------------------------------------------------------*/
StIntersectionRecoverToExit::StIntersectionRecoverToExit(my_context ctx) :
  my_base(ctx), StBase<StIntersectionRecoverToExit>(std::string("StIntersectionRecoverToExit")), taredge(0) {
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = planner.topology_;

  throw VLRException("Recover mode not reimplemented..");

  if (context<StIntersection> ().recover_mode == StIntersection::RECOVER_TO_EXIT) {

    bool inIntersection = false;
    RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);

    while (taredge == 0 && anno_edge_it != topology->route.route.end()) {
      assert((*anno_edge_it));
      assert((*anno_edge_it)->edge());
      RndfEdge* edge = (*anno_edge_it)->edge();
      if (!inIntersection && edge->intersection()) {
        inIntersection = true;
      }
      if (inIntersection && (edge->intersection() == 0)) {

        taredge = edge;

        // start plannning
        //   TODO: reactivate gpp / emergency planner

        break;
      }
      ++anno_edge_it;
    }
  }
  else {

    RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);

    while (taredge == 0 && anno_edge_it != topology->route.route.end()) {
      assert((*anno_edge_it));
      assert((*anno_edge_it)->edge());
      RndfEdge* edge = (*anno_edge_it)->edge();
      if ((edge->intersection())) {

        taredge = edge;

        double x = edge->fromVertex()->x();
        double y = edge->fromVertex()->y();
        double psi = atan2(edge->toVertex()->y() - edge->fromVertex()->y(), edge->toVertex()->x() - edge->fromVertex()->x());

        // substract front bumper distance to stop at stopline/intersection
        x -= (FRONT_BUMPER_DELTA + 0.9) * cos(psi);
        y -= (FRONT_BUMPER_DELTA + 0.9) * sin(psi);

         //   TODO: reactivate gpp / emergency planner
        break;
      }
      ++anno_edge_it;
    }
  }
  planner.generateStopTrajectory();

  // reset recover indicator of intersection state
  context<StIntersection> ().clearRecoveryIndicator();
}

StIntersectionRecoverToExit::~StIntersectionRecoverToExit() {
  //   TODO: reactivate gpp / emergency planner
}

sc::result StIntersectionRecoverToExit::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  throw VLRException("Recover mode not reimplemented..");

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = planner.topology_;

  // Transition: Recovery Mode
  if (taredge == 0 || checkRecovery()) { // recover mode has its own recover timer
    planner.addMessage("Recover to exit was not successful");
    return transit<StIntersectionRecoverBackupToEntry> ();
  }

  // check if vehicle is at exit
  RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);

  //   TODO: reactivate gpp / emergency planner
  return transit<StReplan> ();
  return discard_event();
}

/*---------------------------------------------------------------------------
 * StIntersectionRecoverBackupToEntry
 *---------------------------------------------------------------------------*/
StIntersectionRecoverBackupToEntry::StIntersectionRecoverBackupToEntry(my_context ctx) :
  my_base(ctx), StBase<StIntersectionRecoverBackupToEntry>(std::string("StIntersectionRecoverBackupToEntry")) {

  throw VLRException("Recover mode not reimplemented..");

  ChsmPlanner& planner = context<ChsmPlanner>();
  RndfEdge* edge = context<StIntersection>().recover_recover_edge;

  if (edge) {
    //   TODO: reactivate gpp / emergency planner
  }
  planner.generateStopTrajectory();

  // reset recover indicator of intersection state
  context<StIntersection> ().clearRecoveryIndicator();
}

StIntersectionRecoverBackupToEntry::~StIntersectionRecoverBackupToEntry() {
  //   TODO: reactivate gpp / emergency planner
}

sc::result StIntersectionRecoverBackupToEntry::react(const EvProcess&) {

  throw VLRException("Recover mode not reimplemented..");

  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // Transition: Recovery Mode
  if (context<StIntersection> ().recover_recover_edge == 0) {
    context<ChsmPlanner>().addMessage("Recover was not successful -> don't know what to do");
    return transit<StError> ();
  }

  //   TODO: reactivate gpp / emergency planner

  return discard_event();
}

} // namespace vlr

