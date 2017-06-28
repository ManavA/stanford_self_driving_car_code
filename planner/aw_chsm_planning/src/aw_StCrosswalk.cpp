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
#include <aw_StCrosswalk.hpp>
#include <aw_StLaneChange.hpp>
#include <aw_StError.hpp>
#include <aw_RndfVertex.h>
#include <aw_RndfEdge.h>

namespace drc = driving_common;

namespace vlr {

/*---------------------------------------------------------------------------
 * StCrosswalk
 *---------------------------------------------------------------------------*/
StCrosswalk::StCrosswalk(my_context ctx) :
  my_base(ctx), StBase<StCrosswalk>(std::string("StCrosswalk")) {
  cwm_ = new CrosswalkManager(context<ChsmPlanner> ().topology_);
}

StCrosswalk::~StCrosswalk() {
  delete cwm_;
}

sc::result StCrosswalk::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();
  return forward_event();
}

void StCrosswalk::generateCurvepoints(const double stop_distance, const double max_speed) {
  ChsmPlanner& planner = context<ChsmPlanner> ();
  double sv_veh_dist, mv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  //double obstacle_dist = min(nextVehicle.first, std_dist);
  planner.generateCurvePoints(stop_distance, std_dist, max_speed);
}

/*---------------------------------------------------------------------------
 * StCrosswalkApproach
 *---------------------------------------------------------------------------*/
StCrosswalkApproach::StCrosswalkApproach(my_context ctx) :
  my_base(ctx), StBase<StCrosswalkApproach>(std::string("StCrosswalkApproach")) {

}

StCrosswalkApproach::~StCrosswalkApproach() {

}

sc::result StCrosswalkApproach::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // get context data
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  CrosswalkManager* cwm = context<StCrosswalk> ().cwm_;

  CurvePoint cw_point;
  double crosswalk_dist = topology->distToNextCrosswalk(NULL, &cw_point);

  planner.stop_point_ = cw_point;

  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  bool hasToStop = cwm->isOccupied(planner.predicted_pedestrians_);

  // transition: replanning (vehicle is off track)
  if (topology->isOffTrack()) return transit<StReplan> ();

  // transition: replanning (route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  // transition: queueing
  if (mv_veh_dist < crosswalk_dist || sv_veh_dist < crosswalk_dist) return transit<StCrosswalkQueue> ();

  // transition: stop at occupied crosswalk
  if (hasToStop && crosswalk_dist < TRIGGER_DIST_CROSSWALK) { // && crosswalk_dist > CROSSWALK_DIST_THRESHOLD) {
    return transit<StCrosswalkStop> ();
  }

  if (!hasToStop && (crosswalk_dist < 0 || crosswalk_dist == std::numeric_limits<double>::infinity())) { //CROSSWALK_DIST_THRESHOLD) {
    return transit<StDrive> ();
  }

  double max_speed;

  // generate curvepoints
  if (!hasToStop) {
    crosswalk_dist = std::numeric_limits<double>::infinity();
    max_speed = planner.params().max_speed_crosswalk_approach_empty;
  }
  else {
    max_speed = planner.params().max_speed_crosswalk_approach_occupied;
  }
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  planner.generateCurvePoints(crosswalk_dist, std_dist, max_speed);
  return forward_event();
}

/*---------------------------------------------------------------------------
 * StCrosswalkQueue
 *---------------------------------------------------------------------------*/
StCrosswalkQueue::StCrosswalkQueue(my_context ctx) :
  my_base(ctx), StBase<StCrosswalkQueue>(std::string("StCrosswalkQueue")) {
  congestionTimeout = drc::Time::current();
  congestionTimeout += RECOVERY_TIMEOUT;
  congestion_last_pose_ = context<ChsmPlanner> ().currentPose();
}

StCrosswalkQueue::~StCrosswalkQueue() {
}

sc::result StCrosswalkQueue::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // get context data
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  CrosswalkManager* cwm = context<StCrosswalk> ().cwm_;

  CurvePoint cw_point;
  double crosswalk_dist = topology->distToNextCrosswalk(NULL, &cw_point);
  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  //  planner.stop_point_ = cw_point;

  // transition: replanning (because ego vehicle is off track)
  if (topology->isOffTrack()) return transit<StReplan> ();

  // transition: replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  bool hasToStop = cwm->isOccupied(planner.predicted_pedestrians_);

  // transition: crosswalk approach (vehicles ahead disappeared)
  if (crosswalk_dist < std::min(sv_veh_dist, mv_veh_dist)) {
    return transit<StCrosswalkApproach> ();
  }

  // transition: stop at crosswalk
  if (hasToStop && crosswalk_dist <= TRIGGER_DIST_CROSSWALK && crosswalk_dist >= CROSSWALK_DIST_THRESHOLD && crosswalk_dist <= sv_veh_dist && crosswalk_dist
      <= mv_veh_dist) {
    return transit<StCrosswalkStop> ();
  }

  if (!hasToStop && (crosswalk_dist < 0 || crosswalk_dist == std::numeric_limits<double>::infinity())) { //CROSSWALK_DIST_THRESHOLD) {
    return transit<StDrive> ();
  }

  double max_speed;
  if (!hasToStop) {
    max_speed = planner.params().max_speed_crosswalk_approach_empty;
  }
  else {
    max_speed = planner.params().max_speed_crosswalk_approach_occupied;
  }
  // generate curvepoints
  context<StCrosswalk> ().generateCurvepoints(crosswalk_dist, max_speed);

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StCrosswalkStop
 *---------------------------------------------------------------------------*/
StCrosswalkStop::StCrosswalkStop(my_context ctx) :
  my_base(ctx), StBase<StCrosswalkStop>(std::string("StCrosswalkStop")) {
}

StCrosswalkStop::~StCrosswalkStop() {
}

sc::result StCrosswalkStop::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // get context data
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  CrosswalkManager* cwm = context<StCrosswalk> ().cwm_;

  // transition: replanning (because ego vehicle is off track)
  if (topology->isOffTrack()) {
    return transit<StReplan> ();
  }

  // transition: replanning (because route is blocked)
  if (topology->isRouteBlocked()) {
    return transit<StReplan> ();
  }

  CurvePoint cw_point;
  double crosswalk_dist = topology->distToNextCrosswalk(NULL, &cw_point);

  planner.stop_point_ = cw_point;

  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  bool hasToStop = cwm->isOccupied(planner.predicted_pedestrians_);

  // transition: wait at crosswalk (because we stopped already)
  if (hasToStop && crosswalk_dist < CROSSWALK_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD) {
    return transit<StCrosswalkWait> ();
  }
  else if (hasToStop && crosswalk_dist == std::numeric_limits<double>::infinity()) {
    printf("WE RAN OVER A CROSSWALK!!\n");
    return transit<StDrive> ();
  }
  else if (!hasToStop && (crosswalk_dist < 0 || crosswalk_dist == std::numeric_limits<double>::infinity())) { //CROSSWALK_DIST_THRESHOLD) {
    return transit<StDrive> ();
  }

  // transition: queueing (in case vehicle backed up)
  if (crosswalk_dist < TRIGGER_DIST_CROSSWALK && (mv_veh_dist < crosswalk_dist || sv_veh_dist < crosswalk_dist)) {
    return transit<StCrosswalkQueue> ();
  }

  // generate curvepoints
  //	context<StCrosswalk>().generateCurvepoints(crosswalk_dist);
  double max_speed;
  if (!hasToStop) {
    crosswalk_dist = std::numeric_limits<double>::infinity();
    max_speed = planner.params().max_speed_crosswalk_approach_empty;
  }
  else {
    max_speed = planner.params().max_speed_crosswalk_approach_occupied;
  }
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  planner.generateCurvePoints(crosswalk_dist, std_dist, max_speed);
  return forward_event();
}

/*---------------------------------------------------------------------------
 * StCrosswalkWait
 *---------------------------------------------------------------------------*/
StCrosswalkWait::StCrosswalkWait(my_context ctx) :
  my_base(ctx), StBase<StCrosswalkWait>(std::string("StCrosswalkWait")) {
  //	stop_time.now();
  //	stop_time += MIN_WAIT_STOPLINE;
}

StCrosswalkWait::~StCrosswalkWait() {
}

sc::result StCrosswalkWait::react(const EvProcess&) {
  if (detectedErrornousTransitions()) {
    return transit<StGlobalRecover> ();
  }

  // get context data
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  CrosswalkManager* cwm = context<StCrosswalk> ().cwm_;

  // Transition: Replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  // stop
  planner.generateStopTrajectory();

  bool hasToStop = cwm->isOccupied(planner.predicted_pedestrians_);

  // Transition: drive on intersection
  if (!hasToStop) { // && isExpired(stop_time) && !isec_man->isVehicleOnCrosswalkInFront()) {
    return transit<StDrive> ();
  }

  return forward_event();
}

} // namespace vlr

