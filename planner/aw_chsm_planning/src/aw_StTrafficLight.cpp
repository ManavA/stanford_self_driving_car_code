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

#include <driving_common/TrafficLightPose.h>

#include <aw_StPause.hpp>
#include <aw_StDrive.hpp>
#include <aw_StReplan.hpp>
#include <aw_StTrafficLight.hpp>

namespace drc = driving_common;

namespace vlr {

/*---------------------------------------------------------------------------
 * StTrafficLight
 *---------------------------------------------------------------------------*/
StTrafficLight::StTrafficLight(my_context ctx) :
  my_base(ctx), StBase<StTrafficLight> (std::string("StTrafficLight")) {

  try {
    tlm_ = new TrafficLightManager(context<ChsmPlanner> ().topology_);
  }
  catch (vlr::Ex<>& e) {
    throw e;
  }

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;

  std::vector < std::string > tl_names;
  topology->distToNextTrafficLight(&tl_names, NULL);
  std::vector<std::string>::const_iterator tlnit = tl_names.begin(), tlnit_end = tl_names.end();
  driving_common::TrafficLightPose tl_pose;

  pthread_mutex_lock(&planner.traffic_light_poses_mutex_);
  planner.publish_traffic_lights_ = true;
  planner.traffic_light_poses_.clear();

  for (; tlnit != tlnit_end; tlnit++) {
    rndf::TrafficLight* tl = const_cast<rndf::RoadNetwork*> (&topology->roadNetwork())->trafficLight(*tlnit);
    if (tl) {
      tl_pose.lat = tl->lat();
      tl_pose.lon = tl->lon();
      tl_pose.z = tl->z();
      tl_pose.orientation = tl->orientation();
      tl_pose.name = tl->name();

      planner.traffic_light_poses_.push_back(tl_pose);
    }
  }

  planner.publish_traffic_lights_ = true;

  pthread_mutex_unlock(&planner.traffic_light_poses_mutex_);

  //  max_wait_at_intersection.now();
  //  max_wait_at_intersection += INTERSECTION_MAX_WAIT_TIME;
  //
  //  setRecoveryTime(INTERSECTION_RECOVERY_TIMEOUT);
  //  clearRecoveryIndicator();
}

StTrafficLight::~StTrafficLight() {
  ChsmPlanner& planner = context<ChsmPlanner> ();
  pthread_mutex_lock(&planner.traffic_light_poses_mutex_);
  planner.publish_traffic_lights_ = false;
  planner.traffic_light_poses_.clear();
  pthread_mutex_unlock(&planner.traffic_light_poses_mutex_);
  delete tlm_;
}

sc::result StTrafficLight::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();
  return forward_event();
}

void StTrafficLight::generateCurvepoints(const double stop_distance, const double max_speed) {
  ChsmPlanner& planner = context<ChsmPlanner> ();
  double sv_veh_dist, mv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  //double obstacle_dist = min(nextVehicle.first, std_dist);
  planner.generateCurvePoints(stop_distance, std_dist, max_speed);
}

/*---------------------------------------------------------------------------
 * StTrafficLightApproach
 *---------------------------------------------------------------------------*/
StTrafficLightApproach::StTrafficLightApproach(my_context ctx) :
  my_base(ctx), StBase<StTrafficLightApproach> (std::string("StTrafficLightApproach")) {

}

StTrafficLightApproach::~StTrafficLightApproach() {

}

sc::result StTrafficLightApproach::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // get context data
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  TrafficLightManager* tlm = context<StTrafficLight> ().tlm_;

  CurvePoint tl_point;
  double traffic_light_dist = topology->distToNextTrafficLight(NULL, &tl_point);

  planner.stop_point_ = tl_point;

  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  bool hasToStop = tlm->hasToStop(planner.traffic_light_states_);

  // transition: replanning (vehicle is off track)
  if (topology->isOffTrack()) return transit<StReplan> ();

  // transition: replanning (route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  // transition: queueing
  if (mv_veh_dist < traffic_light_dist || sv_veh_dist < traffic_light_dist) return transit<StTrafficLightQueue> ();

  // transition: stop at occupied traffic light
  if (hasToStop && traffic_light_dist < TRIGGER_DIST_TRAFFIC_LIGHT) { // && traffic_light_dist > TRAFFIC_LIGHT_DIST_THRESHOLD) {
    return transit<StTrafficLightStop> ();
  }

  if (!hasToStop && (traffic_light_dist < 0 || traffic_light_dist == std::numeric_limits<double>::infinity())) { //TRAFFIC_LIGHT_DIST_THRESHOLD) {
    return transit<StDrive> ();
  }

  double max_speed = planner.params().max_speed_traffic_light_approach;

  // generate curve points
  if (!hasToStop) {
    traffic_light_dist = std::numeric_limits<double>::infinity();
  }
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  planner.generateCurvePoints(traffic_light_dist, std_dist, max_speed);
  return forward_event();
}

/*---------------------------------------------------------------------------
 * StTrafficLightQueue
 *---------------------------------------------------------------------------*/
StTrafficLightQueue::StTrafficLightQueue(my_context ctx) :
  my_base(ctx), StBase<StTrafficLightQueue> (std::string("StTrafficLightQueue")) {

  congestionTimeout = drc::Time::current() + RECOVERY_TIMEOUT;
  congestion_last_pose_ = context<ChsmPlanner> ().currentPose();
}

StTrafficLightQueue::~StTrafficLightQueue() {
}

sc::result StTrafficLightQueue::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // get context data
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  TrafficLightManager* tlm = context<StTrafficLight> ().tlm_;

  CurvePoint tl_point;
  double traffic_light_dist = topology->distToNextTrafficLight(NULL, &tl_point);
  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  //  planner.stop_point_ = tl_point;

  // transition: replanning (because ego vehicle is off track)
  if (topology->isOffTrack()) return transit<StReplan> ();

  // transition: replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  bool hasToStop = tlm->hasToStop(planner.traffic_light_states_);

  // transition: traffic light approach (vehicles ahead disappeared)
  if (traffic_light_dist < std::min(sv_veh_dist, mv_veh_dist)) {
    return transit<StTrafficLightApproach> ();
  }

  // transition: stop at traffic light
  if (hasToStop && traffic_light_dist <= TRIGGER_DIST_TRAFFIC_LIGHT && traffic_light_dist >= TRAFFIC_LIGHT_DIST_THRESHOLD && traffic_light_dist <= sv_veh_dist
      && traffic_light_dist <= mv_veh_dist) {
    return transit<StTrafficLightStop> ();
  }

  if (!hasToStop && (traffic_light_dist < 0 || traffic_light_dist == std::numeric_limits<double>::infinity())) { //TRAFFIC_LIGHT_DIST_THRESHOLD) {
    return transit<StDrive> ();
  }

  // generate curve points
  context<StTrafficLight> ().generateCurvepoints(traffic_light_dist, planner.params().max_speed_traffic_light_approach);

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StTrafficLightStop
 *---------------------------------------------------------------------------*/
StTrafficLightStop::StTrafficLightStop(my_context ctx) :
  my_base(ctx), StBase<StTrafficLightStop>(std::string("StTrafficLightStop")) {
}

StTrafficLightStop::~StTrafficLightStop() {
}

sc::result StTrafficLightStop::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // get context data
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  TrafficLightManager* tlm = context<StTrafficLight> ().tlm_;

  // transition: replanning (because ego vehicle is off track)
  if (topology->isOffTrack()) {
    return transit<StReplan> ();
  }

  // transition: replanning (because route is blocked)
  if (topology->isRouteBlocked()) {
    return transit<StReplan> ();
  }

  CurvePoint tl_point;
  double traffic_light_dist = topology->distToNextTrafficLight(NULL, &tl_point);

  planner.stop_point_ = tl_point;

  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  bool hasToStop = tlm->hasToStop(planner.traffic_light_states_);

  // transition: wait at traffic light (because we stopped already)
  if (hasToStop && traffic_light_dist < TRAFFIC_LIGHT_DIST_THRESHOLD && planner.currentPose().v() < STOP_SPEED_THRESHOLD) {
    return transit<StTrafficLightWait> ();
  }
  else if (hasToStop && traffic_light_dist == std::numeric_limits<double>::infinity()) {
    printf("WE RAN OVER A TRAFFIC LIGHT!!\n");
    return transit<StDrive> ();
  }
  else if ((traffic_light_dist < 0 || traffic_light_dist == std::numeric_limits<double>::infinity())) { //TRAFFIC_LIGHT_DIST_THRESHOLD) {
    return transit<StDrive> ();
  }

  // transition: queueing (in case vehicle backed up)
  if (traffic_light_dist < TRIGGER_DIST_TRAFFIC_LIGHT && (mv_veh_dist < traffic_light_dist || sv_veh_dist < traffic_light_dist)) {
    return transit<StTrafficLightQueue> ();
  }

  // generate curvepoints
  //  context<StTrafficLight>().generateCurvepoints(traffic_light_dist);
  if (!hasToStop) {
    traffic_light_dist = std::numeric_limits<double>::infinity();
  }
  double std_dist = std::min(sv_veh_dist, mv_veh_dist);
  planner.generateCurvePoints(traffic_light_dist, std_dist, planner.params().max_speed_traffic_light_approach);
  return forward_event();
}

/*---------------------------------------------------------------------------
 * StTrafficLightWait
 *---------------------------------------------------------------------------*/
StTrafficLightWait::StTrafficLightWait(my_context ctx) :
  my_base(ctx), StBase<StTrafficLightWait>(std::string("StTrafficLightWait")) {
  //  stop_time.now();
  //  stop_time += MIN_WAIT_STOPLINE;
}

StTrafficLightWait::~StTrafficLightWait() {
}

sc::result StTrafficLightWait::react(const EvProcess&) {
  if (detectedErrornousTransitions()) {
    return transit<StGlobalRecover> ();
  }

  // get context data
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  TrafficLightManager* tlm = context<StTrafficLight> ().tlm_;

  // Transition: Replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  // stopping
  planner.generateStopTrajectory();

  bool hasToStop = tlm->hasToStop(planner.traffic_light_states_);
  CurvePoint tl_point;
  double traffic_light_dist = topology->distToNextTrafficLight(NULL, &tl_point);

  // Transition: drive on intersection
  if (!hasToStop || traffic_light_dist < -.5 || traffic_light_dist == std::numeric_limits<double>::infinity()) { // && isExpired(stop_time) && !isec_man->isVehicleOnCrosswalkInFront()) {
    return transit<StDrive> ();
  }

  return forward_event();
}

} // namespace vlr

