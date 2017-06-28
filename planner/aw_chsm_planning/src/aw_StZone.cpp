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

#include <aw_graph_tools.hpp>

#include <aw_StZone.hpp>
#include <aw_StPause.hpp>

namespace drc = driving_common;

namespace vlr {

/*---------------------------------------------------------------------------
 * StZone
 *---------------------------------------------------------------------------*/
StZone::StZone(my_context ctx) :
  my_base(ctx), StBase<StZone> (std::string("StZone")), no_perimeter_points(0) {
  //   TODO: reactivate gpp
  throw VLRException("GPP mode not reimplemented..");

  ChsmPlanner& planner = context<ChsmPlanner> ();

  // extract all parking spots
  Topology* topology = planner.topology_;
  RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);
  RoutePlanner::Route::RouteEdgeList::iterator scan_for_zone_it(anno_edge_it);

  RoutePlanner::AnnotatedRouteEdge* entry_edge = topology->next_edge_with_maneuver(UC_MANEUVER_ZONE_ENTRY);
  RoutePlanner::AnnotatedRouteEdge* exit_edge = topology->next_edge_with_maneuver(UC_MANEUVER_ZONE_EXIT, 100000);

  std::string entry_point_name = "";
  if (entry_edge) entry_point_name = entry_edge->edge()->toVertex()->name();
  if (exit_edge) entry_point_name = exit_edge->edge()->fromVertex()->name();

  while (anno_edge_it != topology->route.route.end()) {
    // add park maneuver
    if ((*anno_edge_it)->hasAnnotation(UC_MANEUVER_PARKING)) {
      assert((*anno_edge_it)->edge()->isZoneEdge());
      zone_maneuvers.push_back(*anno_edge_it);
    }
    // add exit maneuver
    if ((*anno_edge_it)->hasAnnotation(UC_MANEUVER_ZONE_EXIT)) {
      assert((*anno_edge_it)->edge()->isZoneEdge());
      zone_maneuvers.push_back(*anno_edge_it);
      break;
    }
    // add mission end
    if ((*anno_edge_it)->hasAnnotation(UC_MANEUVER_GOAL_REACHED)) {
      assert((*anno_edge_it)->edge()->isZoneEdge());
      zone_maneuvers.push_back(*anno_edge_it);
      break;
    }
    ++anno_edge_it;
  }
}

StZone::~StZone() {
  //   TODO: reactivate gpp
  throw VLRException("Recover mode not reimplemented..");
}

sc::result StZone::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();
  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneApproach
 *---------------------------------------------------------------------------*/
StZoneApproach::StZoneApproach(my_context ctx) :
  my_base(ctx), StBase<StZoneApproach> (std::string("StZoneApproach")) {

}

StZoneApproach::~StZoneApproach() {

}

sc::result StZoneApproach::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;

  // calculate distances
  double zone_dist = topology->distToNextZone();
  double mv_veh_dist, sv_veh_dist;
  planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);

  // Transition: Intersection Drive
  if (std::abs(zone_dist) <= 1.0) return transit<StZoneEntering> ();

  // generate curvepoints
  double stop_distance = zone_dist;
  double follow_distance = std::min(sv_veh_dist, mv_veh_dist);
  planner.generateCurvePoints(stop_distance, follow_distance, planner.params().max_speed_enter_zone);

  //   TODO: reactivate gpp
  throw VLRException("GPP mode not reimplemented..");
  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneEntering
 *---------------------------------------------------------------------------*/
StZoneEntering::StZoneEntering(my_context ctx) :
  my_base(ctx), StBase<StZoneEntering> (std::string("StZoneEntering")) {

}

StZoneEntering::~StZoneEntering() {
}

sc::result StZoneEntering::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  std::vector<AnnotatedRouteEdge*>& zone_maneuvers = context<StZone> ().zone_maneuvers;

  if (zone_maneuvers.empty()) return transit<StDrive> ();
  else if ((*zone_maneuvers.begin())->hasAnnotation(UC_MANEUVER_GOAL_REACHED)) return transit<StZoneParking> ();
  else if ((*zone_maneuvers.begin())->hasAnnotation(UC_MANEUVER_PARKING)) return transit<StZoneParking> ();
  else if ((*zone_maneuvers.begin())->hasAnnotation(UC_MANEUVER_ZONE_EXIT)) return transit<StZoneDriveToExit> ();
  else if ((*zone_maneuvers.begin())->hasAnnotation(UC_MANEUVER_TRAVEL)) return transit<StZoneDriveToExit> ();
  else assert(false);

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneParking
 *---------------------------------------------------------------------------*/
StZoneParking::StZoneParking(my_context ctx) :
  my_base(ctx), StBase<StZoneParking> (std::string("StZoneParking")) {
  double x, y, psi;
  //  ChsmPlanner& planner = context<ChsmPlanner>();
  std::vector<AnnotatedRouteEdge*>& zone_maneuvers = context<StZone> ().zone_maneuvers;

  // get parking spot
  assert(!zone_maneuvers.empty());
  RoutePlanner::AnnotatedRouteEdge* edge = zone_maneuvers[0];
  x = edge->edge()->toVertex()->x();
  y = edge->edge()->toVertex()->y();
  psi = atan2(edge->edge()->toVertex()->y() - edge->edge()->fromVertex()->y(), edge->edge()->toVertex()->x() - edge->edge()->fromVertex()->x());

  // subtract front bumper distance, because we want to park the car
  // with the front bumber on this waypoint
  x -= (FRONT_BUMPER_DELTA - 1.5) * cos(psi);
  y -= (FRONT_BUMPER_DELTA - 1.5) * sin(psi);

  //   TODO: reactivate gpp
  throw VLRException("GPP mode not reimplemented..");
}

StZoneParking::~StZoneParking() {
}

sc::result StZoneParking::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  //   TODO: reactivate gpp
  throw VLRException("GPP mode not reimplemented..");

  // Transition: Recovery Mode
  if (checkRecovery()) {
    return transit<StZoneRecover> ();
  }

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneParked
 *---------------------------------------------------------------------------*/
StZoneParked::StZoneParked(my_context ctx) :
  my_base(ctx), StBase<StZoneParked> (std::string("StZoneParked")) {

  stop_time = drc::Time::current() + MIN_WAIT_PARKED;
}

StZoneParked::~StZoneParked() {
}

sc::result StZoneParked::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  if (isExpired(stop_time)) {
    std::vector<AnnotatedRouteEdge*>& zone_maneuvers = context<StZone> ().zone_maneuvers;
    assert(!zone_maneuvers.empty());

    // erase previous maneuver
    zone_maneuvers.erase(zone_maneuvers.begin());

    if (zone_maneuvers.empty()) return transit<StDrive> ();
    else if ((*zone_maneuvers.begin())->hasAnnotation(UC_MANEUVER_GOAL_REACHED)) return transit<StZoneParking> ();
    else if ((*zone_maneuvers.begin())->hasAnnotation(UC_MANEUVER_PARKING)) return transit<StZoneParking> ();
    else if ((*zone_maneuvers.begin())->hasAnnotation(UC_MANEUVER_ZONE_EXIT)) return transit<StZoneDriveToExit> ();
    else if ((*zone_maneuvers.begin())->hasAnnotation(UC_MANEUVER_TRAVEL)) return transit<StZoneDriveToExit> ();
    else assert(false);
  }

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneDriveToExit
 *---------------------------------------------------------------------------*/
StZoneDriveToExit::StZoneDriveToExit(my_context ctx) :
  my_base(ctx), StBase<StZoneDriveToExit>(std::string("StZoneDriveToExit")) {

  //  ChsmPlanner& planner = context<ChsmPlanner>();

  std::vector<AnnotatedRouteEdge*>& zone_maneuvers = context<StZone> ().zone_maneuvers;

  // get zone exit
  assert(!zone_maneuvers.empty());
  RoutePlanner::AnnotatedRouteEdge* edge = (*zone_maneuvers.begin());

//  double x = edge->edge()->fromVertex()->x();
//  double y = edge->edge()->fromVertex()->y();
//  double psi = atan2(edge->edge()->toVertex()->y() - edge->edge()->fromVertex()->y(), edge->edge()->toVertex()->x() - edge->edge()->fromVertex()->x());

  //   TODO: reactivate gpp
  throw VLRException("GPP mode not reimplemented..");
}

StZoneDriveToExit::~StZoneDriveToExit() {
}

sc::result StZoneDriveToExit::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  //   TODO: reactivate gpp
  throw VLRException("GPP mode not reimplemented..");

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = planner.topology_;

  // check if vehicle is at exit
  RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);

  return transit<StDrive> ();

  // Transition: Recovery Mode
  if (checkRecovery()) {
    return transit<StZoneRecover> ();
  }

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StZoneRecover
 *---------------------------------------------------------------------------*/
StZoneRecover::StZoneRecover(my_context ctx) :
  my_base(ctx), StBase<StZoneRecover>(std::string("StZoneRecover")) {
}

StZoneRecover::~StZoneRecover() {
}

sc::result StZoneRecover::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = planner.topology_;

  std::vector<AnnotatedRouteEdge*>& zone_maneuvers = context<StZone> ().zone_maneuvers;

  if (zone_maneuvers.size() > 1) {
    planner.addMessage("Resuming to next parking maneuver.");
    if (!zone_maneuvers.empty()) return transit<StZoneParked> ();
  }

  // add a new maneuver
  planner.addMessage("Adding a new maneuver to next way point outside the zone.");
  RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);
  while (anno_edge_it != topology->route.route.end()) {
    // add next travel maneuver
    if ((*anno_edge_it)->hasAnnotation(UC_MANEUVER_TRAVEL)) {
      zone_maneuvers.push_back(*anno_edge_it);
      return transit<StZoneParked> ();
    }
  }

  // if nothing helps...global recover! :-(
  return transit<StGlobalRecover> ();
}

} // namespace vlr
