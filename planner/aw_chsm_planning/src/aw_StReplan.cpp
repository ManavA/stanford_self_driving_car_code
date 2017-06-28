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


#include <iostream>

#include <aw_StPause.hpp>
#include <aw_StReplan.hpp>
#include <aw_StStop.hpp>
#include <aw_match_to_graph.hpp>

namespace drc = driving_common;

namespace vlr {

#undef TRACE
#define TRACE(str) std::cout << "[StReplan] "<< str << std::endl

/*---------------------------------------------------------------------------
 * StReplan
 *---------------------------------------------------------------------------*/
StReplan::StReplan(my_context ctx) :
  my_base(ctx), StBase<StReplan>(std::string("StReplan")) {
}

StReplan::~StReplan() {
  ChsmPlanner& planner = context<ChsmPlanner> ();
  planner.stop_before_replanning = true;
  planner.forced_start_edge = NULL;
}

sc::result StReplan::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  return forward_event();

}

/*---------------------------------------------------------------------------
 * StReplanStop
 *---------------------------------------------------------------------------*/
StReplanStop::StReplanStop(my_context ctx) :
  my_base(ctx), StBase<StReplanStop>(std::string("StReplanStop")) {
}

StReplanStop::~StReplanStop() {
}

sc::result StReplanStop::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();

  if (planner.currentPose().v() < 0.5 || !planner.stop_before_replanning) {
    return (transit<StReroute> ());
  }
  else {
    planner.generateCurvePoints(0.001);
  }
  return forward_event();

}

/*---------------------------------------------------------------------------
 * StReroute
 *---------------------------------------------------------------------------*/
StReroute::StReroute(my_context ctx) :
  my_base(ctx), StBase<StReroute>(std::string("StReroute")) {
}

StReroute::~StReroute() {
}

sc::result StReroute::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  TRACE("************** Replanning now !!! ***********************");

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = context<ChsmPlanner> ().topology_;
  Vehicle& ego_veh = topology->ego_vehicle;

  // check for end of mission
  if (topology->route_is_finished()) return (transit<StPause> ());

  //==================== determine best edge ==============================

  // Beste Kante und nächsten Checkpoint ermitteln
  topology->getBestReplanningEdge();
  deque<RndfVertex*>& checkpoints = topology->checkpoints;
  deque<RndfVertex*>::iterator cp_it = topology->next_check_point_it;
  //	deque<RndfVertex*>::iterator cp_it = topology->nextCheckPointIt();

  // dump check points
  TRACE("Checkpoints:");
  int i = 0;
  for (deque<RndfVertex*>::iterator it = checkpoints.begin(); it != checkpoints.end(); ++it, ++i)
    TRACE("  "<< i << ".  "<< (*it)->name() << ( it == topology->next_check_point_it ? "  <-- next Checkpoint" : "") );

  if (cp_it == topology->checkpoints.end()) {
    TRACE("-> No check point left => Stop");
    return transit<StPause> (); // route is probably broken, so we don't drive until the end
  }

  RndfEdge* best_edge = (planner.forced_start_edge ? planner.forced_start_edge : topology->best_alternative_edge);
  while (!best_edge && cp_it != checkpoints.end()) {
    topology->getBestReplanningEdge();
    best_edge = topology->best_alternative_edge;

    // skip next check point since it's unreachable
    if (!best_edge) {
      TRACE("  -> Check point unreachable (skip)");
      cp_it = checkpoints.erase(cp_it);
      if (cp_it != checkpoints.end()) TRACE("  -> Next check point is "<< (*cp_it)->name());
      else
      TRACE("  -> No check point left");
    }
  }

  if (!best_edge) {
    TRACE("=> No alternative edge found -> Stop");
    return transit<StPause> ();
  }

  TRACE("Best edge: "<< best_edge->name());

  //==================== Determine next check point ======================

  //	// Remove past check points and edges from route
  //	deque<RndfVertex*>& checkpoints = topology->checkpoints;
  ////	deque<RndfVertex*>::iterator cp_it = topology->next_check_point_it;
  //	deque<RndfVertex*>::iterator cp_it = topology->nextCheckPointIt();
  //	assert( cp_it != checkpoints.end() );
  cp_it = checkpoints.erase(checkpoints.begin(), cp_it);
  //	while ( (*edge_it)->edge()->toVertex() != *cp_it ) ++edge_it;
  //	route.erase( route.begin(), ++edge_it );
  checkpoints.erase(checkpoints.begin(), cp_it);

  //==================== Generate new route ==============================

  // Generate new complete round and insert best_edge
  RoutePlanner::Route new_route;
  new_route.addEdge(best_edge);

  // Calculate new partial route
  RndfGraph::EdgeList* edgeList = topology->complete_graph->searchPath(best_edge->toVertex(), *cp_it);
  assert(edgeList);
  //	TRACE(" New route from best edge to new check point:");
  //	for (RndfGraph::EdgeList::iterator it = edgeList->begin(); it != edgeList->end(); ++it) {
  //		TRACE("  "<< (*it)->name());
  //	}

  // add new route part to route
  new_route.addEdges(edgeList);
  delete edgeList;

  // Verify driveability for all succeeding route parts and replan if necessary (e.g. because of road blockade)
  // TODO: Only replan parts that are not driveable anymore
  for (cp_it = checkpoints.begin(); cp_it + 1 != checkpoints.end();) {
    deque<RndfVertex*>::iterator n_it = cp_it + 1;

    // plan route between to check points
    TRACE("Planning route segment from "<< (*cp_it)->name() <<" to "<< (*n_it)->name() );
    RndfGraph::EdgeList* edgeList = topology->complete_graph->searchPath(*cp_it, *n_it);

    // forget about blockades and replan if no valid route for a segment was found
    if (!edgeList) {
      TRACE("-> did not find valid route => Ignoring blockades and replan");
      edgeList = topology->complete_graph->searchPath(*cp_it, *n_it, true);
    }
    assert( edgeList ); // point unreachable

    // skip check point if no route could be found
    if (!edgeList) {
      TRACE("-> again no route found => skipping check point");
      checkpoints.erase(n_it);
      continue;
    }

    // add edge to route
    new_route.addEdges(edgeList);
    delete edgeList;

    ++cp_it;
  }

  // annotate new route
  // CHECK ob die Member Vars der Route aktualisiert werden müssen
  new_route.init();
  new_route.annotateRoute();

  //==================== update topology =============================

  // delete old route
  for (Route::RouteEdgeList::iterator anno_it = topology->complete_mission_graph.begin(); anno_it != topology->complete_mission_graph.end(); ++anno_it) {
    delete *anno_it;
  }

  // set new route in topology TODO: move to topology
  topology->route.route = new_route.route;
  topology->complete_mission_graph = new_route.route;
  topology->current_edge_it = topology->route.route.begin();
  topology->current_edge_it_on_complete_mission_graph = topology->complete_mission_graph.begin();

  // remove "visited" flag from all edges
  for (RndfGraph::EdgeMap::iterator it = topology->complete_graph->edgeMap.begin(); it != topology->complete_graph->edgeMap.end(); ++it) {
    it->second->setVisited(false);
  }

  // rematch ego vehicle and update internal data structure of topology TODO: move to topology
  // Calls must be in this order to to ensure that matching works correctly
  topology->extract_relevant_part_of_route();
  topology->handle_ego_vehicle_update();

  // reset best edge for next replanning
  topology->best_alternative_edge = NULL;

  //==================== transitions ============================

  // Transition: GetBackOnTRack
  if (ego_veh.distToMatchedEdge() > TRIGGER_DIST_GET_BACK_ON_TRACK || ego_veh.angleToMatchedEdge() > M_PI_4) return transit<StGetBackOnTrack> ();

  // Transition: Drive
  // TODO: Analyze matched edge and make transition accordingly
  return transit<StDrive> ();

  //	return transit<StPause>();
}

/*---------------------------------------------------------------------------
 * StGetBackOnTrack
 *---------------------------------------------------------------------------*/
StGetBackOnTrack::StGetBackOnTrack(my_context ctx) :
  my_base(ctx), StBase<StGetBackOnTrack>(std::string("StGetBackOnTrack")) {
  map_timer = drc::Time::current() + MIN_WAIT_FOR_OBSTACLEMAP;
}

StGetBackOnTrack::~StGetBackOnTrack() {
  //   TODO: reactivate gpp / emergency planner
}

sc::result StGetBackOnTrack::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  return forward_event();
}

/*---------------------------------------------------------------------------
 * StGetBackOnTrackPrepare
 *---------------------------------------------------------------------------*/
StGetBackOnTrackPrepare::StGetBackOnTrackPrepare(my_context ctx) :
  my_base(ctx), StBase<StGetBackOnTrackPrepare>(std::string("StGetBackOnTrackPrepare")) {
  ChsmPlanner& planner = context<ChsmPlanner> ();
  planner.addMessage("Stop for navigation");
}

StGetBackOnTrackPrepare::~StGetBackOnTrackPrepare() {
}

sc::result StGetBackOnTrackPrepare::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  //	return transit<StDrive>();
  //	return transit<StPause>();

  ChsmPlanner& planner = context<ChsmPlanner> ();

  if (isExpired(context<StGetBackOnTrack> ().map_timer)) {
    return transit<StGetBackOnTrackAStar> ();
  }
  else {
    planner.generateStopTrajectory();
    return forward_event();
  }
}

/*---------------------------------------------------------------------------
 * StGetBackOnTrackAStar
 *---------------------------------------------------------------------------*/
StGetBackOnTrackAStar::StGetBackOnTrackAStar(my_context ctx) :
  my_base(ctx), StBase<StGetBackOnTrackAStar>(std::string("StGetBackOnTrackAStar")) {

  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = planner.topology_;

  assert( ! topology->route_is_finished() );
  RndfEdge* edge = (*topology->complete_mission_graph.begin())->edge();

  double x, y, psi;

  // calculate entry point
  Point_2 pp = edge->line().projection(topology->ego_vehicle.point());
  Vector_2 nvec = edge->vector();
  nvec = nvec / sqrt(nvec.squared_length());
  pp = pp + nvec * 12.;
  Segment_2 seg = edge->segment();
  if (squared_distance(seg, pp) > 0.001) {
    if (squared_distance(pp, edge->fromVertex()->point()) < squared_distance(pp, edge->toVertex()->point())) pp = edge->fromVertex()->point();
    else pp = edge->toVertex()->point();
  }

  // check distance to blocking edge
  if (sqrt(squared_distance(pp, edge->fromVertex()->point())) < 5.) {
    bool bl_edge = false;
    for (set<RndfEdge*>::iterator it = edge->fromVertex()->inEdges().begin(); it != edge->fromVertex()->inEdges().end(); ++it) {
      if ((*it)->isBlockedEdge()) bl_edge = true;
    }
    if (bl_edge) pp = edge->toVertex()->point();
  }

  // set destination point
  x = pp.x();
  y = pp.y();
  psi = atan2(edge->toVertex()->y() - edge->fromVertex()->y(), edge->toVertex()->x() - edge->fromVertex()->x());

  //   TODO: reactivate gpp / emergency planner
  throw VLRException("Recover mode not reimplemented..");

  next_replan = drc::Time::current() + GETBACKONTRACK_REPLAN_INTERVAL;

}

StGetBackOnTrackAStar::~StGetBackOnTrackAStar() {
}

sc::result StGetBackOnTrackAStar::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  //   TODO: reactivate gpp / emergency planner
  throw VLRException("Recover mode not reimplemented..");
  return transit<StDrive> ();
  return forward_event();
}

} // namespace vlr
