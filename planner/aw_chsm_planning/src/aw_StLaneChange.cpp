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

#include <aw_StLaneChange.hpp>
#include <aw_StPause.hpp>
#include <aw_StStop.hpp>
#include <aw_StReplan.hpp>
#include <aw_StDrive.hpp>
#include <aw_StActive.hpp>


namespace drc = driving_common;

namespace vlr {

#undef TRACE
#define TRACE(str) std::cout << "[StLaneChange] "<< str << std::endl

/*---------------------------------------------------------------------------
 * StLaneChange
 *---------------------------------------------------------------------------*/
StLaneChange::StLaneChange(my_context ctx) : my_base(ctx), StBase<StLaneChange>(std::string("StLaneChange")),
                                             change_type(LC_IMPOSSIBLE), merge_allowed(false), has_to_stop(false),
                                             recover_type(LC_RECOVER_TO_ENDPOINT) {
	mfc = new MergeFeasabilityCheck(mfcPassObstacle, MergeFeasabilityCheck::Merge);
}


StLaneChange::~StLaneChange()
{
	context<ChsmPlanner>().turn_signal_.signal = driving_common::TurnSignal::NONE;

	delete mfc;
	context<ChsmPlanner>().lane_change_data = NULL;
}

sc::result StLaneChange::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	//	return forward_event();
	return discard_event();
}

sc::result StLaneChange::react(const EvPause&)
{
	ChsmPlanner& planner = context<ChsmPlanner>();
	planner.has_pause_lanechange = true;
	planner.pause_lanechange_change_point = change_point;
	planner.pause_lanechange_merge_point = merge_point;
	planner.pause_lanechange_merge_end_point = merge_end_point;
	planner.pause_lanechange_change_reason = change_reason;
	planner.pause_lanechange_change_type = change_type;
	planner.pause_lanechange_lateral_offset = lateral_offset;               // offset between the two lanes
	planner.pause_lanechange_change_length = change_length;                // length of the change
	planner.pause_lanechange_merge_length = merge_length;                 // length of the merging zone
	planner.pause_lanechange_merge_allowed = merge_allowed;
	planner.pause_lanechange_has_to_stop = has_to_stop;
	planner.pause_lanechange_recover_type = recover_type;

//	planner.pause_lanechange_old_params = old_params; // for tentacles

	planner.pause_lanechange_obstacles_in_merge_zone = obstacles_in_merge_zone;
	planner.pause_lanechange_merging_suckers = merging_suckers;

	return transit<StPause>();
}

void StLaneChange::restorePauseState()
{
	ChsmPlanner& planner = context<ChsmPlanner>();
	if (planner.has_pause_lanechange) {
		change_point = planner.pause_lanechange_change_point ;
		merge_point = planner.pause_lanechange_merge_point;
		merge_end_point = planner.pause_lanechange_merge_end_point;
		change_reason = planner.pause_lanechange_change_reason;
		change_type = planner.pause_lanechange_change_type;
		lateral_offset = planner.pause_lanechange_lateral_offset;               // offset between the two lanes
		change_length = planner.pause_lanechange_change_length;                // length of the change
		merge_length = planner.pause_lanechange_merge_length;                 // length of the merging zone
		merge_allowed = planner.pause_lanechange_merge_allowed;
		has_to_stop = planner.pause_lanechange_has_to_stop;
		recover_type = planner.pause_lanechange_recover_type;

//		old_params = planner.pause_lanechange_old_params;

		obstacles_in_merge_zone = planner.pause_lanechange_obstacles_in_merge_zone;
		merging_suckers = planner.pause_lanechange_merging_suckers;
	}
}

bool StLaneChange::mergePossible(double desired_speed, bool check_only_forward)
{
	if (change_type == LC_IMPOSSIBLE) return false; // we cannot do the impossible (yet ;) )

	if ( ! merge_point.valid ) {
		mfc->setState(MergeFeasabilityCheck::Merge);
		return true;
	}

	Point_2 mp = merge_point.point();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = planner.topology_;

	if (change_type == LC_LEFT_OPPOSITE) {
		mfc->setEgoGeoConstrain(lateral_offset + SAFETY_DISTANCE, merge_length);
		mfc->setOtherGeoConstrain(SAFETY_DISTANCE, merge_length);
	} else {
		mfc->setEgoGeoConstrain(lateral_offset + merge_length, SAFETY_DISTANCE);
		mfc->setOtherGeoConstrain(merge_length, SAFETY_DISTANCE );
	}

	size_t merge_allowed = 0;
	//	MergeFeasabilityCheck::Entity ego(hypot(mp.x() - planner.robot_pose.x, mp.y() - planner.robot_pose.y), planner.currentPose().v() ); // TODO: use better distance
	MergeFeasabilityCheck::Entity ego = MergeFeasabilityCheck::getEntity(merge_point.point(), topology->ego_vehicle.point(), topology->ego_vehicle.yawMatchedFrom(), topology->ego_vehicle.speed());
	if (change_type == LC_LEFT_OPPOSITE)
		ego = MergeFeasabilityCheck::getEntity(merge_point.point(), topology->ego_vehicle.point(), topology->ego_vehicle.yawMatchedFrom(), topology->ego_vehicle.speed());
	else
		ego = MergeFeasabilityCheck::getEntity(merge_end_point.point(), topology->ego_vehicle.point(), topology->ego_vehicle.yawMatchedFrom(), topology->ego_vehicle.speed());

	double forward_scan_dist = TRIGGER_DIST_MAX_LOOKAHEAD;
	double backward_scan_dist = TRIGGER_DIST_MAX_LOOKAHEAD;
	if (check_only_forward) backward_scan_dist = 0.;
	MergeFeasabilityCheck::Entities others = MergeFeasabilityCheck::getEntities(merge_point.edge, merge_point.offset(), ( change_type == LC_LEFT_OPPOSITE ? forward_scan_dist : backward_scan_dist ), ( change_type == LC_LEFT_OPPOSITE ? backward_scan_dist : forward_scan_dist ));

	// round small velocities to zero (supress sensor noise) TODO: should be in perception
	for (MergeFeasabilityCheck::Entities::iterator iter = others.begin(); iter != others.end(); ++iter) {
		TRACE("other distance = "<< iter->distance << " speed = " << iter->speed);
		if ( fabs( iter->speed ) < 0.5 )
			iter->speed = 0.;
	}

	// adapt velocities and distances for oncoming vehicles
	if (change_type == LC_LEFT_OPPOSITE) {
		for (MergeFeasabilityCheck::Entities::iterator iter = others.begin(); iter != others.end(); ++iter) {
			iter->distance *= -1.;
			iter->speed *= -1.;
		}
	}

	// initialize list of vehicles to remove
	std::set< Vehicle* > non_blocking_vehicles;
	for (std::map<Vehicle*, double>::iterator it = obstacles_in_merge_zone.begin(); it != obstacles_in_merge_zone.end(); ++it)
		non_blocking_vehicles.insert( it->first );
	std::set< Vehicle* > sucks_no_more;
	for (std::map<Vehicle*, double>::iterator it = merging_suckers.begin(); it != merging_suckers.end(); ++it)
		sucks_no_more.insert( it->first );

	// Merging Checks
	for (MergeFeasabilityCheck::Entities::const_iterator iter = others.begin(); iter != others.end(); ++iter)
	{
		if ( merging_suckers.find( iter->veh ) != merging_suckers.end() && merging_suckers[ iter->veh ] + VEHICLE_STAND_TILL_IGNORED < drc::Time::current() ) {
			topology->debug_distances.push_back( Topology::DistanceVisualisation(mp.x(), mp.y(), iter->distance, 1.0, 0.9, 0.0 ) );
			++merge_allowed;
			sucks_no_more.erase( iter->veh );
			continue;
		}

		MergeFeasabilityCheck::Result r = mfc->test(ego, *iter, desired_speed);

		if (r == MergeFeasabilityCheck::Merge) {
			TRACE("  -> merge allowed.");
			++merge_allowed;
			topology->debug_distances.push_back(Topology::DistanceVisualisation(mp.x(), mp.y(), iter->distance, 0.0, 1.0, iter->distance < 0.0 ? 1.0 : 0.0));
		} else {
			TRACE("  -> merge NOT allowed.");
			topology->debug_distances.push_back(Topology::DistanceVisualisation(mp.x(), mp.y(), iter->distance, 1.0, 0.0, iter->distance < 0.0 ? 1.0 : 0.0));

			// update blocking vehicles in merging zone
			if ( iter->veh->speed() < 0.5) {
				if ( obstacles_in_merge_zone.find( iter->veh ) == obstacles_in_merge_zone.end() )
					obstacles_in_merge_zone.insert( std::make_pair( iter->veh, drc::Time::current() ) );
			} else
				non_blocking_vehicles.erase( iter->veh );

			// update list of blocking vehicles
			if ( merging_suckers.find( iter->veh ) == merging_suckers.end() )
				merging_suckers.insert( std::make_pair( iter->veh, drc::Time::current() ) );
			sucks_no_more.erase( iter->veh );
		}
	}

	// remove non-blocking vehicles from list
	for (std::set< Vehicle* >::iterator it = non_blocking_vehicles.begin(); it != non_blocking_vehicles.end(); ++it) {
		obstacles_in_merge_zone.erase( *it );
	}
	for (std::set< Vehicle* >::iterator it = sucks_no_more.begin(); it != sucks_no_more.end(); ++it) {
		merging_suckers.erase( *it );
	}

	if ( merge_allowed == others.size() ) {
		mfc->setState(MergeFeasabilityCheck::Merge);
		return true;
	} else {
		mfc->setState(MergeFeasabilityCheck::Stop);
		return false;
	}
}

bool StLaneChange::snailThroughPossible( const Vehicle& base_vehicle, StLaneChangeLaneChangeType type ) const
{
	Topology* topology = context<ChsmPlanner>().topology_;
	VehicleManager* vman = topology->vehicle_manager;
	assert(vman);

	if ( type == LC_IMPOSSIBLE || type == LC_NO_CHANGE )
		return false;

	double angle = base_vehicle.edge()->getAngle();
	Vector_2 yaw_v( cos( angle ), sin( angle ) );
	Vector_2 forw_v = yaw_v * (EGO_VEHICLE_LENGTH + 1.0);
	Vector_2 left_v = forw_v.perpendicular( CGAL::LEFT_TURN ) * (EGO_VEHICLE_WIDTH + 2.0);

	// check if left side is empty
	Vehicle veh(base_vehicle);
	veh.move( ( type == LC_RIGHT ? - left_v : left_v ) );
	if ( ! vman->intersectsWithAnotherVehicle( veh ) ) return false;

	// check if space ahead is empty
	veh.move( ( type == LC_LEFT_OPPOSITE ? forw_v : -forw_v ) );
	if ( ! vman->intersectsWithAnotherVehicle( veh ) ) return false;

	// check if space behind is empty
	if ( type == LC_LEFT_OPPOSITE ) {
		veh = base_vehicle;
		veh.move( - forw_v );
		if ( ! vman->intersectsWithAnotherVehicle( veh ) ) return false;

		veh.move( - forw_v );
		if ( ! vman->intersectsWithAnotherVehicle( veh ) ) return false;
	}

	return true;
}

bool StLaneChange::placeIsValid(const GraphPlace& place) const
{
	Topology* topology = context<ChsmPlanner>().topology_;

	if ( ! place.valid ) {
		TRACE("  -> GraphPlace not valid");
		return false;
	}

	Point_2 p = place.point();
	double yaw = place.edge->getAngle();
	Vehicle veh_at_end_point( p.x() + cos(yaw)*CENTER_DELTA, p.y() + sin(yaw)*CENTER_DELTA, yaw, EGO_VEHICLE_WIDTH, EGO_VEHICLE_LENGTH );
	if ( topology->vehicle_manager->intersectsWithAnotherVehicle( veh_at_end_point ) ) {
		TRACE("  -> GraphPlace intersects with other vehicle");
		return false;
	}

	return true;
}


/*---------------------------------------------------------------------------
 * StPrepareLaneChange
 *---------------------------------------------------------------------------*/
StPrepareLaneChange::StPrepareLaneChange(my_context ctx) : my_base(ctx), StBase<StPrepareLaneChange>(std::string("StPrepareLaneChange")), prep_start(0)
{
	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology_;
	Vehicle& ego_veh = topology->ego_vehicle;
	//	RndfEdge* ego_edge = ego_veh.edge;

	// reset pause state
	planner.has_pause_lanechange = false;

	// determine changePoint and set change_reason
	double mv_veh_dist, sv_veh_dist;
	planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
	mv_veh_dist -= LANE_CHANGE_LENGTH;
	//	if (mv_veh_dist < 0.) mv_veh_dist = 0.;
	sv_veh_dist -= LANE_CHANGE_LENGTH;
	//	if (sv_veh_dist < 0.) sv_veh_dist = 0.;
	double route_lc_dist  = topology->distToNextLaneChange() + FRONT_BUMPER_DELTA;
	//	if (route_lc_dist < 0.) route_lc_dist = 0.;
	std::map< double, StLaneChangeLaneChangeReason > dists;
	dists.insert(std::make_pair( mv_veh_dist, LC_CAUSE_OF_MOVING_VEHICLE ) );
	dists.insert(std::make_pair( sv_veh_dist, LC_CAUSE_OF_OBSTACLE ) );
	dists.insert(std::make_pair( route_lc_dist, LC_CAUSE_OF_ROUTE ) );
	lc.change_reason = dists.begin()->second;
	lc.change_point = PlaceOnGraph( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );
	lc.change_point += dists.begin()->first;
	RndfEdge* edge = (*lc.change_point.edge_)->edge();
	TRACE("Changing lane from "<< edge->name());

	// stop in case there's a vehicle ahead
	if (lc.change_reason == LC_CAUSE_OF_OBSTACLE)
		lc.has_to_stop = true;

	// set change_reason
	if (lc.change_reason == LC_CAUSE_OF_ROUTE) {
		double route_lc_left_dist  = topology->distToNextManeuverStart(UC_MANEUVER_LANECHANGE_LEFT);
		double route_lc_right_dist = topology->distToNextManeuverStart(UC_MANEUVER_LANECHANGE_RIGHT);
		if ( route_lc_left_dist < route_lc_right_dist ) {
			TRACE("  -> change to left because of route");
			lc.change_type = LC_LEFT;
		} else {
			TRACE("  -> change to right because of route");
			lc.change_type = LC_RIGHT;
		}
	} else {
		  // Assumption here: 2 lanes per direction max
		lc.change_type = LC_IMPOSSIBLE;
		if      ( ! edge->leftEdges().empty() ) {
			TRACE("  Current lane has left neighboring lane");
			if ( leftLaneAllowsLaneChange(lc.change_point, LANE_CHANGE_STD_MERGING_LENGTH) ) {
				TRACE("  -> left lane is suited for lane change");
				lc.change_type = LC_LEFT;
			} else { TRACE("  -> left lane is NOT suited for lane change"); }
		}
		if ( ! edge->rightEdges().empty() && lc.change_type == LC_IMPOSSIBLE ) {
			TRACE("  Current lane has right neighboring lane");
			if ( rightLaneAllowsLaneChange(lc.change_point, LANE_CHANGE_STD_MERGING_LENGTH) ) {
				TRACE("  -> right lane is suited for lane change");
				lc.change_type = LC_RIGHT;
			} else { TRACE("  -> right lane is NOT suited for lane change"); }
		}
		if ( ! edge->leftOncomingEdges().empty() && lc.change_type == LC_IMPOSSIBLE ) {
			TRACE("  Current lane has left oncoming neighboring lane");
			if ( leftOncomingLaneAllowsLaneChange(lc.change_point, LANE_CHANGE_OPPOSITE_MERGING_LENGTH) ) {
				TRACE("  -> left oncoming lane is suited for lane change");
				lc.change_type = LC_LEFT_OPPOSITE;
			} else { TRACE("  -> left oncoming lane is NOT suited for lane change"); }
		}
	}

	  // Cancel if lane change is not feasible
	if (lc.change_type == LC_IMPOSSIBLE) {
		TRACE("=> No suited lane for lane change found");
		if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
			TRACE("Lane change because of blocking obstacle");
			TRACE("-> marking lane as blocked");
			Vehicle* veh = topology->get_next_vehicle();
			assert(veh);
			if (planner.params().enable_blockade_detection && veh) veh->blockMatchedEdges();
			TRACE("=> Replanning");
			return;
		} else if ( lc.change_reason == LC_CAUSE_OF_MOVING_VEHICLE ) {
			TRACE("Lane change because of moving vehicle -> Drive");
			lc.change_type = LC_NO_CHANGE;
			return;
		} else return;
	}

	// Determine merge point
	lc.merge_point = GraphPlace((*lc.change_point.edge_)->edge(), lc.change_point.offset());
	assert(lc.merge_point.valid);
	lc.merge_length = LANE_CHANGE_STD_MERGING_LENGTH;
	switch (lc.change_type) {
		case LC_LEFT:
			lc.merge_point.goToLeftEdge();
			break;
		case LC_RIGHT:
			lc.merge_point.goToRightEdge();
			break;
		case LC_LEFT_OPPOSITE:
			lc.merge_point.goToLeftOncomingEdge();
			lc.merge_length = LANE_CHANGE_OPPOSITE_MERGING_LENGTH;
			break;
		default: assert(false); break;
	}
//	assert(lc.merge_point.valid);
	if ( ! lc.merge_point.valid )
		lc.change_type = LC_IMPOSSIBLE;


	// determine MergeEndPoint
	if (lc.merge_point.valid) {
		if ( lc.change_type != LC_LEFT_OPPOSITE )
			lc.merge_end_point = searchDistOnLane(lc.merge_point, GraphSearchTraits::FORWARD, lc.merge_length );
		else
			lc.merge_end_point = searchDistOnLane(lc.merge_point, GraphSearchTraits::BACKWARD, lc.merge_length );
	}

	// calculate sampling offset
	RndfEdge* change_edge = NULL;
	if (lc.change_reason == LC_CAUSE_OF_ROUTE) {
		if (lc.change_type == LC_LEFT)
			change_edge = (topology->next_edge_with_maneuver(UC_MANEUVER_LANECHANGE_LEFT))->edge();
		else if (lc.change_type == LC_RIGHT)
			change_edge = (topology->next_edge_with_maneuver(UC_MANEUVER_LANECHANGE_RIGHT))->edge();
		assert(change_edge);
		assert(change_edge->isLaneChangeEdge());
	} else {
		TRACE("Looking for change_edge:");
		// TODO determine ChangeEdge with Graphplace
		switch ( lc.change_type ) {
			case LC_LEFT          : change_edge = *edge->leftEdges().begin(); break;
			case LC_RIGHT         : change_edge = *edge->rightEdges().begin(); break;
			case LC_LEFT_OPPOSITE : change_edge = *edge->leftOncomingEdges().begin(); break;
			default: assert(false); break;
		}
		if (change_edge) TRACE("  -> "<< change_edge->name());
	}
	assert(change_edge);

	// set change length
	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
		lc.change_length = LANE_CHANGE_LENGTH;
	else
		lc.change_length = change_edge->length()+1.0;

	TRACE("  change_length: "<< lc.change_length);

	lc.lateral_offset = std::sqrt( squared_distance( lc.change_point.point(), change_edge->line() ) );
	if (lc.change_type == LC_RIGHT) lc.lateral_offset *= -1;

	planner.lane_change_data = &lc;
}

StPrepareLaneChange::~StPrepareLaneChange()
{
}

sc::result StPrepareLaneChange::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	// Daten holen
	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology_;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	  // replan if lane change is impossible
	if ( lc.change_type == LC_IMPOSSIBLE ) {

		if ( prep_start == 0 ) {
			prep_start = drc::Time::current();
		}

		  // Check if waiting period is over
		if ( prep_start + VEHICLE_STAND_TILL_BLOCKADE_TIME < drc::Time::current() ) {
			planner.generateCurvePoints(planner.params().max_speed_pass_obstacle);
		} else {
			TRACE("lane change because of non-moving obstacle");
			Vehicle* veh = topology->get_next_vehicle();
			if ( veh == NULL) {
				TRACE("-> no obstacle ahead anymore");
				TRACE("=> Drive");
				return transit<StDrive>();
			} else if ( veh->speed() > 0.5) {
				TRACE("-> obstacle moves again");
				TRACE("=> Drive");
				return transit<StDrive>();
			} else {
				TRACE("-> marking lane as blocked");
				TRACE("=> Replanning");
				if (planner.params().enable_blockade_detection && veh) veh->blockMatchedEdges();
				return transit<StReplan>();
			}
		}
	}

	  // change state to Drive if lane change was impossible and reason was a moving vehicle
	if ( lc.change_type == LC_NO_CHANGE )
		return transit<StDrive>();

	  // calculate distance to change point
	double dist = difference( lc.change_point, ego_place );
	TRACE("Dist to change_point (P): "<< dist);

	// Prüfen ob sich das Vehicle bewegt hat und das Überholen überhaupt noch notwendig ist
	if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
		double mv_veh_dist, sv_veh_dist;
		planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
		double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
		veh_dist -= lc.change_length;
		if (veh_dist < 0.) veh_dist = 0.;
		if ( veh_dist > dist + 2.0 )
			return transit<StDrive>();
	}

	// Transition: Replanning (because route is blocked)
	if ( topology->isRouteBlocked(dist + lc.change_length + 2.0) )
		return transit<StReplan>();

	// set turn signal
	if ( dist < TRIGGER_DIST_BLINK )
		planner.turn_signal_.signal = (lc.change_type == LC_RIGHT ? (uint8_t)driving_common::TurnSignal::RIGHT : (uint8_t)driving_common::TurnSignal::LEFT);

	// reset time stamp
	if (lc.change_reason == LC_CAUSE_OF_OBSTACLE && planner.currentPose().v() < STOP_SPEED_THRESHOLD && prep_start == 0) {
		prep_start = drc::Time::current();
	}

  // reset time stamp if vehicle moved
	if (lc.change_reason == LC_CAUSE_OF_OBSTACLE && planner.currentPose().v() > STOP_SPEED_THRESHOLD) {
		prep_start = 0;
	}

	// Transition: back up if we moved already to far
	if ( dist < - STD_VEHICLE_LENGTH ) {
		lc.recover_type = LC_RECOVER_TO_STARTPOINT;
		return transit<StLaneChangeRecover>();
	}

	// Merge Check
	lc.merge_allowed = lc.mergePossible(planner.params().max_speed_lane_merge_false);
	//lc.merge_allowed = true; // HACK

	if ( ! lc.merge_allowed || (lc.has_to_stop && (prep_start == 0 || prep_start + MIN_WAIT_OBSTACLE > drc::Time::current()) ) ) {
		// Check if snailing is possible
		Vehicle* next_veh = topology->get_next_vehicle();
		bool snail_possible = ( next_veh ? lc.snailThroughPossible( *next_veh, lc.change_type ) : false );

//		if (snail_possible && prep_start != Timestamp(0)) {
//			lc.recover_type = LC_RECOVER_TO_ENDPOINT;
//			return transit<StLaneChangeRecover>();
//		}

		// Check if zone was occupied by a standing vehicle for a while and if yes mark lane a blocked and replan
		bool replan = false;

		for (std::map<Vehicle*, double>::iterator it = lc.obstacles_in_merge_zone.begin(); it != lc.obstacles_in_merge_zone.end(); ++it) {
			if ( it->second + VEHICLE_STAND_TILL_BLOCKADE_TIME < drc::Time::current() && ! snail_possible ) {
				Vehicle* veh = it->first;
				if (planner.params().enable_blockade_detection) veh->blockMatchedEdges();

				// determine if vehicle ahead should be marked as blocking
				double mv_veh_dist, sv_veh_dist;
				planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
				double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
				veh_dist -= lc.change_length;
				// if (veh_dist < 0.) veh_dist = 0.;
				if ( veh_dist < dist + 2.0 && next_veh )
					if (planner.params().enable_blockade_detection) next_veh->blockMatchedEdges();

				replan = true;
			}
		}
		if (replan)
			return transit<StReplan>();

		// Generate Curvepoints
		double st_veh_dist, mv_veh_dist;
		planner.getVehicleDistances(st_veh_dist, mv_veh_dist);
		planner.generateCurvePoints(dist, std::min(st_veh_dist, mv_veh_dist), planner.params().max_speed_lane_merge_false);//( stop at change_point );

		return forward_event();
	}

	//
	if ( ! lc.merging_suckers.empty() ) {
//		bool do_recover = true;
//		for (map<Vehicle*, Timestamp>::iterator it = merging_suckers.begin(); it != merging_suckers.end(); ++it) {
//			if ( it->second + VEHICLE_STAND_TILL_IGNORED > drc::Time::current() )
//				do_recover = false;
//		}
//		if ( do_recover ) {
			lc.recover_type = LC_RECOVER_TO_ENDPOINT;
			return transit<StLaneChangeRecover>();
//		}
	}

	// Change lane if change point is reached
	if ( dist < 0.2 && dist >= - STD_VEHICLE_LENGTH )
	{
		// Transition: change to left lane
		if (lc.change_type == LC_LEFT)
			return transit<StChangeToLeftLane>();
		// Transition: change to left lane
		if (lc.change_type == LC_RIGHT)
			return transit<StChangeToRightLane>();
		// Transition: change to left lane
		if (lc.change_type == LC_LEFT_OPPOSITE)
			return transit<StChangeToLeftOncomingLane>();
	}

	// generate Curvepoints
	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
		std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
//	planner.generateCurvePoints( 0., dist, lc.change_length, lc.lateral_offset, MAX_SPEED_PASS_OBSTACLE );
	else {
		planner.generateCurvePoints(planner.params().max_speed_pass_obstacle);
	}

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StChangeToLeftLane
 *---------------------------------------------------------------------------*/
StChangeToLeftLane::StChangeToLeftLane(my_context ctx) : my_base(ctx), StBase<StChangeToLeftLane>(std::string("StChangeToLeftLane"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lane change state if coming from pause mode
	lc.restorePauseState();
//	if (lc.change_reason != LC_CAUSE_OF_ROUTE) {
//		context<StLaneChange>().storeTentacleParams();
//		context<StLaneChange>().setLaneChangeTentacleParams();
//	}
}

StChangeToLeftLane::~StChangeToLeftLane()
{
//	StLaneChange& lc = context<StLaneChange>();
//	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
//		context<StLaneChange>().restoreTentacleParams();
}

sc::result StChangeToLeftLane::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology_;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// set turn signal
	planner.turn_signal_.signal = driving_common::TurnSignal::LEFT;

	// calc distance to change point
	double dist = - difference( ego_place, lc.change_point );
	TRACE("Dist to change_point (L): "<< dist);

	// TODO: Assure that current edge is chosen when replanning

	// Transition: Replan (since new lane was reached)
	//		if ( fabs(lc.lateral_offset) - ego_veh.distToMatchedEdge() < 0.2  )
	if ( - dist > lc.change_length ) {
		if (lc.change_reason != LC_CAUSE_OF_ROUTE) {
			planner.forced_start_edge = lc.merge_point.edge;
			planner.stop_before_replanning = false;
			return transit<StReplan>();
		} else
			return transit<StDrive>();
	}

	// check if vehicle moved and lane change is still necessary
	if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
		double mv_veh_dist, sv_veh_dist;
		planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
		double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
		veh_dist -= lc.change_length;
		//		if (veh_dist < 0.) veh_dist = 0.;
		Vehicle* veh = topology->get_next_vehicle();
		if ( veh_dist > dist + 2.0 && (veh == NULL || veh->speed() > 0.5 ) )
			return transit<StDrive>();
	}

	// Merge Check
	lc.merge_allowed = lc.mergePossible(planner.params().max_speed_pass_switch_lane);
	//lc.merge_allowed = true; // HACK
	if ( ! lc.merge_allowed )
		return transit<StAbortLaneChange>();

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_ENDPOINT;
		return transit<StLaneChangeRecover>();
	}


	// generate Curvepoints
	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
    std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
		//planner.generateCurvePoints( 0., dist, lc.change_length, lc.lateral_offset, MAX_SPEED_PASS_SWITCH_LANE );
	else
		planner.generateCurvePoints(planner.params().max_speed_pass_switch_lane);

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StChangeToRightLane
 *---------------------------------------------------------------------------*/
StChangeToRightLane::StChangeToRightLane(my_context ctx) : my_base(ctx), StBase<StChangeToRightLane>(std::string("StChangeToRightLane"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lanechange state if comming from pause mode
	lc.restorePauseState();
//	if (lc.change_reason != LC_CAUSE_OF_ROUTE) {
//		context<StLaneChange>().storeTentacleParams();
//		context<StLaneChange>().setLaneChangeTentacleParams();
//	}
}

StChangeToRightLane::~StChangeToRightLane()
{
//	StLaneChange& lc = context<StLaneChange>();
//	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
//		context<StLaneChange>().restoreTentacleParams();
}

sc::result StChangeToRightLane::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology_;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// set turn signal
	planner.turn_signal_.signal = driving_common::TurnSignal::RIGHT;

	// calc distance to change point
	double dist = - difference( ego_place, lc.change_point );
	TRACE("Dist to change_point (R): "<< dist);

	// Transition: Replan (since new lane was reached)
	if ( - dist > lc.change_length ) {
		if (lc.change_reason != LC_CAUSE_OF_ROUTE) {
			planner.forced_start_edge = lc.merge_point.edge;
			planner.stop_before_replanning = false;
			return transit<StReplan>();
		} else
			return transit<StDrive>();
	}

  // check if vehicle moved and lane change is still necessary
	if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
		double mv_veh_dist, sv_veh_dist;
		planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
		double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
		veh_dist -= lc.change_length;
		//		if (veh_dist < 0.) veh_dist = 0.;
		Vehicle* veh = topology->get_next_vehicle();
		if ( veh_dist > dist + 2.0 && (veh == NULL || veh->speed() > 0.5 ) )
			return transit<StDrive>();
	}


	// Merge Check
	lc.merge_allowed = lc.mergePossible(planner.params().max_speed_pass_switch_lane);
	//lc.merge_allowed = true; // HACK
	if ( ! lc.merge_allowed )
		return transit<StAbortLaneChange>();

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_ENDPOINT;
		return transit<StLaneChangeRecover>();
	}


	// generate Curvepoints
	if (lc.change_reason != LC_CAUSE_OF_ROUTE)
    std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
		//planner.generateCurvePoints( 0., dist, lc.change_length, lc.lateral_offset, MAX_SPEED_PASS_SWITCH_LANE );
	else
		planner.generateCurvePoints(planner.params().max_speed_pass_switch_lane);

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StChangeToLeftOncomingLane
 *---------------------------------------------------------------------------*/
StChangeToLeftOncomingLane::StChangeToLeftOncomingLane(my_context ctx) : my_base(ctx), StBase<StChangeToLeftOncomingLane>(std::string("StChangeToLeftOncomingLane"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lanechange state if comming from pause mode
	lc.restorePauseState();
//	context<StLaneChange>().storeTentacleParams();
//	context<StLaneChange>().setLaneChangeTentacleParams();
}

StChangeToLeftOncomingLane::~StChangeToLeftOncomingLane()
{
//	context<StLaneChange>().restoreTentacleParams();
}

sc::result StChangeToLeftOncomingLane::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology_;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// set turn signal
	planner.turn_signal_.signal = driving_common::TurnSignal::LEFT;

	// calculate distance to change point
	double dist = - difference( ego_place, lc.change_point );
	TRACE("Dist to change_point (L): "<< dist);

	// Transition: drive on oncoming lane (since new lane was reached)
	if ( - dist > lc.change_length ) {
		TRACE("reached oncoming lane");
		planner.addMessage("oncoming lane reached");
		return transit<StDriveOnOncomingLane>();
	}

  // check if vehicle moved and lane change is still necessary
	if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
		double mv_veh_dist, sv_veh_dist;
		planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
		double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
		veh_dist -= lc.change_length;
		//		if (veh_dist < 0.) veh_dist = 0.;
		Vehicle* veh = topology->get_next_vehicle();
		TRACE("  veh_dist: "<< veh_dist << "  dist: "<< dist);
		if ( veh_dist > dist + 2.0 && ( veh == NULL || veh->speed() > 0.5 ) ) {
			TRACE("Obstacle moved => Drive");
			planner.addMessage("Obstacle moved");
			return transit<StDrive>();
		}
	}


	// Merge Check
	lc.merge_allowed = lc.mergePossible(planner.params().max_speed_pass_switch_lane);
	//lc.merge_allowed = true; // HACK
	if ( ! lc.merge_allowed )
		return transit<StAbortLaneChange>();

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_ENDPOINT;
		return transit<StLaneChangeRecover>();
	}


	// generate Curvepoints
  std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
  //planner.generateCurvePoints( 0., dist, lc.change_length, lc.lateral_offset, MAX_SPEED_PASS_SWITCH_LANE );

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StDriveOnOncomingLane
 *---------------------------------------------------------------------------*/
StDriveOnOncomingLane::StDriveOnOncomingLane(my_context ctx) : my_base(ctx), StBase<StDriveOnOncomingLane>(std::string("StDriveOnOncomingLane"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lanechange state if comming from pause mode
	lc.restorePauseState();
}

StDriveOnOncomingLane::~StDriveOnOncomingLane()
{
}

sc::result StDriveOnOncomingLane::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology_;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// calculate distance to change point
	//	double dist = - difference( ego_place, lc.change_point );
	//	TRACE("Dist to change_point (L): "<< dist);

	// Simple merge check. TODO: replace with real merge check
	bool merge_allowed = true;
	//	const map<int, Vehicle>& vehicles = topology->vehicle_manager->vehicle_map;
	//	for (map<int, Vehicle>::const_iterator it = vehicles.begin(); it != vehicles.end(); ++it) {
	double min_dist = LANE_CHANGE_LENGTH + STD_VEHICLE_LENGTH;
	if (topology->distToPreviousVehicle()          < 0.5 ||
			topology->distToNextMovingVehicle()   < min_dist ||
			topology->distToNextStandingVehicle() < min_dist )
		merge_allowed = false;

	// Transition: return to original lane (since obstacle was passed)
	if (merge_allowed) {
		lc.change_point = PlaceOnGraph( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );
		return transit<StReturnFromOncomingLane>();
	}

	// Merge check for oncoming traffic
	lc.merge_point = GraphPlace( (*ego_place.edge_)->edge(), ego_place.offset());
	lc.merge_point.goToLeftOncomingEdge();
	lc.merge_length = LANE_CHANGE_OPPOSITE_MERGING_LENGTH - LANE_CHANGE_LENGTH;
	PlaceOnGraph end_place = ego_place;
	end_place += lc.merge_length;
	lc.merge_end_point = GraphPlace( (*end_place.edge_)->edge(), end_place.offset());
	lc.merge_end_point.goToLeftOncomingEdge();

	lc.merge_allowed = lc.mergePossible(planner.params().max_speed_pass_obstacle, true);
	//	if ( ! lc.merge_allowed )
	//		return transit<StAbortLaneChange>();

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_ENDPOINT;
		return transit<StLaneChangeRecover>();
	}


	// generate Curvepoints
	if ( lc.merge_allowed )
    std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
		//planner.generateCurvePoints( lc.lateral_offset, -20., 1., lc.lateral_offset, MAX_SPEED_PASS_OBSTACLE );
	else
		context<ChsmPlanner>().generateStopTrajectory();

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StReturnFromOncomingLane
 *---------------------------------------------------------------------------*/
StReturnFromOncomingLane::StReturnFromOncomingLane(my_context ctx) : my_base(ctx), StBase<StReturnFromOncomingLane>(std::string("StReturnFromOncomingLane"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lanechange state if coming from pause mode
	lc.restorePauseState();

//	context<StLaneChange>().storeTentacleParams();
//	context<StLaneChange>().setLaneChangeTentacleParams();

	context<StLaneChange>().merge_length = LANE_CHANGE_LENGTH;
}

StReturnFromOncomingLane::~StReturnFromOncomingLane()
{
//	context<StLaneChange>().restoreTentacleParams();
}

sc::result StReturnFromOncomingLane::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology_;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// set turn signal
	planner.turn_signal_.signal = driving_common::TurnSignal::RIGHT;

	// calculate distance to change point
	double dist = - difference( ego_place, lc.change_point );
	TRACE("Dist to change_point: "<< dist);

	// Transition: Drive (returned to original lane)
	if ( - dist > lc.change_length ) {
		return transit<StDrive>();
	}

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_ENDPOINT;
		return transit<StLaneChangeRecover>();
	}


	// generate Curvepoints
  std::cout << "Error: Lane change trajectory generation not implemented yet.\n";
	//planner.generateCurvePoints( lc.lateral_offset, dist, lc.change_length, 0., MAX_SPEED_PASS_SWITCH_LANE );

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StAbortLaneChange
 *---------------------------------------------------------------------------*/
StAbortLaneChange::StAbortLaneChange(my_context ctx) : my_base(ctx), StBase<StAbortLaneChange>(std::string("StAbortLaneChange"))
{
	setRecoveryTime(10.);

	StLaneChange& lc = context<StLaneChange>();
	// restore lane change state if coming from pause mode
	lc.restorePauseState();
}

StAbortLaneChange::~StAbortLaneChange()
{
}

sc::result StAbortLaneChange::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	StLaneChange& lc = context<StLaneChange>();
	ChsmPlanner& planner = context<ChsmPlanner>();
	Topology* topology = context<ChsmPlanner>().topology_;
	Vehicle& ego_veh = topology->ego_vehicle;
	GraphTools::PlaceOnGraph ego_place( topology->current_edge_it, ego_veh.distFromStart(), topology->route.route );

	// set turn signal
	planner.turn_signal_.signal = ( lc.change_type == LC_RIGHT ? (uint8_t)driving_common::TurnSignal::LEFT : (uint8_t)driving_common::TurnSignal::RIGHT );

	// calculate distance to change point
	double dist = - difference( ego_place, lc.change_point );
	TRACE("Dist to change_point: "<< dist);

  // check if vehicle moved and 2nd lane change is still necessary
	if ( lc.change_reason == LC_CAUSE_OF_OBSTACLE ) {
		double mv_veh_dist, sv_veh_dist;
		planner.getVehicleDistances(sv_veh_dist, mv_veh_dist);
		double veh_dist = std::min( sv_veh_dist, mv_veh_dist );
		veh_dist -= lc.change_length;
		if (veh_dist < 0.) veh_dist = 0.;
		Vehicle* veh = topology->get_next_vehicle();
		if ( veh_dist > dist + 2.0 && (veh == NULL || veh->speed() > 0.5 ) )
			return transit<StDrive>();
	}

	// Transition:  (since we returned to original lane but cannot continue driving)
	if (ego_veh.distToMatchedEdge() < 0.5 || ego_veh.speed() < 0.2 ) {
		lc.recover_type = LC_RECOVER_TO_STARTPOINT;
		return transit<StLaneChangeRecover>();
	}

	// Transition: recover
	if (checkRecovery()) {
		lc.recover_type = LC_RECOVER_TO_STARTPOINT;
		return transit<StLaneChangeRecover>();
	}


	// generate Curvepoints
	planner.generateCurvePoints( planner.params().max_speed_pass_switch_lane);

	return forward_event();
}


/*---------------------------------------------------------------------------
 * StLaneChangeRecover
 *---------------------------------------------------------------------------*/
StLaneChangeRecover::StLaneChangeRecover(my_context ctx) : my_base(ctx), StBase<StLaneChangeRecover>(std::string("StLaneChangeRecover")) {
	map_timer = drc::Time::current() + MIN_WAIT_FOR_OBSTACLEMAP;

	StLaneChange& lc = context<StLaneChange>();
	// restore lane change state if coming from pause mode
	lc.restorePauseState();
}

StLaneChangeRecover::~StLaneChangeRecover() {
  //   TODO: reactivate gpp / emergency planner
}

sc::result StLaneChangeRecover::react(const EvProcess&)
{
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	return discard_event(); // lane change states don't get this
}

/*---------------------------------------------------------------------------
 * StLaneChangeRecoverPrepare
 *---------------------------------------------------------------------------*/
StLaneChangeRecoverPrepare::StLaneChangeRecoverPrepare(my_context ctx) : my_base(ctx), StBase<StLaneChangeRecoverPrepare>(std::string("StLaneChangeRecoverPrepare")) {
}

StLaneChangeRecoverPrepare::~StLaneChangeRecoverPrepare() {
}

sc::result StLaneChangeRecoverPrepare::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  context<ChsmPlanner>().generateStopTrajectory();
  return forward_event();
}

//=============================================================================
//		Convenience Functions
//=============================================================================

bool leftLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length)
{
	if ( ! lc_start.isValid() ) {
		TRACE("    LaneChange start point invalid");
		return false;
	}

	GraphPlace merge_start( (*lc_start.edge_)->edge(), lc_start.offset());
	GraphPlace merge_left( merge_start );

	merge_left.goToLeftEdge();
	if ( ! merge_left.valid ) {
		TRACE("    Merge start point invalid");
		return false;
	}

	if ( merge_start.edge->leftBoundaryType() == rndf::Lane::DoubleYellow ||
			merge_start.edge->leftBoundaryType() == rndf::Lane::SolidWhite )
	{
		TRACE("    Lane markings do not allow changing to left lane");
		return false;
	}

	GraphPlace merge_end = searchDistOnLane(merge_left, GraphSearchTraits::FORWARD, length );
	if ( ! merge_end.valid ) {
		TRACE("    Merge end point invalid");
		return false;
	}

	merge_end.goToRightEdge();
	if ( ! merge_end.valid ) {
		TRACE("    LaneChange end point invalid");
		return false;
	}

	bool result = isOnSameLaneNoChange(merge_start, merge_end.edge, GraphSearchTraits::FORWARD, length + 30.);
	if ( ! result ) {
		TRACE("    Lanes diverge. (No direct connection between "<< merge_start.edge->name() <<" and "<< merge_end.edge->name() <<" could be found)");
	}

	return result;
}

bool rightLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length)
{
	if ( ! lc_start.isValid() ) {
		TRACE("    LaneChange start point invalid");
		return false;
	}

	GraphPlace merge_start( (*lc_start.edge_)->edge(), lc_start.offset());
	GraphPlace merge_right( merge_start );

	// change to right lane
	merge_right.goToRightEdge();
	if ( ! merge_right.valid ) {
		TRACE("    Merge start point invalid");
		return false;
	}

	// check lane markings
	if ( merge_start.edge->rightBoundaryType() == rndf::Lane::DoubleYellow ||
			merge_start.edge->rightBoundaryType() == rndf::Lane::SolidWhite )
	{
		TRACE("    Lane markings do not allow changing to right lane");
		return false;
	}

	// retrieve distance on lane
	GraphPlace merge_end = searchDistOnLane(merge_right, GraphSearchTraits::FORWARD, length );
	if ( ! merge_end.valid ) {
		TRACE("    Merge end point invalid");
		return false;
	}

	// change to left lane
	merge_end.goToLeftEdge();
	if ( ! merge_end.valid ) {
		TRACE("    LaneChange end point invalid");
		return false;
	}

	// Search destination edge starting from start edge
	bool result = isOnSameLaneNoChange(merge_start, merge_end.edge, GraphSearchTraits::FORWARD, length + 30.);
	if ( ! result ) {
		TRACE("    Lanes diverge. (No direct path from "<< merge_start.edge->name() <<" to "<< merge_end.edge->name() <<" found)");
	}

	return result;
}

bool leftOncomingLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length)
{
	if ( ! lc_start.isValid() ) {
		TRACE("    LaneChange start point invalid");
		return false;
	}

	GraphPlace merge_start( (*lc_start.edge_)->edge(), lc_start.offset() );
	GraphPlace merge_left_opp( merge_start );

	// change to left lane
	merge_left_opp.goToLeftOncomingEdge();
	if ( ! merge_left_opp.valid ) {
		TRACE("    Merge start point invalid");
		return false;
	}

  // check lane markings
	if ( merge_start.edge->leftBoundaryType() == rndf::Lane::DoubleYellow ||
			merge_start.edge->leftBoundaryType() == rndf::Lane::SolidWhite )
	{
		TRACE("    Lane markings do not allow changing to left oncoming lane");
		return false;
	}

  // retrieve distance on lane
	GraphPlace merge_end = searchDistOnLane(merge_left_opp, GraphSearchTraits::BACKWARD, length );
	if ( ! merge_end.valid ) {
		TRACE("    Merge start point invalid");
		return false;
	}

	// change to right lane
	merge_end.goToRightEdge();
  if ( ! merge_end.valid ) {
    TRACE("    LaneChange end point invalid");
    return false;
  }

  // Search destination edge starting from start edge
	bool result = isOnSameLaneNoChange(merge_start, merge_end.edge, GraphSearchTraits::FORWARD, length + 30.);
	if ( ! result ) {
    TRACE("    Lanes diverge. (No direct path from "<< merge_start.edge->name() <<" to "<< merge_end.edge->name() <<" found)");
	}

	return result;
}

} // namespace vlr
