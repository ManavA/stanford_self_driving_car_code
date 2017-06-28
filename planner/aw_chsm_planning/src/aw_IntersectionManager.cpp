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


#include <cmath>
#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()
#include <sys/types.h>
#include <algorithm>
#include <aw_CGAL.h>
#include <aw_Topology.hpp>
#include <aw_Topology.hpp>
#include <aw_roadNetwork.h>
#include <aw_roadNetworkSearch.h>

#include "aw_ChsmPlanner.hpp"
#include "aw_IntersectionManager.hpp"

using namespace std;

namespace drc = driving_common;

namespace vlr {

#undef TRACE
//#define TRACE(str) cout << "[IntersectionManager] " << str << endl;
#define TRACE(str)

//#define USE_GERMAN_RIGHT_OF_WAY

const double IntersectionManager::last_pose_pose_threshold = 0.75; // [m]
const double IntersectionManager::last_pose_time_threshold = 20;   // [s]
const double IntersectionManager::last_pose_time_diffusion = 2;    // [s]

IntersectionManager::IntersectionManager(Topology& topology, VehicleManager& vehicle_manager, double max_merge_speed,
                                          std::map<std::string, driving_common::TrafficLightState>& tl_states, pthread_mutex_t& intersection_predictor_mutex)
: topology_(&topology), graph(NULL), vehicle_manager_(&vehicle_manager), intersection_(NULL), mfc(NULL), max_merge_speed_(max_merge_speed),
  vehicles_with_row(), right_to_drive(true), turn_dir(0), traffic_light_states_(tl_states) {
	graph = topology_->complete_graph;
	if(!graph) {
	   throw VLRException("Graph not initialized.");
	}
    if(!topology_->ego_vehicle.edge()) {
       throw VLRException("Cannot find edge for ego vehicle (that's our car).");
    }
	intersection_ = topology_->get_next_intersection();

	if(!intersection_) {
       throw VLRException("Intersection is invalid.");
    }

    mfc = new MergeFeasabilityCheck(mfcIntersection, MergeFeasabilityCheck::Merge, intersection_->radius());

	turn_dir = topology_->nextTurnDirection();
    if(turn_dir != driving_common::TurnSignal::NONE && turn_dir != driving_common::TurnSignal::LEFT && turn_dir != driving_common::TurnSignal::RIGHT) {
       throw VLRException("Invalid turn direction.");
    }
}

IntersectionManager::~IntersectionManager() {
	delete mfc;
}


void
IntersectionManager::stoppedOnStopline()
{
	TRACE("---- INITIAL LOOKAROUND ----");

	map<RndfEdge*, Vehicle*> vehicles_at_stopline;
	map<RndfEdge*, double> vehicles_dists;

	// which other vehicle is on the intersection_
	vehicles_with_row.clear();
	TRACE("Looking for Vehicles with right of way:")
	for (VehicleMap::iterator it = vehicle_manager_->vehicle_map.begin(); it != vehicle_manager_->vehicle_map.end(); ++it) {
		Vehicle& veh = it->second;

		double intersec_dist = veh.distToIntersection(intersection_);
		double stopline_dist = veh.distToStopLine(intersection_);

		TRACE("  " << veh << "width:" << veh.width() << " length:" << veh.length());

		// directly at the stopline
		if ( std::abs(stopline_dist) <= VEH_DIST_FROM_STOPLINE_THRESHOLD ) {
			TRACE("  -> at stopline. stopline dist: "<< stopline_dist << " (added)");
			vehicles_with_row[it->first] = veh;
			vehicles_on_stopline[it->first] = veh;

			// TODO: Make sure that for each lane only the car with minimum distance to stop line is used
//			TRACE("  -> at stopline (remembered)");
//			map<RndfEdge*, double>::iterator dist_it = vehicles_dists.find(veh.edge);
//			if (dist_it == vehicles_dists.end() || veh.dist_to_end < dist_it->second) {
//				vehicles_at_stopline[veh.edge] = &veh;
//				vehicles_dists[veh.edge] = veh.dist_to_end;
//			}
			continue;
		}

		// inside intersection_
		if ( fabs(intersec_dist) < 0.01 ) {
			TRACE("  ->inside intersection_ (added)");
			vehicles_with_row[it->first] = veh;
			continue;
		}

		// in proximity of intersection_
		if ( stopline_dist > VEH_DIST_FROM_STOPLINE_THRESHOLD && stopline_dist < TRIGGER_DIST_STOPLINE ) {
			TRACE("  -> close to stop line. distance: "<< (stopline_dist) <<". (not added)");
			continue;
		}

		// is on priority lane
		if ( intersec_dist > 0. && intersec_dist < 20. && veh.edge()->isPriorityLaneEdge() ) {
			TRACE("  -> is on priority lane. intersection distance: "<< (intersec_dist) <<". (added)");
			vehicles_with_row[it->first] = veh;
			continue;
		}

		// somewhere else
		TRACE("  -> somewhere else (not added)");
	}

	TRACE(vehicles_with_row.size() << " with right of way found");
}


bool
IntersectionManager::hasRightOfWay()
{
	TRACE("---- IS_ALLOWED_TO_DRIVE? ----");

	assert(intersection_);

	// update vehicle positions
	VehicleIdSet notUpdated;
	for (VehicleMap::iterator it = vehicles_with_row.begin(); it != vehicles_with_row.end(); ++it)
		notUpdated.insert(it->first);

	TRACE("Updating Positions")
	for (VehicleMap::iterator it = vehicle_manager_->vehicle_map.begin(); it != vehicle_manager_->vehicle_map.end(); ++it) {
		// Graph abgrasen
		if (vehicles_with_row.find(it->first) != vehicles_with_row.end()) {
			vehicles_with_row[it->first] = it->second;
			notUpdated.erase(it->first);
			TRACE("  " << it->second);
		}
	}


	TRACE("Trying to assign unassigned vehicles:");
	// Alle Vehicles matchen die nicht upgedated wurden
	for (VehicleIdSet::iterator nit = notUpdated.begin(); nit != notUpdated.end(); ++nit) {
		Vehicle& row_veh = vehicles_with_row[*nit];
		TRACE("  "<< row_veh);
		double min_dist = std::numeric_limits<double>::max();
		VehId id = -1;
		for (VehicleMap::iterator it = vehicle_manager_->vehicle_map.begin(); it != vehicle_manager_->vehicle_map.end(); ++it) {
			// skip vehicles already assigned
			if (vehicles_with_row.find(it->first) != vehicles_with_row.end()) continue;
			// Abstand ermitteln
			Vehicle& veh = it->second;
			double dx = row_veh.xMatchedFrom() - veh.xMatchedFrom();
			double dy = row_veh.yMatchedFrom() - veh.yMatchedFrom();
			// double dyaw = min(rwo_veh.);
			double dist = sqrt(dx*dx + dy*dy);
			if (veh.isAtStopline(intersection_) && dist < min_dist) {
				min_dist = dist;
				id = it->first;
			}
		}
		// replace vehicles if matched
		if (id >= 0 && min_dist < 1.0) {
			vehicles_with_row.erase(row_veh.id());
			vehicles_with_row[id] = vehicle_manager_->vehicle_map[id];
			TRACE("  -> "<< vehicle_manager_->vehicle_map[id]);
		} else {
			TRACE("  -> could not be assigned");
		}
	}


	// Check vehicle current positions
	TRACE("Check if vehicles with right of way already left intersection:");
	VehicleIdSet gone;
	VehicleIdSet mergecheck;
	for (VehicleMap::iterator it = vehicles_with_row.begin(); it != vehicles_with_row.end(); ++it) {
		Vehicle& veh = it->second;
		assert(&veh != &topology_->ego_vehicle);
		assert(!(veh.id() == topology_->ego_vehicle.id()));
		TRACE("  "<< veh);

		double intersec_dist = veh.distToIntersection(intersection_);
		double stopline_dist = veh.distToStopLine(intersection_);



		if ( fabs(stopline_dist) <= VEH_DIST_FROM_STOPLINE_THRESHOLD ) {
			TRACE("  ->at stop line");
		} else

		// inside intersection_
		if ( fabs(intersec_dist) < 0.01 ) {
			TRACE("  ->inside intersection");
			//mergecheck.insert(veh.id());
		} else

		// in proximity of intersection_
		if ( stopline_dist > VEH_DIST_FROM_STOPLINE_THRESHOLD && stopline_dist < TRIGGER_DIST_STOPLINE ) {
			TRACE("  -> close to stop line. distance: "<< (stopline_dist) <<".");
		} else

		// is on priority lane
		if ( intersec_dist > 0. && stopline_dist > intersec_dist + 30. ) {
			TRACE("  -> is on priority lane. isec distance: "<< (intersec_dist) <<".");
			mergecheck.insert(veh.id());
		} else
			gone.insert(veh.id());

	}

	// merge check with all vehicles on priority lanes, populates vehicles_with_priority
	bool merge_allowed = isMergeAllowed();

	// If a vehicle left intersection, remove from observation list
	for (VehicleIdSet::iterator it = gone.begin(); it != gone.end(); ++it) {
		vehicles_with_row.erase(*it);
	}

	// only if we do not wait for prio vehicles purge ROW vehicles that do not move (e.g. cones)
	if (merge_allowed) {
		size_t dummy = 123;
		purgeFakeVehicles(vehicles_with_row, dummy);
	} else {
	  clearTimestamps(vehicles_with_row);
	}

	right_to_drive = merge_allowed && vehicles_with_row.size()==0;
	TRACE("");
	TRACE("Ok to continue driving? "<< (right_to_drive ? "YES" : "NO"));
	TRACE("");
	return right_to_drive;
}

bool IntersectionManager::hasToStop()
{
	TRACE("----  hasToStop? ---- ");

	assert(intersection_);

	double stop_dist = topology_->distToStopLine(intersection_);
	double isec_dist = topology_->distToIntersection(intersection_);
  std::vector<std::string> tl_names;
  double traffic_light_dist = topology_->distToTrafficLight(intersection_, &tl_names, NULL);
	bool merge_allowed = isMergeAllowed();

  if(traffic_light_dist<stop_dist) {
    std::map<std::string, driving_common::TrafficLightState>::const_iterator tlsit = traffic_light_states_.find(tl_names[0]);
    if(tlsit != traffic_light_states_.end()) {
      printf("Current TLID: %s (%c)\n", tl_names[0].c_str(), (*tlsit).second.state);
      return (*tlsit).second.state != 'g';
    }
  }
  else {
    TRACE("   stop_dist = "<<stop_dist <<" isec-dist="<<isec_dist);
    if (isOnPrio()) {
      TRACE("  on priority lane");
      return !merge_allowed;
    } else {
      TRACE("  on yield lane");
      if (stop_dist < isec_dist+5.0) {
        TRACE("  has stopline");
        return true;
      } else {
        if (merge_allowed) {
          TRACE("  merge allowed");
          return false;
        } else {
          TRACE("  merge not allowed");
          return true;
        }
      }
    }
  }

  return true;  // should never be reached
}

// TODO: move to a better place
bool isOnRightSide(const Vehicle& ego, const Vehicle& other) {
	Line_2 ego_line(Point_2(ego.xMatched(), ego.yMatched()), Vector_2(cos(ego.yawMatchedFrom()), sin(ego.yawMatchedFrom())));
	return ego_line.has_on_negative_side(Point_2(other.xMatched(), other.yMatched()));
}
// TODO: move to a better place
// TODO: make a better version. this one compares the yaw of the cars it should compare the direction of the edges where the cars are matched on
// TODO: make an even better version that uses a better semantic than just the geometric one because this semantic fails for some crazy intersections (an intersection_ in a curve for instance)
bool isCommingFromFront(const Vehicle& ego, const Vehicle& other) {
	using namespace CGAL_Geometry;

	Direction_2   ego_dir(cos(  ego.yawMatchedFrom()), sin(  ego.yawMatchedFrom()));
	Direction_2 other_dir(cos(other.yawMatchedFrom()), sin(other.yawMatchedFrom()));
	Direction_2 bound1 = (-ego_dir).transform(Transformation(CGAL::ROTATION, sin(-M_PI_4), cos(-M_PI_4)));
	Direction_2 bound2 = (-ego_dir).transform(Transformation(CGAL::ROTATION, sin( M_PI_4), cos( M_PI_4)));
	return other_dir.counterclockwise_in_between(bound1, bound2);
}

void IntersectionManager::do_merge_check(MergeFeasabilityCheck::Entity& ego, GraphPlace& ego_place, bool on_prio, size_t& merge_allowed, const Vehicle& veh, const RndfEdge* edge)
{

	double intersec_dist = veh.distToIntersection(intersection_);
	bool veh_on_prio = edge->isPriorityLaneEdge() && intersec_dist > -0.1 && intersec_dist < 70.;
	bool veh_on_same_prio = veh_on_prio && isOnSamePrioLane( ego_place, edge, intersection_, 100. );
#ifdef USE_GERMAN_RIGHT_OF_WAY
	bool veh_on_right_side = isOnRightSide(topology_->ego_vehicle, veh);
#endif
	bool veh_is_comming_from_front = isCommingFromFront(topology_->ego_vehicle, veh);

	TRACE("    " << veh);// << "  - isec dist: "<< intersec_dist);

	topology_->addMessage(boost::format("%1%: onPrio %2% vehOnPrio %3% vehOnSame %4%")
		% veh.id() % on_prio % veh_on_prio % veh_on_same_prio);

#ifdef USE_GERMAN_RIGHT_OF_WAY
	// Rechts vor Links precendence rule
	if (on_prio && veh_on_prio && !veh_on_same_prio && veh_on_right_side) { // CHECK: is that really all it takes to determin the rechts-vor-links precendence?
		TRACE("  -> need to obey rechts-vor-links precedence with this vehicle");
		vehicles_on_opposite_prio[veh.id()] = veh;
		topology_->addMessage(boost::format("%1% i am left of") % veh.id());
	} else
#endif
	  // vehicle on oncoming priority lane and turn direction is right or straight
    if ( on_prio && veh_on_prio && ! veh_on_same_prio && veh_is_comming_from_front) {
      TRACE("  -> is on oncoming priority lane");
			vehicles_on_opposite_prio[veh.id()] = veh;
			if (turn_dir >= 0) {
				TRACE("  -> is oncoming priority lane and self is "<< (turn_dir == 0 ? "driving straight" : "turning right") <<" at intersection.");
				TRACE("  -> merge allowed.");
				topology_->addMessage(boost::format("%1% on oncoming priority lane&me!left: merge allowed!") % veh.id());
				return;
			} else {
				topology_->addMessage(boost::format("%1% on oncoming priority lane&me left") % veh.id());
			}
		}

	// is on priority lane
#ifdef USE_GERMAN_RIGHT_OF_WAY
    if ( vehicles_with_priority.find(veh.id()) == vehicles_with_priority.end() && ((veh_on_prio && !on_prio) || (on_prio && veh_on_prio && ! veh_on_same_prio && veh_is_comming_from_front) || (on_prio && veh_on_prio && !veh_on_same_prio && veh_on_right_side)) ) {
#else
    if ( vehicles_with_priority.find(veh.id()) == vehicles_with_priority.end() && ((veh_on_prio && !on_prio) || (on_prio && veh_on_prio && ! veh_on_same_prio && veh_is_comming_from_front)) ) {
#endif
		TRACE("    -> is on priority lane. intersection distance: "<< (intersec_dist) <<". (added)");
		vehicles_with_priority[veh.id()] = veh;
		MergeFeasabilityCheck::Entity other = MergeFeasabilityCheck::getEntity(intersection_->center(), CGAL_Geometry::Point_2(veh.xMatched(), veh.yMatched()), veh.yawMatchedFrom(), veh.speed());
		MergeFeasabilityCheck::Result r = mfc->test(ego, other, max_merge_speed_);
		if (r == MergeFeasabilityCheck::Merge) {
			TRACE("  -> merge allowed.");
			topology_->addMessage(boost::format("%1% on priority: merge allowed") % veh.id());
			++merge_allowed;
			topology_->debug_distances.push_back(Topology::DistanceVisualisation(intersection_->center().x(), intersection_->center().y(), other.distance, 0.0, 1.0, other.distance<0.0?1.0:0.0));
		} else {
			TRACE("  -> merge NOT allowed.");
			topology_->addMessage(boost::format("%1% on priority: merge NOT allowed") % veh.id());
			topology_->debug_distances.push_back(Topology::DistanceVisualisation(intersection_->center().x(), intersection_->center().y(), other.distance, 1.0, 0.0, other.distance<0.0?1.0:0.0));
		}
	} else {
		topology_->addMessage(boost::format("%1% somewhere else (dist = %2%)") % veh.id() % intersec_dist);
	}

	// Somewhere else
	TRACE("    -> somewhere else (not added)");
}

bool IntersectionManager::isMergeAllowed()
{
	using namespace CGAL_Geometry;
	assert(intersection_);
	TRACE("  Looking for Vehicles with priority:");

	// Check if we are on priority lane
	bool on_prio = isOnPrio();
	GraphPlace ego_place(topology_->ego_vehicle.edge(), topology_->ego_vehicle.distFromStart());

	vehicles_with_priority.clear();
	vehicles_on_opposite_prio.clear();
	size_t merge_allowed = 0;
	MergeFeasabilityCheck::Entity ego = MergeFeasabilityCheck::getEntity(intersection_->center(), CGAL_Geometry::Point_2(topology_->ego_vehicle.xMatched(),topology_->ego_vehicle.yMatched()), topology_->ego_vehicle.yawMatchedFrom(), topology_->ego_vehicle.speed());
	for (VehicleMap::iterator it = vehicle_manager_->vehicle_map.begin(); it != vehicle_manager_->vehicle_map.end(); ++it) {
		Vehicle& veh = it->second;
		assert(veh.edge());
//		if (veh.edges.size()) {
//			map< RoutePlanner::RndfEdge*, double >::const_iterator edges_iter, edges_end;
//			for (edges_iter = veh.edges.begin(), edges_end = veh.edges.end(); edges_iter != edges_end; ++edges_iter) {
//				do_merge_check(ego, ego_place, on_prio, merge_allowed, veh, edges_iter->first);
//			}
//		} else {
			do_merge_check(ego, ego_place, on_prio, merge_allowed, veh, veh.edge());
//		}
	}

	//purgeFakeVehicles(vehicles_with_priority, merge_allowed);

	mfc->setState(merge_allowed == vehicles_with_priority.size()?MergeFeasabilityCheck::Merge:MergeFeasabilityCheck::Stop);
	return mfc->getState()==MergeFeasabilityCheck::Merge;
}

bool IntersectionManager::isOnPrio()
{
	assert(topology_->ego_vehicle.edge());
	return topology_->ego_vehicle.edge()->isPriorityLaneEdge();
}

void IntersectionManager::purgeFakeVehicles(VehicleMap& map, size_t& counter)
{
	// kick non-moving vehicles out of map
  // if there is one moving vehicle we clear all timestamps
	VehicleIdSet nonMovers;
	double now=drc::Time::current();
	for (VehicleMap::iterator it = map.begin(); it != map.end(); ++it) {
		Vehicle& veh = it->second;
		if (vehicles_on_stopline.find(veh.id())!=vehicles_on_stopline.end()) { // only vehicles that were initially at the stopline are considered
			if (now > last_pose[veh.id()].time) {
				if (std::abs(veh.xMatched() - last_pose[veh.id()].pose.x()) < last_pose_pose_threshold && std::abs(veh.yMatched() - last_pose[veh.id()].pose.y()) < last_pose_pose_threshold) {
					TRACE(veh << " did not move for a longer period" << endl);
					nonMovers.insert(veh.id());
					//ignored_vehicles.insert(veh.id());
				} else {
					last_pose[veh.id()].pose.x() = veh.xMatched();
					last_pose[veh.id()].pose.y() = veh.yMatched();
					last_pose[veh.id()].time = drc::Time::current();
					srand(time(0));
					double rand_time_offset = ( (double)rand() / ((double)(RAND_MAX)+(double)(1)) ) * 2 * last_pose_time_diffusion - last_pose_time_diffusion;
					last_pose[veh.id()].time += last_pose_time_threshold + rand_time_offset;
				}
			}
		}

	}
	if (nonMovers.size() == map.size()) {
	    clearTimestamps(map);
	}
	for (VehicleIdSet::iterator it = nonMovers.begin(); it != nonMovers.end(); ++it) {
		map.erase(*it);
		--counter;
	}
}

void IntersectionManager::clearTimestamps(VehicleMap& map)
{
  for (VehicleMap::iterator it = map.begin(); it != map.end(); ++it) {
    Vehicle& veh = it->second;
    last_pose[veh.id()].pose.x() = veh.xMatched();
    last_pose[veh.id()].pose.y() = veh.yMatched();
    last_pose[veh.id()].time = drc::Time::current();
    srand(time(0));
    double rand_time_offset = ( (double)rand() / ((double)(RAND_MAX)+(double)(1)) ) * 2 * last_pose_time_diffusion - last_pose_time_diffusion;
    last_pose[veh.id()].time += last_pose_time_threshold + rand_time_offset;
  }
}

bool IntersectionManager::isVehicleOnIntersectionInFront()
{
	using namespace CGAL_Geometry;

	TRACE("---------isVehicleOnIntersectionInFront-------------");

	Point_2 ego(topology_->ego_vehicle.xMatchedFrom(), topology_->ego_vehicle.yMatchedFrom());
	Vector_2 yaw(cos(topology_->ego_vehicle.yawMatchedFrom()), sin(topology_->ego_vehicle.yawMatchedFrom()));
	//Vector_2 ego_to_center_norm = (intersection_->center() - ego) / intersection_->radius();
	double rho = M_PI_2 / 8.0; // 22.5 degree
	if (isOnPrio()) rho = M_PI_2 / 4.0; // 45 degree

	Line_2 line_right = Line_2(ego, yaw.perpendicular(CGAL::CLOCKWISE).transform(Transformation(CGAL::ROTATION, sin(rho), cos(rho))));
	Line_2 line_left = Line_2(ego, yaw.perpendicular(CGAL::CLOCKWISE).transform(Transformation(CGAL::ROTATION, sin(M_PI-rho), cos(M_PI-rho))));

	Point_2 debug_left = ego + line_left.to_vector()*3.0;
	Point_2 debug_right = ego + line_right.to_vector()*3.0;
	topology_->debug_distances.push_back(Topology::DistanceVisualisation(debug_left.x(), debug_left.y(), 0.2, 0.0, 0.70, 0.0));
	topology_->debug_distances.push_back(Topology::DistanceVisualisation(debug_right.x(), debug_right.y(), 0.2, 0.70, 0.0, 0.0));

	for (VehicleMap::iterator it = vehicle_manager_->vehicle_map.begin(); it != vehicle_manager_->vehicle_map.end(); ++it) {
		Vehicle& veh = it->second;
		assert(veh.edge());
		double intersec_dist = veh.distToIntersection(intersection_);
		double stopline_dist = veh.distToStopLine(intersection_);
		Point_2 veh_point = Point_2(veh.xMatchedFrom(), veh.yMatchedFrom());
		  // TODO: Check abs distances
		if (std::abs(intersec_dist) < 0.001 && (stopline_dist < -STOP_DIST_THRESHOLD || !isfinite(stopline_dist))
				&& ignored_vehicles.find(veh.id()) == ignored_vehicles.end()
				&& line_left.has_on_negative_side(veh_point)
				&& line_right.has_on_positive_side(veh_point)) {
			TRACE("Vehicle " << veh << " is in front and on intersection");
			topology_->debug_distances.push_back(Topology::DistanceVisualisation(veh_point.x(), veh_point.y(), 3, 0.0, 0.0, 0.70));
			return true;
		}
	}

	TRACE("    no vehicle in front");
	return false;
}

bool IntersectionManager::isInfrontMergePoint()
{
	return MergeFeasabilityCheck::getEntity(intersection_->center(), CGAL_Geometry::Point_2(topology_->ego_vehicle.xMatched(),topology_->ego_vehicle.yMatched()), topology_->ego_vehicle.yawMatchedFrom(), topology_->ego_vehicle.speed()).distance > 0.0;
}

// needs vehicles_on_opposite_prio populated in isMergeAllowed()
std::pair< bool, double > IntersectionManager::isPrioOppositeMergeAllowed(double velocity_desired)
{

	isMergeAllowed(); //need the datastructures

	double min_dist = std::numeric_limits<double>::infinity();
	MergeFeasabilityCheck check(mfcPassObstacle, MergeFeasabilityCheck::Merge);
	GraphPlace ego_place( topology_->ego_vehicle.edge(), topology_->ego_vehicle.distFromStart() );
	Point_2 ego_point(topology_->ego_vehicle.xMatchedFrom(), topology_->ego_vehicle.yMatchedFrom());
	for (VehicleMap::iterator it = vehicles_on_opposite_prio.begin(); it != vehicles_on_opposite_prio.end(); ++it) {
		Vehicle& veh = it->second;
		assert(veh.edge());
		GraphPlace veh_place( veh.edge(), veh.distFromStart() );
		GraphPlace cross_place = searchCrossingPoint(ego_place, veh_place, intersection_, 50.);
		if (cross_place.valid) {
			MergeFeasabilityCheck::Entity ego_ent = MergeFeasabilityCheck::getEntity(cross_place.point(), ego_point, topology_->ego_vehicle.yawMatchedFrom(), topology_->ego_vehicle.speed());
			if (ego_ent.distance > 0.0) { // if we are infront mergepoint
				Point_2 veh_point(veh.xMatchedFrom(), veh.yMatchedFrom());
				MergeFeasabilityCheck::Entity veh_ent = MergeFeasabilityCheck::getEntity(cross_place.point(), veh_point, veh.yawMatchedFrom(), veh.speed());
				if (fabs(veh_ent.speed) < 0.2) {
					veh_ent.speed = 0;
				}
				check.setEgoGeoConstrain(FRONT_BUMPER_DELTA + 1.0, BACK_BUMPER_DELTA + 1.0);
				double veh_geoconstr = cross_place.edge->width()/2.0+veh.width()/2.0; // some random default for lane width
				check.setOtherGeoConstrain(veh_geoconstr, veh_geoconstr);
				if (check.test(ego_ent, veh_ent, velocity_desired) == MergeFeasabilityCheck::Stop) {
					if (min_dist > ego_ent.distance) {
						min_dist = ego_ent.distance;
					}
					topology_->debug_distances.push_back(Topology::DistanceVisualisation(cross_place.point().x(), cross_place.point().y(), veh_ent.distance, 1.0, 0.0, veh_ent.distance<0.0?1.0:0.0));
				} else {
					topology_->debug_distances.push_back(Topology::DistanceVisualisation(cross_place.point().x(), cross_place.point().y(), veh_ent.distance, 0.0, 1.0, veh_ent.distance<0.0?1.0:0.0));
				}
			}

		}
	}

	return make_pair(!isfinite(min_dist), min_dist - STD_VEHICLE_LENGTH);
}

bool IntersectionManager::hasPrioMovement()
{
	isMergeAllowed(); //need the datastructures
	// indicates that there is some progress on the prio lanes, call this after isMergeAllowed! it needs vehicles_with_priority
	return _hasPrioMovement(vehicles_with_priority) || _hasPrioMovement(vehicles_on_opposite_prio);
}

bool IntersectionManager::_hasPrioMovement(VehicleMap& map)
{
	// indicates that there is some progress on the prio lanes, call this after isMergeAllowed! it needs vehicles_with_priority
	size_t nonMovers = 0;
	double now = drc::Time::current();
	for (VehicleMap::iterator it = map.begin(); it != map.end(); ++it) {
		Vehicle& veh = it->second;
		if (now > last_prio_pose[veh.id()].time) {
			if (fabs(veh.xMatched() - last_prio_pose[veh.id()].pose.x()) < last_pose_pose_threshold && fabs(veh.yMatched() - last_prio_pose[veh.id()].pose.y()) < last_pose_pose_threshold) {
				TRACE(veh << " didn't move for a longer period" << endl);
				++nonMovers;
			} else {
				last_prio_pose[veh.id()].pose.x() = veh.xMatched();
				last_prio_pose[veh.id()].pose.y() = veh.yMatched();
				last_prio_pose[veh.id()].time = drc::Time::current();
				srand(time(0));
				double rand_time_offset = ( (double)rand() / ((double)(RAND_MAX)+(double)(1)) ) * 2 * last_pose_time_diffusion - last_pose_time_diffusion;
				last_prio_pose[veh.id()].time += last_pose_time_threshold*1.5 + rand_time_offset;
			}
		}
	}
	return nonMovers != map.size();
}

} // namespace vlr
