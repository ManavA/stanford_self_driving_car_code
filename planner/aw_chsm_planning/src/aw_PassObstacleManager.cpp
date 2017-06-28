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


#include <global.h>
#include <aw_CGAL.h>
#include <aw_PassObstacleManager.hpp>
#include <aw_Topology.hpp>
#include <aw_match_to_graph.hpp>
#include <aw_RouteSampler.hpp>
#include <aw_RndfGraphSearch.h>

namespace drc = driving_common;

namespace vlr {

#define TRACE(str) std::cout << "[PassObstacleManager] " << str << std::endl;

PassObstacleManager::PassObstacleManager(Topology* top)
: inited(false), top(top), vman(0), passEdge(0), edgeIterationDirection(INVALID), mfc(0)
{
	assert(top);
	vman = top->vehicle_manager;
	assert(vman);

	mfc = new MergeFeasabilityCheck(mfcPassObstacle, MergeFeasabilityCheck::Stop);

}

PassObstacleManager::~PassObstacleManager()
{
	cleanup();
	delete mfc;
}

void PassObstacleManager::setObstacle(Vehicle* veh)
{
	obstacle = *veh; // save a local copy

	RndfEdge* edge = obstacle.edge();
	assert(edge);

	// determine passing possiblity and way
	passEdge = 0; // means that no pass is possible
	edgeIterationDirection = INVALID;
	if (edge->leftEdges().size()) {
		TRndfEdgeSet::const_iterator passEdgeIt = (edge->leftEdges().begin());
		TRndfEdgeSet::const_iterator passEdgeEnd = (edge->leftEdges().end());
		while (passEdgeIt != passEdgeEnd && (*passEdgeIt) == edge) {
			++passEdgeIt;
		}
		assert(passEdgeIt != passEdgeEnd);
		passEdge = *passEdgeIt;
		edgeIterationDirection = FORWARD;
	} else if (edge->leftOncomingEdges().size()) {
		TRndfEdgeSet::const_iterator passEdgeIt = (edge->leftOncomingEdges().begin());
		TRndfEdgeSet::const_iterator passEdgeEnd = (edge->leftOncomingEdges().end());
		while (passEdgeIt != passEdgeEnd && (*passEdgeIt) == edge) {
			++passEdgeIt;
		}
		assert(passEdgeIt != passEdgeEnd);
		passEdge = *passEdgeIt;
		edgeIterationDirection = BACKWARD;
	} else if (edge->rightEdges().size()) {
		TRndfEdgeSet::const_iterator passEdgeIt = (edge->rightEdges().begin());
		TRndfEdgeSet::const_iterator passEdgeEnd = (edge->rightEdges().end());
		while (passEdgeIt != passEdgeEnd && (*passEdgeIt) == edge) {
			++passEdgeIt;
		}
		assert(passEdgeIt != passEdgeEnd);
		passEdge = *passEdgeIt;
		edgeIterationDirection = FORWARD;
	}
	TRACE("passing obstacle on "<<edge->name());
	if (passEdge) {
		TRACE("using edge " << passEdge->name());
	} else {
		TRACE("NO PASS POSSIBLE");
	}
	assert(edge != passEdge);

	// go back to next split point
	// TODO: what to do when we are on a priority lane and another lane merges with it?
	RndfEdge* currEdge = passEdge;
	int idx = 0;
	cleanup();
	if (edgeIterationDirection == FORWARD) {
		while (currEdge && currEdge->fromVertex() && currEdge->fromVertex()->numInEdges()==1) {
			if (*(currEdge->fromVertex()->inEdges().begin()) == passEdge) { // abort if there are loops
				TRACE("loop detected");
				break;
			} else {
				currEdge = *(currEdge->fromVertex()->inEdges().begin());
				TRACE("going one backwards: " << currEdge->name());
			}
		}
		assert(currEdge);
		edges_map[idx] = currEdge;
		edges_list.push_back(new AnnotatedRouteEdge(currEdge, 0.0));
		bool foundPassEdge = false;
		while (currEdge && currEdge->toVertex() && currEdge->toVertex()->numOutEdges()==1 && (!foundPassEdge||currEdge != passEdge)) {
			currEdge = *(currEdge->toVertex()->outEdges().begin());
			edges_map[++idx] = currEdge;
			edges_list.push_back(new AnnotatedRouteEdge(currEdge, 0.0));
			if (currEdge == passEdge) foundPassEdge = true;
			TRACE("going one forward: " << currEdge->name() << " " << foundPassEdge);
		}
		assert(edges_map.size());
	} else if (edgeIterationDirection == BACKWARD) {
		while (currEdge && currEdge->toVertex() && currEdge->toVertex()->numOutEdges()==1) {
			if (*(currEdge->toVertex()->outEdges().begin()) == passEdge) { // abort if there are loops
				break;
				TRACE("loop detected");
			} else {
				currEdge = *(currEdge->toVertex()->outEdges().begin());
				TRACE("going one backwards: " << currEdge->name());
			}
		}
		assert(currEdge);
		edges_map[idx] = currEdge;
		edges_list.push_back(new AnnotatedRouteEdge(currEdge, 0.0));
		bool foundPassEdge = false;
		while (currEdge && currEdge->fromVertex() && currEdge->fromVertex()->numInEdges()==1 && (!foundPassEdge||currEdge != passEdge)) {
			currEdge = *(currEdge->fromVertex()->inEdges().begin());
			edges_map[++idx] = currEdge;
			edges_list.push_back(new AnnotatedRouteEdge(currEdge, 0.0));
			if (currEdge == passEdge) foundPassEdge = true;
			TRACE("going one forward: " << currEdge->name() << " " << foundPassEdge);
		}
		assert(edges_map.size());
	} else {
		assert(passEdge == 0);
	}

	inited = true;
}

// get called every cycle, updates internals
void PassObstacleManager::update() {
	if (!inited) return;
}

// true if there is a neigbor lane
bool PassObstacleManager::isPassPossible() {
	return inited && passEdge != 0;
	// BETTER: if no neighbor lane try to pass with offset of current lane or A*
}

// true if passing is allowed
bool PassObstacleManager::mayPass(drc::GlobalPose pose, double speed, double desired_speed) {
	using namespace CGAL_Geometry;

	if (!inited) return false;
	if (!isPassPossible()) return false;

	double d;
	double cp_x;
	double cp_y;
	double offsetOnCurrEdge;
	int sign;
	double yaw = pose.yaw();
	if (edgeIterationDirection == BACKWARD) {
		yaw = normalizeAngle(yaw - M_PI);
	}
	RndfEdge* currEdge = match_2_graph(pose.utmX(), pose.utmY(), yaw, edges_map, d, cp_x, cp_y, offsetOnCurrEdge, sign);
	assert(currEdge);
	assert(currEdge->fromVertex());
	assert(currEdge->toVertex());
	Point_2 from = Point_2(currEdge->fromVertex()->x(), currEdge->fromVertex()->y());
	Point_2 to   = Point_2(currEdge->toVertex()->x(), currEdge->toVertex()->y());
	Vector_2 f_t = to - from;
	Point_2 mp = from + (f_t / currEdge->length())*offsetOnCurrEdge;

	size_t merge_allowed = 0;
	MergeFeasabilityCheck::Entity ego(hypot(mp.x() - pose.utmX(), mp.y() - pose.utmY()), speed); // TODO: use better distance
	MergeFeasabilityCheck::Entities others = MergeFeasabilityCheck::getEntities(currEdge, offsetOnCurrEdge, TRIGGER_DIST_MAX_LOOKAHEAD, TRIGGER_DIST_MAX_LOOKAHEAD);
	for (MergeFeasabilityCheck::Entities::const_iterator iter = others.begin(); iter != others.end(); ++iter) {
		MergeFeasabilityCheck::Result r = mfc->test(ego, *iter, desired_speed);
		if (r == MergeFeasabilityCheck::Merge) {
			TRACE("  -> merge allowed.");
			++merge_allowed;
			top->debug_distances.push_back(Topology::DistanceVisualisation(mp.x(), mp.y(), iter->distance, 0.0, 1.0, iter->distance<0.0?1.0:0.0));
		} else {
			TRACE("  -> merge NOT allowed.");
			top->debug_distances.push_back(Topology::DistanceVisualisation(mp.x(), mp.y(), iter->distance, 1.0, 0.0, iter->distance<0.0?1.0:0.0));
		}
	}
	if (merge_allowed == others.size()) {
		mfc->setState(MergeFeasabilityCheck::Merge);
		return true;
	} else {
		mfc->setState(MergeFeasabilityCheck::Stop);
		return false;
	}
}

// generates curvepoints for passing an obstacle
void PassObstacleManager::generateTrajectory(drc::GlobalPose pose, std::vector<CurvePoint>& /*center_line*/,
                                                double /*front_sample_length*/, double /*back_sample_length*/) {
	using namespace CGAL_Geometry;

	double d;
	double cp_x;
	double cp_y;
	double offsetOnCurrEdge;
	int sign;
	double yaw = pose.yaw();
	if (edgeIterationDirection == BACKWARD) {
		yaw = normalizeAngle(yaw - M_PI);
	}
	RndfEdge* currEdge = match_2_graph(pose.utmX(), pose.utmY(), yaw, edges_map, d, cp_x, cp_y, offsetOnCurrEdge, sign);
	assert(currEdge);
	assert(currEdge->fromVertex());
	assert(currEdge->toVertex());
	Point_2 from = Point_2(currEdge->fromVertex()->x(), currEdge->fromVertex()->y());
	Point_2 to   = Point_2(currEdge->toVertex()->x(), currEdge->toVertex()->y());
	Vector_2 f_t = to - from;
	Point_2 mp = from + (f_t / currEdge->length())*offsetOnCurrEdge;

	TRACE("searching for " << currEdge->name() << " in edges_list ("<<edges_list.size()<<")");
	Route::RouteEdgeList::iterator currEdgeIt = edges_list.begin();
	while (currEdgeIt!=edges_list.end() && (*currEdgeIt)->edge()!= currEdge) {
		TRACE("searching for " << (*currEdgeIt)->edge()->name());
		++currEdgeIt;
	}

	std::cout << "Sampling / smoothing of neighboring lane NOT IMPLEMENTED YET\n";

//	bool valid = RouteSampler::samplePoints(currEdgeIt, offsetOnCurrEdge, &edges_list, curve, curvepoints_length_front, curvepoints_length_back, curvepoints_number);
//	assert(valid);
}

double PassObstacleManager::getFollowingSpeed() {
	double d;
	double cp_x;
	double cp_y;
	double offsetOnCurrEdge;
	int sign;
	double yaw = obstacle.yawMatchedFrom();
	if (edgeIterationDirection == BACKWARD) {
		yaw = normalizeAngle(yaw - M_PI);
	}
	RndfEdge* currEdge = match_2_graph(obstacle.xMatched(), obstacle.yMatched(), yaw, edges_map, d, cp_x, cp_y, offsetOnCurrEdge, sign);
	assert(currEdge);
	assert(currEdge->fromVertex());
	assert(currEdge->toVertex());

	GraphPlace passEdgePlace(currEdge, offsetOnCurrEdge);
	GraphPlace veh_place = searchForVehicleOnSameLane(passEdgePlace, edgeIterationDirection == BACKWARD);

	if (isfinite(veh_place.traveled_dist)) {
		std::map<int, Vehicle*>::const_iterator iter=veh_place.edge->vehicles_on_edge.begin();
		std::map<int, Vehicle*>::const_iterator end=veh_place.edge->vehicles_on_edge.end();
		for (;iter != end; ++iter) {
			if (fabs(iter->second->distFromStart() - veh_place.offset()) < 0.01) {
				return iter->second->speed();
			}
		}
	}
	return 0.0;
}

void PassObstacleManager::cleanup() {
	edges_map.clear();
	// delete our fake AnnotedEdges
	for (Route::RouteEdgeList::iterator currEdgeIt = edges_list.begin(); currEdgeIt!=edges_list.end(); ++currEdgeIt) {
		delete *currEdgeIt;
	}
	edges_list.clear();
}

} // namespace vlr
