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


#ifndef AW_GRAPH_TOOLS_HPP
#define AW_GRAPH_TOOLS_HPP

#include <aw_CGAL.h>
#include <aw_Route.h>
#include <aw_RndfGraph.h>
#include <aw_RndfEdge.h>

namespace vlr {

namespace RoutePlanner {
class RndfIntersection;
class RndfEdge;
}

using namespace RoutePlanner;
using CGAL_Geometry::Point_2;

// some functions to easily traverse the mission graph,
// distance calculations on graph... (jz)
namespace GraphTools
{
// only used internally
struct Point {
	Point( double x_, double y_ ): x( x_ ), y( y_ ) {}
	double x;
	double y;
};

// describes a place on the mission graph (edge and offset on edge)
class PlaceOnGraph
{
public:
	// constructor
	PlaceOnGraph();
	PlaceOnGraph( const PlaceOnGraph& original );
	PlaceOnGraph& operator=(const PlaceOnGraph& original);

	bool isValid() const { return valid; };
	double offset() const {return offset_;}

	PlaceOnGraph( RoutePlanner::Route::RouteEdgeList::const_iterator edge, double offset, const RoutePlanner::Route::RouteEdgeList& edges );

	/*!
	 * @remark: if u use this constructor, vehicle _must_ have been matched to the mission graph
	 * (not the "whole" graph!)
	 * u can do so with vehicle.match_to_graph( Topology.mission_graph_map ) (but this might not be what you want)
	 */
	//PlaceOnGraph( Vehicle& vehicle, RoutePlanner::Route::RouteEdgeList& edges );

	// move forward
	// meter _must_ be positive!
	void operator+=( double meter );
	Vehicle* prev_obstacle( double& d, PlaceOnGraph& place, double max_scan_distance );

	// move backward
	// meter _must_ be positive!
	void operator-=( double meter );
	RoutePlanner::AnnotatedRouteEdge* next_zone_exit( double& d, PlaceOnGraph& place, double max_scan_distance );

	// return the place with the next obstacle. d will contain distance to obstalce. d will be <0 if no obstacle found
	Vehicle* next_obstacle( double& d, PlaceOnGraph& place, double max_scan_distance );

	RoutePlanner::AnnotatedRouteEdge* next_manuever( double& d, PlaceOnGraph& place, double max_scan_distance, const maneuver_t maneuver );
	RoutePlanner::AnnotatedRouteEdge* next_manuever( double& d, PlaceOnGraph& place, double max_scan_distance, const std::vector<maneuver_t>& maneuvers );
	RoutePlanner::AnnotatedRouteEdge* next_area( double& d, PlaceOnGraph& place, double max_scan_distance, const area_type_t area );

	//! returns the first edge of the given intersection \a isec forward on the route
	RoutePlanner::AnnotatedRouteEdge* go_fwd_to_intersection( double& d, double max_scan_distance, const RoutePlanner::RndfIntersection* isec );
	//! returns the first edge of the given intersection \a isec backward on the route
	RoutePlanner::AnnotatedRouteEdge* go_bwd_to_intersection( double& d, double max_scan_distance, const RoutePlanner::RndfIntersection* isec );

  RoutePlanner::AnnotatedRouteEdge* go_fwd_to_crosswalk(double& d, double max_scan_distance);
  RoutePlanner::AnnotatedRouteEdge* go_bwd_to_crosswalk(double& d, double max_scan_distance);

	//! returns the edge of which the toPoint is a stoppoint and accords to the given intersection \a isec forward on the route
	RoutePlanner::AnnotatedRouteEdge* go_fwd_to_stopline( double& d, double max_scan_distance, const RoutePlanner::RndfIntersection* isec );
	//! returns the edge of which the toPoint is a stoppoint and accords to the given intersection \a isec backward on the route
	RoutePlanner::AnnotatedRouteEdge* go_bwd_to_stopline( double& d, double max_scan_distance, const RoutePlanner::RndfIntersection* isec );

	RoutePlanner::AnnotatedRouteEdge* go_fwd_to_traffic_light( double& d, double max_scan_distance, const RoutePlanner::RndfIntersection* isec );
	RoutePlanner::AnnotatedRouteEdge* go_bwd_to_traffic_light( double& d, double max_scan_distance, const RoutePlanner::RndfIntersection* isec );

    // distances to traffic light that does not belong to any intersection (e.g. meter light)
	RoutePlanner::AnnotatedRouteEdge* go_fwd_to_traffic_light(double& d, double max_scan_distance);
  RoutePlanner::AnnotatedRouteEdge* go_bwd_to_traffic_light(double& d, double max_scan_distance);

	// make a 2D point with x and y, for visualisation
	Point make_point() const;

	Point_2 point() const;

public:
	const RoutePlanner::Route::RouteEdgeList* graph;
	RoutePlanner::Route::RouteEdgeList::const_iterator edge_;
	double offset_;

private:
	bool valid;
};

// distance from first to second. second _must_ be after first (else returns -1.0).
double difference( const PlaceOnGraph& second, const PlaceOnGraph& first );

std::set<RoutePlanner::RndfEdge*> getSurroundingEdges(RoutePlanner::RndfEdge* edge, RoutePlanner::RndfGraph* complete_graph, double search_distance);

std::map<int, RoutePlanner::RndfEdge*> getSiblingEdges(RoutePlanner::RndfEdge* edge, double max_search_distance);


} // namespace GraphTools

} // namespace vlr

#endif
