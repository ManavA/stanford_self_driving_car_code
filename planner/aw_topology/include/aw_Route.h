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


#ifndef AW_ROUTE_H
#define AW_ROUTE_H

#include <list>
#include <deque>
#include <string>
#include <iostream>

#include <aw_roadNetwork.h>
#include <aw_Maneuver.h>
#include <aw_RndfGraph.h>

namespace vlr {

#define UC_RNDF_LANE_SAMPLING_INTERVAL 1.
#define UC_RNDF_LANE_PREVIEW 200

namespace RoutePlanner {

class RouteAnnotation  {
public:
    RouteAnnotation( const std::string & objname, double abs_pos, double x, double y, double min_speed, double max_speed);
    RouteAnnotation( const std::string & objname, double abs_pos, double x, double y, double min_speed, double max_speed, maneuver_t m, area_type_t at, const std::string& way_point_name);
	virtual ~RouteAnnotation();

	inline double absoluteStartPosition() const { return absolute_start_position_; }

    inline maneuver_t maneuver() const {return maneuver_;}
	inline void maneuver(maneuver_t m) {maneuver_ = m;}

	inline area_type_t areaType() const {return area_type_;}
	inline void areaType(area_type_t at) {area_type_ = at;}

	inline void length(double l) {length_ = l;}

	inline void updatePosition(double vehicle_pos) {
	    start_distance_ = absolute_start_position_ - vehicle_pos;
	}

	void dump();
	static std::string maneuverToString(maneuver_t maneuver);
	static std::string areaTypeToString(area_type_t areaType);

private:
    maneuver_t maneuver_;
    area_type_t area_type_;     //parking, safety or travel (road) area
    double start_distance_;     // distance from now on (time when this description becomes valid);  not the euclidean distance between the car and the starting pos
    double start_x_, start_y_;  // start position of this maneuver (absolute coordinates)
    std::string way_point_name_; // RNDF Waypoint name (eg "1.1.3") TODO: really needed?!?
    double length_;
    double min_speed_, max_speed_;  // speed limits

    double absolute_start_position_;
};

class AnnotatedRouteEdge {
public:
	typedef std::list<RouteAnnotation*> AnnotationList;

	AnnotatedRouteEdge(RndfEdge * edge, double startPosition);
	AnnotatedRouteEdge( const AnnotatedRouteEdge& );
	virtual ~AnnotatedRouteEdge();

	double getStartPosition() const { return startPosition; }
	bool isFromCheckpointEdge() const { return fromCheckpointEdge; }
	bool isToCheckpointEdge() const { return toCheckpointEdge; }

	void addAnnotation(RouteAnnotation * annotation) {
		if( annotation ) annotations_.push_back(annotation);
		else printf( "zero annotation occured!\n" );
		//std::cout << "Add Edge Annotation:  "<< edge->name() <<" -> " << annotation->getManeuver().maneuver << std::endl;
	}
	std::string getNextObjectName();
	double annotateLength(double prevStartPos);
	void updateRtdb(double vehiclePosition);
	void setFromCheckpointEdge() { fromCheckpointEdge = true; }
	void setToCheckpointEdge() { toCheckpointEdge = true; }

	const AnnotationList& annotations() {return annotations_;}
	bool hasAnnotation(maneuver_t anno);
	void deleteAnnotation(maneuver_t anno);

	RndfEdge* edge() { return edge_; }
	const RndfEdge* edge() const { return edge_; }

	void dump();

	bool was_reached; // was the from-Node of this edge reached already? (used for memorizing which checkpoints had been reached btw road_planner crashes
protected:
	RndfEdge * edge_;
public:
	double startPosition;
protected:
	AnnotationList annotations_;
	bool fromCheckpointEdge, toCheckpointEdge;
};

class Route {
public:
	typedef std::deque<AnnotatedRouteEdge*> RouteEdgeList;
	typedef RndfGraph::EdgeList EdgeList;

	Route();
	virtual ~Route();

	void addEdges( EdgeList * edgeList );
	void addEdge( RndfEdge * edge ); //!< for single-checkpoint missions
	void annotateRoute();
	void init();

	double * computeIntersectionAngles(RndfEdge * edge);
	maneuver_t addVertexAnnotations(AnnotatedRouteEdge* routeEdge, RndfVertex * vertex, RndfEdge * nextEdge, bool isCheckpoint, double currentPosition, area_type_t areaType, double minSpeed, double maxSpeed, bool firstVertex);

  const RouteEdgeList& edges() const {return route;}
  RouteEdgeList& edges() {return route;}
	//RouteEdgeList::iterator getCurrentEdgeIt() { return currentEdge; }

	void dump();

	RouteEdgeList route;

protected:
	double veh_x, veh_y, veh_route;
	double routeLength;
	double minSpeed, maxSpeed;
	maneuver_t currentManeuver;
	area_type_t currentArea;
};

} // namespace vlr

} // namspace RoutePlanner

#endif
