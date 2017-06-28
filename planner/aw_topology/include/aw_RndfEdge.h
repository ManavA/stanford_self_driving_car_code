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


#ifndef AW_RNDFEDGE_H
#define AW_RNDFEDGE_H

#include <string>
#include <iostream>
#include <aw_lane.h>
#include <aw_Graph.h>
#include <aw_CGAL.h>

namespace vlr {

// forward declarations
struct Vehicle;

namespace RoutePlanner {
using CGAL_Geometry::Point_2;
using CGAL_Geometry::Vector_2;
using CGAL_Geometry::Segment_2;
using CGAL_Geometry::Line_2;

class RndfGraphBuilder;
class RndfVertex;
class RndfIntersection;
class RndfEdge;

typedef std::set<RndfEdge*> TRndfEdgeSet;


class RndfEdge : public Edge<RndfVertex> {
public:
	friend class RndfGraph;
	friend class RndfGraphBuilder;

	RndfEdge(RndfVertex* v1, RndfVertex* v2, std::string name, bool isLane, bool isVirtual, bool isLaneChange, double minSpeed, double maxSpeed, bool isOffroad, double weight, int id);

	// create a plain edge with new vertices, no name, no annotaions, no lat-long...
	// I use this as a quick hack and only for doing graph matching
	RndfEdge( double from_x, double from_y, double to_x, double to_y );

	virtual ~RndfEdge();

	virtual RndfVertex* fromVertex() { return from; }
	virtual RndfVertex const * fromVertex() const { return from; }
	virtual RndfVertex* toVertex() { return to; }
	virtual RndfVertex const * toVertex() const { return to; }
	std::string name() const { return name_; }
	void copy_attributes_from( RndfEdge& );
	bool isBlockedEdge() const { return isBlocked; }
	bool isLaneEdge() const { return isLane; }
	bool isZoneEdge() const { return isZone; }
	bool isCircleEdge() const { return isCircle; }
	bool isVirtualEdge() const { return isVirtual; }
	bool isLaneChangeEdge() const { return RndfEdge::isLaneChange; }
	bool isLeftLaneChangeEdge() const { return isLeftLaneChange; }
	bool isRightLaneChangeEdge() const { return isRightLaneChange; }
	bool isLeftOppositeLaneChangeEdge() const { return isLeftOppositeLaneChange; }
	bool isUTurnEdge() const { return isUTurn; }
	bool isStopLaneEdge() const { return isStopLane; }
	bool isPriorityLaneEdge() const { return !isStopLane; }
	bool isIntersectionEdge() const { return (intersection_ != NULL); }
	bool isOffroadEdge() const { return isOffroad; }
	bool hasLaneWidth() const { return hasWidth; }
	double width() const { return width_; }
	double length() const { return length_; }
	double travelTime() const { return travel_time_; }
	double minSpeed() const { return min_speed_; }
	double maxSpeed() const { return max_speed_; }
	double laneWidth() const { return width_; }
	rndf::Lane::eBoundaryTypes leftBoundaryType() const { return left_boundary_; }
	rndf::Lane::eBoundaryTypes rightBoundaryType() const { return right_boundary_; }
	RndfIntersection* intersection() const { return intersection_; }

	void setBlocked(bool v = true) { isBlocked = v; }
	void setZone() { isZone = true; }
	void setUTurn(bool v = true) { isUTurn = v; }
	void setStopLane(bool v = true) { isStopLane = v; }
	void setLeftLaneChange(bool v = true) { isLeftLaneChange = v; }
	void setRightLaneChange(bool v = true) { isRightLaneChange = v; }
	void setLeftoppositeLaneChange(bool v = true) { isLeftOppositeLaneChange = v; }
	void setOffroad(bool v = true) { isOffroad = v; }

	void setWidth(double width);
	void setLeftBoundary(rndf::Lane::eBoundaryTypes boundary) { left_boundary_ = boundary; }
	void setRightBoundary(rndf::Lane::eBoundaryTypes boundary) { right_boundary_ = boundary; }

	void setVirtual() { isVirtual = true; }

	virtual void computePosition(double offset, double & x, double & y);
	virtual void computePosition(double offset, double & x, double & y, double & yawangle);
	virtual void computeDistance(double x, double y, double & latDist, double & lonDist, double & lonOffset);

	std::map<int, Vehicle*> vehicles_on_edge;

	bool hasLeftEdge(RndfEdge* edge) const			{ return left_edges_.find(edge) != left_edges_.end(); }
	bool hasRightEdge(RndfEdge* edge) const			{ return right_edges_.find(edge) != right_edges_.end(); }
	bool hasLeftOncomingEdge(RndfEdge* edge) const	{ return left_oncoming_edges_.find(edge) != left_oncoming_edges_.end(); }
	bool hasCrossingEdge(RndfEdge* edge) const		{ return crossing_edges_.find(edge) != crossing_edges_.end(); }

	const TRndfEdgeSet& leftEdges() const			{ return left_edges_; }
	const TRndfEdgeSet& rightEdges() const			{ return right_edges_; }
	const TRndfEdgeSet& leftOncomingEdges() const	{ return left_oncoming_edges_; }
	const TRndfEdgeSet& crossingEdges() const 		{ return crossing_edges_; }

  void addLeftEdge(RndfEdge* edge) {left_edges_.insert(edge);}
  void addRightEdge(RndfEdge* edge) {right_edges_.insert(edge);}
  void addLeftOncomingEdge(RndfEdge* edge) {left_oncoming_edges_.insert(edge);}
  void addCrossingEdge(RndfEdge* edge) {crossing_edges_.insert(edge);}

  void addVehicle(int id, Vehicle* vehicle) {vehicles_on_edge.insert(std::make_pair(id, vehicle));}
  void removeVehicle(int id) {vehicles_on_edge.erase(id);}
  const std::map<int, Vehicle*>& vehicles() const {return vehicles_on_edge;}

	Point_2 point(double offset) const;
	Point_2 fromVertexPoint() const;
	Point_2 toVertexPoint() const;
	Vector_2 vector() const;
	Segment_2 segment() const;
	Line_2 line() const;
	double getAngle() const { return CGAL_Geometry::angle( vector() ); }

	bool is_crossing_virtual_segment;
public:
	virtual void computeLength();
	virtual void computeTravelTime();

	std::string name_;

	bool isLane;
	bool isCircle;
	bool isVirtual;
	bool isZone;
	bool isUTurn;
	bool isLaneChange;
	bool isLeftLaneChange;
	bool isRightLaneChange;
	bool isLeftOppositeLaneChange;
	bool isStopLane;
	bool isBlocked;
	bool isOffroad;
	bool hasWidth;

	double min_speed_, max_speed_;   //!< [m/s]
	double length_;               //!< [m]
	double travel_time_;           //!< [s]
	double width_;                //!< [m]

	rndf::Lane::eBoundaryTypes left_boundary_;   // TODO: make independent of rndf library...
	rndf::Lane::eBoundaryTypes right_boundary_;

	RndfIntersection* intersection_;

	//! set of left adjacent eges in the same direction
	TRndfEdgeSet left_edges_;
	//! set of right adjacent edges in the same direction
	TRndfEdgeSet right_edges_;
	//! set of left adjacent edges in the opposite direction
	TRndfEdgeSet left_oncoming_edges_;

	//! set of crossing edges
	TRndfEdgeSet crossing_edges_;
};

//--------------------------------------------------------
//             Operators
//--------------------------------------------------------
std::ostream& operator << (std::ostream& ostrm, const RndfEdge& obj);

}

} // namespace vlr

#endif // AW_RNDFEDGE_H
