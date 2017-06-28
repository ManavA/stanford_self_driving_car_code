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

#include <aw_RndfVertex.h>
#include <aw_RndfEdge.h>
#include <aw_RndfIntersection.h>

using namespace std;

namespace vlr {

namespace RoutePlanner {

RndfEdge::RndfEdge(RndfVertex * v1, RndfVertex * v2, string name, bool isLane, bool isVirtual, bool isLaneChange, double minSpeed,
		double maxSpeed, bool isOffroad, double weight, int id) :
      Edge<RndfVertex>(v1, v2, weight, id), is_crossing_virtual_segment( false ), name_(name),
      isLane(isLane), isCircle(false), isVirtual(isVirtual), isZone(false), isUTurn(false),
      isLaneChange(isLaneChange), isLeftLaneChange(false), isRightLaneChange(false),
      isLeftOppositeLaneChange(false), isStopLane(false), isBlocked(false), isOffroad(isOffroad),
      hasWidth(false), min_speed_(minSpeed), max_speed_(maxSpeed), length_(0), width_(4.5),
			left_boundary_(rndf::Lane::UnknownBoundary), right_boundary_(rndf::Lane::UnknownBoundary), intersection_(NULL)
{
	computeLength();
	computeTravelTime();
	if (from) {from->addOutEdge(this);}
	if (to) {to->addInEdge(this);}
}

RndfEdge::RndfEdge(double from_x, double from_y, double to_x, double to_y) :
  Edge<RndfVertex>(new RndfVertex( 0, 0, from_x, from_y ), new RndfVertex( 0, 0, to_x, to_y ), 1., 0), is_crossing_virtual_segment( false ),
	name_("plane"), isLane(true), isCircle(false), isVirtual(false), isZone(false), isUTurn(false),
	isLaneChange(false), isLeftLaneChange(false), isRightLaneChange(false), isLeftOppositeLaneChange(false), isStopLane(false), isBlocked(false), isOffroad(false),
	hasWidth(false), min_speed_(0), max_speed_(0), length_(0), width_(4.5),
	left_boundary_(rndf::Lane::UnknownBoundary), right_boundary_(rndf::Lane::UnknownBoundary), intersection_(NULL)
{
  computeLength();
  computeTravelTime();
  if (from) from->addOutEdge(this);
  if (to) to->addInEdge(this);
}

  void RndfEdge::copy_attributes_from( RndfEdge& old_edge )
  {

    isLane = old_edge.isLane;
    isCircle = old_edge.isCircle;

    isZone = old_edge.isZone;
    isUTurn = old_edge.isUTurn;
    isLaneChange = old_edge.isLaneChange;
    isLeftLaneChange = old_edge.isLeftLaneChange;
    isRightLaneChange = old_edge.isRightLaneChange;
    isLeftOppositeLaneChange = old_edge.isLeftOppositeLaneChange;
    isStopLane = old_edge.isStopLane;
    isBlocked = old_edge.isBlocked;
    hasWidth = old_edge.hasWidth;

  }

RndfEdge::~RndfEdge()
{
	// remove links
	if (intersection_) {intersection_->removeEdge(this);}
	for (TRndfEdgeSet::iterator it = left_edges_.begin(); it != left_edges_.end(); ++it)
		(*it)->right_edges_.erase(this);
	for (TRndfEdgeSet::iterator it = right_edges_.begin(); it != right_edges_.end(); ++it)
		(*it)->left_edges_.erase(this);
	for (TRndfEdgeSet::iterator it = left_oncoming_edges_.begin(); it != left_oncoming_edges_.end(); ++it)
		(*it)->left_oncoming_edges_.erase(this);
	for (TRndfEdgeSet::iterator it = crossing_edges_.begin(); it != crossing_edges_.end(); ++it)
		(*it)->crossing_edges_.erase(this);
}


void RndfEdge::computeLength() {
	RndfVertex * from = this->from;
	RndfVertex * to = this->to;
	length_ = hypot(from->x() - to->x(), from->y() - to->y());
}

void RndfEdge::computeTravelTime() {
	travel_time_ = length_ / max_speed_;
}

void RndfEdge::setWidth(double width) {
	width_ = width;
	hasWidth = true;
}

void RndfEdge::computePosition(double offset, double & x, double & y) {
	double factor = offset/length_; // check: must be in [0,1] !
	x = factor * toVertex()->x() + (1-factor) * fromVertex()->x();
	y = factor * toVertex()->y() + (1-factor) * fromVertex()->y();
}

void RndfEdge::computePosition(double offset, double & x, double & y, double & yawangle) {
	computePosition(offset, x, y);
	yawangle = atan2(toVertex()->y() - fromVertex()->y(), toVertex()->x() - fromVertex()->x());
}

void RndfEdge::computeDistance(double x, double y, double & latDist, double & lonDist, double & lonOffset) {
	// better: precompute and store dx, dy, c
	double dx = toVertex()->y() - fromVertex()->y();
	double dy = fromVertex()->x() - toVertex()->x();
	double normalLength = hypot(dx, dy);
	dx /= normalLength;
	dy /= normalLength;
	double c = -toVertex()->x() * dx - toVertex()->y() * dy;
	latDist = x * dx + y * dy + c;

	// orthogonal projection onto lane's center line
	double opx = x - dx * latDist;
	double opy = y - dy * latDist;
	assert(fabs(opx * dx + opy * dy + c) < .0001);
	double xdiff = fromVertex()->x() - opx;
	double ydiff = fromVertex()->y() - opy;
	bool forward;
	if (fabs(xdiff) > fabs(ydiff)) {
		if ( (xdiff<0 && dy<0) || (xdiff>0 && dy>0) ) {
			forward = true;
		} else {
			forward = false;
		}
	} else {
		if ( (ydiff<0 && dx>0) || (ydiff>0 && dx<0) ) {
			forward = true;
		} else {
			forward = false;
		}
	}
	lonOffset = hypot(xdiff, ydiff);
	if (forward) {
		lonDist = max(lonOffset - length_, 0.);
	} else {
		lonOffset = - lonOffset;
		lonDist = lonOffset;
	}
}


Point_2 RndfEdge::point(double offset) const
{
	Point_2 p1( from->x(), from->y() );
	Point_2 p2( to->x(), to->y() );
	if (offset > length_) offset = length_;
	if (offset < 0.) offset = 0.;
	return p1 + ( p2 - p1 ) * offset / length_;
}

Point_2 RndfEdge::fromVertexPoint() const
{
	return from->point();
}

Point_2 RndfEdge::toVertexPoint() const
{
	return to->point();
}

Vector_2 RndfEdge::vector() const
{
	return Vector_2( Point_2( from->x(), from->y() ), Point_2( to->x(), to->y() ) );
}

Segment_2 RndfEdge::segment() const
{
	return Segment_2( Point_2( from->x(), from->y() ), Point_2( to->x(), to->y() ) );
}

Line_2 RndfEdge::line() const
{
	return Line_2( Point_2( from->x(), from->y() ), Point_2( to->x(), to->y() ) );
}

//--------------------------------------------------------
//             Operators
//--------------------------------------------------------

std::ostream& operator << (std::ostream& ostrm, const RndfEdge& obj)
{
	ostrm << "RndfEdge ["<< obj.name() <<"]";
	if (obj.isLaneEdge()) ostrm << " isLaneEdge";
	if (obj.isVirtualEdge()) ostrm << " isVirtualEdge";
	if (obj.toVertex() && obj.toVertex()->isStopVertex())  ostrm << " hasStopLine";
	return ostrm;
}

}

} // namespace vlr
