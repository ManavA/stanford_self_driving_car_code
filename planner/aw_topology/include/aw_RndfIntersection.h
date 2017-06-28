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


#ifndef AW_RNDFINTERSECTION_H
#define AW_RNDFINTERSECTION_H

#include <string>
#include <set>
#include <boost/utility.hpp>
#include <aw_CGAL.h>

namespace vlr {

namespace RoutePlanner {
class RndfGraphBuilder;
class RndfEdge;
typedef std::set<RndfEdge*> TRndfEdgeSet;


class RndfIntersection : boost::noncopyable
{
  struct point {
    double x;
    double y;
  };

public:
	RndfIntersection(int id);
	~RndfIntersection();

	bool addEdge(RndfEdge* edge);
	void removeEdge(RndfEdge* edge);
	size_t numEdges() const;
	bool hasEdge(RndfEdge* edge) const;

	const TRndfEdgeSet& edges() const;

	int getId() const {return id; }

  void updateCenter();
  CGAL_Geometry::Point_2 center() const;

  void updateRadius();    // also updates center
  double radius() const;

	void paint_segment_with_offset( RndfEdge* scanning_edge, double offset );

	std::vector<point> get_boundary_lines();
protected:
	int id;
private:
	TRndfEdgeSet edges_;
	CGAL_Geometry::Vector_2 sum_;
	double sum_length_;
	unsigned int center_count_;
	CGAL_Geometry::Point_2 center_;
	double radius_;
	bool center_calculated_;
	bool radius_calculated_;
};


} // namespace RoutePlanner

} // namespace vlr

#endif // AW_RNDFINTERSECTION_H
