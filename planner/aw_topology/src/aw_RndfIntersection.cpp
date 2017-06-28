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


#include <assert.h>
#include <vlrException.h>
#include <aw_RndfIntersection.h>
#include <aw_RndfEdge.h>
#include <aw_RndfVertex.h>

using namespace std;

namespace vlr {

namespace RoutePlanner {

RndfIntersection::RndfIntersection(int id) :
	id(id),
	sum_(CGAL_Geometry::Vector_2(0,0)),
	sum_length_(0),
	center_count_(0),
	center_(CGAL_Geometry::Point_2(0,0)),
	center_calculated_(false),
	radius_(0),
	radius_calculated_(false) {
}

RndfIntersection::~RndfIntersection() {
}

bool RndfIntersection::addEdge(RndfEdge* edge) {
	assert(edge);
	pair<TRndfEdgeSet::iterator,bool> p = edges_.insert(edge);
	if (p.second) {
		sum_ = sum_ + CGAL_Geometry::Vector_2(edge->fromVertex()->x(), edge->fromVertex()->y())*edge->length() + CGAL_Geometry::Vector_2(edge->toVertex()->x(), edge->toVertex()->y())*edge->length();
		sum_length_ += edge->length();
    center_calculated_ = false;
		radius_calculated_ = false;
	}
	return p.second;
}

void RndfIntersection::removeEdge(RndfEdge* edge)
{
	if (edges_.find(edge) == edges_.end()) return;
	assert(edge->fromVertex() && edge->toVertex());
	sum_ = sum_ - CGAL_Geometry::Vector_2(edge->fromVertex()->x(), edge->fromVertex()->y())*edge->length() - CGAL_Geometry::Vector_2(edge->toVertex()->x(), edge->toVertex()->y())*edge->length();
	sum_length_ -= edge->length();
  center_calculated_ = false;
	radius_calculated_= false;
	edges_.erase(edge);
}

size_t RndfIntersection::numEdges() const {
	return edges_.size();
}

bool RndfIntersection::hasEdge(RndfEdge* edge) const {
	return (edges_.find(edge) != edges_.end());
}

const TRndfEdgeSet& RndfIntersection::edges() const {
	return edges_;
}

void RndfIntersection::updateCenter() {
	if (!center_calculated_) {
		center_ = CGAL_Geometry::Point_2(0,0) + (sum_ / (sum_length_*2.0));
		center_count_ = edges_.size();
		center_calculated_ = true;
	}
}

CGAL_Geometry::Point_2 RndfIntersection::center() const {
  if (!center_calculated_) {
    throw VLRException("Center not up to date. Call updateCenter() first.");
	}
	return center_;
}

void RndfIntersection::updateRadius() {
  updateCenter(); // ensure that center is up-to-date
		radius_ = 0;
		for (TRndfEdgeSet::const_iterator iter = edges_.begin(); iter != edges_.end(); ++iter) {
			double r = hypot((*iter)->toVertex()->x() - center_.x(), (*iter)->toVertex()->y() - center_.y());
			if (r > radius_) {
				radius_ = r;
			}
		}
		radius_calculated_ = true;
}

double RndfIntersection::radius() const {
	if (!radius_calculated_) {
	  throw VLRException("Radius not up to date. Call updateRadius() first.");
	}
	return radius_;
}

void RndfIntersection::paint_segment_with_offset( RndfEdge* scanning_edge, double offset )
{
  std::cout << "Hi\n";

  assert( scanning_edge );
  // one intersection entry found!

  // scan forw
  while( true )
    {
      std::cout << "Di\n";
      //	  if( scanning_edge->toVertex()->outEdges().size() != 1 ) break;

      double x1 = scanning_edge->fromVertex()->x();
      double y1 = scanning_edge->fromVertex()->y();

      double x2 = scanning_edge->toVertex()->x();
      double y2 = scanning_edge->toVertex()->y();

      double dx = x2 - x1;
      double dy = y2 - y1;

      double norm = sqrt( dx*dx + dy*dy );

      double n1 = -dy / norm;
      double n2 =  dx / norm;

      std::cout << "ILINE " << x1 + n1 * offset << " " << y1 + n2 * offset << "\n";
      std::cout << "ILINE " << x2 + n1 * offset << " " << y2 + n2 * offset << "\n  \n  \n";


      scanning_edge = *( scanning_edge->toVertex()->outEdges().begin() );

      if( edges_.find( scanning_edge ) == edges_.end() )
	break;

    }
}

std::vector<RndfIntersection::point> RndfIntersection::get_boundary_lines()
{
  std::vector<RndfIntersection::point> result;
  set<RndfEdge*> entries;
  for( set<RndfEdge*>::iterator it = edges_.begin(); it != edges_.end(); it++ )
    {
      RndfEdge* scanning_edge = *it;
      // scan backw
      while( true )
	{
	  std::cout << "hop\n";
	  if( edges_.find( scanning_edge ) == edges_.end() ) break;
	  assert( scanning_edge->fromVertex()->inEdges().size() == 1 );
	  RndfEdge* prev_scanning_edge = *(scanning_edge->fromVertex()->inEdges().begin());
	  //	  if( scanning_edge->fromVertex()->outEdges().size() != 1 ) break;
	  scanning_edge = prev_scanning_edge;
	}
      entries.insert( scanning_edge );
    }

  std::cout << "# ILINE found " << entries.size() << " entries\n"<< "\n  \n  \n";
  for( set<RndfEdge*>::iterator it = entries.begin(); it != entries.end(); it++ )
    {
      std::cout << "Ho!\n";

      RndfEdge* leftest = 0;
      RndfEdge* rightest = 0;

      double a_min = 999999;
      double a_max = -999999;
      for( set<RndfEdge*>::iterator out_it = (*it)->toVertex()->outEdges().begin(); out_it != (*it)->toVertex()->outEdges().end(); out_it++ )
	{
	  RndfEdge* preview_edge = *out_it;
	  for( int i=0; i<2; i++ )
	    {
	      if( preview_edge->toVertex()->outEdges().size() == 1 )
		{
		  RndfEdge* pre_preview_edge = preview_edge;
		  pre_preview_edge = *(preview_edge->toVertex()->outEdges().begin());
		  if( edges_.find( pre_preview_edge ) != edges_.end() )
		    preview_edge = pre_preview_edge;
		}
	    }
	  double dx = preview_edge->toVertex()->x() -  (*out_it)->fromVertex()->x();
	  double dy = preview_edge->toVertex()->y() -  (*out_it)->fromVertex()->y();
	  double a = atan2( dy, dx );
	  if( a <= a_min )
	    {
	      a_min = a;
	      rightest = *out_it;
	    }
	  if( a >= a_max )
	    {
	      a_max = a;
	      leftest = *out_it;
	    }
	}
      std::cout << "a_min a_max " << a_min << " " << a_max << "\n";
#warning add tolerance here!

      //      paint_segment_with_offset( leftest, 0 );

      //      if( (*it)->leftBoundaryType() == rndf::lane::SolidWhite )

      // einer zurueck!
      RndfEdge* approaching_edge = *it;
      if( approaching_edge->fromVertex()->inEdges().size() == 1 )
	approaching_edge = *(approaching_edge->fromVertex()->inEdges().begin());

      if( approaching_edge->leftOncomingEdges().size() == 0 )
	{
	  paint_segment_with_offset( leftest,   0.4 * (*it)->width() );
	  paint_segment_with_offset( rightest, -0.4 * (*it)->width() );      	}
      else
	paint_segment_with_offset( rightest, -0.4 * (*it)->width() );
      //      if( approaching_edge->rightEdges().size() == 0 )


    }
  return result;
}


} //namespace RoutePlanner

} // namespace vlr
