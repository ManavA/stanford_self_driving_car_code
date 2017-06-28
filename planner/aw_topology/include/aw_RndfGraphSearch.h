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


#ifndef AW_RNDFGRAPHSEARCH_H
#define AW_RNDFGRAPHSEARCH_H

#include <math.h>
#include <bitset>
#include <limits>
#include <aw_Route.h>
#include <aw_RndfGraph.h>
#include <aw_RndfEdge.h>
#include <aw_Vehicle.h>

using std::bitset;

namespace vlr {

namespace RoutePlanner {

//=============================================================================
//
//							Places
//
//=============================================================================

//-----------------------------------------------------------------------------
//							GraphPlace
//-----------------------------------------------------------------------------

struct GraphPlace
{
	GraphPlace() : edge( NULL ), offset_( 0. ), traveled_dist( std::numeric_limits<double>::infinity() ), valid( false) {}
	GraphPlace(RndfEdge* edge, double offset = 0.) : edge( edge ), offset_( offset ), traveled_dist( 0. ), valid( edge != NULL ) {
		if (offset_ < 0.0) offset_ = 0.0;
		if (edge && edge->length() < offset_) offset_ = edge->length();
	}

	double offset() const {return offset_;}

	GraphPlace& goToEdgeStart() {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		traveled_dist += offset_;
		offset_ = 0.;
		return *this;
	}

	GraphPlace& goToEdgeEnd() {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		traveled_dist += edge->length() - offset_;
		offset_ = edge->length();
		return *this;
	}

	GraphPlace& goToOffset(double new_offset) {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		assert(new_offset >= 0 && new_offset <=edge->length());
		traveled_dist += fabs( new_offset - offset_ );
		offset_ = new_offset;
		return *this;
	}

	GraphPlace& goToPrevEdge(RndfEdge* prev_edge) {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		assert(edge->fromVertex()); assert(edge->fromVertex()->hasInEdge(prev_edge));
		assert( ! prev_edge->isLaneChangeEdge() ); assert(edge != prev_edge);
		traveled_dist += offset_;
		offset_ = prev_edge->length();
		edge = prev_edge;
		return *this;
	}

	GraphPlace& goToNextEdge(RndfEdge* next_edge) {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		assert(edge->toVertex()); assert(edge->toVertex()->hasOutEdge(next_edge));
		assert( ! next_edge->isLaneChangeEdge() ); assert(edge != next_edge);
		traveled_dist += edge->length() - offset_;
		offset_ = 0;
		edge = next_edge;
		return *this;
	}

	GraphPlace& goToLeftEdge(RndfEdge* left_edge, double new_offset = 0.) {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		assert(edge->hasLeftEdge(left_edge)); assert(edge != left_edge);
		assert(edge->laneWidth() > 0.); assert(left_edge->laneWidth() > 0.);
		// BETTER Distanz über Projektion bestimmen und Offset entsprechend setzen
		traveled_dist += (edge->laneWidth() + left_edge->laneWidth()) / 2.;
		offset_ = new_offset;
		edge = left_edge;
		if (offset_ < 0.) offset_ = 0.;
		if (offset_ > edge->length()) offset_ = edge->length();
		return *this;
	}

	GraphPlace& goToRightEdge(RndfEdge* right_edge, double new_offset = 0.) {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		assert(edge->hasRightEdge(right_edge)); assert(edge != right_edge);
		assert(edge->laneWidth() > 0.); assert(right_edge->laneWidth() > 0.);
		// BETTER Distanz über Projektion bestimmen und Offset entsprechend setzen
		traveled_dist += (edge->laneWidth() + right_edge->laneWidth()) / 2.;
		offset_ = new_offset;
		edge = right_edge;
		if (offset_ < 0.) offset_ = 0.;
		if (offset_ > edge->length()) offset_ = edge->length();
		return *this;
	}

	GraphPlace& goToLeftOncomingEdge(RndfEdge* left_opposite_edge, double new_offset = 0.) {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		assert(edge->hasLeftOncomingEdge(left_opposite_edge)); assert(edge != left_opposite_edge);
		assert(edge->laneWidth() > 0.); assert(left_opposite_edge->laneWidth() > 0.);
		// BETTER Distanz über Projektion bestimmen und Offset entsprechend setzen
		traveled_dist += (edge->laneWidth() + left_opposite_edge->laneWidth()) / 2.;
		offset_ = new_offset;
		edge = left_opposite_edge;
		if (offset_ < 0.) offset_ = 0.;
		if (offset_ > edge->length()) offset_ = edge->length();
		return *this;
	}

	GraphPlace& goToCrossingEdge(RndfEdge* crossing_edge, double new_offset = 0.) {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		assert(edge->hasCrossingEdge(crossing_edge)); assert(edge != crossing_edge);
		assert(edge->laneWidth() > 0.); assert(crossing_edge->laneWidth() > 0.);
		// BETTER Distanz über Schnitt bestimmen und Offset entsprechend setzen
		traveled_dist += fabs(edge->length()/2. - offset_) + crossing_edge->length()/2.;
		offset_ = new_offset;
		edge = crossing_edge;
		if (offset_ < 0.) offset_ = 0.;
		if (offset_ > edge->length()) offset_ = edge->length();
		return *this;
	}


	GraphPlace& goToLeftEdge() {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		if ( edge->leftEdges().empty() ) {
			*this = GraphPlace::invalid();
			return *this;
		}
		const TRndfEdgeSet& next_edges = edge->leftEdges();
		double min_dist = std::numeric_limits<double>::infinity();
		RndfEdge* best_edge = NULL;
		double min_alt_dist = std::numeric_limits<double>::infinity();
		RndfEdge* best_alt_edge = NULL;
		for (TRndfEdgeSet::const_iterator edge_it = next_edges.begin(); edge_it != next_edges.end(); ++edge_it) {
			RndfEdge* next_edge = *edge_it;
			Point_2 pp = next_edge->line().projection( point() );
			double dist = squared_distance( next_edge->segment(), pp );
			if ( dist < 0.00001 ) {
				dist = squared_distance( next_edge->fromVertexPoint(), pp );
				if (dist < min_dist) {
					min_dist = dist;
					best_edge = next_edge;
				}
			} else {
				if (dist < min_alt_dist) {
					min_alt_dist = dist;
					best_alt_edge = next_edge;
				}
			}
		}
		if ( best_edge ) return this->goToLeftEdge( best_edge, sqrt(min_dist) );
		if ( best_alt_edge ) {
			Point_2 pp = best_alt_edge->line().projection( point() );
			double offset_ = ( squared_distance( pp, best_alt_edge->fromVertexPoint() ) < squared_distance( pp, best_alt_edge->toVertexPoint() ) ? 0. : best_alt_edge->length() );
			return this->goToLeftEdge( best_alt_edge, offset_ );
		}
		*this = GraphPlace::invalid();
		return *this;
	}

	GraphPlace& goToRightEdge() {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		if ( edge->rightEdges().empty() ) {
			*this = GraphPlace::invalid();
			return *this;
		}
		const TRndfEdgeSet& next_edges = edge->rightEdges();
		double min_dist = std::numeric_limits<double>::infinity();
		RndfEdge* best_edge = NULL;
		double min_alt_dist = std::numeric_limits<double>::infinity();
		RndfEdge* best_alt_edge = NULL;
		for (TRndfEdgeSet::const_iterator edge_it = next_edges.begin(); edge_it != next_edges.end(); ++edge_it) {
			RndfEdge* next_edge = *edge_it;
			Point_2 pp = next_edge->line().projection( point() );
			double dist = squared_distance( next_edge->segment(), pp );
			if ( dist < 0.00001 ) {
				dist = squared_distance( next_edge->fromVertexPoint(), pp );
				if (dist < min_dist) {
					min_dist = dist;
					best_edge = next_edge;
				}
			} else {
				if (dist < min_alt_dist) {
					min_alt_dist = dist;
					best_alt_edge = next_edge;
				}
			}
		}
		if ( best_edge ) return this->goToRightEdge( best_edge, sqrt(min_dist) );
		if ( best_alt_edge ) {
			Point_2 pp = best_alt_edge->line().projection( point() );
			double offset_ = ( squared_distance( pp, best_alt_edge->fromVertexPoint() ) < squared_distance( pp, best_alt_edge->toVertexPoint() ) ? 0. : best_alt_edge->length() );
			return this->goToRightEdge( best_alt_edge, offset_ );
		}
		*this = GraphPlace::invalid();
		return *this;
	}

	GraphPlace& goToLeftOncomingEdge() {
		assert(valid); assert(edge); assert(offset_ >= -0.001 && offset_ <= edge->length()+0.001);
		if ( edge->leftOncomingEdges().empty() ) {
			*this = GraphPlace::invalid();
			return *this;
		}
		const TRndfEdgeSet& next_edges = edge->leftOncomingEdges();
		double min_dist = std::numeric_limits<double>::infinity();
		RndfEdge* best_edge = NULL;
		double min_alt_dist = std::numeric_limits<double>::infinity();
		RndfEdge* best_alt_edge = NULL;
		for (TRndfEdgeSet::const_iterator edge_it = next_edges.begin(); edge_it != next_edges.end(); ++edge_it) {
			RndfEdge* next_edge = *edge_it;
			Point_2 pp = next_edge->line().projection( point() );
			double dist = squared_distance( next_edge->segment(), pp );
			if ( dist < 0.00001 ) {
				dist = squared_distance( next_edge->fromVertexPoint(), pp );
				if (dist < min_dist) {
					min_dist = dist;
					best_edge = next_edge;
				}
			} else {
				if (dist < min_alt_dist) {
					min_alt_dist = dist;
					best_alt_edge = next_edge;
				}
			}
		}
		if ( best_edge ) return this->goToLeftOncomingEdge( best_edge, sqrt(min_dist) );
		if ( best_alt_edge ) {
			Point_2 pp = best_alt_edge->line().projection( point() );
			double offset_ = ( squared_distance( pp, best_alt_edge->fromVertexPoint() ) < squared_distance( pp, best_alt_edge->toVertexPoint() ) ? 0. : best_alt_edge->length() );
			return this->goToLeftOncomingEdge( best_alt_edge, offset_ );
		}
		*this = GraphPlace::invalid();
		return *this;
	}





	Point_2 point() const { return (valid && edge ? edge->point(offset_) : Point_2( std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() ) ); }

	static GraphPlace invalid() { return GraphPlace(); }


	RndfEdge* edge;
	double offset_;
	double traveled_dist;
	bool valid;
};


//-----------------------------------------------------------------------------
//							RoutePlace
//-----------------------------------------------------------------------------

struct RoutePlace
{
	RoutePlace(const RoutePlanner::Route::RouteEdgeList& route, RoutePlanner::Route::RouteEdgeList::const_iterator edge_it, double offset = 0., bool valid = true) :
		route(route), edge_it(edge_it), offset_(offset), valid(valid)
	{}

	const RoutePlanner::Route::RouteEdgeList& route;
	RoutePlanner::Route::RouteEdgeList::const_iterator edge_it;
	double offset_;
	bool valid;
};

// typedef PlaceOnGraph		RoutePlace;


//=============================================================================
//
//							Search Traits
//
//=============================================================================

//-----------------------------------------------------------------------------
//							GraphSearchTraits
//-----------------------------------------------------------------------------

struct GraphSearchTraits;
typedef std::vector< const GraphSearchTraits* >	TGraphSearchTraitsVec;


struct GraphSearchTraits
{
	enum SearchDir {
		FORWARD,
		BACKWARD,
		LEFT,
		RIGHT,
		LEFT_OPPOSITE,
		CROSSING,
		SEARCH_DIR_COUNT
	};

	GraphSearchTraits(double max_scan_dist = 0.) : max_scan_dist(max_scan_dist) {}
	virtual ~GraphSearchTraits() {}

	virtual GraphSearchTraits* copy() const { return new GraphSearchTraits(*this); }
	virtual GraphSearchTraits* clear_copy() const {
		GraphSearchTraits* c = copy();
		c->search_dirs.reset();
		return c;
	}


	virtual bool testEdge(const GraphPlace&) const			{ return false; }
	virtual bool testFromVertex(const RndfVertex*) const 	{ return false; }
	virtual bool testToVertex(const RndfVertex*) const		{ return false; }

	virtual bool expandEdge(const RndfEdge*) const				{ return true; }
	virtual bool expandFromVertex(const RndfVertex*) const	{ return true; }
	virtual bool expandToVertex(const RndfVertex*) const		{ return true; }


	virtual const TGraphSearchTraitsVec changeSearchDir() const		{ return TGraphSearchTraitsVec(1, this); }

	GraphSearchTraits* clearSearchDirs() 							{ search_dirs.reset(); return this; }

	void setSearchDir(SearchDir dir, bool v = true)					{ search_dirs.set(dir, v); }
	void unsetSearchDir(SearchDir dir)								{ search_dirs.set(dir, 0); }
	bool isSearchAllowed(SearchDir dir)	const						{ return search_dirs.test(dir); }

	// Convenience Functions
	void setForwardSearch(bool v = true)							{ search_dirs.set(FORWARD, v); }
	void setBackwardSearch(bool v = true)							{ search_dirs.set(BACKWARD, v); }
	void setLeftSearch(bool v = true)								{ search_dirs.set(LEFT, v); }
	void setRightSearch(bool v = true)								{ search_dirs.set(RIGHT, v); }
	void setLeftOppositeSearch(bool v = true)						{ search_dirs.set(LEFT_OPPOSITE, v); }
	void setCrossingSearch(bool v = true)							{ search_dirs.set(CROSSING, v); }


	bitset<SEARCH_DIR_COUNT> search_dirs;
	double max_scan_dist;
};

//-----------------------------------------------------------------------------
//							RouteSearchTraits
//-----------------------------------------------------------------------------

struct RouteSearchTraits : GraphSearchTraits
{
	virtual ~RouteSearchTraits() {}

	virtual bool testRouteEdge(const AnnotatedRouteEdge& /*route_edge*/) const { return true; }
};


//=============================================================================
//
//							Search Routines
//
//=============================================================================

typedef std::pair< RndfEdge*, GraphSearchTraits::SearchDir >    TVisitedPair;
typedef std::map< TVisitedPair, double >                        TVisitedMap;
typedef std::vector<GraphPlace>                                 TGraphPlaceVec;

#undef TRACE
#define TRACE(str) std::cout << "[RndfGraphSearch] "<< str << std::endl
//#define TRACE(str)

//-----------------------------------------------------------------------------
//							searchOnGraph
//-----------------------------------------------------------------------------

//! main search routine for finding places on the graph
/*! the graph is searched recursively and returns the the place that fits the search traits
 * 	with the minimal distance to the \a starting_place.
 */
GraphPlace searchOnGraph(GraphPlace starting_place, const GraphSearchTraits& traits, double max_scan_dist);

//! internalliy used recursiv search funktion
GraphPlace searchOnGraph(const GraphPlace& place, const GraphSearchTraits& traits, double max_scan_dist, RndfEdge* last_edge, TVisitedMap& visited);


//-----------------------------------------------------------------------------
//							searchOnRoute
//-----------------------------------------------------------------------------

RoutePlace searchOnRoute();


//=============================================================================
//
//							User Search Traits
//
//=============================================================================

//-----------------------------------------------------------------------------
//							SameLaneSearchTraits
//-----------------------------------------------------------------------------

struct SameLaneSearchTraits : public GraphSearchTraits
{
	SameLaneSearchTraits(const RndfEdge* target_edge) : GraphSearchTraits(), target_edge(target_edge) {
		assert(target_edge);
		setForwardSearch();
//		setBackwardSearch();
		setLeftSearch();
		setRightSearch();
	}

	virtual GraphSearchTraits* copy() const { return new SameLaneSearchTraits(*this); }

	virtual bool testEdge(const GraphPlace& place) const	{ return place.edge == target_edge; }

	const RndfEdge* target_edge;
};

//-----------------------------------------------------------------------------
//							SamePrioLaneSearchTraits
//-----------------------------------------------------------------------------

struct SamePrioLaneSearchTraits : public GraphSearchTraits
{
	SamePrioLaneSearchTraits(const RndfEdge* target_edge, bool backwards, const RndfIntersection* isec = NULL) : GraphSearchTraits(), target_edge(target_edge), isec(isec) {
		assert(target_edge);
		if (backwards) {
			setBackwardSearch();
			setLeftSearch();
			setRightSearch();
		} else {
			setForwardSearch();
			setLeftSearch();
			setRightSearch();
		}
	}

	virtual GraphSearchTraits* copy() const { return new SamePrioLaneSearchTraits(*this); }

	virtual bool testEdge(const GraphPlace& place) const	{
		return place.edge == target_edge;
	}

	virtual bool expandEdge(const RndfEdge* edge) const	{
		return (edge->isPriorityLaneEdge() &&
				! edge->isZoneEdge() && !
				edge->isUTurnEdge() &&
				(edge->intersection() == NULL || isec == NULL || edge->intersection() == isec));
	}

	const RndfEdge* target_edge;
	const RndfIntersection* isec;
};



//-----------------------------------------------------------------------------
//							SearchIntersectionEdgeTraits
//-----------------------------------------------------------------------------

struct SearchIntersectionEdgeTraits : public GraphSearchTraits
{
	SearchIntersectionEdgeTraits(const RndfIntersection* target_intersection = NULL) : GraphSearchTraits(), target_intersection(target_intersection) {
		setForwardSearch();
		setLeftSearch();
		setRightSearch();
	}

	virtual bool testEdge(const GraphPlace& place) const	{ return (place.edge->intersection() && (target_intersection == NULL || place.edge->intersection()->getId() == target_intersection->getId()) ); };

	virtual bool expandEdge(const RndfEdge* edge) const	{ return (edge->intersection() == NULL || target_intersection == NULL || edge->intersection()->getId() == target_intersection->getId()); };


	const RndfIntersection* target_intersection;
};

//-----------------------------------------------------------------------------
//							VehicleOnSameLaneSearchTraits
//-----------------------------------------------------------------------------

struct VehicleOnSameLaneSearchTraits : public GraphSearchTraits
{
	VehicleOnSameLaneSearchTraits(bool backwards) : GraphSearchTraits(), backwards(backwards) {
		if (!backwards) {
			setForwardSearch();
		} else {
			setBackwardSearch();
		}
	}

	virtual GraphSearchTraits* copy() const { return new VehicleOnSameLaneSearchTraits(*this); }

	virtual bool testEdge(const GraphPlace& place) const	{
		if (!backwards) {
			double min_offset = std::numeric_limits<double>::infinity();
			std::map<int, Vehicle*>::const_iterator iter=place.edge->vehicles_on_edge.begin();
			std::map<int, Vehicle*>::const_iterator end=place.edge->vehicles_on_edge.end();
			for (;iter != end; ++iter) {
				if (iter->second->distFromStart() < min_offset) {
					min_offset = iter->second->distFromStart();
				}
			}
			if (std::isfinite(min_offset) && (min_offset >= place.offset())) {
				return true;
			}
		} else {
			double max_offset = -1;
			std::map<int, Vehicle*>::const_iterator iter=place.edge->vehicles_on_edge.begin();
			std::map<int, Vehicle*>::const_iterator end=place.edge->vehicles_on_edge.end();
			for (;iter != end; ++iter) {
				if (iter->second->distFromStart() > max_offset) {
					max_offset = iter->second->distFromStart();
				}
			}
			if ((max_offset >= place.offset())) {
				return true;
			}
		}
		return false;
	}

	bool backwards;
};

//-----------------------------------------------------------------------------
//							EdgeOnSameLaneNoChangeSearchTraits
//-----------------------------------------------------------------------------

struct EdgeOnSameLaneNoChangeSearchTraits : public GraphSearchTraits
{
	EdgeOnSameLaneNoChangeSearchTraits(const RndfEdge* edge, SearchDir dir = FORWARD) :
                                      GraphSearchTraits(), target_edge(edge) {
//                                      GraphSearchTraits(), target_edge(target_edge) {
		setSearchDir( dir );
	}

	virtual GraphSearchTraits* copy() const { return new EdgeOnSameLaneNoChangeSearchTraits(*this); }

	virtual bool testEdge(const GraphPlace& place) const { return ( place.edge == target_edge ); }

	const RndfEdge* target_edge;
};

//-----------------------------------------------------------------------------
//							SearchDistOnLaneTraits
//-----------------------------------------------------------------------------

struct SearchDistOnLaneTraits : public GraphSearchTraits
{
	SearchDistOnLaneTraits(SearchDir dir, double max_scan_dist) : GraphSearchTraits(max_scan_dist) {
		setSearchDir( dir );
	}

	virtual bool testEdge(const GraphPlace& place) const {
//		TRACE("Traveled_Dist: "<< place.traveled_dist<< "   ("<< max_scan_dist <<")");
		if ( isSearchAllowed( FORWARD ) )
			return ( place.traveled_dist + place.edge->length() - place.offset() >= max_scan_dist );
		else if ( isSearchAllowed( BACKWARD ) )
			return ( place.traveled_dist + place.offset() >= max_scan_dist );
		else return false;
	}

	virtual bool expandEdge(const RndfEdge* edge) const	            { return ( ! edge->isIntersectionEdge() && ! edge->isBlockedEdge() && ! edge->isZoneEdge() && ! edge->isUTurnEdge() ); } // Evtl noch Virtual Edges prüfen
};


//-----------------------------------------------------------------------------
//							SearchCrossingEdgeTraits
//-----------------------------------------------------------------------------

struct SearchCrossingEdgeTraits : public GraphSearchTraits
{
	SearchCrossingEdgeTraits(const RndfEdge* target_edge, const RndfIntersection* target_intersection = NULL) : GraphSearchTraits(), target_edge(target_edge), target_intersection(target_intersection) {
		setForwardSearch();
	}

	virtual bool testEdge(const GraphPlace& place) const	{
		if (place.edge->intersection() && (target_intersection == NULL || place.edge->intersection()->getId() == target_intersection->getId()) ) {
//			TRACE(" Starting Search over crossing Edges ("<< place.edge->crossingEdges().size() <<"):");
			for (TRndfEdgeSet::iterator it = place.edge->crossingEdges().begin(); it != place.edge->crossingEdges().end(); ++it) {
//				TRACE("  -> "<< (*it)->name() );
				if ( searchOnGraph( GraphPlace( *it, 0. ), EdgeOnSameLaneNoChangeSearchTraits( target_edge, BACKWARD ), 80. ).valid )
					return true;
			}
		}
		return false;
	}

	virtual bool expandEdge(const RndfEdge* edge) const	{ return (edge->intersection() == NULL || target_intersection == NULL || edge->intersection()->getId() == target_intersection->getId()); }

	const RndfEdge* target_edge;
	const RndfIntersection* target_intersection;
};


//=============================================================================
//
//							User Search Routines
//
//=============================================================================

//-----------------------------------------------------------------------------
//							graph search functions
//-----------------------------------------------------------------------------

bool isOnSameLane(const GraphPlace& place, RndfEdge* target_edge, double max_scan_dist);

bool isOnSamePrioLane(const GraphPlace& place, const RndfEdge* target_edge, const RndfIntersection* target_intersection, double max_scan_dist);


//! searches for intersection \a isec
GraphPlace searchForIntersection(const GraphPlace& place, RndfIntersection* isec = NULL, double max_scan_dist = 100.);

double distToIntersection(const GraphPlace& place, RndfIntersection* isec = NULL, double max_scan_dist = 100.);

GraphPlace searchForVehicleOnSameLane(const GraphPlace& place, bool backwards, double max_scan_dist = 100.);

bool isOnSameLaneNoChange(const GraphPlace& place, RndfEdge* edge, GraphSearchTraits::SearchDir dir = GraphSearchTraits::FORWARD, double max_scan_dist = 100.);
GraphPlace searchEdgeOnSameLaneNoChange(const GraphPlace& place, RndfEdge* edge, GraphSearchTraits::SearchDir dir = GraphSearchTraits::FORWARD, double max_scan_dist = 100.);

//! searches for the crossing point on intersection \a isec going forward from each place
GraphPlace searchCrossingPoint(const GraphPlace& place_1, const GraphPlace& place_2, const RndfIntersection* isec = NULL, double max_scan_dist = 100.);

GraphPlace searchDistOnLane(const GraphPlace& start_place, GraphSearchTraits::SearchDir dir, double max_scan_dist = 100. );

}  // namespace RoutePlanner

#undef TRACE

} // namespace vlr

#endif /*RNDFGRAPHSEARCH_H_*/
