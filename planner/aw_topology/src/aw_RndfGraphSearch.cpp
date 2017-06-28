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
#include <aw_CGAL.h>
#include <aw_RndfGraphSearch.h>

namespace vlr {

namespace RoutePlanner {

using namespace CGAL_Geometry;

#undef TRACE
//#define TRACE(str) std::cout << "[RndfGraphSearch] "<< str << std::endl
#define TRACE(str)

//-----------------------------------------------------------------------------
//							searchOnGraph
//-----------------------------------------------------------------------------

GraphPlace searchOnGraph(GraphPlace starting_place, const GraphSearchTraits& traits, double max_scan_dist)
{
	// Suche initalisieren initalisieren
	starting_place.traveled_dist = 0.;

	// Map initialisieren die die besuchten Kanten speichert
	TVisitedMap visited;

	if (!starting_place.valid) { return GraphPlace::invalid(); }

	// Suche starten
	return searchOnGraph(starting_place, traits, max_scan_dist, NULL, visited);

}

GraphPlace searchOnGraph(const GraphPlace& place, const GraphSearchTraits& traits, double max_scan_dist, RndfEdge* last_edge, TVisitedMap& visited)
{
	assert(place.valid);
	TRACE("Testing Edge "<< place.edge->name() << ( last_edge ? "   coming from (" + last_edge->name() + ")" : "") );

	// Scandistanz testen
	if ( place.traveled_dist > max_scan_dist ) {
		TRACE(" -|  Traveldist (" << place.traveled_dist << ") > Scandist ("<< max_scan_dist <<")");
		return GraphPlace::invalid();
	}

	// Kante testen
	if ( traits.testEdge( place ) ) {
		TRACE(" -o  Edge found ("<< place.traveled_dist <<")");
		return place;
	}

	// Testen ob man die Kante expandieren darf
	if ( ! traits.expandEdge( place.edge ) ) {
		TRACE(" -|  edge not expandable");
		return GraphPlace::invalid();
	}

	// Neue Suchrichtungen erzeugen
	const TGraphSearchTraitsVec& new_dirs = traits.changeSearchDir();
	TGraphPlaceVec results;

	// Suchen starten
	for (TGraphSearchTraitsVec::const_iterator it = new_dirs.begin(); it != new_dirs.end(); ++it)
	{
		const GraphSearchTraits& traits = **it;

		// Vorwärts suchen
		if ( traits.isSearchAllowed( GraphSearchTraits::FORWARD ) )
		{
			TRACE("  -> search FORWARD");

			// Vertex testen
			RndfVertex* vertex = place.edge->toVertex();
			if ( traits.testToVertex( vertex ) ) {				// Vertex testen
				TRACE(" -o  Edge found (toVertex) ("<< place.traveled_dist <<")");
				results.push_back( GraphPlace(place).goToEdgeEnd() );
				continue;
			}
			if ( ! traits.expandToVertex( vertex ) ) {
				TRACE(" -|  toVertex not expandable");
				goto next_dir_1;
			}

			// Überprüfen ob die Kante schon besucht wurde
			TVisitedPair v_pair(place.edge, GraphSearchTraits::FORWARD);
			TVisitedMap::iterator v_it = visited.find( v_pair );
			if ( v_it != visited.end() && place.traveled_dist > v_it->second ) {
				TRACE(" -|  edge alreadry visited on a better way");
				goto next_dir_1;
			}
			visited[ v_pair ] = place.traveled_dist;

			// Suche ausweiten
			const TRndfEdgeSet& next_edges = vertex->outEdges();
			for (TRndfEdgeSet::const_iterator edge_it = next_edges.begin(); edge_it != next_edges.end(); ++edge_it) {
				RndfEdge* next_edge = *edge_it;
				if (next_edge == last_edge) continue;
				if (next_edge->isLaneChangeEdge()) continue;
				results.push_back( searchOnGraph( GraphPlace(place).goToNextEdge( next_edge ), traits, max_scan_dist, place.edge, visited ) );
			}
		}
		next_dir_1:

		// Rückwärts suchen
		if ( traits.isSearchAllowed( GraphSearchTraits::BACKWARD ) )
		{
			// Vertex testen
			RndfVertex* vertex = place.edge->fromVertex();
			if ( traits.testToVertex( vertex ) ) {				// Vertex testen
				results.push_back( GraphPlace(place).goToEdgeStart() );
				continue;
			}
			if ( ! traits.expandFromVertex( vertex ) )  goto next_dir_2;

			// Überprüfen ob die Kante schon besucht wurde
			TVisitedPair v_pair(place.edge, GraphSearchTraits::BACKWARD);
			TVisitedMap::iterator v_it = visited.find( v_pair );
			if ( v_it != visited.end() && place.traveled_dist > v_it->second ) goto next_dir_2;
			visited[ v_pair ] = place.traveled_dist;

			// Suche ausweiten
			const TRndfEdgeSet& next_edges = vertex->inEdges();
			for (TRndfEdgeSet::const_iterator edge_it = next_edges.begin(); edge_it != next_edges.end(); ++edge_it) {
				RndfEdge* next_edge = *edge_it;
				if (next_edge == last_edge) continue;
				if (next_edge->isLaneChangeEdge()) continue;
				results.push_back( searchOnGraph( GraphPlace(place).goToPrevEdge( next_edge ), traits, max_scan_dist, place.edge, visited ) );
			}
		}
		next_dir_2:

		// Auf linken Nachbarspuren suchen
		if ( traits.isSearchAllowed( GraphSearchTraits::LEFT ) )
		{
			// Überprüfen ob die Kante schon besucht wurde
			TVisitedPair v_pair(place.edge, GraphSearchTraits::LEFT);
			TVisitedMap::iterator v_it = visited.find( v_pair );
			if ( v_it != visited.end() && place.traveled_dist > v_it->second ) goto next_dir_3;
			visited[ v_pair ] = place.traveled_dist;

			// Suche ausweiten
			const TRndfEdgeSet& next_edges = place.edge->leftEdges();
			for (TRndfEdgeSet::const_iterator edge_it = next_edges.begin(); edge_it != next_edges.end(); ++edge_it) {
				RndfEdge* next_edge = *edge_it;
				if (next_edge == last_edge) continue;
				results.push_back( searchOnGraph( GraphPlace(place).goToLeftEdge( next_edge ), traits, max_scan_dist, place.edge, visited ) );
			}
		}
		next_dir_3:

		// Auf rechen Nachbarspuren suchen
		if ( traits.isSearchAllowed( GraphSearchTraits::RIGHT ) )
		{
			// Überprüfen ob die Kante schon besucht wurde
			TVisitedPair v_pair(place.edge, GraphSearchTraits::RIGHT);
			TVisitedMap::iterator v_it = visited.find( v_pair );
			if ( v_it != visited.end() && place.traveled_dist > v_it->second ) goto next_dir_4;
			visited[ v_pair ] = place.traveled_dist;

			// Suche ausweiten
			const TRndfEdgeSet& next_edges = place.edge->rightEdges();
			for (TRndfEdgeSet::const_iterator edge_it = next_edges.begin(); edge_it != next_edges.end(); ++edge_it) {
				RndfEdge* next_edge = *edge_it;
				if (next_edge == last_edge) continue;
				results.push_back( searchOnGraph( GraphPlace(place).goToRightEdge( next_edge ), traits, max_scan_dist, place.edge, visited ) );
			}
		}
		next_dir_4:

		// Auf linken Gegenfahrspuren suchen
		if ( traits.isSearchAllowed( GraphSearchTraits::LEFT_OPPOSITE ) )
		{
			// Überprüfen ob die Kante schon besucht wurde
			TVisitedPair v_pair(place.edge, GraphSearchTraits::LEFT_OPPOSITE);
			TVisitedMap::iterator v_it = visited.find( v_pair );
			if ( v_it != visited.end() && place.traveled_dist > v_it->second ) goto next_dir_5;
			visited[ v_pair ] = place.traveled_dist;

			// Suche ausweiten
			const TRndfEdgeSet& next_edges = place.edge->leftOncomingEdges();
			for (TRndfEdgeSet::const_iterator edge_it = next_edges.begin(); edge_it != next_edges.end(); ++edge_it) {
				RndfEdge* next_edge = *edge_it;
				if (next_edge == last_edge) continue;
				results.push_back( searchOnGraph( GraphPlace(place).goToLeftOncomingEdge( next_edge ), traits, max_scan_dist, place.edge, visited ) );
			}
		}
		next_dir_5:

		// Auf linken Gegenfahrspuren suchen
		if ( traits.isSearchAllowed( GraphSearchTraits::CROSSING ) )
		{
			// Überprüfen ob die Kante schon besucht wurde
			TVisitedPair v_pair(place.edge, GraphSearchTraits::CROSSING);
			TVisitedMap::iterator v_it = visited.find( v_pair );
			if ( v_it != visited.end() && place.traveled_dist > v_it->second ) goto next_dir_6;
			visited[ v_pair ] = place.traveled_dist;

			// Suche ausweiten
			const TRndfEdgeSet& next_edges = place.edge->crossingEdges();
			for (TRndfEdgeSet::const_iterator edge_it = next_edges.begin(); edge_it != next_edges.end(); ++edge_it) {
				RndfEdge* next_edge = *edge_it;
				if (next_edge == last_edge) continue;
				results.push_back( searchOnGraph( GraphPlace(place).goToCrossingEdge( next_edge ), traits, max_scan_dist, place.edge, visited ) );
			}
		}
		next_dir_6:;

	}

	// Traits Speicher freigeben
	for (TGraphSearchTraitsVec::const_iterator it = new_dirs.begin(); it != new_dirs.end(); ++it)
		if (*it != &traits) delete *it;

	// Ergebnisse auswerten
	GraphPlace res = GraphPlace::invalid();
	TRACE("Invalid Sol TravelDist: "<< res.traveled_dist);
	for (TGraphPlaceVec::iterator it = results.begin(); it != results.end(); ++it) {
		if ( ! it->valid) {
			TRACE("Solution invalid");
			continue;
		}
		if ( it->traveled_dist > res.traveled_dist) {
			TRACE("Bereits bessere Solution gefunden ("<< res.traveled_dist <<" < "<< it->traveled_dist <<")");
			continue;
		}
		res = *it;
	}

	TRACE(" => "<< (res.valid ? "TARGET FOUND" : "NO SOLUTION FOUND") << " (" << res.traveled_dist << ")");

	return res;
}

//-----------------------------------------------------------------------------
//							graph search functions
//-----------------------------------------------------------------------------

bool isOnSameLane(const GraphPlace& place, RndfEdge* target_edge, double max_scan_dist)
{
	return searchOnGraph(place, SameLaneSearchTraits(target_edge), max_scan_dist).valid;
}

bool isOnSamePrioLane(const GraphPlace& place, const RndfEdge* target_edge, const RndfIntersection* target_intersection, double max_scan_dist)
{
	return searchOnGraph(place, SamePrioLaneSearchTraits(target_edge, true, target_intersection), max_scan_dist).valid || searchOnGraph(place, SamePrioLaneSearchTraits(target_edge, false, target_intersection), max_scan_dist).valid;
}


GraphPlace searchForIntersection(const GraphPlace& place, RndfIntersection* isec, double max_scan_dist)
{
	return searchOnGraph(place, SearchIntersectionEdgeTraits(isec), max_scan_dist);
}

double distToIntersection(const GraphPlace& place, RndfIntersection* isec, double max_scan_dist)
{
	GraphPlace dist_place = searchForIntersection(place, isec, max_scan_dist);
	return dist_place.traveled_dist;
}

GraphPlace searchForVehicleOnSameLane(const GraphPlace& place, bool backwards, double max_scan_dist)
{
	return searchOnGraph(place, VehicleOnSameLaneSearchTraits(backwards), max_scan_dist);
}

GraphPlace searchCrossingPoint(const GraphPlace& place_1, const GraphPlace& place_2, const RndfIntersection* isec, double max_scan_dist)
{
	GraphPlace cross_place =  searchOnGraph(place_1, SearchCrossingEdgeTraits( place_2.edge, isec ), max_scan_dist);
	if ( ! cross_place.valid ) return cross_place;

	assert(cross_place.edge->crossingEdges().size()>0);

	// Exakten Crossing Point berechnen
	for (TRndfEdgeSet::iterator it = cross_place.edge->crossingEdges().begin(); it != cross_place.edge->crossingEdges().end(); ++it) {
		TRACE("searchCrossingPoint: " << (*it)->name());
		if ( searchOnGraph( GraphPlace( *it, 0. ), EdgeOnSameLaneNoChangeSearchTraits( place_2.edge, GraphSearchTraits::BACKWARD ), 80. ).valid )
		{
			TRACE("found");
			// Vertexes holen
			RndfVertex* v1s = cross_place.edge->fromVertex();
			RndfVertex* v1e = cross_place.edge->toVertex();
			RndfVertex* v2s = (*it)->fromVertex();
			RndfVertex* v2e = (*it)->toVertex();

			// Punkte berechnen
			Point_2 p1s( v1s->x(), v1s->y() );
			Point_2 p1e( v1e->x(), v1e->y() );
			Point_2 p2s( v2s->x(), v2s->y() );
			Point_2 p2e( v2e->x(), v2e->y() );
			Segment_2 seg_1( p1s, p1e );
			Segment_2 seg_2( p2s, p2e );

		    // Schnittpunkt berechnen
		    CGAL::Object result;
		    CGAL::Point_2<Kernel> ipoint;
		    result = CGAL::intersection(seg_1, seg_2);
		    if (CGAL::assign(ipoint, result)) {
		    	double offset = std::sqrt( (ipoint - p1s).squared_length() );
				return GraphPlace( cross_place.edge, offset);
		    }
		}
	}
	assert(false);
	return cross_place;
}

bool isOnSameLaneNoChange(const GraphPlace& place, RndfEdge* edge, GraphSearchTraits::SearchDir dir, double max_scan_dist)
{
	return searchOnGraph(place, EdgeOnSameLaneNoChangeSearchTraits(edge, dir), max_scan_dist).valid;
}

GraphPlace searchEdgeOnSameLane(const GraphPlace& place, RndfEdge* edge, GraphSearchTraits::SearchDir dir, double max_scan_dist)
{
	return searchOnGraph(place, EdgeOnSameLaneNoChangeSearchTraits(edge, dir), max_scan_dist);
}

GraphPlace searchDistOnLane(const GraphPlace& start_place, GraphSearchTraits::SearchDir dir, double max_scan_dist )
{
	GraphPlace res = searchOnGraph(start_place, SearchDistOnLaneTraits( dir, max_scan_dist ), max_scan_dist);
	if ( res.valid && res.traveled_dist < max_scan_dist ) {
		assert( res.traveled_dist <= max_scan_dist +0.01);
		if ( dir == GraphSearchTraits::FORWARD )
			res.offset_ += max_scan_dist - res.traveled_dist;
		else if ( dir == GraphSearchTraits::BACKWARD )
			res.offset_ -= max_scan_dist - res.traveled_dist;
		res.traveled_dist = max_scan_dist;
		assert( res.offset_ <= res.edge->length() +0.01 );
		assert( res.offset_ >= -0.01 );
	}
	return res;
}


//-----------------------------------------------------------------------------
//							searchOnRoute
//-----------------------------------------------------------------------------

RoutePlace searchOnRoute()
{
	// TODO impln
	assert(false);
}


}  // namespace RoutePlanner

} // namespace vlr
