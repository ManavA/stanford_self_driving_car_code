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


#include <limits>
#include <iostream>
#include <set>

#include <Eigen/Dense>

#include <global.h>
#include <aw_geometry_2d.h>

#include <aw_Vehicle.h>
#include <aw_VehicleManager.hpp>
#include <aw_Topology.hpp>
#include <aw_match_to_graph.hpp>
#include <aw_RndfGraphSearch.h>

using std::set;
using namespace Eigen;

namespace vlr {

using namespace RoutePlanner;

#undef TRACE
#define TRACE(str) std::cout << "[Vehicle] "<< str << std::endl


const double MAX_INTERSECTION_FWD_SEARCH_DIST		= 50.; // [m]
const double MAX_INTERSECTION_BWD_SEARCH_DIST		= 30.; // [m]

const double MAX_STOPLINE_FWD_SEARCH_DIST			= 50.; // [m]
const double MAX_STOPLINE_BWD_SEARCH_DIST			=  5.; // [m]


//-----------------------------------------------------------------------------
//             Vehicle
//-----------------------------------------------------------------------------

Vehicle::Vehicle() : id_(-665), x_matched_from_(-1.), y_matched_from_(-1.), yaw_matched_from_(-1.),
                     speed_(-1.), width_(-1.), length_(-1.),
                     edge_(NULL), dist_from_start_(-1.), dist_to_end_(-1.), is_blocking_(false), vman_(NULL) {
}

Vehicle::Vehicle(const perception::DynamicObstacle& vehicle, double offset_x, double offset_y) :
	id_(vehicle.id), width_(vehicle.width), length_(vehicle.length), edge_(NULL), // <- not yet matched
	vman_(NULL)
	{
	update( vehicle, offset_x, offset_y);
	}

Vehicle::Vehicle(double x, double y, double yaw, double width, double length, double speed) :
    id_(-667), x_matched_from_(x), y_matched_from_(y), yaw_matched_from_(yaw), speed_(speed), width_(width), length_(length),
    edge_(NULL), dist_from_start_(-1.), dist_to_end_(-1.), is_blocking_(false), vman_(NULL) {
}


Vehicle::~Vehicle() {
}

void Vehicle::update(const perception::DynamicObstacle& vehicle, double offset_x, double offset_y)
{
//	std::cout << "update vehicle " << vehicle.id << " to ( " << vehicle.position.x << " " << vehicle.position.y << " )\n";
	//  std::cerr << vehicle.id << " vs " << id << "\n";

	assert( vehicle.id == id_ );

	x_matched_from_ = vehicle.x + offset_x;
	y_matched_from_ = vehicle.y + offset_y;
	yaw_matched_from_ = vehicle.direction;
	speed_ = vehicle.velocity;
	turn_signal_ = vehicle.turn_signal;
	width_  = vehicle.width;
	length_ = vehicle.length;
//	printf("New vehicle @%f, %f\n",x_matched_from_, y_matched_from_);
}

void Vehicle::update( const double x, const double y, const double yaw, const double speed )
{
	x_matched_from_ = x;
	y_matched_from_ = y;
	yaw_matched_from_ = yaw;
	speed_= speed;
}



void Vehicle::match_to_graph( std::map<int, RndfEdge*>& edge_map, bool skip_graph_annotation )
{
	// Edge Matching
	double d;

	RndfEdge* old_edge = edge_;

	// no penalty for obstacles
	double angle_penalty = 0;
	if( skip_graph_annotation ) {angle_penalty = PENALTY_METER_PER_DEGREE;}

	int sign;
	edge_ = match_2_graph( x_matched_from_, y_matched_from_, yaw_matched_from_, edge_map, d, x_matched_, y_matched_, dist_from_start_, sign, angle_penalty );

	if( edge_ ) {
		dist_to_end_ = edge_->length() - dist_from_start_;

		// Vehicle mit Graph verknüpfen
		if( !skip_graph_annotation  ) {

			// alte Verknüpfungen löschen
//			if( edge != old_edge ) {
				if( old_edge ) {old_edge->vehicles_on_edge.erase( id_ );}
//			}
			if ( ! edges_.empty() ) {
				for (map< RndfEdge*, double >::iterator it = edges_.begin(); it != edges_.end(); ++it)
					it->first->vehicles_on_edge.erase( id_ );
			}


			edge_->vehicles_on_edge[ id_ ] = this;

			// Multimatching
			if ( edge_->intersection() )
			{
				// multi matching in intersections
				edges_ = multi_match_2_graph( *this, edge_->intersection()->edges() );

				// Matchings eintragen
				edges_.erase( edge_ ); // remove main edge
				for (map< RndfEdge*, double >::iterator it = edges_.begin(); it != edges_.end(); ++it) {
					it->first->vehicles_on_edge[ id_ ] = this;
//					TRACE(">>>>>>>>>>>>>>>>>>>>>  " << id << " to " << it->first->name());
				}
			} else
			{
				// multimatching für Lanechange Edges
				edges_.clear();
				set< RndfEdge* > lc_edges;
				for (set< RndfEdge* >::iterator it = edge_->fromVertex()->outEdges().begin(); it != edge_->fromVertex()->outEdges().end(); ++it)
					if ( edge_->isLaneChangeEdge() || (*it)->isLaneChangeEdge() ) lc_edges.insert( *it );
				for (set< RndfEdge* >::iterator it = edge_->toVertex()->inEdges().begin(); it != edge_->toVertex()->inEdges().end(); ++it)
					if ( edge_->isLaneChangeEdge() || (*it)->isLaneChangeEdge() ) lc_edges.insert( *it );
				edges_ = multi_match_2_graph( *this, lc_edges );

				// Matchings eintragen
				edges_.erase(edge_); // Hauptmatch-Kante rauslöschen
				for (map< RndfEdge*, double >::iterator it = edges_.begin(); it != edges_.end(); ++it) {
					it->first->vehicles_on_edge[ id_ ] = this;
//					TRACE(">>>>>>>>>>>>>>>>>>>>>  " << id << " to " << it->first->name());
				}
			}

		}
	}
}

void Vehicle::rematch_in_intersection( bool skip_graph_annotation )
{
	if (!edge_ || !edge_->isIntersectionEdge()) {return;}

	// Intersection holen und edgemap erstellen
	RndfIntersection* isec = edge_->intersection();
	map<int, RndfEdge*> edge_map;
	int i = 0;
	for (TRndfEdgeSet::const_iterator it = isec->edges().begin(); it != isec->edges().end(); ++it, ++i) {
		edge_map.insert( make_pair(i, *it) );
	}

	// Einzelne Kanten Matchings für Intersection Edges ermitteln
	map<double, RndfEdge*> matchings = getBestMatchings( *this, edge_map );

	// Kanten rausschmeißen die zu weit vom Fahrzeug weg sind
	vector< double > dels;
	for (map<double, RndfEdge*>::iterator it = matchings.begin(); it != matchings.end(); ++it) {
		if ( distToEdge( it->second ) > 5. ) {
			dels.push_back( it->first );
//			TRACE("  Del Edge: "<< it->second->name() << " ("<< distToEdge( it->second ) << ")");
		}
	}
	for (vector< double >::iterator it = dels.begin(); it != dels.end(); ++it)
		matchings.erase( *it );

	if ( matchings.empty() ) return;


	// Erlaubte Edges merken
	set< RndfEdge* > allowed_edges;
	for (map< double, RndfEdge* >::iterator it = matchings.begin(); it != matchings.end(); ++it)
		allowed_edges.insert( it->second );

//	// Kantenscores ausgeben
//	TRACE("Kanten Scores: ");
//	for (map< double, RndfEdge* >::iterator it = matchings.begin(); it != matchings.end(); ++it) {
//		TRACE("  "<< it->first <<"  "<< it->second->name());
//	}
//	TRACE("");

	// Kantenzüge initalisieren
	map<RndfEdge*, double> cum_scores;
	map<RndfEdge*, double> cum_length;
	for (map<double, RndfEdge*>::iterator it = matchings.begin(); it != matchings.end(); ++it) {
		RndfEdge* base_edge = it->second;
		double score = (it->first) * base_edge->length();
		cum_scores[ base_edge ] = score;
		cum_length[ base_edge ] = base_edge->length();
	}
	// Scores innnerhalb der einzelnen Kantenzüge aufsummieren
	for (map<double, RndfEdge*>::iterator it = matchings.begin(); it != matchings.end(); ++it) {
		RndfEdge* base_edge = it->second;
		double score = (it->first) * base_edge->length();

		// vorwärts aufsummieren
		RndfEdge* act_edge = base_edge;
		while( act_edge->fromVertex()->inEdges().size() <= 1 && act_edge->isIntersectionEdge() && allowed_edges.find( act_edge ) !=  allowed_edges.end() ) {
			if (act_edge != base_edge) {
				cum_scores[ act_edge ] += score;
				cum_length[ act_edge ] += base_edge->length();
			}
			if ( act_edge->toVertex()->outEdges().size() != 1 ) break;
			RndfEdge* next_edge = *act_edge->toVertex()->outEdges().begin();
			act_edge = next_edge;
		}

		// rückwärts aufsummieren
		act_edge = base_edge;
		while( act_edge->toVertex()->outEdges().size() <= 1 && act_edge->isIntersectionEdge() && allowed_edges.find( act_edge ) !=  allowed_edges.end() ) {
			if (act_edge != base_edge) {
				cum_scores[ act_edge ] += score;
				cum_length[ act_edge ] += base_edge->length();
			}
			if ( act_edge->fromVertex()->inEdges().size() != 1 ) break;
			RndfEdge* next_edge = *act_edge->fromVertex()->inEdges().begin();
			act_edge = next_edge;
		}
	}

	// scores mitteln und sortieren
	map< double, RndfEdge* > sorted_sums;
	for (map<RndfEdge*, double>::iterator it = cum_scores.begin(); it != cum_scores.end(); ++it)
		sorted_sums[ (it->second / cum_length[ it->first ]) ] = it->first;

//	// Summierte Kantenscores ausgeben
//	TRACE("Summierte Kanten Scores: ");
//	for (map< double, RndfEdge* >::iterator it = sorted_sums.begin(); it != sorted_sums.end(); ++it) {
//		TRACE("  "<< it->first <<"  "<< it->second->name());
//	}
//	TRACE("");

	// Besten Kantenzug erzeugen
	map<int, RndfEdge*> route_edges;
	RndfEdge* base_edge = sorted_sums.begin()->second;
	i = 0;
	route_edges.insert( make_pair( ++i, base_edge) );
	// vorwärts Kanten hinzufügen
	RndfEdge* act_edge = base_edge;
	while( act_edge->fromVertex()->inEdges().size() <= 1 && act_edge->isIntersectionEdge() && allowed_edges.find( act_edge ) !=  allowed_edges.end() ) {
		if (act_edge != base_edge)
			route_edges.insert( make_pair( ++i, act_edge) );
		if ( act_edge->toVertex()->outEdges().size() != 1 ) break;
		RndfEdge* next_edge = *act_edge->toVertex()->outEdges().begin();
		act_edge = next_edge;
	}

	// rückwärts Kanten hinzufügen
	act_edge = base_edge;
	while( act_edge->toVertex()->outEdges().size() <= 1 && act_edge->isIntersectionEdge() && allowed_edges.find( act_edge ) !=  allowed_edges.end() ) {
		if (act_edge != base_edge)
			route_edges.insert( make_pair( ++i, act_edge) );
		if ( act_edge->fromVertex()->inEdges().size() != 1 ) break;
		RndfEdge* next_edge = *act_edge->fromVertex()->inEdges().begin();
		act_edge = next_edge;
	}

//	// Kantenzug ausgeben
//	TRACE("Intersection Remacht Kantenzug: ");
//	for (map<int, RndfEdge*>::iterator it = route_edges.begin(); it != route_edges.end(); ++it) {
//		TRACE("  "<< it->second->name());
//	}
//	TRACE("");

	// Alte Graphannotierung löschen
//	if( ! skip_graph_annotation )
//		edge->vehicles_on_edge.erase( id );

	// Auf besten Kantenzug matchen und Edges annotieren
	RndfEdge* old_edge = edge_;
	double old_offset = dist_from_start_;
	match_to_graph( route_edges, true );
	if( edge_ != old_edge && ! skip_graph_annotation ) {
		if( old_edge )
			edges_.insert( make_pair( old_edge, old_offset ) );
			//old_edge->vehicles_on_edge.erase( id );
		edge_->vehicles_on_edge[ id_ ] = this;
	}
}


//-----------------------------------------------------------------------------
//		SiteVisit hacked funcs
//-----------------------------------------------------------------------------

bool
Vehicle::isAtStopline() const
{
	assert(edge_);
	if (!edge_) return false;

	RndfVertex* waypoint = edge_->toVertex();
	// TODO evtl noch die length noch besser einbinden
	if (waypoint && waypoint->isStopVertex() && dist_to_end_ - length_/2 < VEH_DIST_FROM_STOPLINE_THRESHOLD) // && dist_to_end_ - length/2> -1.0)
		return true;
	return false;
}

bool
Vehicle::isOnIntersection(RndfGraph* graph) const
{
	if( !edge_->isVirtualEdge() ) return false;

	assert(edge_);
	assert(vman_);
	assert(&(vman_->topology));
	if (graph == NULL) {
		assert(vman_);
		graph = vman_->topology.completeGraph();
	}
	assert(graph);

	RndfEdge* sedge = edge_;
	while (sedge && sedge->isVirtualEdge()) {
		sedge = graph->findIncomingEdge( sedge->fromVertex() );
	}
	RndfVertex* waypoint = sedge->toVertex();
	if (waypoint && waypoint->isStopVertex())
		return true;
	return false;

}

bool
Vehicle::isOnIntersection(double min_offset) const
{
	if( !edge_->isVirtualEdge() ) return false;

	assert(edge_);
	assert(vman_);
	RndfGraph* graph = vman_->topology.completeGraph();
	assert(graph);

	// traverese all virtual edges backward. if afterwards, im on a stop vertex,
	// vehicle must be on intersection. BUT ONLY IF IT WAS ON A VIRTUAL
	// EDGE IN THE FIRST PLACE! (jz)

	RndfEdge* sedge = edge_;
	double dist = dist_from_start_;

	while (sedge && sedge->isVirtualEdge()) {
		if (sedge != edge_) dist += sedge->length();
		sedge = graph->findIncomingEdge( sedge->fromVertex() );
	}
	RndfVertex* waypoint = sedge->toVertex();

	if( waypoint && waypoint->isStopVertex() && dist > min_offset )
		return true;
	return false;

}

bool
Vehicle::isOnStopLane() const
{
	assert(edge_);
	RndfVertex* waypoint = edge_->toVertex();
	// TODO evtl noch die length noch besser einbinden
	if (waypoint && waypoint->isStopVertex())
		return true;
	return false;
}


//-----------------------------------------------------------------------------
//		New generalized funcs
//-----------------------------------------------------------------------------


bool Vehicle::isAtStopline(const RndfIntersection* intersection) const
{
	return fabs(distToStopLine(intersection)) < VEH_DIST_FROM_STOPLINE_THRESHOLD;
}

bool Vehicle::isOnIntersection(const RndfIntersection* intersection) const
{
	return fabs(distToIntersection(intersection)) < 0.01;
}

//bool Vehicle::isOnIntersection(const RndfIntersection* intersection, double min_offset) const
//{
//	assert(false);
//	return false;
//}
//
//bool Vehicle::isOnStopLane(const RndfIntersection* intersection) const
//{
//	assert(false);
//	return false;
//}


double Vehicle::distToIntersection(const RndfIntersection* intersection) const
{
	// Prüfen ob sich das Fahrzeug auf der Intersection befindet
	assert(edge_);
//	std::cout << "  -> is on Edge ("<< edge->name() <<")"<< std::endl;
	if (edge_->intersection()) {
//		std::cout << "  -> is on Intersection ("<< edge->intersection()->getId() <<")"<< std::endl;
		if (intersection == NULL || edge_->intersection()->getId() == intersection->getId()) return 0.;
	}

	// Distanz voraus berechnen
//	std::cout << "Vehicle "<< id << ": Calc Fwd Dist to Intersection.. " << std::flush;
	double fwd_dist = fwdDistToIntersection(intersection, *edge_, -dist_from_start_);
	assert(fwd_dist >= -0.01);
	fwd_dist -= length_/2.;
	if (fwd_dist < 0.) fwd_dist = 0;
//	std::cout << fwd_dist << std::endl;

	// Distanz rückwärts berechnen
//	std::cout << "Vehicle "<< id << ": Calc Bwd Dist to Intersection.. " << std::flush;
	double bwd_dist = bwdDistToIntersection(intersection, *edge_, -dist_to_end_);
	assert(bwd_dist >= -0.01);
	bwd_dist -= length_/2.;
	if (bwd_dist < 0.) bwd_dist = 0;
//	std::cout << bwd_dist << std::endl;

	// Vorzeichen richtig setzen
	return (fwd_dist <= bwd_dist ? fwd_dist : - bwd_dist);
//	return fwd_dist;
}


double Vehicle::distToStopLine(const RndfIntersection* intersection) const
{
	// Distanz voraus berechnen
	double fwd_dist = fwdDistToStopLine(intersection, *edge_, -dist_from_start_);
	assert(fwd_dist >= 0.);
	fwd_dist -= length_/2.;

	// Distanz rückwärts berechnen
	double bwd_dist = bwdDistToStopLine(intersection, *edge_, -dist_to_end_);
	assert(bwd_dist >= 0.);
	bwd_dist += length_/2.;

	// Vorzeichen richtig setzen
	return (fwd_dist <= bwd_dist ? fwd_dist : - bwd_dist);
}


double Vehicle::fwdDistToIntersection(const RndfIntersection* intersection, const RndfEdge& edge, double act_dist) const
{
//	std::cout << "  -> examine Edge "<< edge.name() << std::endl;

	// Pfad Länge berechnen
	assert( edge.length() > 0. );
	act_dist += edge.length();
	if (act_dist > MAX_INTERSECTION_FWD_SEARCH_DIST) {
//		std::cout << "    -> max search dist reached: "<< act_dist << std::endl;
		return std::numeric_limits<double>::infinity();
	}

	// Edge auf Intersection überprüfen
	if (edge.intersection()) {
		if (intersection == NULL || edge.intersection()->getId() == intersection->getId()) {
//		std::cout << "    -> Intersection ("<< edge.intersection()->getId() <<")"<< std::endl;
		return act_dist;
		} else return std::numeric_limits<double>::infinity();
	}

	// Weitersuchen und Pfad explorieren
	double min = std::numeric_limits<double>::infinity();
	const RndfVertex* waypoint = edge.toVertex();
	assert(waypoint);
	const RndfVertex::TEdgeSet& nextEdges = waypoint->outEdges();
//	std::cout << "    -> exploring "<< nextEdges.size() << " edges" << std::endl;
	for (RndfVertex::TEdgeSet::iterator it=nextEdges.begin(); it != nextEdges.end(); ++it) {
		double res = fwdDistToIntersection(intersection, **it, act_dist);
		if (res < min) min = res;
	}
//	std::cout << "  => returning min: "<< min << std::endl;
	return (min <= MAX_INTERSECTION_FWD_SEARCH_DIST ? min : std::numeric_limits<double>::infinity());
}

double Vehicle::bwdDistToIntersection(const RndfIntersection* intersection, const RndfEdge& edge, double act_dist) const
{
//	std::cout << "  -> examine Edge "<< edge.name() << std::endl;

	// Pfad Länge berechnen
	assert( edge.length() > 0. );
	act_dist += edge.length();
	if (act_dist > MAX_INTERSECTION_FWD_SEARCH_DIST) {
//		std::cout << "    -> max search dist reached: "<< act_dist << std::endl;
		return std::numeric_limits<double>::infinity();
	}

	// Edge auf Intersection überprüfen
	if (edge.intersection()) {
		if (intersection == NULL || edge.intersection()->getId() == intersection->getId()) {
//		std::cout << "    -> Intersection ("<< edge.intersection()->getId() <<")"<< std::endl;
		return act_dist;
		} else return std::numeric_limits<double>::infinity();
	}

	// Weitersuchen und Pfad explorieren
	double min = std::numeric_limits<double>::infinity();
	const RndfVertex* waypoint = edge.fromVertex();
	assert(waypoint);
	const RndfVertex::TEdgeSet& nextEdges = waypoint->inEdges();
	for (RndfVertex::TEdgeSet::iterator it=nextEdges.begin(); it != nextEdges.end(); ++it) {
		double res = bwdDistToIntersection(intersection, **it, act_dist);
		if (res < min) min = res;
	}
	return (min <= MAX_INTERSECTION_BWD_SEARCH_DIST ? min : std::numeric_limits<double>::infinity());
}


double Vehicle::fwdDistToStopLine(const RndfIntersection* intersection, const RndfEdge& edge, double act_dist) const
{
	// Pfad Länge berechnen
	assert( edge.length() > 0. );
	act_dist += edge.length();
	if (act_dist > MAX_STOPLINE_FWD_SEARCH_DIST) return std::numeric_limits<double>::infinity();

	// Vertex am Ende der Edge holen und auf Stoppoint überprüfen
	const RndfVertex* waypoint = edge.toVertex();
	assert(waypoint);
	if (waypoint->isStopVertex()) {
		if (waypoint->isStopVertex() && ( intersection == NULL || waypoint->hasIntersectionOutEdge(intersection)))
			return act_dist;
		else return std::numeric_limits<double>::infinity();
	}

	// Weitersuchen und Pfad explorieren
	double min = std::numeric_limits<double>::infinity();
	const RndfVertex::TEdgeSet& nextEdges = waypoint->outEdges();
	for (RndfVertex::TEdgeSet::iterator it=nextEdges.begin(); it != nextEdges.end(); ++it) {
		double res = fwdDistToStopLine(intersection, **it, act_dist);
		if (res < min) min = res;
	}
	return (min <= MAX_STOPLINE_FWD_SEARCH_DIST ? min : std::numeric_limits<double>::infinity());
}


double Vehicle::bwdDistToStopLine(const RndfIntersection* intersection, const RndfEdge& edge, double act_dist) const
{
	// Pfad Länge berechnen
	assert( edge.length() > 0. );
	act_dist += edge.length();
	if (act_dist > MAX_STOPLINE_BWD_SEARCH_DIST) return std::numeric_limits<double>::infinity();

	// Vertex am Ende der Edge holen und auf Stoppoint überprüfen
	const RndfVertex* waypoint = edge.fromVertex();
	assert(waypoint);
	if (waypoint->isStopVertex()) {
		if (waypoint->isStopVertex() && ( intersection == NULL || waypoint->hasIntersectionOutEdge(intersection)))
//		if (waypoint->isStopVertex() && (intersection == NULL || (waypoint->intersection() && waypoint->intersection()->getId() == intersection->getId())))
			return act_dist;
		else return std::numeric_limits<double>::infinity();
	}

	// Weitersuchen und Pfad explorieren
	double min = std::numeric_limits<double>::infinity();
	const RndfVertex::TEdgeSet& nextEdges = waypoint->inEdges();
	for (RndfVertex::TEdgeSet::iterator it=nextEdges.begin(); it != nextEdges.end(); ++it) {
		double res = bwdDistToStopLine(intersection, **it, act_dist);
		if (res < min) min = res;
	}
	return (min <= MAX_STOPLINE_BWD_SEARCH_DIST ? min : std::numeric_limits<double>::infinity());
}


void Vehicle::blockMatchedEdges(bool /*block_multi_matching_edges*/)
{
	if (vman_) {
		vman_->setBlocked(id_);
	}
}


double Vehicle::distToMatchedEdge() const
{
	return std::sqrt( sqr(x_matched_from_ - x_matched_) + sqr(y_matched_from_ - y_matched_) );
}

double Vehicle::distToEdge(RndfEdge* edge) const
{
	RndfVertex* v1 = edge->fromVertex();
	RndfVertex* v2 = edge->toVertex();
	Segment_2 seg( Point_2( v1->x(), v1->y() ), Point_2( v2->x(), v2->y() ) );

	// Score berechen und Edge einsortieren
	return std::sqrt( squared_distance( Point_2(x_matched_from_, y_matched_from_), seg) );
}

double Vehicle::angleToMatchedEdge() const
{
	const RndfVertex* v1 = edge_->fromVertex();
	const RndfVertex* v2 = edge_->toVertex();
	Vector_2 vec( Point_2( v1->x(), v1->y() ), Point_2( v2->x(), v2->y() ) );
	return deltaAngle( angle( vec ), yaw_matched_from_ );
}

double Vehicle::angleToEdge(RndfEdge* edge) const
{
	const RndfVertex* v1 = edge->fromVertex();
	const RndfVertex* v2 = edge->toVertex();
	Vector_2 vec( Point_2( v1->x(), v1->y() ), Point_2( v2->x(), v2->y() ) );
	return deltaAngle( angle( vec ), yaw_matched_from_ );
}


void Vehicle::move(Vector_2 delta_v)
{
	x_matched_from_ += delta_v.x();
	y_matched_from_ += delta_v.y();
}


bool Vehicle::intersects(const Vehicle& veh) const
{
	Vector2d veh_m_1( x_matched_from_, y_matched_from_ );
	Vector2d veh_m_2( veh.xMatchedFrom(), veh.yMatchedFrom() );

	return rect_rect_X( veh_m_1, yaw_matched_from_, width_, length_,
			veh_m_2, veh.yawMatchedFrom(), veh.width(), veh.length());
}

//GraphPlace Vehicle::getGraphPlace() const
//{
//	return GraphPlace( edge, offset );
//}

RoutePlanner::RndfEdge* Vehicle::bestPredictedEdge(const std::map<RoutePlanner::RndfEdge*, double>& edges, double& dist_to_end) {
  if(edges.empty()) {return NULL;}
  std::map<RoutePlanner::RndfEdge*, double>::const_iterator eit = edges.begin(), eit_end = edges.end();
  for(; eit!=eit_end; eit++) {
    if(!(*eit).first->isVirtualEdge()) {
      dist_to_end = (*eit).first->length() - (*eit).second;
      return eit->first;
    }
  }

    // only virtual edges available, pick first one
  dist_to_end = (*edges.begin()).first->length() - (*edges.begin()).second;
  return edges.begin()->first;
}

RoutePlanner::RndfEdge* Vehicle::bestPredictedEdge(const RoutePlanner::TRndfEdgeSet& edges) {
if(edges.empty()) {return NULL;}
RoutePlanner::TRndfEdgeSet::const_iterator eit = edges.begin(), eit_end = edges.end();
for(; eit!=eit_end; eit++) {
  if(!(*eit)->isVirtualEdge()) {return *eit;}
}

  // only virtual edges available, pick first one
return *edges.begin();
}

void Vehicle::createCircles(MovingBox& mb) {

    double r = 0.5*mb.width;
    double delta_d = mb.length / (mb.num_circles_-1);

    double cos_psi = cos(mb.psi);
    double sin_psi = sin(mb.psi);

    double d_rear_circle = -mb.ref_offset;

    for(uint32_t i=0; i< mb.num_circles_; i++) {
    double d = d_rear_circle + i * delta_d;
    mb.circles_[i].x = mb.x + d*cos_psi;
    mb.circles_[i].y = mb.y + d*sin_psi;
    mb.circles_[i].r = r;
    }
}

void Vehicle::createCircumCircle(MovingBox& mb) {

  const double car_length_half = 0.5 * mb.length;
  mb.circum_circle_.r = car_length_half + 0.5*mb.width ; // depends on car2Circles()
  const double deltal = - mb.ref_offset + car_length_half; // go to the car center
  mb.circum_circle_.x = mb.x + deltal * cos(mb.psi);
  mb.circum_circle_.y = mb.y + deltal * sin(mb.psi);
}

void Vehicle::predictOnRNDF(double t0, double checked_horizon, double time_sample_res) {
  MovingBox mb;

  if(speed() > dgc::dgc_mph2ms(25)) {
    printf(" Found FAST (v: %f) vehicle (w: %f, l: %f) @ (%f, %f / %f):\n matched on lane %s (%f, %f; dist from start: %f\n", dgc::dgc_ms2mph(speed()), width(),
        length(), xMatchedFrom(), yMatchedFrom(), yawMatchedFrom(), edge()->name().c_str(), xMatched(), yMatched(),
        distFromStart());
  }

  mb.length = length();
  mb.width = width();
  mb.ref_offset = mb.length * 0.5;

  double dist_step = speed();

  for (double t = 0; t < checked_horizon; t += time_sample_res) {

    mb.t = t0 + t;

    double predicted_dist = dist_step*t;
    double dist_to_end=0;
    RoutePlanner::RndfEdge* pred_edge = bestPredictedEdge(edges(), dist_to_end);

      // TODO: Make this more consistent (edge vs edges) in topology library
//      if(!edge) {
    pred_edge=edge();
      if(!pred_edge) {
        throw VLRException("Found unmatched vehicle while determining vehicle predictions.");
      }
      dist_to_end=distToEnd();
//      }

    if(predicted_dist < dist_to_end) {
      std::map<int, Vehicle*>::const_iterator evit = pred_edge->vehicles_on_edge.find(id());
      if(evit==pred_edge->vehicles_on_edge.end()) {
      throw VLRException("Buhuh... :-(");
      }
      mb.x = evit->second->xMatched() + predicted_dist*cos(pred_edge->getAngle());
      mb.y = evit->second->yMatched() + predicted_dist*sin(pred_edge->getAngle());
//        mb.x = v.xMatched() + predicted_dist*cos(edge->getAngle());
//        mb.y = v.yMatched() + predicted_dist*sin(edge->getAngle());
      mb.psi = pred_edge->getAngle();
    }
    else {
      RndfVertex* vert = pred_edge->toVertex();
      if(vert->numOutEdges()==0) {
//          printf("End of graph..predicting linear.\n");
        mb.x = xMatched() + predicted_dist*cos(pred_edge->getAngle());
        mb.y = yMatched() + predicted_dist*sin(pred_edge->getAngle());
        mb.psi = pred_edge->getAngle();
      }
      else {
        predicted_dist -= dist_to_end;
        pred_edge = bestPredictedEdge(vert->outEdges());
        while (predicted_dist>pred_edge->length()) {
          vert = pred_edge->toVertex();
          predicted_dist-=pred_edge->length();
          if(vert->numOutEdges()==0) {
//              printf("End of graph..predicting linear.\n");
            break;
          }
          pred_edge = bestPredictedEdge(vert->outEdges());
        }
        mb.x = vert->x() + predicted_dist*cos(pred_edge->getAngle());
        mb.y = vert->y() + predicted_dist*sin(pred_edge->getAngle());
        mb.psi = pred_edge->getAngle();
      }
    }

    createCircumCircle(mb);
    createCircles(mb);
    predicted_traj_.push_back(mb);
  }
}

void Vehicle::predict(double t0, double checked_horizon, double time_sample_res) {
  MovingBox mb;

  if(speed() > dgc::dgc_mph2ms(25)) {
    printf(" Found FAST (v: %f) vehicle (w: %f, l: %f) @ (%f, %f / %f):\n matched on lane %s (%f, %f; dist from start: %f\n", dgc::dgc_ms2mph(speed()), width(),
        length(), xMatchedFrom(), yMatchedFrom(), yawMatchedFrom(), edge()->name().c_str(), xMatched(), yMatched(),
        distFromStart());
  }

  mb.length       = length();
  mb.width        = width();
  mb.ref_offset = mb.length * 0.5;

    double x_step = speed() * cos((double)yawMatchedFrom());
    double y_step = speed() * sin((double)yawMatchedFrom());

    for (double t = 0; t < checked_horizon; t += time_sample_res) {
      mb.t            = t0 + t;
      mb.x            = xMatchedFrom() + t*x_step;
      mb.y            = yMatchedFrom() + t*y_step;
//      mb.psi          = (speed() >=0 ? yawMatchedFrom : yawMatchedFrom-M_PI);
      mb.psi          = yawMatchedFrom();

    createCircumCircle(mb);
    createCircles(mb);
    predicted_traj_.push_back(mb);
  }
}

} // namespace vlr
