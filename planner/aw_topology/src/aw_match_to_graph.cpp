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
#include <assert.h>
#include <set>
#include <map>
#include <sstream>
#include <Eigen/Dense>

#include <vlrException.h>
#include <aw_geometry_2d.h>

#include <aw_match_to_graph.hpp>
#include <aw_Route.h>
#include <aw_Vehicle.h>

using std::set;
using std::map;
using namespace Eigen;

namespace vlr {

#define TRACE(str) std::cout << "[M2G] "<< str << std::endl


RoutePlanner::RndfEdge* match_2_graph( const double x, const double y,
               const double yaw, const std::map<int, RoutePlanner::RndfEdge*>& edges,
               double& d,
               double& cp_x,
               double& cp_y,
               double& offset_to_first_point,
               int& sign,
               const double penalty_meter_per_degree  )
{
  //  std::cout << "penalty_meter_per_degree " << penalty_meter_per_degree << "\n";

	if (edges.empty()) {return NULL;}

  double yaw_r = yaw * ( 180. / M_PI );

  RoutePlanner::RndfEdge* best_edge = NULL;
  d = std::numeric_limits<double>::max();
  offset_to_first_point = -1;
  double yaw_r_cropped = yaw_r;

  while( yaw_r_cropped < 0. ) yaw_r_cropped += 360.;
  while( yaw_r_cropped > 360. ) yaw_r_cropped -= 360.;
  // BETTER would be  yaw_r_cropped -= floor( yaw_r_cropped/360 ) * 360;

  double best_angle = 123;

//  std::cout << "edges.size == " << edges.size() << "\n";

  for( std::map<int, RoutePlanner::RndfEdge*>::const_iterator it = edges.begin(); it != edges.end(); it++ )
    {
//      assert( it->second != 0 );

      if(!it->second) {
        char* a=NULL; *a=1;
      }
      double x1 = it->second->fromVertex()->x();
      double y1 = it->second->fromVertex()->y();

      double x2 = it->second->toVertex()->x();
      double y2 = it->second->toVertex()->y();

      double pc_x, pc_y, offset;

      closest_point_on_segment( x1, y1, x2, y2, x, y, pc_x, pc_y, offset );

      double delta_x =  x2 - x1;
      double delta_y =  y2 - y1;



      double dist = sqrt( (pc_y - y) * (pc_y - y) + (pc_x - x) * (pc_x - x) );


      // std::cout << "signed dist " << dist << " vs " << dist_signded << " vs " << dist_signded2 << "\n";


      double angle = atan2( delta_y, delta_x ) * 180./M_PI;



      double angle_diff = fabs( angle - yaw_r_cropped );

      while( angle_diff > 360. ) angle_diff -= 360;
      while( angle_diff < 0. ) angle_diff += 360;

//       assert( angle_diff < 360. );
//       assert( angle_diff > 0. );

      if( angle_diff > 180. ) angle_diff = 360. - angle_diff;

      dist += penalty_meter_per_degree * angle_diff;

      if( fabs( dist ) < fabs( d ) )
  {
    d = dist;
    best_edge = it->second;
    offset_to_first_point = offset;
    cp_x = pc_x;
    cp_y = pc_y;
    best_angle = angle;
  }
    }


//  assert(best_edge != 0);
  if(!best_edge) {
    std::stringstream s;
    s << "Object at " << x << ", " << y << " (" << yaw << ") could not be matched to graph.";
    throw VLRException(s.str());
  }
  double x1 = best_edge->fromVertex()->x();
  double y1 = best_edge->fromVertex()->y();

  double x2 = best_edge->toVertex()->x();
  double y2 = best_edge->toVertex()->y();

  double delta_x =  x2 - x1;
  double delta_y =  y2 - y1;

  double norm = sqrt( delta_x*delta_x + delta_y*delta_y );
  if (norm < 0.0001) {
//  	std::cerr << " norm very small! " << norm << std::endl;
  }
  double nx = -delta_y / norm;
  double ny = delta_x / norm;

  double dist_signded = nx * ( x - cp_x ) + ny * ( y - cp_y );

  if( dist_signded < 0 ) sign = -1;
  else sign = 1;

  //  std::cout << "signed distance " << sign * d << "\n";

  return best_edge;
}

std::map< RoutePlanner::RndfEdge*, double > multi_match_2_graph( const Vehicle& veh, const set< RndfEdge* >& edges )
{
	map< RndfEdge*, double > matched_edges;

	Vector2d veh_m( veh.xMatchedFrom(), veh.yMatchedFrom() );
	Point_2 veh_p( veh.xMatchedFrom(), veh.yMatchedFrom() );

	for (set< RndfEdge* >::iterator it = edges.begin(); it != edges.end(); ++it)
	{
		RndfEdge* edge = *it;
		RndfVertex* v1 = edge->fromVertex();
		RndfVertex* v2 = edge->toVertex();
		Point_2 p1( v1->x(), v1->y() );
		Point_2 p2( v2->x(), v2->y() );
		Vector_2 vec = p2 - p1;
		Point_2 m = p1 + vec/2.;

		if ( rect_rect_X( Vector2d( m.x(), m.y() ), angle(vec), edge->width(), edge->length(),
							veh_m, veh.yawMatchedFrom(), veh.width(), veh.length()) )
		{
			//TRACE(edge->name() << " intersects with " << veh.id);
			Line_2 l(p1, p2);
			Point_2 pp = l.projection( veh_p );
			double offset = std::sqrt( (pp - p1).squared_length() );
			if ( offset < 0. ) offset = 0.;
			if ( offset > edge->length() ) offset = edge->length();

			matched_edges.insert( make_pair( edge, offset ) );

		} else {
			//TRACE(edge->name() << " NOT intersects with " << veh.id);
		}
	}

	return matched_edges;

//	************ Alte Version die auch die Ausrichtungsdiffernz berücksichtigt hat
//
//	Point_2 pos(veh.xMatchedFrom(), veh.yMatchedFrom());
//	Vector_2 vec_y( cos(veh.yawMatchedFrom()         ) * veh.length / 2. + 1.0, sin(veh.yawMatchedFrom()         ) * veh.length / 2. + 1.0) ;
//	Vector_2 vec_x( cos(veh.yawMatchedFrom() + M_PI_2) * veh.width  / 2. + 1.0, sin(veh.yawMatchedFrom() + M_PI_2) * veh.width  / 2. + 1.0) ;
//	vector< Point_2 > coords(5, pos);
//	coords[1] = pos + vec_x + vec_y;
//	coords[2] = pos + vec_x - vec_y;
//	coords[3] = pos - vec_x + vec_y;
//	coords[4] = pos - vec_x - vec_y;
//	map< RndfEdge*, double > matched_edges;
//
//	for (vector< Point_2 >::iterator pos_it = coords.begin(); pos_it != coords.end(); ++pos_it) {
//		Point_2 pos = *pos_it;
//		for (set<RndfEdge*>::iterator it = edges.begin(); it != edges.end(); ++it)
//		{
//			RndfEdge* edge = *it;
//			RndfVertex* v1 = edge->fromVertex();
//			RndfVertex* v2 = edge->toVertex();
//			Segment_2 seg( Point_2( v1->x(), v1->y() ), Point_2( v2->x(), v2->y() ) );
//
//			// Abstand überprüfen
//			if ( squared_distance(seg, pos) > sqr( edge->laneWidth()/2. ) ) continue;
//
//			// Winkelabweichung überprüfen (max Abweichung 60°)
//			//		if ( deltaAngle( yaw, angle( seg.to_vector() ) ) > M_PI_4 * 0.66 ) continue;
//
//			// Offset bestimmen
//			Line_2 lin =  seg.supporting_line();
//			Point_2 pp = lin.projection( pos );
//			//		if ( ! seg.has_on( pp ) ) continue;
//			if ( squared_distance( seg, pp ) > 0.0001) continue;
//			double offset = sqrt( (pp - seg.source()).squared_length() );
////			if (offset < 0.) offset = 0.;
////			if (offset > edge->length()) offset = edge->length();
//			if (matched_edges.find(edge) == matched_edges.end() || matched_edges[ edge ] > offset) {
//				matched_edges[ edge ] = offset;
//			}
//		}
//	}
//
//	return matched_edges;
}

map<double, RndfEdge*> getBestMatchings( const Vehicle& veh, const map<int, RndfEdge*>& edges, const double penalty_meter_per_degree, double min_score, double max_score )
{
	Point_2 pos( veh.xMatchedFrom(), veh.yMatchedFrom() );
	map<double, RndfEdge*> result;

	for (map<int, RndfEdge*>::const_iterator it = edges.begin(); it != edges.end(); ++it) {
		RndfEdge* edge = it->second;
		RndfVertex* v1 = edge->fromVertex();
		RndfVertex* v2 = edge->toVertex();
		Segment_2 seg( Point_2( v1->x(), v1->y() ), Point_2( v2->x(), v2->y() ) );

		// Score berechen und Edge einsortieren
		double score = std::sqrt( squared_distance( pos, seg) ) + deltaAngle( angle( seg.to_vector() ), veh.yawMatchedFrom() ) * penalty_meter_per_degree;
		if (score <= min_score || score > max_score) continue;
		result[ score ] = edge;
	}

	return result;
}


double calcDeltaAngle(const Vehicle& veh, const RndfEdge* edge)
{
	const RndfVertex* v1 = edge->fromVertex();
	const RndfVertex* v2 = edge->toVertex();
	Vector_2 vec( Point_2( v1->x(), v1->y() ), Point_2( v2->x(), v2->y() ) );
	return deltaAngle( angle( vec ), veh.yawMatchedFrom() );
}


void closest_point_on_segment( double p1_x, double p1_y, double p2_x, double p2_y, double p_x, double p_y, double& pc_x, double& pc_y, double& offset_to_first_point )
{
  double vx = p2_x - p1_x;
  double vy = p2_y - p1_y;

  double wx = p_x - p1_x;
  double wy = p_y - p1_y;

  double c1 = dot_product( wx, wy, vx, vy );
  if( c1 <= 0 )
    {
      pc_x = p1_x;
      pc_y = p1_y;
      offset_to_first_point = 0;
      return;
    }

  double c2 = dot_product( vx, vy, vx, vy );
  if( c2 <= c1 )
    {
      pc_x = p2_x;
      pc_y = p2_y;
      offset_to_first_point = sqrt( c2 );
      return;
    }

  double b = c1/c2;

  offset_to_first_point = b * sqrt( c2 );

  //assert(offset_to_first_point >= 0.0);
  //assert(offset_to_first_point <= sqrt(vx*vx+vy*vy));

  pc_x = p1_x + b * vx;
  pc_y = p1_y + b * vy;

}

double dot_product( double p1_x, double p1_y, double p2_x, double p2_y )
{
  return p1_x * p2_x + p1_y * p2_y;
}

} // namespace vlr
