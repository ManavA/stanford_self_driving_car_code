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


#ifndef MATCH_TO_GRAPH_HPP
#define MATCH_TO_GRAPH_HPP

#include <map>
#include <limits>
#include <aw_CGAL.h>
#include <aw_RndfGraph.h>
#include <aw_RndfVertex.h>
#include <aw_RndfEdge.h>

using namespace CGAL_Geometry;

namespace vlr {

using namespace RoutePlanner;

const double PENALTY_METER_PER_DEGREE = 4.5/90;
const double REPLANNING_PENALTY_METER_PER_RAD = 5./M_PI_2;
const double THRESHOLD_OLD_POSITION = 15.;


RoutePlanner::RndfEdge* match_2_graph( const double x, const double y,
				       const double yaw, const std::map<int, RoutePlanner::RndfEdge*>& edges,
				       double& d,
				       double& cp_x,
				       double& cp_y,
				       double& offset_to_first_point,
				       int& sign,
				       const double penalty_meter_per_angle = PENALTY_METER_PER_DEGREE
				       );

//! matches a vehicle to multiple edges if there is no single best solution
/*! used to get better matching results in intersections. The result is a map of
 * 	mached edges and the offeset of the matched positions
 */
std::map< RoutePlanner::RndfEdge*, double >
						multi_match_2_graph( const Vehicle& veh, const std::set<RoutePlanner::RndfEdge*>& edges );

//! matches the \a vehicle to \edges and returns a sorted map with scores and the according edges
/*! the smaller the score the better the matching
 * */
std::map<double, RoutePlanner::RndfEdge*>
						getBestMatchings( const Vehicle& vehicle, const std::map<int, RoutePlanner::RndfEdge*>& edges, const double penalty_meter_per_degree = REPLANNING_PENALTY_METER_PER_RAD, double min_score = -1.0, double max_score = numeric_limits<double>::infinity() );

//! calculates the minimal delta angle (rad) between the \vehilce s yaw and the edge out of [0, PI]
double calcDeltaAngle(const Vehicle& vehicle, const RndfEdge* edge);


void closest_point_on_segment( const double p1_x, const double p1_y, const double p2_x, const double p2_y, const double p_x, double p_y, double& pc_x, double& pc_y, double& offset_to_first_point );

double dot_product( const double p1_x, const double p1_y, const double p2_x, const double p2_y );

} // namespace vlr

#endif
