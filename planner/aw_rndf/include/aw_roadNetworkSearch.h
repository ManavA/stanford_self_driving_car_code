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


#ifndef AW_RNDFSEARCH_H_
#define AW_RNDFSEARCH_H_

#include <aw_laneQuadTree.h>

namespace vlr {

namespace rndf
{
class WayPoint;
class PerimeterPoint;
class Lane;
class TrafficLight;
class Crosswalk;

class RoadNetworkSearch {

public:
	RoadNetworkSearch(RoadNetwork* rndf);
	~RoadNetworkSearch();
  RoadNetworkSearch(const RoadNetworkSearch& rns,  RoadNetwork* rndf=NULL);

  // methods for searching  way points

	//  // returns the closest way point to a point utm_x, utm_y
	//  rndf_waypoint_p closest_waypoint(double utm_x, double utm_y) const;
	//  // returns the closest way point and the distance to a point utm_x, utm_y
	//  void closest_waypoint(double utm_x, double utm_y, rndf_waypoint_p& waypoint, double& distance) const;

	// returns the closest way point to a point utm_x, utm_y
	WayPoint* closest_waypoint(double utm_x, double utm_y);

  // returns the closest perimeter point to a point utm_x, utm_y
  PerimeterPoint* closest_perimeterpoint(double utm_x, double utm_y);

  // returns the closest perimeter point to a point utm_x, utm_y
  TrafficLight* closestTrafficLight(double utm_x, double utm_y);

  Crosswalk* closestCrosswalk(double utm_x, double utm_y);

    // returns the closest Lane to a point utm_x, utm_y
	Lane* closest_lane(double utm_x, double utm_y) const;
	// returns the closest Lane and the distance to a point utm_x, utm_y
	void closest_lane(double utm_x, double utm_y, Lane*& l, double& distance) const;
	// returns true if point utm_x, utm_y is inside a Lane
	bool within_lane(double utm_x, double utm_y) const;

	LaneQuadTree* get_lane_quadtree() { return lane_tree_; }

private:
	RoadNetwork*   rndf_;
	LaneQuadTree*  lane_tree_;
};

} // NAMESPACE rndf

} // namespace vlr

#endif /*RNDFSEARCH_H_*/
