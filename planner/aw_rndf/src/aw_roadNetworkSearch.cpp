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


#define DEBUG_LEVEL 0

#include <float.h>
#include <Eigen/Dense>
#include <aw_roadNetworkSearch.h>
#include <aw_lane.h>
#include <aw_perimeterPoint.h>
#include <aw_roadNetwork.h>

using namespace std;
using namespace Eigen;

namespace vlr {

namespace rndf {

RoadNetworkSearch::RoadNetworkSearch(RoadNetwork* rndf) :
  rndf_(rndf) {
  lane_tree_ = new LaneQuadTree(rndf, 1.0);
}

RoadNetworkSearch::~RoadNetworkSearch() {
  delete lane_tree_;
}

RoadNetworkSearch::RoadNetworkSearch(const RoadNetworkSearch& rns, RoadNetwork* rndf) {
  if (!rndf) {
    rndf_ = rns.rndf_;
  }
  else {
    rndf_ = rndf;
  }

  lane_tree_ = new LaneQuadTree(*rns.lane_tree_);
}

rndf::Lane* RoadNetworkSearch::closest_lane(double utm_x, double utm_y) const {
  Lane* l;
  double distance = DBL_MAX;
  closest_lane(utm_x, utm_y, l, distance);
  return l;
}

void RoadNetworkSearch::closest_lane(double utm_x, double utm_y, Lane*& l, double& distance) const {
  LaneQuadTree::Element* closest_element = NULL;
  Vector2d p(utm_x, utm_y);
  Vector2d closest_point(DBL_MAX, DBL_MAX);
  distance = DBL_MAX;
  l = NULL;
  if (!lane_tree_->search_nearest_neighbor(p, closest_element, distance, closest_point)) if (DEBUG_LEVEL > 0) fprintf(stderr,
      "BUG!!! RNDF NN SEARCH RETURNED FALSE!\n");
  if (closest_element != NULL && closest_element->l != NULL) l = closest_element->l;
  else if (DEBUG_LEVEL > 0) fprintf(stderr, "BUG!!! CLOSEST ELEMENT / LANE IS NULL!\n");
}

bool RoadNetworkSearch::within_lane(double utm_x, double utm_y) const {
  Lane* l = NULL;
  double distance;
  closest_lane(utm_x, utm_y, l, distance);
  if (l == NULL) {
    if (DEBUG_LEVEL > 0) fprintf(stderr, "BUG!!!! LANE IS NULL!\n");
    return false;
  }
  distance = sqrt(distance);
  if (distance > 0.5 * l->laneWidth() + 0.5) return false;
  else return true;
}

WayPoint* RoadNetworkSearch::closest_waypoint(double utm_x, double utm_y) {
  double min_dist2 = DBL_MAX;
  WayPoint* w = NULL;

  const TWayPointMap& waypoints = rndf_->wayPoints();
  TWayPointMap::const_iterator it, it_end;

  for (it = waypoints.begin(), it_end = waypoints.end(); it != it_end; ++it) {
    double x = utm_x - (*it).second->utmX();
    double y = utm_y - (*it).second->utmY();
    double dist2 = x * x + y * y;
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      w = (*it).second;
    }

  }

  return w;
}

PerimeterPoint* RoadNetworkSearch::closest_perimeterpoint(double utm_x, double utm_y) {
  double min_dist2 = DBL_MAX;
  PerimeterPoint* p = NULL;

  const TPerimeterPointMap& perimeterpoints = rndf_->perimeterPoints();
  TPerimeterPointMap::const_iterator it, it_end;

  for (it = perimeterpoints.begin(), it_end = perimeterpoints.end(); it != it_end; ++it) {
    double x = utm_x - (*it).second->utmX();
    double y = utm_y - (*it).second->utmY();
    double dist2 = x * x + y * y;
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      p = (*it).second;
    }
  }

  return p;
}

TrafficLight* RoadNetworkSearch::closestTrafficLight(double utm_x, double utm_y) {
  double min_dist2 = DBL_MAX;
  TrafficLight* tl = NULL;

  const TTrafficLightMap& traffic_lights = rndf_->trafficLights();
  TTrafficLightMap::const_iterator it, it_end;

  for (it = traffic_lights.begin(), it_end = traffic_lights.end(); it != it_end; ++it) {
    double x = utm_x - (*it).second->utmX();
    double y = utm_y - (*it).second->utmY();
    double dist2 = x * x + y * y;
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      tl = (*it).second;
    }
  }

  return tl;
}

Crosswalk* RoadNetworkSearch::closestCrosswalk(double utm_x, double utm_y) {
  double min_dist2 = DBL_MAX;
  Crosswalk* cw = NULL;

  const TCrosswalkMap& crosswalks = rndf_->crosswalks();
  TCrosswalkMap::const_iterator it, it_end;

  for (it = crosswalks.begin(), it_end = crosswalks.end(); it != it_end; ++it) {
    double center_x = 0.5 * ((*it).second->utmX1() + (*it).second->utmX2());
    double center_y = 0.5 * ((*it).second->utmY1() + (*it).second->utmY2());
    double x = utm_x - center_x;
    double y = utm_y - center_y;
    double dist2 = x * x + y * y;
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      cw = (*it).second;
    }
  }

  return cw;
}
} // namespace rndf

} // namespace vlr

