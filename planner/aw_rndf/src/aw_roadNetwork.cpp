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


#include <time.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <set>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <aw_CGAL.h>
#include <CGAL/Sweep_line_2_algorithms.h>
#include <global.h>
#include <lltransform.h>

#include <aw_roadNetwork.h>
#include <aw_StringTools.h>
#include <aw_intersection.h>

#include <gl_support.h>
#include "aw_roadNetworkGL.h"

namespace drc = driving_common;

using namespace std;
//using namespace CGAL;
using namespace vlr::CStringTools;
using namespace CGAL_Geometry;

//using CGAL_Geometry::Point_2;
//using CGAL_Geometry::Vector_2;
//using CGAL_Geometry::Line_2;
//using CGAL_Geometry::Segment_2;

//typedef	Cartesian<double>	CS;
//typedef CS::Point_2			Point_2;
//typedef CS::Vector_2		Vector_2;
//typedef CS::Line_2			Line_2;
//typedef CS::Segment_2		Segment_2;

// ------- Konstanten ---------------------------------------------------------
const double EDGE_SAMPLE_DISTANCE = 20.0;
const uint32_t MAX_TRANSITIVE_EDGES_SEARCH_DEPTH = 5;

// ------- Macros -------------------------------------------------------------
//#define TRACE(str) cout << str << endl;
#define TRACE(str)

namespace vlr {

namespace rndf {

RoadNetwork::RoadNetwork(const string& name, bool create_visualization) :
  format_version_(0), creation_date_(""), rndfgl_(NULL) {
  setName(name);

  if (create_visualization) {
    rndfgl_ = new RoadNetworkGL;
  }
}

RoadNetwork::RoadNetwork(const RoadNetwork& other) {
  copy(other);
}

RoadNetwork::~RoadNetwork() {
}

RoadNetwork& RoadNetwork::operator=(const RoadNetwork& other) {
  return (copy(other));
}

RoadNetwork& RoadNetwork::copy(const RoadNetwork& other) {
  clear();

  name_ = other.name_;
  return *this;
}

RoadNetwork* RoadNetwork::clone() const {
  return new RoadNetwork(*this);
}

void RoadNetwork::createVisualization() {
  if (!rndfgl_) {
    rndfgl_ = new RoadNetworkGL;
  }
}

bool RoadNetwork::saveRNDF(const string& strFileName) {
  // save file
  std::cout << "Saving RNDF to " << strFileName;

  ofstream file(strFileName.c_str());
  if (!file.is_open()) {
    cout << " failed. Could not open file." << endl;
    return false;
  }
  file << *this;
  std::cout << " done." << std::endl;
  return true;
}

bool RoadNetwork::loadRNDF(const string& strFileName) {
  // clear road network
  clear();

  // load file
  ifstream file(strFileName.c_str());
  string line;
  vector<string> tokens;
  int numZones = -1;
  int numSegments = -1;
  int numIntersections = -1;

  if (!file.is_open() || !file.good()) {
    setStatus("Cannot open '" + strFileName + "'");
    return false;
  }

  // FIRST PASS: insert segments, zones, lanes and WayPoints
  while (true) {
    tokens.clear();
    if (!getline(file, line)) break;
    if (!line.length()) continue;
    line = clearCComments(line);
    splitString(line, tokens, RNDF_DELIMITER);
    if (tokens.empty()) continue;

    if (tokens[0] == RNDF_ROADNETWORK_NAME && tokens.size() > 1) setName(tokens[1]);
    else if (tokens[0] == SRNDF_HEADER) { // ignore SRNDF addons for now
    }
    else if (tokens[0] == SRNDF_IDSTRING) { // ignore SRNDF addons for now
    }
    else if (tokens[0] == SRNDF_LIBVERSION) { // ignore SRNDF addons for now
    }
    else if (tokens[0] == RNDF_ROADNETWORK_NUM_SEGMENTS && tokens.size() > 1) numSegments = CStringTools::gnCInt(tokens[1]);
    else if (tokens[0] == RNDF_ROADNETWORK_NUM_ZONES && tokens.size() > 1) numZones = CStringTools::gnCInt(tokens[1]);
    else if (tokens[0] == RNDF_ROADNETWORK_NUM_INTERSECTIONS && tokens.size() > 1) numIntersections = CStringTools::gnCInt(tokens[1]);
    else if (tokens[0] == RNDF_ROADNETWORK_FORMAT_VERSION && tokens.size() > 1) format_version_ = CStringTools::gdCDouble(tokens[1]);
    else if (tokens[0] == RNDF_ROADNETWORK_CREATION_DATE && tokens.size() > 1) creation_date_ = tokens[1];
    else if (tokens[0] == RNDF_SEGMENT_BEGIN && tokens.size() > 1) {
      string strSegmentName = tokens[1];
      string strData = section(file, RNDF_SEGMENT_BEGIN, RNDF_SEGMENT_END);
      if (!strData.length()) continue;
      addSegment(strSegmentName, strData);
    }
    else if (tokens[0] == RNDF_INTERSECTION_BEGIN && tokens.size() > 1) {
      string strIntersectionName = tokens[1];
      string strData = section(file, RNDF_INTERSECTION_BEGIN, RNDF_INTERSECTION_END);
      if (!strData.length()) continue;
      addIntersection(strIntersectionName, strData);
    }
    else if (tokens[0] == RNDF_CROSSWALK_BEGIN && tokens.size() > 1) {
      string strCrosswalkName = tokens[1];
      string strData = section(file, RNDF_CROSSWALK_BEGIN, RNDF_CROSSWALK_END);
      if (!strData.length()) continue;
      addCrosswalk(strCrosswalkName, strData);
    }
    else if (tokens[0] == RNDF_TRAFFIC_LIGHT_BEGIN && tokens.size() > 1) {
      string strTrafficLightName = tokens[1];
      string strData = section(file, RNDF_TRAFFIC_LIGHT_BEGIN, RNDF_TRAFFIC_LIGHT_END);
      if (!strData.length()) continue;
      addTrafficLight(strTrafficLightName, strData);
      //      addTrafficLight(is, strTrafficLightName, strData);
    }
    else if (tokens[0] == RNDF_ZONE_BEGIN && tokens.size() > 1) {
      string strZoneName = tokens[1];
      string strData = section(file, RNDF_ZONE_BEGIN, RNDF_ZONE_END);
      if (!strData.length()) continue;
      addZone(strZoneName, strData);
    }
    else if (tokens[0] == RNDF_ROADNETWORK_END_FILE) {
      // end file
    }
    else printf("Unknown token %s in RNDF data.\n", line.c_str());
  }
  file.close();

  // SECOND PASS: insert exits, CheckPoints, stops, traffic lights and crosswalks
  Lane* pLane = NULL;
  Perimeter* pPerimeter = NULL;
  Spot* pSpot = NULL;

  ifstream file2(strFileName.c_str());
  while (true) {
    tokens.clear();
    if (!getline(file2, line)) break;
    if (!line.length()) continue;
    line = clearCComments(line);
    splitString(line, tokens, RNDF_DELIMITER);
    if (tokens.empty()) continue;

    if (tokens[0] == RNDF_LANE_BEGIN && tokens.size() > 1) {
      pLane = getLane(tokens[1]);
      assert(pLane);
    }
    else if (tokens[0] == RNDF_LANE_END) {
      pLane = NULL;
    }
    else if (tokens[0] == RNDF_PERIMETER_BEGIN && tokens.size() > 1) {
      pPerimeter = perimeter(tokens[1]);
      assert(pPerimeter);
    }
    else if (tokens[0] == RNDF_PERIMETER_END) pPerimeter = NULL;
    else if (tokens[0] == RNDF_SPOT_BEGIN && tokens.size() > 1) pSpot = getSpot(tokens[1]);
    else if (tokens[0] == RNDF_SPOT_END) pSpot = NULL;
    else if (tokens[0] == RNDF_EXIT && tokens.size() > 2) {
      assert(pLane || pPerimeter);
      WayPoint* wp_from = wayPoint(tokens[1]);
      PerimeterPoint* pp_from = perimeterPoint(tokens[1]);
      WayPoint* wp_to = wayPoint(tokens[2]);
      PerimeterPoint* pp_to = perimeterPoint(tokens[2]);
      //      printf("%s, %s\n", tokens[1].c_str(), tokens[2].c_str());
      assert( (wp_from != NULL && pp_from == NULL) || (wp_from == NULL && pp_from != NULL) );
      if (!((wp_to != NULL && pp_to == NULL) || (wp_to == NULL && pp_to != NULL))) {
        if (wp_from) {
          printf("wp_from: %s\n", wp_from->name().c_str());
        }
        if (wp_to) {
          printf("wp_to: %s\n", wp_to->name().c_str());
        }
        if (pp_from) {
          printf("pp_from: %s\n", pp_from->name().c_str());
        }
        if (pp_to) {
          printf("pp_to: %s\n", pp_to->name().c_str());
        }
        printf("Nothing?!?\n");
      }
      assert( (wp_to != NULL && pp_to == NULL) || (wp_to == NULL && pp_to != NULL) );
      if (wp_from && wp_to) addExit(wp_from, wp_to);
      if (wp_from && pp_to) addExit(wp_from, pp_to);
      if (pp_from && wp_to) addExit(pp_from, wp_to);
      // Perimeter to Perimeter is not allowed in specification
    }
    else if (tokens[0] == RNDF_STOP && tokens.size() > 1) {
      addStop(tokens[1]);
    }
    else if (tokens[0] == RNDF_CROSS && tokens.size() > 3) {
      WayPoint* wp = wayPoint(tokens[1]);
      if(wp->isVirtual()) {
        throw VLRException("Cannot add crosswalk to virtual way point " +  wp->name());
      }
      Crosswalk* cw = crosswalk(tokens[2]);
      if (wp && cw) {
        (tokens[3] == RNDF_CROSSWALK_TYPE_STOP ? wp->addCrosswalk(cw, stop_waypoint) : wp->addCrosswalk(cw, incoming_waypoint)); // also adds wp to cw
        //      cw->dump();
      }
    }
    else if (tokens[0] == RNDF_LIGHT && tokens.size() > 2) {
      WayPoint* wp = wayPoint(tokens[1]);
      if(wp->isVirtual()) {
        throw VLRException("Cannot add traffic light to virtual way point " +  wp->name());
      }
      TrafficLight* tl = trafficLight(tokens[2]);
      if (wp && tl) {
        wp->addTrafficLight(tl);
      }
    }
    else if (tokens[0] == RNDF_CHECKPOINT && tokens.size() > 2) {
      WayPoint* wp = wayPoint(tokens[1]);
      addCheckPoint(wp, tokens[2]);
    }

  }
  file2.close();

  return true;
}

Segment* RoadNetwork::addSegment() {
  Segment* s = addSegment(nextSegmentStr());
  assert(s);
  return s;
}

Segment* RoadNetwork::addSegment(const string& strName) {
  TRACE("adding Segment " << strName);

  // prüfen ob das Roadnet schon ein Segment mit der gleichen Id besitzt
  uint32_t s_id = getIdFromStr(strName, 0);
  if (segments_.find(strName) != segments_.end()) {
    cout << "Das Roadnet besitzt bereits ein Segment mit der Id " << s_id << endl;
    return NULL;
  }

  Segment* pSegment = new Segment(s_id, strName);
  segments_.insert(make_pair(strName, pSegment));

  return pSegment;
}

Segment* RoadNetwork::addSegment(const string& strName, const string& strData) {
  Segment* pSegment = addSegment(strName);
  if (!pSegment) return NULL;

  // initialize values
  istringstream iStream(strData);
  string line;
  vector<string> tokens;
  int nLanes;
  int nCrossWalks;

  while (true) {
    tokens.clear();
    if (!getline(iStream, line)) break;
    if (!line.length()) continue;
    line = clearCComments(line);
    if (!line.length()) continue;
    splitString(line, tokens, RNDF_DELIMITER);
    if (tokens.size() < 2) continue;

    if (tokens[0] == RNDF_SEGMENT_NAME) {
      if (tokens.size() < 2) pSegment->setDescription("");
      else pSegment->setDescription(tokens[1]);
    }
    else if (tokens[0] == RNDF_SEGMENT_NUM_LANES) {
      if (tokens.size() < 2) nLanes = 0;
      else nLanes = CStringTools::gnCInt(tokens[1]);
    }
    else if (tokens[0] == RNDF_SEGMENT_NUM_CROSSWALKS) {
      if (tokens.size() < 2) nCrossWalks = 0;
      else nCrossWalks = CStringTools::gnCInt(tokens[1]);
    }
    else if (tokens[0] == RNDF_SEGMENT_SPEED_LIMIT && tokens.size() > 1) {
      std::string sl_name = "1"; // there's only one sl per Segment, so name is always 1
      SpeedLimit* sl = new SpeedLimit(1, sl_name);
      sl->maxSpeed(dgc::dgc_mph2ms(CStringTools::gfCFloat(tokens[1])));
      if (tokens.size() > 2) {
        sl->minSpeed(dgc::dgc_mph2ms(CStringTools::gfCFloat(tokens[2])));
      }
      else {
        sl->minSpeed(0.0);
      }
      sl->setSegment(pSegment);
      pSegment->setSpeedLimit(sl);
    }
    else if (tokens[0] == RNDF_LANE_BEGIN && tokens.size() > 1) {
      string strLaneName = tokens[1];
      string strData = section(iStream, RNDF_LANE_BEGIN, RNDF_LANE_END);
      if (!strData.length()) continue;
      addLane(pSegment, strLaneName, strData);
    }
    else if (tokens[0] == RNDF_CROSSWALK_BEGIN && tokens.size() > 1) {
      string strCrosswalkName = tokens[1];
      string strData = section(iStream, RNDF_CROSSWALK_BEGIN, RNDF_CROSSWALK_END);
      if (!strData.length()) continue;
      //      addCrosswalk(pSegment, strCrosswalkName, strData);
      addCrosswalk(strCrosswalkName, strData);
    }
    else if (tokens[0] == RNDF_SEGMENT_OFFROAD && tokens.size() > 1) {
      bool isOffroad = CStringTools::gnCInt(tokens[1]);
      if (isOffroad) pSegment->setOffroad();
    }
    else {
      throw VLRException("Unknown token " + tokens[0] + std::string("in RNDF data."));
    }
  }
  return pSegment;
}

void RoadNetwork::delSegment(Segment* s) {
  if (s == NULL) return;
  cout << "  deleting Segment " << s->name() << endl;
  while (s->lanes_.begin() != s->lanes_.end())
    delLane(*s->lanes_.begin());
  if (s->speedLimit()) delete s->speedLimit();
  segments_.erase(s->name());
  delete s;
}

uint32_t RoadNetwork::nextSegmentId() const {
  vector<uint32_t> ids;
  for (TSegmentMap::const_iterator it = segments_.begin(); it != segments_.end(); ++it)
    ids.push_back(it->second->id());
  for (TZoneMap::const_iterator it = zones_.begin(); it != zones_.end(); ++it)
    ids.push_back(it->second->id());

  sort(ids.begin(), ids.end());

  for (uint32_t i = 1; i <= ids.size(); ++i)
    if (i != ids[i - 1]) return i;

  return ids.size() + 1;
}

std::string RoadNetwork::nextSegmentStr() const {
  return boost::lexical_cast<std::string>(nextSegmentId());
}

Intersection* RoadNetwork::addIntersection() {
  Intersection* i = new Intersection(nextIntersectionId(), "Intersection." + nextIntersectionStr());
  intersections_.insert(i);
  return i;
}

Intersection* RoadNetwork::addIntersection(const string& /* strName */, const string& strData) {
  // Currently intersections are not read from the RNDF but are generated in a second pass
  //  Intersection* is = addIntersection(strName);
  //  if (!is) {return NULL;}

  // initialize values
  istringstream iStream(strData);
  string line;
  vector<string> tokens;
  uint32_t num_traffic_lights = 0;

  while (true) {
    tokens.clear();
    if (!getline(iStream, line)) break;
    if (!line.length()) continue;
    line = clearCComments(line);
    if (!line.length()) continue;
    splitString(line, tokens, RNDF_DELIMITER);
    if (tokens.size() < 2) continue;

    if (tokens[0] == RNDF_INTERSECTION_NUM_LIGHTS && tokens.size() > 1) {
      num_traffic_lights = CStringTools::gnCInt(tokens[1]);
    }
    else if (tokens[0] == RNDF_TRAFFIC_LIGHT_BEGIN && tokens.size() > 1) {
      string strTrafficLightName = tokens[1];
      string strData = section(iStream, RNDF_TRAFFIC_LIGHT_BEGIN, RNDF_TRAFFIC_LIGHT_END);
      if (!strData.length()) continue;
      addTrafficLight(strTrafficLightName, strData);
      //      addTrafficLight(is, strTrafficLightName, strData);
    }
    else {
      throw VLRException("Unknown token " + tokens[0] + std::string("in RNDF data."));
    }
  }
  return NULL; // is;
}

void RoadNetwork::delIntersection(Intersection* i) {
//  cout << "  deleting intersection " << i->name() << endl;
  TLaneSegmentSet* lane_segments = const_cast<TLaneSegmentSet*> (&(i->laneSegments()));
  for (TLaneSegmentSet::const_iterator lsit = lane_segments->begin(); lsit != lane_segments->end(); lsit++) {
    LaneSegment* ls = (*lsit);
    ls->setIntersection(NULL);
  }

  intersections_.erase(i);
  delete i;
}

uint32_t RoadNetwork::nextIntersectionId() const {
  return nextId(intersections_);
}

std::string RoadNetwork::nextIntersectionStr() const {
  return boost::lexical_cast<std::string>(nextIntersectionId());
}

Lane* RoadNetwork::addLane(Segment* pSegment) {
  Lane* l = addLane(pSegment, pSegment->nextLaneStr());
  return l;
}

Lane* RoadNetwork::addLane(Segment* pSegment, const string& strName) {
  TRACE("adding Lane " << strName);

  // check if segment already contains lane with given id
  uint32_t l_id = getIdFromStr(strName, 1);
  if (pSegment->getLaneById(l_id) != NULL) {
    cout << "segment already contains lane with id  " << l_id << endl;
    return NULL;
  }

  Lane* pLane = new Lane(l_id, strName);
  lanes_.insert(make_pair(strName, pLane));

  pSegment->addLane(pLane);
  pLane->setSegment(pSegment);

  return pLane;
}

Lane* RoadNetwork::addLane(Segment* pSegment, const string& strName, const string& strData) {
  Lane* pLane = addLane(pSegment, strName);

  // initialize values
  istringstream iStream(strData);
  string line;
  vector<string> tokens;
  int nWayPoints;
  while (true) {
    tokens.clear();
    if (!getline(iStream, line)) break;
    if (!line.length()) continue;
    line = clearCComments(line);
    if (!line.length()) continue;
    splitString(line, tokens, RNDF_DELIMITER);
    if (tokens.size() < 2) continue;

    if (tokens[0] == RNDF_LANE_NUM_WAYPOINTS) nWayPoints = CStringTools::gnCInt(tokens[1]);
    else if (tokens[0] == RNDF_LANE_WIDTH) {
      int width = CStringTools::gnCInt(tokens[1]);
      pLane->setLaneWidth(dgc::dgc_feet2meters((double) width));
    }
    else if (tokens[0] == RNDF_LANE_SPEED_LIMIT && tokens.size() > 1) {
      std::string sl_name = "1"; // there's only one sl per Segment, so name is always 1
      SpeedLimit* sl = new SpeedLimit(1, sl_name);
      sl->maxSpeed(dgc::dgc_mph2ms(CStringTools::gfCFloat(tokens[1])));
      if (tokens.size() > 2) {
        sl->minSpeed(dgc::dgc_mph2ms(CStringTools::gfCFloat(tokens[2])));
      }
      else {
        sl->minSpeed(0.0);
      }
      sl->setLane(pLane);
      pLane->setSpeedLimit(sl);
    }
    else if (tokens[0] == RNDF_LANE_TYPE && tokens.size() > 1) {
      if (tokens[1] == RNDF_LANE_TYPE_CARLANE) {
        pLane->setLaneType(Lane::car_lane);
      }
      else if (tokens[1] == RNDF_LANE_TYPE_BIKELANE) {
        pLane->setLaneType(Lane::bike_lane);
      }
    }
    else if (tokens[0] == RNDF_LANE_LEFT_BOUNDARY) {
      if (tokens[1] == RNDF_LANE_BOUNDARYTYPE_DOUBLEYELLOW) pLane->setLeftBoundaryType(Lane::DoubleYellow);
      else if (tokens[1] == RNDF_LANE_BOUNDARYTYPE_BROKENWHITE) pLane->setLeftBoundaryType(Lane::BrokenWhite);
      else if (tokens[1] == RNDF_LANE_BOUNDARYTYPE_SOLIDWHITE) pLane->setLeftBoundaryType(Lane::SolidWhite);
      else pLane->setLeftBoundaryType(Lane::NoBoundary);
    }
    else if (tokens[0] == RNDF_LANE_RIGHT_BOUNDARY) {
      if (tokens[1] == RNDF_LANE_BOUNDARYTYPE_DOUBLEYELLOW) pLane->setRightBoundaryType(Lane::DoubleYellow);
      else if (tokens[1] == RNDF_LANE_BOUNDARYTYPE_BROKENWHITE) pLane->setRightBoundaryType(Lane::BrokenWhite);
      else if (tokens[1] == RNDF_LANE_BOUNDARYTYPE_SOLIDWHITE) pLane->setRightBoundaryType(Lane::SolidWhite);
      else pLane->setRightBoundaryType(Lane::NoBoundary);
    }
    else if (tokens[0] == RNDF_EXIT) {
    }
    else if (tokens[0] == RNDF_STOP) {
    }
    else if (tokens[0] == RNDF_CROSS) {
    }
    else if (tokens[0] == RNDF_LIGHT) {
    }
    else if (tokens[0] == RNDF_CHECKPOINT) {
    }
    else // assume WayPoints
    {
      if (tokens.size() < 3) {
        throw VLRException("Unknown way point format for line " + line + std::string("in RNDF data."));
      }
      bool virtual_waypoint = false;
      if (tokens.size() > 3) {
        if (tokens[3] == "v") {
          virtual_waypoint = true;
        }
      }
      addWayPoint(pLane, tokens[0], CStringTools::gdCDouble(tokens[1]), CStringTools::gdCDouble(tokens[2]), virtual_waypoint);
    }
  } // while lines

  pSegment->addLane(pLane);
  pLane->setSegment(pSegment);
  return pLane;
}

void RoadNetwork::delLane(Lane* l) {
  if (l == NULL) return;
  cout << "  deleting Lane " << l->name() << endl;
  if (l->segment_) {
    l->segment_->removeLane(l);
  }
  while (l->waypoints_.begin() != l->waypoints_.end())
    delWayPoint(*l->waypoints_.begin());
  lanes_.erase(l->name());
  delete l;
}

WayPoint* RoadNetwork::addWayPoint(Lane* pLane, const double& lat, const double& lon, bool is_virtual) {
  return addWayPoint(pLane, pLane->nextWayPointStr(), lat, lon, pLane->numWayPoints(), is_virtual);
}

WayPoint* RoadNetwork::addWayPoint(Lane* pLane, const double& lat, const double& lon, uint32_t insert_before, bool is_virtual) {
  return addWayPoint(pLane, pLane->nextWayPointStr(), lat, lon, insert_before, is_virtual);
}

WayPoint* RoadNetwork::addWayPoint(Lane* pLane, const string& strName, const double& lat, const double& lon, bool is_virtual) {
  return addWayPoint(pLane, strName, lat, lon, pLane->numWayPoints(), is_virtual);
}

WayPoint* RoadNetwork::addWayPoint(Lane* pLane, const string& strName, const double& lat, const double& lon, uint32_t insert_before, bool is_virtual) {
  assert(pLane);

  TRACE("adding Waypoint " << strName << " to Lane "<< pLane->name());

  // prüfen ob die Lane schon einen Waypoint mit der gleichen Id besitzt
  uint32_t wp_id = getIdFromStr(strName, 2);
  //	cout << "-> Id = " << wp_id << endl;
  if (pLane->wayPointFromId(wp_id) != NULL) {
    cout << "Die Lane besitzt bereits einen Waypoint mit der Id " << wp_id << endl;
    return NULL;
  }

  WayPoint* wp = new WayPoint(wp_id, strName);

  wp->setLatLon(lat, lon);
  wp->setParentLane(pLane);
  wp->setVirtual(is_virtual);
  waypoints_.insert(make_pair(strName, wp));
  pLane->addWayPoint(wp, insert_before);

  return wp;
}

WayPoint* RoadNetwork::addWayPoint(Spot* pSpot, const double& lat, const double& lon) {
  return addWayPoint(pSpot, pSpot->nextSpotPointStr(), lat, lon);
}

WayPoint* RoadNetwork::addWayPoint(Spot* pSpot, const string& strName, const double& lat, const double& lon) {
  assert(pSpot);

  TRACE("adding Waypoint " << strName << " to Spot "<< pSpot->name());
  cout << "adding Waypoint " << strName << " to Spot " << pSpot->name() << "\n";

  uint32_t wp_id = getIdFromStr(strName, 2);
  WayPoint* pWayPoint = new WayPoint(wp_id, strName);
  pair<TWayPointMap::iterator, bool> result = waypoints_.insert(make_pair(strName, pWayPoint));
  if (!result.second) {
    setStatus("addWayPoint: WayPoint with name " + strName + " was not added to the network.");
    pWayPoint->destroy();
    return NULL;
  }
  pWayPoint->setLatLon(lat, lon);
  pWayPoint->setParentSpot(pSpot);
  waypoints_.insert(make_pair(strName, pWayPoint));
  printf("Adding way point %s from spot to way point map.\n", pWayPoint->name().c_str());
  pSpot->addWayPoint(pWayPoint);
  return pWayPoint;
}

void RoadNetwork::delWayPoint(WayPoint* wp) {
  if (wp == NULL) return;

  cout << "  deleting waypoint " << wp->name() << endl;

  if (wp->parentLane()) wp->parentLane()->removeWayPoint(wp);
  if (wp->parentSpot()) wp->parentSpot()->removeWayPoint(wp);
  while (wp->exits().begin() != wp->exits().end()) {
    delExit(wp->exits().begin()->second);
  }
  while (wp->entries().begin() != wp->entries().end()) {
    delExit(wp->entries().begin()->second);
  }
  while (wp->crosswalks().begin() != wp->crosswalks().end()) {
    delCrosswalk(wp->crosswalks().begin()->second.crosswalk_);
  }
  while (wp->trafficLights().begin() != wp->trafficLights().end()) {
    delTrafficLight(wp->trafficLights().begin()->second);
  }

  if (wp->checkPoint()) {
    delCheckPoint(wp->checkPoint());
  }
  if (wp->stop()) {
    delStop(wp->stop());
  }

  waypoints_.erase(wp->name());
  delete wp;
}

Exit* RoadNetwork::addExit(string strName) {
  Exit* pExit = new Exit(exits_.size() + 1, strName);
  pair<TExitMap::iterator, bool> result = exits_.insert(make_pair(strName, pExit));
  if (!result.second) {
    setStatus("addExit: Exit with name " + strName + " was not added to the network.");
    delete pExit;
    return NULL;
  }
  return pExit;
}

Exit* RoadNetwork::addExit(WayPoint* pWayPointFrom, WayPoint* pWayPointTo) {
  assert(pWayPointFrom && pWayPointTo);

  TRACE("adding Exit from Waypoint " << pWayPointFrom->name() << " to Waypoint "<< pWayPointTo->name());

  if(pWayPointFrom->isVirtual() || pWayPointTo->isVirtual()) {
    throw VLRException("Cannot add exit to virtual way points.");
  }

  Exit* pExit = addExit(pWayPointFrom->name() + "_" + pWayPointTo->name());
  if (pExit == NULL) return NULL;

  pExit->setExitType(Exit::LaneToLane);
  pExit->setExitFrom(pWayPointFrom);
  pExit->setExitTo(pWayPointTo);
  pWayPointFrom->addExit(pExit);
  pWayPointTo->addEntry(pExit);

  return pExit;
}

Exit* RoadNetwork::addExit(WayPoint* pWayPointFrom, PerimeterPoint* pPerimterPointTo) {
  assert(pWayPointFrom && pPerimterPointTo);

  TRACE("adding Exit from Waypoint " << pWayPointFrom->name() << " to Perimeterpoint "<< pPerimterPointTo->name());

  if(pWayPointFrom->isVirtual()) {
    throw VLRException("Cannot add exit to virtual way points.");
  }

  Exit* pExit = addExit(pWayPointFrom->name() + "_" + pPerimterPointTo->name());
  if (pExit == NULL) return NULL;

  pExit->setExitType(Exit::LaneToPerimeter);
  pExit->setExitFrom(pWayPointFrom);
  pExit->setExitTo(pPerimterPointTo);
  pWayPointFrom->addExit(pExit);
  pPerimterPointTo->addEntry(pExit);

  return pExit;
}

Exit* RoadNetwork::addExit(PerimeterPoint* pPerimterPointFrom, WayPoint* pWayPointTo) {
  assert(pPerimterPointFrom && pWayPointTo);

  TRACE("adding Exit from Perimeterpoint " << pPerimterPointFrom->name() << " to Waypoint "<< pWayPointTo->name());

  if(pWayPointTo->isVirtual()) {
    throw VLRException("Cannot add exit to virtual way points.");
  }

  Exit* pExit = addExit(pPerimterPointFrom->name() + "_" + pWayPointTo->name());
  if (pExit == NULL) return NULL;

  pExit->setExitType(Exit::PerimeterToLane);
  pExit->setExitFrom(pPerimterPointFrom);
  pExit->setExitTo(pWayPointTo);
  pPerimterPointFrom->addExit(pExit);
  pWayPointTo->addEntry(pExit);

  return pExit;
}

Exit* RoadNetwork::addExit(PerimeterPoint* pPerimterPointFrom, PerimeterPoint* pPerimterPointTo) {
  assert(pPerimterPointFrom && pPerimterPointTo);

  TRACE("adding Exit from Perimeter point " << pPerimterPointFrom->name() << " to Perimeter point "<< pPerimterPointTo->name());

  Exit* pExit = addExit(pPerimterPointFrom->name() + "_" + pPerimterPointTo->name());

  if (!pExit) {
    return NULL;
  }

  pExit->setExitType(Exit::PerimeterToPerimeter);
  pExit->setExitFrom(pPerimterPointFrom);
  pExit->setExitTo(pPerimterPointTo);
  pPerimterPointFrom->addExit(pExit);
  pPerimterPointTo->addEntry(pExit);

  return pExit;
}

void RoadNetwork::delExit(Exit* e) {
  assert(e);
  cout << "  deleting Exit " << e->name() << endl;
  if (e->m_exitFromWayPoint) e->m_exitFromWayPoint->removeExit(e);
  if (e->m_exitToWayPoint) e->m_exitToWayPoint->removeEntry(e);
  if (e->m_exitFromPerimeterPoint) e->m_exitFromPerimeterPoint->removeExit(e);
  if (e->m_exitToPerimeterPoint) e->m_exitToPerimeterPoint->removeEntry(e);
  exits_.erase(e->name());
  delete e;
}

Stop* RoadNetwork::addStop(const string& strWayPoint) {
  WayPoint* pWayPoint = wayPoint(strWayPoint);

  if (!pWayPoint) {
    throw VLRException("Could not find waypoint " + strWayPoint);
  }

  if(pWayPoint->isVirtual()) {
    throw VLRException("Cannot add stop to virtual way point " + strWayPoint);
  }

  TRACE("adding Stop " << strWayPoint);

  string strName = strWayPoint;
  Stop* pStop = new Stop(stops_.size(), strName);
  pair<TStopMap::iterator, bool> result = stops_.insert(make_pair(strWayPoint, pStop));
  if (!result.second) {
    setStatus("addStop: Stop with name " + strWayPoint + " was not added to the network.");
    pStop->destroy();
    return NULL;
  }

  pStop->setWayPoint(pWayPoint);
  pWayPoint->setStop(pStop);

  TRACE("-> done")
  return pStop;
}

void RoadNetwork::delStop(Stop* s) {
  if (!s) {
    cout << __FUNCTION__ << ":  no Stop to delete.\n";
  }

  if (s->way_point_) s->way_point_->setStop(NULL);
  stops_.erase(s->name());
  delete s;
}

Stop* RoadNetwork::stop(const std::string& strWayPoint) {
  TStopMap::iterator it = stops_.find(strWayPoint);
  if (it != stops_.end()) return it->second;
  else setStatus("stop: stoppoint with identifier " + strWayPoint + " not found.");
  return NULL;
}

CheckPoint* RoadNetwork::addCheckPoint(WayPoint* wp) {
  return addCheckPoint(wp, nextIdStr(checkpoints_));
}

CheckPoint* RoadNetwork::addCheckPoint(WayPoint* wp, const std::string& strName) {

  if(wp->isVirtual()) {
    throw VLRException("Cannot add check point to virtual way point " +  wp->name());
  }

  CheckPoint* pCheckPoint = new CheckPoint(getIdFromStr(strName, 0), strName);

  TRACE("adding Checkpoint "<< strName <<" to Waypoint " << wp->name());

  assert(pCheckPoint);

  pair<TCheckPointMap::iterator, bool> result = checkpoints_.insert(make_pair(strName, pCheckPoint));
  if (!result.second) {
    setStatus("addCheckPoint: CheckPoint with name " + strName + " was not added to the network.");
    pCheckPoint->destroy();
    return NULL;
  }

  pCheckPoint->setWayPoint(wp);
  wp->setCheckPoint(pCheckPoint);

  return pCheckPoint;
}

void RoadNetwork::delCheckPoint(CheckPoint* cp) {
  assert(cp);
  cout << "  deleting checkpoint " << cp->name() << endl;
  if (cp->m_waypoint) cp->m_waypoint->setCheckPoint(NULL);
  checkpoints_.erase(cp->name());
  delete cp;
}

Zone* RoadNetwork::addZone() {
  Zone* z = addZone(nextZoneStr());
  assert(z);
  return z;
}

Zone* RoadNetwork::addZone(const string& strName) {
  TRACE("adding Zone "<< strName);

  // prüfen ob das Roadnet schon eine Zone mit der gleichen Id besitzt
  uint32_t z_id = getIdFromStr(strName, 0);
  if (zones_.find(strName) != zones_.end()) {
    cout << "Das Roadnet besitzt bereits eine Zone mit der Id " << z_id << endl;
    return NULL;
  }

  Zone* z = new Zone(z_id, strName);
  zones_.insert(make_pair(strName, z));

  return z;
}

Zone* RoadNetwork::addZone(const string& strName, const string& strData) {
  Zone* pZone = addZone(strName);
  if (!pZone) return NULL;

  // initialize values
  istringstream iStream(strData);
  string line;
  vector<string> tokens;
  int nSpots;
  // loop lines
  while (true) {
    tokens.clear();
    if (!getline(iStream, line)) break;
    if (!line.length()) continue;
    line = clearCComments(line);
    splitString(line, tokens, RNDF_DELIMITER);
    if (tokens.empty()) continue;

    if (tokens[0] == RNDF_ZONE_NUM_SPOTS) {
      if (tokens.size() < 2) nSpots = 0;
      else nSpots = CStringTools::gnCInt(tokens[1]);
    }
    else if (tokens[0] == RNDF_ZONE_NAME) {
      if (tokens.size() < 2) pZone->setDescription("");
      else pZone->setDescription(tokens[1]);
    }
    else if (tokens[0] == RNDF_PERIMETER_BEGIN && tokens.size() > 1) {
      string strPerimeterName = tokens[1];
      string strData = section(iStream, RNDF_PERIMETER_BEGIN, RNDF_PERIMETER_END);
      if (!strData.length()) continue;
      addPerimeter(pZone, strPerimeterName, strData);
    }
    else if (tokens[0] == RNDF_SPOT_BEGIN && tokens.size() > 1) {
      string strSpotName = tokens[1];
      string strData = section(iStream, RNDF_SPOT_BEGIN, RNDF_SPOT_END);
      if (!strData.length()) continue;
      addSpot(pZone, strSpotName, strData);
    }
    else if (tokens[0] == RNDF_ZONE_OFFROAD) {
      bool isOffroad = CStringTools::gnCInt(tokens[1]);
      if (isOffroad) pZone->setOffroad();
    }
    else {
      throw VLRException("Unknown token ' " + tokens[0] + std::string("in RNDF data."));
    }
  }
  return pZone;
}

void RoadNetwork::delZone(Zone* z) {
  if (!z) {
    return;
  }
  cout << "  deleting Zone " << z->name() << endl;
  while (z->perimeters().begin() != z->perimeters().end()) {
    delPerimeter((*z->perimeters().begin()).second);
  }

  while (z->spots().begin() != z->spots().end()) {
    delSpot((*z->spots().begin()).second);
  }

  zones_.erase(z->name());
  delete z;
}

Spot* RoadNetwork::addSpot(Zone* pZone) {
  assert(pZone);
  Spot* s = addSpot(pZone, pZone->getNextSpotStr());
  assert(s);
  return s;
}

Spot* RoadNetwork::addSpot(Zone* pZone, const string& strName) {
  assert(pZone);

  TRACE("adding Spot " << strName << " to Zone "<< pZone->name());

  // prüfen ob das Segment schon eine Lane mit der gleichen Id besitzt
  uint32_t s_id = getIdFromStr(strName, 1);
  if (pZone->getSpotById(s_id) != NULL) {
    cout << "Zone already contains Spot with id " << s_id << endl;
    return NULL;
  }

  Spot* s = new Spot(s_id, strName);
  spots_.insert(make_pair(strName, s));

  pZone->addSpot(s);
  s->setZone(pZone);

  return s;
}

Spot* RoadNetwork::addSpot(Zone* pZone, const string& strName, const string& strData) {
  Spot* pSpot = addSpot(pZone, strName);

  pZone->addSpot(pSpot);

  // initialize values
  istringstream iStream(strData);
  string line;
  vector<string> tokens;
  // loop lines
  while (true) {
    tokens.clear();
    if (!getline(iStream, line)) break;
    if (!line.length()) continue;
    line = clearCComments(line);
    splitString(line, tokens, RNDF_DELIMITER);
    if (tokens.empty()) continue;

    if (tokens[0] == RNDF_SPOT_WIDTH) pSpot->setSpotWidth(CStringTools::gnCInt(tokens[1]));
    else if (tokens[0] == RNDF_EXIT) {
    }
    else if (tokens[0] == RNDF_STOP) {
    }
    else if (tokens[0] == RNDF_CROSS) {
    }
    else if (tokens[0] == RNDF_LIGHT) {
    }
    else if (tokens[0] == RNDF_CHECKPOINT) {
    }
    else {
      if (tokens.size() < 3) continue;
      addWayPoint(pSpot, tokens[0], CStringTools::gdCDouble(tokens[1]), CStringTools::gdCDouble(tokens[2]));
    }
  }

  return pSpot;
}

void RoadNetwork::delSpot(Spot* s) {
  if (!s) {
    return;
  }
  cout << "  deleting Spot " << s->name() << endl;

  if (s->zone()) {
    s->zone()->removeSpot(s);
  }

  while (s->wayPoints().begin() != s->wayPoints().end()) {
    delWayPoint(*s->wayPoints().begin());
  }

  spots_.erase(s->name());
  delete s;
}

Perimeter* RoadNetwork::addPerimeter(Zone* Zone) {
  Perimeter* p = addPerimeter(Zone, Zone->getNextPerimeterStr());
  assert(p);
  return p;
}

Perimeter* RoadNetwork::addPerimeter(Zone* Zone, const string& strName) {
  assert(Zone);

  TRACE("adding Perimeter " << strName << " to Zone "<< Zone->name());

  // Perimeter* p = new Perimeter(perimeters_.size(), strName);
  Perimeter* p = new Perimeter(Zone->getNextPerimeterId(), strName);
  pair<TPerimeterMap::iterator, bool> result = perimeters_.insert(make_pair(strName, p));
  if (!result.second) {
    setStatus("addPerimeter: Perimeter with name " + strName + " was not added to the network.");
    p->destroy();
    return NULL;
  }

  p->setZone(Zone);
  Zone->addPerimeter(p);

  return p;
}

Perimeter* RoadNetwork::addPerimeter(Zone* Zone, const string& strName, const string& strData) {
  assert(Zone);

  TRACE("adding Perimeter " << strName << " to Zone "<< Zone->name());

  // Perimeter* p = new Perimeter(perimeters_.size(), strName);
  Perimeter* p = new Perimeter(Zone->getNextPerimeterId(), strName);
  pair<TPerimeterMap::iterator, bool> result = perimeters_.insert(make_pair(strName, p));
  if (!result.second) {
    setStatus("addPerimeter: Perimeter with name " + strName + " was not added to the network.");
    p->destroy();
    return NULL;
  }

  p->setZone(Zone);
  Zone->addPerimeter(p);

  // initialize values
  istringstream iStream(strData);
  string line;
  vector<string> tokens;
  int nPerimeterPoints;
  // loop lines
  while (true) {
    tokens.clear();
    if (!getline(iStream, line)) break;
    if (!line.length()) continue;
    line = clearCComments(line);
    splitString(line, tokens, RNDF_DELIMITER);
    if (tokens.empty()) continue;

    if (tokens[0] == RNDF_PERIMETER_NUM_PERIMETERPOINTS) {
      nPerimeterPoints = CStringTools::gnCInt(tokens[1]);
    }

    else if (tokens[0] == RNDF_EXIT) {
    }
    else if (tokens[0] == RNDF_STOP) {
    }
    else if (tokens[0] == RNDF_CROSS) {
    }
    else if (tokens[0] == RNDF_LIGHT) {
    }
    else if (tokens[0] == RNDF_CHECKPOINT) {
    }
    else // assume PerimeterPoints
    {
      if (tokens.size() < 3) continue;
      addPerimeterPoint(p, tokens[0], CStringTools::gdCDouble(tokens[1]), CStringTools::gdCDouble(tokens[2]));
    }
  }

  return p;
}

void RoadNetwork::delPerimeter(Perimeter* p) {
  if (!p) {
    return;
  }
  cout << "  deleting Perimeter " << p->name() << endl;

  if (p->zone()) {
    p->zone()->removePerimeter(p);
  }

  while (p->perimeterPoints().begin() != p->perimeterPoints().end()) {
    delPerimeterPoint(*p->perimeterPoints().begin());
  }

  perimeters_.erase(p->name());
  delete p;
}

PerimeterPoint* RoadNetwork::addPerimeterPoint(Perimeter* pPerimeter, const string& strName, const double& lat, const double& lon) {
  assert(pPerimeter);
  PerimeterPoint* pp = addPerimeterPoint(pPerimeter, strName, lat, lon, pPerimeter->numPerimeterPoints());
  assert(pp);
  return pp;
}

PerimeterPoint* RoadNetwork::addPerimeterPoint(Perimeter* pPerimeter, const double& lat, const double& lon, uint32_t insert_before) {
  assert(pPerimeter);
  PerimeterPoint* pp = addPerimeterPoint(pPerimeter, pPerimeter->nextPerimeterPointStr(), lat, lon, insert_before);
  assert(pp);
  return pp;
}

PerimeterPoint* RoadNetwork::addPerimeterPoint(Perimeter* pPerimeter, const string& strName, const double& lat, const double& lon, uint32_t insert_before) {
  assert(pPerimeter);

  TRACE("adding PerimeterPoint " << strName << " to Perimeter "<< pPerimeter->name());

  PerimeterPoint* pPerimeterPoint = new PerimeterPoint(pPerimeter->nextPerimeterPointId(), strName);
  pair<TPerimeterPointMap::iterator, bool> result = perimeter_points_.insert(make_pair(strName, pPerimeterPoint));
  if (!result.second) {
    setStatus("addPerimeterPoint: PerimeterPoint with name " + strName + " was not added to the network.");
    pPerimeterPoint->destroy();
    return NULL;
  }

  pPerimeter->addPerimeterPoint(pPerimeterPoint, insert_before);
  pPerimeterPoint->setPerimeter(pPerimeter);
  pPerimeterPoint->setLatLon(lat, lon);
  return pPerimeterPoint;
}

void RoadNetwork::delPerimeterPoint(PerimeterPoint* pp) {
  if (!pp) {
    return;
  }
  cout << "  deleting Perimeter point" << pp->name() << endl;

  if (pp->perimeter()) {
    ((Perimeter*) pp->perimeter())->removePerimeterPoint(pp);
  }

  while (pp->exits().begin() != pp->exits().end()) {
    delExit(pp->exits().begin()->second);
  }

  while (pp->entries().begin() != pp->entries().end()) {
    delExit(pp->entries().begin()->second);
  }

  // TODO: not implemented if (pp->m_checkpoint) delCheckPoint(pp->m_checkpoint);

  perimeter_points_.erase(pp->name());
  delete pp;
}

//Crosswalk* RoadNetwork::addCrosswalk(WayPoint* wp) {
//  if(!wp) {
//    // throw(VLRException("zero pointer argument (waypoint)"));
//    return NULL;
//  }
//
//  TRACE("adding crosswalk " << strName << " to waypoint "<< wp->name());
//
//  Crosswalk* cw = addCrosswalk(wp, wp->nextCrosswalkStr());
//  return cw;
//}


Crosswalk* RoadNetwork::addCrosswalk() {
  return addCrosswalk(nextIdStr(crosswalks_));
}

Crosswalk* RoadNetwork::addCrosswalk(const string& strName) {
  TRACE("creating crosswalk " << strName);

  uint32_t cw_id = getIdFromStr(strName, 1);
  Crosswalk* cw = new Crosswalk(cw_id, strName);

  if (!cw) {
    return NULL;
  }

  crosswalks_.insert(make_pair(strName, cw));

  return cw;
}

Crosswalk* RoadNetwork::addCrosswalk(const string& strName, const string& strData) {
  Crosswalk* cw = addCrosswalk(strName);
  if (!cw) {
    //    throw(VLRException("Failed to add crosswalk."));
    return NULL;
  }

  // initialize values
  istringstream iStream(strData);
  string line;
  vector<string> tokens;
  // loop lines
  while (true) {
    tokens.clear();
    if (!getline(iStream, line)) break;
    if (!line.length()) continue;
    line = clearCComments(line);
    splitString(line, tokens, RNDF_DELIMITER);
    if (tokens.empty()) continue;

    if (tokens[0] == RNDF_CROSSWALK_WIDTH) {
      if (tokens.size() < 2) continue;
      cw->width(dgc::dgc_feet2meters(CStringTools::gfCFloat(tokens[1])));
    }
    else if (tokens[0] == RNDF_CROSSWALK_P1) {
      if (tokens.size() < 3) continue;
      cw->setLatLon1(CStringTools::gdCDouble(tokens[1]), CStringTools::gdCDouble(tokens[2]));
    }
    else if (tokens[0] == RNDF_CROSSWALK_P2) {
      if (tokens.size() < 3) continue;
      cw->setLatLon2(CStringTools::gdCDouble(tokens[1]), CStringTools::gdCDouble(tokens[2]));
    }
  }

  return cw;
}

void RoadNetwork::delCrosswalk(Crosswalk* cw) {
  if (!cw) {
    return;
  }
  cout << "  deleting crosswalk " << cw->name() << endl;

  crosswalks_.erase(cw->name());
  delete cw; // waypoints are unlinked in crosswalk destructor
}

Crosswalk* RoadNetwork::crosswalk(const string& strName) {
  TCrosswalkMap::const_iterator it = crosswalks_.find(strName);
  if (it != crosswalks_.end()) {
    return (*it).second;
  }

  setStatus("crosswalk: crosswalk with identifier " + strName + " not found.");
  return NULL;
}

//TrafficLight* RoadNetwork::addTrafficLight(WayPoint* wp) {
//  if(!wp) {
//    // throw(VLRException("zero pointer argument (waypoint)"));
//    return NULL;
//  }
//  TrafficLight* tl = addTrafficLight(wp, wp->nextTrafficLightStr());
//  return tl;
//}


TrafficLight* RoadNetwork::addTrafficLight() {
  return addTrafficLight(nextIdStr(traffic_lights_));
}

TrafficLight* RoadNetwork::addTrafficLight(const string& strName) {
  TRACE("creating traffic light " << strName);

  uint32_t tl_id = getIdFromStr(strName, 1);
  TrafficLight* tl = new TrafficLight(tl_id, strName);

  traffic_lights_.insert(make_pair(strName, tl));

  return tl;
}

TrafficLight* RoadNetwork::addTrafficLight(const string& strName, const string& strData) {
  TrafficLight* tl = addTrafficLight(strName);
  if (!tl) {
    //    throw(VLRException("Failed to add traffic light."));
    return NULL;
  }

  // initialize values
  istringstream iStream(strData);
  string line;
  vector<string> tokens;
  // loop lines
  while (true) {
    tokens.clear();
    if (!getline(iStream, line)) {
      break;
    }
    if (!line.length()) continue;
    line = clearCComments(line);
    splitString(line, tokens, RNDF_DELIMITER);
    if (tokens.empty()) continue;

    if (tokens[0] == RNDF_TRAFFIC_LIGHT_GROUP_ID) {
      if (tokens.size() < 2) continue;
      tl->groupId(CStringTools::gdCDouble(tokens[1]));
    }
    else if (tokens[0] == RNDF_TRAFFIC_LIGHT_POSITION) {
      if (tokens.size() < 3) {
        continue;
      }
      tl->setLatLon(CStringTools::gdCDouble(tokens[1]), CStringTools::gdCDouble(tokens[2]));
      if (tokens.size() >= 3) {
        tl->z(CStringTools::gdCDouble(tokens[3]));
      }
    }
    else if (tokens[0] == RNDF_TRAFFIC_LIGHT_ORIENTATION) {
      if (tokens.size() < 2) continue;
      tl->orientation(CStringTools::gdCDouble(tokens[1]));
    }
  }

  return tl;
}

void RoadNetwork::delTrafficLight(TrafficLight* tl) {
  if (!tl) {
    return;
  }
  cout << "  deleting traffic light " << tl->name() << endl;

  traffic_lights_.erase(tl->name());
  delete tl; // waypoints are unlinked in traffic light destructor
}

TrafficLight* RoadNetwork::trafficLight(const string& strName) {
  TTrafficLightMap::const_iterator it = traffic_lights_.find(strName);
  if (it != traffic_lights_.end()) {
    return (*it).second;
  }

  setStatus("trafficLight: traffic light with identifier " + strName + " not found.");
  return NULL;
}

void RoadNetwork::addRelations() {
  // DO NOT change the order
  addVirtualLanes();
  //	alignEdgeStarts();		// is irgendwie noch buggy
  //sampleEdges();       // what is that good for?!?
  addLaneSegments();
  double t = drc::Time::current();
  connectAdjacentLanes();
  t = drc::Time::current() - t;
  printf("time for connectAdjacentLanes(): %f\n", t);
  addKTurns();
  addIntersections();
  //	addAdditionalKTurnEdges();   // noch nicht ganz fertig
  addLaneSegemetTypes();
}

void RoadNetwork::addVirtualLanes() {
  cout << "Add Virtual Lanes... " << flush;

  // TODO Check if virtual lanes already exist and do not add then (in case function is called several times)

  // Iterate through all lanes and represent entry-exit links with virtual lanes
  uint32_t id = 0;
  for (TLaneMap::iterator l1_it = lanes_.begin(); l1_it != lanes_.end(); ++l1_it) {
    Lane* lane1 = l1_it->second;
    //		cout << "Lane "<< lane1->name() << endl;
    const TExitMap& exits = lane1->exits();
    for (TExitMap::const_iterator it = exits.begin(); it != exits.end(); ++it) {
      // determine exit lane and way points
      Exit* exit = it->second;
      if (exit->getExitFromLane() == NULL || exit->getExitToLane() == NULL) {
        continue;
      }
      Lane* lane2 = exit->getExitToLane()->parentLane();
      if (lane2 == NULL) {
        continue;
      }
      WayPoint* wf = exit->getExitFromLane();
      WayPoint* wt = exit->getExitToLane();

      // create a new virtual Lane
      // DO NOT change the naming scheme without adapting dependent functions
      Lane* l = new Lane(0, exit->getExitFromLane()->name() + " -> " + exit->getExitToLane()->name(), true);
      m_virtualLanes.insert(l);
      lanes_.insert(make_pair(l->name(), l));

      WayPoint* w1 = new WayPoint(0, "0.0." + boost::lexical_cast<string>(++id));
      w1->setLatLon(wf->lat(), wf->lon());
      w1->setParentLane(l);
      l->addWayPoint(w1);

      WayPoint* w2 = new WayPoint(0, "0.0." + boost::lexical_cast<string>(++id));
      w2->setLatLon(wt->lat(), wt->lon());
      w2->setParentLane(l);
      l->addWayPoint(w2);

      l->setLaneWidth((lane1->laneWidth() + lane2->laneWidth()) / 2);

      //			cout << "  "<< Exit->getExitFromLane()->name() <<" -> "<< Exit->getExitToLane()->name() << endl;
    }
  }
  cout << "Ok" << endl;
}

void RoadNetwork::alignEdgeStarts() {
//  cout << "Aligning Edge Starts... " << flush;
//
//  // Alle Lanes durchgehen und geometrisch prüfen ob sie nebeneinander liegen
//  for (TLaneMap::iterator l1_it = lanes_.begin(); l1_it != lanes_.end(); ++l1_it) {
//    Lane* lane1 = l1_it->second;
//    if (lane1->isVirtual()) continue;
//    for (size_t l1wp_i = 0; l1wp_i < lane1->numWayPoints() - 1; ++l1wp_i) {
//      for (TLaneMap::iterator l2_it = lanes_.begin(); l2_it != lanes_.end(); ++l2_it) {
//        Lane* lane2 = l2_it->second;
//        if (lane1 == lane2) continue;
//        if (lane2->isVirtual()) continue;
//        for (size_t l2wp_i = 0; l2wp_i < lane2->numWayPoints() - 1; ++l2wp_i) {
//          alignEdgeStarts(lane1, l1wp_i, lane2, l2wp_i);
//        }
//      }
//    }
//  }
//
//  cout << "Ok" << endl;
}

void RoadNetwork::alignEdgeStarts(Lane* lane1, uint32_t l1wp_i, Lane* lane2, uint32_t l2wp_i) {
//  assert(lane1 && lane2);
//  assert(l1wp_i < lane1->numWayPoints()-1 && l2wp_i < lane2->numWayPoints()-1);
//  assert(lane1 != lane2);
//
//  // Waypoints holen
//  WayPoint* l1wp1 = lane1->wayPoint(l1wp_i);
//  WayPoint* l1wp2 = lane1->wayPoint(l1wp_i + 1);
//  WayPoint* l2wp1 = lane2->wayPoint(l2wp_i);
//  WayPoint* l2wp2 = lane2->wayPoint(l2wp_i + 1);
//  assert(l1wp1 && l1wp2 && l2wp1 && l2wp2);
//
//  // Punkte ud Linien erzeugen
//  Point_2 p1s = Point_2(l1wp1->utmX(), l1wp1->utmY());
//  Point_2 p1e = Point_2(l1wp2->utmX(), l1wp2->utmY());
//  Point_2 p2s = Point_2(l2wp1->utmX(), l2wp1->utmY());
//  Point_2 p2e = Point_2(l2wp2->utmX(), l2wp2->utmY());
//  Line_2 l1(p1s, p1e);
//  Line_2 l2(p2s, p2e);
//  Segment_2 s1(p1s, p1e);
//  Segment_2 s2(p2s, p2e);
//  Vector_2 v1 = p1e - p1s;
//  Vector_2 v2 = p2e - p2s;
//  Vector_2 v1n = v1 / std::sqrt(v1.squared_length());
//  Vector_2 v2n = v2 / std::sqrt(v2.squared_length());
//
//  // Segemente verkürzen um keine Überkreuznachbarschaften zu bekommen
//  double max_dist = (lane1->laneWidth() + lane2->laneWidth()) / 2 + 1.5;
//  double min_dist = (lane1->laneWidth() + lane2->laneWidth()) / 4;
//  Segment_2 ss1(p1s, p1e - v1n * (max_dist + 1.0));
//  Segment_2 se1(p1s + v1n * (max_dist + 1.0), p1e);
//
//  max_dist *= max_dist; // max_dist wird quadriert um schnellere Vergleiche zu ermöglichen
//  min_dist *= min_dist; // min_dist wird quadriert um schnellere Vergleiche zu ermöglichen
//
//  // Richtungswinkel berechnen
//  double angle = atan2(v1.y(), v1.x()) - atan2(v2.y(), v2.x());
//  angle -= floor(angle * M_1_PI * 0.5) * 2. * M_PI; // Winkel normieren
//  bool same_dir = (angle < M_PI_2 || angle > 3 * M_PI_2);
//
//  // Prüfen ob eine der beiden Spuren ein Nachfolger der anderen ist
//  //	if ((l1pe-l2ps).squared_length() < 0.1) return false;
//  //	if ((l2pe-l1ps).squared_length() < 0.1) return false;
//
//  // Abstände berechnen (squared) und überprüfen
//  double l2ps_l1_dist = squared_distance(p2s, l1);
//  double l2pe_l1_dist = squared_distance(p2e, l1);
//  if (l2ps_l1_dist < min_dist || l2ps_l1_dist > max_dist) return;
//  if (l2pe_l1_dist < min_dist || l2pe_l1_dist > max_dist) return;
//
//  // Auf Nachbarspur überprüfen
//  if (same_dir && ((squared_distance(p2s, ss1) < max_dist) || (squared_distance(p2e, se1) < max_dist))) {
//    if ((l1.has_on_negative_side(p2s) && l1.has_on_negative_side(p2e)) || (l1.has_on_positive_side(p2s) && l1.has_on_positive_side(p2e))) {
//      // Punkte so normieren das Verbindungslinien senkrecht stehen
//      Point_2 pp1s = l1.projection(p2s);
//      //		Point_2 pp1e = l1.projection(p2e);
//      Point_2 pp2s = l2.projection(p1s);
//      //		Point_2 pp2e = l2.projection(p1e);
//
//      // Lage auf der Lane überprüfen und Abstände zu den Rändern überprüfen und Einfügung
//      const double min_border_dist = 4.0 * 4.0; // quadriert für schnelleren vergleich
//      if (squared_distance(pp1s, p1s) < min_border_dist) pp1s = pp1s + v1n * EDGE_SAMPLE_DISTANCE;
//      if (squared_distance(pp1s, s1) < 0.001 && squared_distance(pp1s, p1s) >= min_border_dist && squared_distance(pp1s, p1e) >= min_border_dist) {
//        double lat = 0., lon = 0.;
//        utmToLatLong(pp1s.x(), pp1s.y(), l1wp1->utmZone(), &lat, &lon);
//        addWayPoint(lane1, lat, lon, l1wp_i + 1);
//TRACE      ("Adding intermediate Point ("<< pp1s.x() <<","<< pp1s.y() <<") to Lane "<< lane1->name() )
//    }
//    if (squared_distance(pp2s, p2s) < min_border_dist) pp2s = pp2s + v2n * EDGE_SAMPLE_DISTANCE;
//    if (squared_distance(pp2s, s2) < 0.001 && squared_distance(pp2s, p2s) >= min_border_dist && squared_distance(pp2s, p2e) >= min_border_dist) {
//      double lat=0., lon=0.;
//      utmToLatLong(pp2s.x(), pp2s.y(), l2wp1->utmZone(), &lat, &lon);
//      addWayPoint(lane2, lat, lon, l2wp_i+1);
//      TRACE("Adding intermediate Point ("<< pp2s.x() <<","<< pp2s.y() <<") to Lane "<< lane2->name() )
//    }
//  }
//}

//	// Auf Gegenspur überprüfen
//	if (!same_dir && ((squared_distance(l2pe, ss1) < max_dist) || (squared_distance(l2ps, se1) < max_dist)))
//	{
//		if (l1.has_on_positive_side(l2ps) && l1.has_on_positive_side(l2pe)) {
//			laneSeg1.oncomingLaneSegments().insert( &laneSeg2 );
//			laneSeg2.oncomingLaneSegments().insert( &laneSeg1 );
//			result = true;
//		}
//	}

}

void RoadNetwork::sampleEdges() {
//cout << "Add Intermediate Waypoints... " << flush;
//
//// Alle Lanes durchgehen und für jeweils zwei aufeinanderfolgenden Waypoints
//// Zwischenpunkte einfügen
//for (TLaneMap::iterator l1_it = lanes_.begin(); l1_it != lanes_.end(); ++l1_it) {
//  Lane* l = l1_it->second;
//  if (l->isVirtual()) continue;
//  uint32_t wp_index = 0;
//  while (wp_index < l->numWayPoints()-1) {
//    WayPoint* wp1 = l->wayPoint(wp_index);
//    WayPoint* wp2 = l->wayPoint(++wp_index);
//    assert(wp1 && wp2);
//    //			cout << "  sampling edge between Waypoint "<< wp1->name() <<" and Waypoint "<< wp2->name() << endl;
//
//    // Punkte berechnen
//    Point_2 p1(wp1->utmX(), wp1->utmY());
//    Point_2 p2(wp2->utmX(), wp2->utmY());
//    Vector_2 vec = p2-p1;
//    double dist = std::sqrt( vec.squared_length() );
//    vec = vec / dist;
//    int intermediate_count = (int)( (dist / EDGE_SAMPLE_DISTANCE) + 0.5 );
//    //			cout << "  -> dist: "<< dist <<"  ->  inter_points: "<< intermediate_count-1 << endl;
//
//    // Zwischenpunkte erzeugen und zur Lane hinzufügen
//    for (int i = 1; i < intermediate_count; ++i, ++wp_index) {
//      Point_2 pi = p1 + vec * i * EDGE_SAMPLE_DISTANCE;
//      double lat=0., lon=0.;
//      utmToLatLong(pi.x(), pi.y(), wp1->utmZone(), &lat, &lon);
//      //				WayPoint* wpi =
//      addWayPoint(l, lat, lon, wp_index);
//
//      //				cout << "  Adding intermediate Waypoint "<< wpi->name() <<" to lane_" << l->name() << endl;
//    }
//  }
//}
//cout << "Ok" << endl;
}

void RoadNetwork::addLaneSegments() {
cout << "Add Lane Segments... " << flush;

// Iterate through all lanes and create a lane segment all pairs of consecutive way points
for (TLaneMap::iterator l1_it = lanes_.begin(); l1_it != lanes_.end(); ++l1_it) {
  Lane* lane1 = l1_it->second;
  const TWayPointVec& wpoints = lane1->wayPoints();
  if (wpoints.size() < 2) continue;
  for (uint32_t i= 1; i<wpoints.size(); ++i) {
    LaneSegment* lseg = new LaneSegment(lane1, wpoints[i-1], wpoints[i]);
    lane1->addLaneSegment( lseg );
    //			cout << "  Adding LaneSegment " << lseg->name() << " to lane_" << lane1->name() << endl;
  }
}

// link lane segments
for (TLaneMap::iterator l1_it = lanes_.begin(); l1_it != lanes_.end(); ++l1_it)
{
  Lane* lane1 = l1_it->second;

  for (uint32_t i=0; i< lane1->numLaneSegments(); ++i)
  {
    LaneSegment* prev_lseg = ( i>0 ? lane1->laneSegment(i-1) : NULL );
    LaneSegment* lseg = lane1->laneSegment(i);
    LaneSegment* next_lseg = ( i<lane1->numLaneSegments()-1 ? lane1->laneSegment(i+1) : NULL );
    assert(lseg);

    // links inside lane
    if (prev_lseg) {
      prev_lseg->nextLaneSegments().insert(lseg);
      lseg->prevLaneSegments().insert(prev_lseg);
    }
    if (next_lseg) {
      lseg->nextLaneSegments().insert(next_lseg);
      next_lseg->prevLaneSegments().insert(lseg);
    }

    // links to other lanes (link entries)
    for (TExitMap::iterator it=lseg->fromWayPoint()->entries().begin(); it != lseg->fromWayPoint()->entries().end(); ++it) {
      Exit* l_entry = it->second;
      assert(l_entry);
      if (l_entry->getExitFromLane() == NULL || l_entry->getExitToLane() == NULL) continue;
      Lane* vlane = getLane( l_entry->getExitFromLane()->name()+" -> "+l_entry->getExitToLane()->name() );
      assert( vlane );
      assert( vlane->numLaneSegments() == 1 );
      prev_lseg = vlane->laneSegment(0);
      assert(prev_lseg);
      prev_lseg->nextLaneSegments().insert(lseg);
      lseg->prevLaneSegments().insert(prev_lseg);
    }

    // links to other lanes (link exits)
    for (TExitMap::iterator it=lseg->toWayPoint()->exits().begin(); it != lseg->toWayPoint()->exits().end(); ++it) {
      Exit* l_exit = it->second;
      assert(l_exit);
      if (l_exit->getExitFromLane() == NULL || l_exit->getExitToLane() == NULL) continue;
      Lane* vlane = getLane( l_exit->getExitFromLane()->name()+" -> "+l_exit->getExitToLane()->name() );
      assert( vlane );
      assert( vlane->numLaneSegments() == 1 );
      next_lseg = vlane->laneSegment(0);
      assert(next_lseg);
      lseg->nextLaneSegments().insert(next_lseg);
      next_lseg->prevLaneSegments().insert(lseg);
    }
  }

}

cout << "Ok" << endl;
}

void RoadNetwork::connectAdjacentLanes()
{
cout << "Connecting adjacent Lanes... " << flush;

// Alle Lanes durchgehen und geometrisch prüfen ob sie nebeneinander liegen
uint32_t i=0;
for (TLaneMap::iterator l1_it = lanes_.begin(); l1_it != lanes_.end(); ++l1_it) {
  Lane* lane1 = l1_it->second;
  double xmin1, ymin1, xmax1, ymax1;
  lane1->boundingBox(xmin1, ymin1, xmax1, ymax1);

  //		cout << "Lane " << i << " / " << num_lanes << "\n";
  for (size_t ls1_i=0; ls1_i < lane1->numLaneSegments(); ++ls1_i) {
    LaneSegment* ls1 = lane1->laneSegment(ls1_i);
    for (TLaneMap::iterator l2_it = lanes_.begin(); l2_it != lanes_.end(); ++l2_it) {
      Lane* lane2 = l2_it->second;
      if (lane1 == lane2) continue;
      double xmin2, ymin2, xmax2, ymax2;
      lane2->boundingBox(xmin2, ymin2, xmax2, ymax2);
      double max_dist = (lane1->laneWidth() + lane2->laneWidth())/2 + 2.5;
      if(xmax2 < xmin1 - max_dist || ymax2 < ymin1 - max_dist ||
          xmin2 > xmax1 + max_dist || ymin2 > ymax1 + max_dist) {
        continue;
      }
      for (size_t ls2_i=0; ls2_i < lane2->numLaneSegments(); ++ls2_i) {
        LaneSegment* ls2 = lane2->laneSegment(ls2_i);
        //          connectAdjacentLanes_orig(*ls1, *ls2);
        connectAdjacentLanes(*ls1, *ls2);
      }
    }
  }
  i++;
}

cout << "Ok" << endl;
}

bool RoadNetwork::connectAdjacentLanes_orig(LaneSegment& laneSeg1, LaneSegment& laneSeg2)
{
if (&laneSeg1 == &laneSeg2) return false;
if (!laneSeg1.fromWayPoint() || !laneSeg1.toWayPoint()) return false;
if (!laneSeg2.fromWayPoint() || !laneSeg2.toWayPoint()) return false;

//	cout << "  testing "<< laneSeg1.name() << "  <->  "<< laneSeg2.name() << endl;
assert(laneSeg1.name() != laneSeg2.name());

// Punkte, Linien und Vektoren berechnen
Point_2 l1ps = Point_2(laneSeg1.fromWayPoint()->utmX(), laneSeg1.fromWayPoint()->utmY());
Point_2 l1pe = Point_2(laneSeg1.toWayPoint()->utmX() , laneSeg1.toWayPoint()->utmY());
Point_2 l2ps = Point_2(laneSeg2.fromWayPoint()->utmX(), laneSeg2.fromWayPoint()->utmY());
Point_2 l2pe = Point_2(laneSeg2.toWayPoint()->utmX() , laneSeg2.toWayPoint()->utmY());
Line_2 l1(l1ps, l1pe);
Line_2 l2(l2ps, l2pe);
Vector_2 v1 = l1pe - l1ps;
Vector_2 v2 = l2pe - l2ps;
Vector_2 v1n = v1 / std::sqrt( v1.squared_length() );

// Segemente verkürzen um keine Überkreuznachbarschaften zu bekommen
double max_dist = (laneSeg1.lane()->laneWidth() + laneSeg2.lane()->laneWidth())/2 + 2.5;
double min_dist = (laneSeg1.lane()->laneWidth() + laneSeg2.lane()->laneWidth())/4;
Segment_2 ss1(l1ps, l1pe - v1n*(max_dist + 1.0));
Segment_2 se1(l1ps + v1n*(max_dist + 1.0) , l1pe);

max_dist *= max_dist; // max_dist wird quadriert um schnellere Vergleiche zu ermöglichen
min_dist *= min_dist; // min_dist wird quadriert um schnellere Vergleiche zu ermöglichen

// Abstände berechnen (squared) und überprüfen
double l2ps_l1_dist = squared_distance(l2ps, l1);
double l2pe_l1_dist = squared_distance(l2pe, l1);
if ( l2ps_l1_dist < min_dist || l2ps_l1_dist > max_dist ) return false;
if ( l2pe_l1_dist < min_dist || l2pe_l1_dist > max_dist ) return false;

// Prüfen ob eine der beiden Spuren ein Nachfolger der anderen ist
if ((l1pe-l2ps).squared_length() < 0.1) return false;
if ((l2pe-l1ps).squared_length() < 0.1) return false;

// Projektionen überprüfen
Point_2 pp1s = l1.projection(l2ps);
Point_2 pp1e = l1.projection(l2pe);
Point_2 pp2s = l2.projection(l1ps);
Point_2 pp2e = l2.projection(l1pe);
Segment_2 seg1(l1ps, l1pe);
Segment_2 seg2(l2ps, l2pe);
if ( !( squared_distance(seg1, pp1s) < 0.0001 || squared_distance(seg1, pp1e) < 0.0001 ||
        squared_distance(seg2, pp2s) < 0.0001 || squared_distance(seg2, pp2e) < 0.0001) ) return false;

// Richtungswinkel berechnen
//	double angle = atan2(v1.y(), v1.x()) - atan2(v2.y(), v2.x());
//	angle -= floor(angle * M_1_PI * 0.5) * 2. * M_PI;	// Winkel normieren
double delta_angle = deltaAngle( angle( v1 ), angle( v2 ) );
bool same_dir = (delta_angle < M_PI_4 * 0.5 );
bool oppo_dir = (delta_angle > M_PI_4 * 3.5 );
//	cout << "Angle_1: "<< normAngle(angle( v1 ))*180/M_PI << " Angle_2: "<< normAngle(angle( v2 ))*180/M_PI <<"  ->  DeltaAngle: "<< delta_angle*180/M_PI << "  SameDir: "<< (same_dir ? "true" : "false") << "  OppoDir: "<< (oppo_dir ? "true" : "false") << endl;


bool result = false;

// test for neighboring lanes
if ( same_dir && ((squared_distance(l2ps, ss1) < max_dist) || (squared_distance(l2pe, se1) < max_dist)) )
{
  //		cout << "  neighbor dist restriction fulfilled" << endl;
  if (l1.has_on_negative_side(l2ps) && l1.has_on_negative_side(l2pe)) {
    //			cout << "  -> connect as right Lane" << endl;
    laneSeg1.rightLaneSegments().insert( &laneSeg2 );
    laneSeg2.leftLaneSegments().insert( &laneSeg1 );
    result = true;
  }
  else if (l1.has_on_positive_side(l2ps) && l1.has_on_positive_side(l2pe)) {
    //			cout << "  -> connect as left Lane" << endl;
    laneSeg1.leftLaneSegments().insert( &laneSeg2 );
    laneSeg2.rightLaneSegments().insert( &laneSeg1 );
    result = true;
  }
}

// Auf Gegenspur überprüfen
if ( oppo_dir && ((squared_distance(l2pe, ss1) < max_dist) || (squared_distance(l2ps, se1) < max_dist)))
{
  //		cout << "  opposite dist restriction fullfilled" << endl;
  if (l1.has_on_positive_side(l2ps) && l1.has_on_positive_side(l2pe)) {
    //			cout << "  -> connect as left opposite Lane" << endl;
    laneSeg1.oncomingLaneSegments().insert( &laneSeg2 );
    laneSeg2.oncomingLaneSegments().insert( &laneSeg1 );
    result = true;
  }
}

return result;
}

bool RoadNetwork::connectAdjacentLanes(LaneSegment& laneSeg1, LaneSegment& laneSeg2)
{
if (&laneSeg1 == &laneSeg2) return false;
if (!laneSeg1.fromWayPoint() || !laneSeg1.toWayPoint()) return false;
if (!laneSeg2.fromWayPoint() || !laneSeg2.toWayPoint()) return false;

//  cout << "  testing "<< laneSeg1.name() << "  <->  "<< laneSeg2.name() << endl;
assert(laneSeg1.name() != laneSeg2.name());

// Punkte, Linien und Vektoren berechnen
Point_2 l1ps = Point_2(laneSeg1.fromWayPoint()->utmX(), laneSeg1.fromWayPoint()->utmY());
Point_2 l1pe = Point_2(laneSeg1.toWayPoint()->utmX() , laneSeg1.toWayPoint()->utmY());
Point_2 l2ps = Point_2(laneSeg2.fromWayPoint()->utmX(), laneSeg2.fromWayPoint()->utmY());
Point_2 l2pe = Point_2(laneSeg2.toWayPoint()->utmX() , laneSeg2.toWayPoint()->utmY());

double max_dist = (laneSeg1.lane()->laneWidth() + laneSeg2.lane()->laneWidth())/2 + 2.5;
double min_dist = (laneSeg1.lane()->laneWidth() + laneSeg2.lane()->laneWidth())/4;

// BBox check..
double minx1, miny1, maxx1, maxy1;
if(l1ps.x() < l1pe.x()) {minx1 = l1ps.x(); maxx1 = l1pe.x();}
else {minx1 = l1pe.x(); maxx1 = l1ps.x();}
if(l1ps.y() < l1pe.y()) {miny1 = l1ps.y(); maxy1 = l1pe.y();}
else {miny1 = l1pe.y(); maxy1 = l1ps.y();}
minx1 -= max_dist; miny1 -= max_dist;
maxx1 += max_dist; maxy1 += max_dist;

if((l2ps.x() < minx1 && l2pe.x() < minx1) ||
    (l2ps.x() > maxx1 && l2pe.x() < maxx1)) {return false;}

if((l2ps.y() < miny1 && l2pe.y() < miny1) ||
    (l2ps.y() > maxy1 && l2pe.y() < maxy1)) {return false;}

double max2 = max_dist * max_dist;
double min2 = min_dist * min_dist;

Line_2 l1(l1ps, l1pe);
Line_2 l2(l2ps, l2pe);

// Abstände berechnen (squared) und überprüfen
double l2ps_l1_dist = squared_distance(l2ps, l1);
double l2pe_l1_dist = squared_distance(l2pe, l1);
if ( l2ps_l1_dist < min2 || l2ps_l1_dist > max2 ) return false;
if ( l2pe_l1_dist < min2 || l2pe_l1_dist > max2 ) return false;

// Prüfen ob eine der beiden Spuren ein Nachfolger der anderen ist
if ((l1pe-l2ps).squared_length() < 0.1) return false;
if ((l2pe-l1ps).squared_length() < 0.1) return false;

Vector_2 v1 = l1pe - l1ps;
Vector_2 v2 = l2pe - l2ps;
Vector_2 v1n = v1 / std::sqrt( v1.squared_length() );

// Segemente verkürzen um keine Überkreuznachbarschaften zu bekommen
Segment_2 ss1(l1ps, l1pe - v1n*(max_dist + 1.0));
Segment_2 se1(l1ps + v1n*(max_dist + 1.0) , l1pe);

// Projektionen überprüfen
Point_2 pp1s = l1.projection(l2ps);
Point_2 pp1e = l1.projection(l2pe);
Point_2 pp2s = l2.projection(l1ps);
Point_2 pp2e = l2.projection(l1pe);
Segment_2 seg1(l1ps, l1pe);
Segment_2 seg2(l2ps, l2pe);
if ( !( squared_distance(seg1, pp1s) < 0.0001 || squared_distance(seg1, pp1e) < 0.0001 ||
        squared_distance(seg2, pp2s) < 0.0001 || squared_distance(seg2, pp2e) < 0.0001) ) return false;

// Richtungswinkel berechnen
//  double angle = atan2(v1.y(), v1.x()) - atan2(v2.y(), v2.x());
//  angle -= floor(angle * M_1_PI * 0.5) * 2. * M_PI; // Winkel normieren
double delta_angle = deltaAngle( angle( v1 ), angle( v2 ) );
bool same_dir = (delta_angle < M_PI_4 * 0.5 );
bool oppo_dir = (delta_angle > M_PI_4 * 3.5 );
//  cout << "Angle_1: "<< normAngle(angle( v1 ))*180/M_PI << " Angle_2: "<< normAngle(angle( v2 ))*180/M_PI <<"  ->  DeltaAngle: "<< delta_angle*180/M_PI << "  SameDir: "<< (same_dir ? "true" : "false") << "  OppoDir: "<< (oppo_dir ? "true" : "false") << endl;


bool result = false;

// Auf Nachbarspur überprüfen
if ( same_dir && ((squared_distance(l2ps, ss1) < max2) || (squared_distance(l2pe, se1) < max2)) )
{
  //    cout << "  neighbor dist restriction fullfilled" << endl;
  if (l1.has_on_negative_side(l2ps) && l1.has_on_negative_side(l2pe)) {
    //      cout << "  -> connect as right Lane" << endl;
    laneSeg1.rightLaneSegments().insert( &laneSeg2 );
    laneSeg2.leftLaneSegments().insert( &laneSeg1 );
    result = true;
  }
  else if (l1.has_on_positive_side(l2ps) && l1.has_on_positive_side(l2pe)) {
    //      cout << "  -> connect as left Lane" << endl;
    laneSeg1.leftLaneSegments().insert( &laneSeg2 );
    laneSeg2.rightLaneSegments().insert( &laneSeg1 );
    result = true;
  }
}

// check for oncoming lanes
if ( oppo_dir && ((squared_distance(l2pe, ss1) < max2) || (squared_distance(l2ps, se1) < max2)))
{
  //    cout << "  opposite dist restriction fullfilled" << endl;
  if (l1.has_on_positive_side(l2ps) && l1.has_on_positive_side(l2pe)) {
    //      cout << "  -> connect as left opposite Lane" << endl;
    laneSeg1.oncomingLaneSegments().insert( &laneSeg2 );
    laneSeg2.oncomingLaneSegments().insert( &laneSeg1 );
    result = true;
  }
}

return result;
}

void RoadNetwork::addKTurns()
{
cout << "Adding KTurns... " << flush;

// TODO KTurns evtl. verallgemeinern über mehrere Spuren.
//		Könnte man dann auch als UTurn markieren

// lanes auf KTurn Kanten überprüfen
for (TLaneMap::iterator l1_it = lanes_.begin(); l1_it != lanes_.end(); ++l1_it) {
  Lane* lane1 = l1_it->second;
  if ( lane1->isVirtual() ) continue;
  for (size_t ls1_i=0; ls1_i < lane1->numLaneSegments(); ++ls1_i) {
    LaneSegment* ls1 = lane1->laneSegment(ls1_i);
    for (TLaneSegmentSet::iterator l2_it=ls1->nextLaneSegments().begin(); l2_it != ls1->nextLaneSegments().end(); ++l2_it) {
      LaneSegment* ls2 = *l2_it;
      if (ls2->lane() == lane1) continue;
      if (!ls2->lane()->isVirtual()) continue;
      for (TLaneSegmentSet::iterator l3_it=ls2->nextLaneSegments().begin(); l3_it != ls2->nextLaneSegments().end(); ++l3_it) {
        LaneSegment* ls3 = *l3_it;
        if ( ls3->lane()->isVirtual() ) continue;
        if ( ls1->hasOncomingLaneSegment(ls3) ) ls2->kturnLane() = true;
      }
    }
  }
}

cout << "Ok" << endl;
}

void RoadNetwork::addAdditionalKTurnEdges()
{
cout << "Adding Additional KTurn Edges... " << flush;

// lanes auf überschneidungen überprüfen
for (TLaneMap::iterator l1_it = lanes_.begin(); l1_it != lanes_.end(); ++l1_it) {
  Lane* lane1 = l1_it->second;
  if ( lane1->isVirtual() ) continue;
  for (size_t ls1_i=0; ls1_i < lane1->numLaneSegments(); ++ls1_i) {
    LaneSegment* ls1 = lane1->laneSegment(ls1_i);
    for (TLaneMap::iterator l2_it = l1_it; l2_it != lanes_.end(); ++l2_it) {
      Lane* lane2 = l2_it->second;
      if ( lane1 == lane2 ) continue;
      if ( lane2->isVirtual() ) continue;
      for (size_t ls2_i=0; ls2_i < lane2->numLaneSegments(); ++ls2_i) {
        LaneSegment* ls2 = lane2->laneSegment(ls2_i);
        //					cout << "Testing lseg_"<< lane1->name() <<"__"<<ls1_i<<"  <kturn>  "<< lane2->name() <<"__"<< ls2_i << endl;
        testKTurn(*ls1, *ls2);
      }
    }
  }
}

cout << "Ok" << endl;
}

void RoadNetwork::testKTurn(LaneSegment& lseg1, LaneSegment& lseg2)
{
if ( ! lseg1.hasOncomingLaneSegment( &lseg2 ) ) return;
assert( lseg2.hasOncomingLaneSegment( &lseg1 ) );

//	cout << "-> benachbart" << endl;

// Prüfen ob die Kanten zu einer Intersection gehören
if ( lseg1.intersection() != NULL || lseg2.intersection() != NULL ) return;

// TODO Prüfen ob die Kanten nahe bei einer Intersection sind
for (TLaneSegmentSet::iterator it = lseg1.nextLaneSegments().begin(); it != lseg1.nextLaneSegments().end(); ++it)
if ( (*it)->intersection() || (*it)->lane()->isVirtual() ) return;
for (TLaneSegmentSet::iterator it = lseg1.prevLaneSegments().begin(); it != lseg1.prevLaneSegments().end(); ++it)
if ( (*it)->intersection() || (*it)->lane()->isVirtual() ) return;
for (TLaneSegmentSet::iterator it = lseg2.nextLaneSegments().begin(); it != lseg2.nextLaneSegments().end(); ++it)
if ( (*it)->intersection() || (*it)->lane()->isVirtual() ) return;
for (TLaneSegmentSet::iterator it = lseg2.prevLaneSegments().begin(); it != lseg2.prevLaneSegments().end(); ++it)
if ( (*it)->intersection() || (*it)->lane()->isVirtual() ) return;

for (int i = 0; i < 2; ++i)
{
  WayPoint* wf, *wt;
  Line_2 lin;
  if (i == 0) {
    wf = lseg1.toWayPoint();
    wt = lseg2.fromWayPoint();
    lin = Line_2( Point_2(lseg1.fromWayPoint()->utmX(), lseg1.fromWayPoint()->utmY()),
        Point_2(lseg1.toWayPoint()->utmX() , lseg1.toWayPoint()->utmY()) );
  }
  else {
    wf = lseg2.toWayPoint();
    wt = lseg1.fromWayPoint();
    lin = Line_2( Point_2(lseg2.fromWayPoint()->utmX(), lseg2.fromWayPoint()->utmY()),
        Point_2(lseg2.toWayPoint()->utmX() , lseg2.toWayPoint()->utmY()) );
  }

  // check boundaries
  if ( wf->parentLane()->leftBoundaryType() == Lane::DoubleYellow || wf->parentLane()->leftBoundaryType() == Lane::SolidYellow) continue;
  if ( wt->parentLane()->leftBoundaryType() == Lane::DoubleYellow || wt->parentLane()->leftBoundaryType() == Lane::SolidYellow) continue;

  // check distances
  Point_2 pp = lin.projection( Point_2(wt->utmX() , wt->utmY()) );
  if ( squared_distance(pp, Point_2(wf->utmX() , wf->utmY())) > sqr(2.0) ) continue;

  // check if link already exists
  if ( exits_.find( wf->name() + "_" + wt->name() ) != exits_.end() ) continue;
  if ( lanes_.find( wf->name()+" -> "+wt->name() ) != lanes_.end() ) continue;

  // create link
  addExit( wf, wt );

  // create a new virtual Lane
  Lane* l = new Lane(0, wf->name()+" -> "+wt->name(), true);
  m_virtualLanes.insert(l);
  lanes_.insert( make_pair(l->name(), l) );
  WayPoint* w1 = new WayPoint(0, "0.1." + boost::lexical_cast<string>( m_virtualLanes.size() ));
  WayPoint* w2 = new WayPoint(0, "0.2." + boost::lexical_cast<string>( m_virtualLanes.size() ));
  w1->setLatLon(wf->lat(), wf->lon());
  w2->setLatLon(wt->lat(), wt->lon());
  l->addWayPoint(w1);
  l->addWayPoint(w2);
  l->setLaneWidth( 4.5 );

  // create lanesegment
  LaneSegment* lseg = new LaneSegment(l, w1, w2);
  lseg->kturnLane() = true;
  l->addLaneSegment( lseg );
}
}

void RoadNetwork::entryAndExitPoints(LaneSegment& ls, WayPoint*& entry, WayPoint*& exit) {
TLaneSegmentSet& prev_lss = ls.prevLaneSegments();
if(!prev_lss.empty()) {
  entry=(*prev_lss.begin())->toWayPoint();
}
else {
  entry=ls.toWayPoint();
}

TLaneSegmentSet& succ_lss = ls.nextLaneSegments();
if(!succ_lss.empty()) {
  exit=(*succ_lss.begin())->fromWayPoint();
}
else {
  exit=ls.fromWayPoint();
}
}

void RoadNetwork::numEntriesAndExits(Intersection& is, uint32_t& num_entries, uint32_t& num_exits) {
const TLaneSegmentSet lane_segments = is.laneSegments();
TWayPointMap is_points;
std::multimap<std::string, WayPoint*> real_wps;
for(TLaneSegmentSet::const_iterator lsit=lane_segments.begin(); lsit != lane_segments.end(); lsit++) {
  if((*lsit)->lane()->isVirtual()) {
    WayPoint* entry_wp, *exit_wp;
    entryAndExitPoints(*(*lsit), entry_wp, exit_wp);
    is_points.insert(std::make_pair(entry_wp->name(), entry_wp));
    is_points.insert(std::make_pair(exit_wp->name(), exit_wp));
  }
  else {
    real_wps.insert(std::make_pair((*lsit)->fromWayPoint()->name(), (*lsit)->fromWayPoint()));
    real_wps.insert(std::make_pair((*lsit)->toWayPoint()->name(), (*lsit)->toWayPoint()));
  }
}
std::multimap<std::string, WayPoint*>::const_iterator rwpit=real_wps.begin();
while(rwpit != real_wps.end()) {
  WayPoint* wp = rwpit->second;
  Lane* lane = wp->parentLane();
  uint32_t idx = lane->wayPointIndex(wp);
  rwpit++; idx++;
  is_points.insert(std::make_pair(wp->name(), wp));
  while(rwpit != real_wps.end() && idx < lane->wayPoints().size()) {
    if (!(rwpit->second->name() == lane->wayPoints()[idx]->name() || rwpit->second->name() == lane->wayPoints()[idx-1]->name())) {break;}
    rwpit++; idx++;
  }
  if(rwpit != real_wps.end()) {
    std::multimap<std::string, WayPoint*>::const_iterator last_rwpit=rwpit;
    last_rwpit--;
    is_points.insert(std::make_pair(last_rwpit->second->name(), last_rwpit->second));
  }
  else {
    std::multimap<std::string, WayPoint*>::const_iterator last_rwpit=--real_wps.end();
    is_points.insert(std::make_pair(last_rwpit->second->name(), last_rwpit->second));
    break;
  }
}

TWayPointMap::const_iterator wpit=is_points.begin(), wpit_end=is_points.end();
TWayPointMap entry_map, exit_map;
for(; wpit != wpit_end; wpit++) {
  WayPoint* wp = (*wpit).second;
  Lane* lane = wp->parentLane();

  TExitMap::const_iterator eit=wp->entries().begin();

  for(; eit != wp->entries().end(); eit++) {
    WayPoint* ewp = eit->second->getExitFromLane();
    if(ewp) {
      entry_map.insert(std::make_pair(ewp->name(), ewp));
    }
  }

  eit=wp->exits().begin();

  for(; eit != wp->exits().end(); eit++) {
    WayPoint* ewp = eit->second->getExitToLane();
    if(ewp) {
      exit_map.insert(std::make_pair(ewp->name(), ewp));
    }
  }

  uint32_t index=lane->wayPointIndex(wp);
  if(index==0) {
    WayPoint* next_wp=lane->wayPoint(index+1);
    exit_map.insert(std::make_pair(next_wp->name(), next_wp));
  }
  else if(index==lane->numWayPoints()-1) {
    WayPoint* prev_wp=lane->wayPoint(index-1);
    entry_map.insert(std::make_pair(prev_wp->name(), prev_wp));
  }
  else {
    WayPoint* prev_wp=lane->wayPoint(index-1);
    entry_map.insert(std::make_pair(prev_wp->name(), prev_wp));
    WayPoint* next_wp=lane->wayPoint(index+1);
    exit_map.insert(std::make_pair(next_wp->name(), next_wp));
  }
}

for(TWayPointMap::const_iterator sit=entry_map.begin(); sit != entry_map.end(); ) {
  Lane* lane = sit->second->parentLane();
  num_entries++;
  do {sit++;}while(sit != entry_map.end() && sit->second->parentLane() == lane);
}

for(TWayPointMap::const_reverse_iterator sit=exit_map.rbegin(); sit != exit_map.rend();) {
  Lane* lane = sit->second->parentLane();
  num_exits++;
  do {sit++;}while(sit != exit_map.rend() && sit->second->parentLane() == lane);
}

//printf("num entries: %u, num exits: %u\n", num_entries, num_exits);

// check if all angles between connected entry and exit lane segments are
// small than a given threshold (in this case it's not real intersection)
double angle_thresh = 45.0;
// first compare all entry with all other entry and all exit lane segments
for(TWayPointMap::const_iterator enit=entry_map.begin(); enit != entry_map.end(); enit++) {
  LaneSegment* ls1 =NULL;
  ls1 = enit->second->parentLane()->laneSegmentWithToPoint(enit->second);
  if(!ls1) {
    ls1 = enit->second->parentLane()->laneSegmentWithFromPoint(enit->second);
  }
  double dx1=ls1->toWayPoint()->x() - ls1->fromWayPoint()->x();
  double dy1=ls1->toWayPoint()->y() - ls1->fromWayPoint()->y();
  double len1 = hypot(dx1, dy1);

  for(TWayPointMap::const_iterator enit2=entry_map.begin(); enit2 != entry_map.end(); enit2++) {
    if(enit == enit2) {continue;}
    LaneSegment* ls2 =NULL;
    ls2 = enit2->second->parentLane()->laneSegmentWithToPoint(enit2->second);
    if(!ls2) {
      ls2 = enit2->second->parentLane()->laneSegmentWithFromPoint(enit2->second);
    }
    double dx2=ls2->toWayPoint()->x() - ls2->fromWayPoint()->x();
    double dy2=ls2->toWayPoint()->y() - ls2->fromWayPoint()->y();
    double len2 = hypot(dx2, dy2);
    double angle = acos( (dx1*dx2+dy1*dy2)/(len1*len2) )/M_PI*180.0;
    if(std::abs(angle) > angle_thresh && std::abs(angle) < 180.0-angle_thresh) {
//      printf("%s %s: %f\n", ls1->name().c_str(), ls2->name().c_str(), angle);
      return;
    }
  }

  for(TWayPointMap::const_iterator enit2=exit_map.begin(); enit2 != exit_map.end(); enit2++) {
    if(enit == enit2) {continue;}
    LaneSegment* ls2=NULL;
    ls2 = enit2->second->parentLane()->laneSegmentWithFromPoint(enit2->second);
    if(!ls2) {
      ls2 = enit2->second->parentLane()->laneSegmentWithToPoint(enit2->second);
    }
    double dx2=ls2->toWayPoint()->x() - ls2->fromWayPoint()->x();
    double dy2=ls2->toWayPoint()->y() - ls2->fromWayPoint()->y();
    double len2 = hypot(dx2, dy2);
    double angle = acos( (dx1*dx2+dy1*dy2)/(len1*len2) )/M_PI*180.0;
    if(std::abs(angle) > angle_thresh && std::abs(angle) < 180.0-angle_thresh) {
//      printf("%s %s: %f\n", ls1->name().c_str(), ls2->name().c_str(), angle);
      return;
    }
  }

}

// no compare all exit lane segments with all other exit lane segments
for(TWayPointMap::const_iterator enit=exit_map.begin(); enit != exit_map.end(); enit++) {
  LaneSegment* ls1 =NULL;
  ls1 = enit->second->parentLane()->laneSegmentWithFromPoint(enit->second);
  if(!ls1) {
    ls1 = enit->second->parentLane()->laneSegmentWithToPoint(enit->second);
  }
  double dx1=ls1->toWayPoint()->x() - ls1->fromWayPoint()->x();
  double dy1=ls1->toWayPoint()->y() - ls1->fromWayPoint()->y();
  double len1 = hypot(dx1, dy1);

  for(TWayPointMap::const_iterator enit2=exit_map.begin(); enit2 != exit_map.end(); enit2++) {
    if(enit == enit2) {continue;}
    LaneSegment* ls2 =NULL;
    ls2 = enit2->second->parentLane()->laneSegmentWithFromPoint(enit2->second);
    if(!ls2) {
      ls2 = enit2->second->parentLane()->laneSegmentWithToPoint(enit2->second);
    }
    double dx2=ls2->toWayPoint()->x() - ls2->fromWayPoint()->x();
    double dy2=ls2->toWayPoint()->y() - ls2->fromWayPoint()->y();
    double len2 = hypot(dx2, dy2);
    double angle = acos( (dx1*dx2+dy1*dy2)/(len1*len2) )/M_PI*180.0;
    if(std::abs(angle) > angle_thresh && std::abs(angle) < 180.0-angle_thresh) {
//      printf("%s %s: %f\n", ls1->name().c_str(), ls2->name().c_str(), angle);
      return;
    }
  }
}

num_entries=num_exits=0;
}

static const double big_epsilon_ = 0.01;
bool segmentIntersect(double x1, double y1, double x2, double y2,
  double x3, double y3, double x4, double y4,
  double& intx, double& inty, bool& end_point1, bool& end_point2) {
double Ax, Bx, Cx, Ay, By, Cy, d, e, f, num;
double x1lo, x1hi, y1lo, y1hi;

Ax = x2 - x1;
Bx = x3 - x4;

// X bound box test
if (Ax < 0) {
  x1lo = x2;
  x1hi = x1;
}
else {
  x1hi = x2;
  x1lo = x1;
}
if (Bx > 0) {
  if (x1hi < x4 || x3 < x1lo) {
    return false;
  }
}
else {
  if (x1hi < x3 || x4 < x1lo) {
    return false;
  }
}

Ay = y2 - y1;
By = y3 - y4;

// Y bound box test
if (Ay < 0) {
  y1lo = y2;
  y1hi = y1;
}
else {
  y1hi = y2;
  y1lo = y1;
}
if (By > 0) {
  if (y1hi < y4 || y3 < y1lo) {
    return false;
  }
}
else {
  if (y1hi < y3 || y4 < y1lo) {
    return false;
  }
}

Cx = x1 - x3;
Cy = y1 - y3;
d = By * Cx - Bx * Cy; // alpha numerator
f = Ay * Bx - Ax * By; // both denominator
if (f > 0) { // alpha tests
  if (d < 0 || d > f) {
    return false;
  }
}
else {
  if (d > 0 || d < f) {
    return false;
  }
}

e = Ax * Cy - Ay * Cx; // beta numerator
if (f > 0) { // beta tests
  if (e < 0 || e > f) {
    return false;
  }
}
else {
  if (e > 0 || e < f) {
    return false;
  }
}

// compute intersection coordinates
if (f == 0) {
  return false; // parallel
}

num = d * Ax; // numerator
intx = x1 + num / f;

num = d * Ay;
inty = y1 + num / f;

end_point1=false;
if(num<big_epsilon_) {end_point1=true;}

end_point2=false;
if(std::abs(x2-intx) < big_epsilon_ && std::abs(y2-inty) < big_epsilon_) {
  end_point2=true;
}

return true;
}

bool intersectionsShareWaypoint(Intersection& is1, Intersection& is2) {

return false;
}

void RoadNetwork::addIntersections()
{
cout << "Adding Intersections... " << flush;

// check for crossing lanes
for (TLaneMap::iterator l1_it = lanes_.begin(); l1_it != lanes_.end(); ++l1_it) {
  Lane* lane1 = l1_it->second;
  for (size_t ls1_i=0; ls1_i < lane1->numLaneSegments(); ++ls1_i) {
    LaneSegment* ls1 = lane1->laneSegment(ls1_i);
    for (TLaneMap::iterator l2_it = l1_it; l2_it != lanes_.end(); ++l2_it) {
      Lane* lane2 = l2_it->second;
      for (size_t ls2_i=0; ls2_i < lane2->numLaneSegments(); ++ls2_i) {
        LaneSegment* ls2 = lane2->laneSegment(ls2_i);
        testIntersection(*ls1, *ls2);
      }
    }
  }
}

// Intersection lanes transitiv verknüpfen
for (TLaneMap::iterator l1_it = lanes_.begin(); l1_it != lanes_.end(); ++l1_it) {
  Lane* lane1 = l1_it->second;
  for (size_t ls1_i=0; ls1_i < lane1->numLaneSegments(); ++ls1_i) {
    LaneSegment* ls1 = lane1->laneSegment(ls1_i);
    vector<LaneSegment*> lsegs;
    lsegs.push_back(ls1);
    if (ls1->intersection()) addTransitivInterectionLanes(lsegs);
  }
}

//  bool checked_all_for_merge = false;
//  while(!checked_all_for_merge) {
//    checked_all_for_merge = true;
//    for(TIntersectionSet::const_iterator iit=intersections().begin(); iit != intersections().end(); iit++) {
//      for(TIntersectionSet::const_iterator iit2=intersections().begin(); iit2 != intersections().end(); iit2++) {
//        if(iit != iit2) {
//          Intersection& is1 = *(*iit);
//          Intersection& is2 = *(*iit2);
//          if(intersectionsShareWaypoint(is1, is2)) {
//            //mergeIntersections(is1, is2);
//            cout << "  -> fusing "<< is1 <<" and "<< is2 << endl;
//            const TLaneSegmentSet& ilanes2 = is2.laneSegments();
//            for (TLaneSegmentSet::iterator it=ilanes2.begin(); it != ilanes2.end(); ++it) {
//                LaneSegment* switch_lane = *it;
//                is1.addLaneSegment(switch_lane);
//                switch_lane->intersection() = is1;
//            }
//            while(!ilanes2.empty()) {
//              ilanes2.erase(ilanes2.begin());
//            }
//            delIntersection(isec2);
//            iit = intersections().end();
//            iit2 = intersections().end();
//            checked_all_for_merge = false;
//            continue;
//          }
//        }
//      }
//    }

//  }

// removing lane connections that are not really intersections
std::vector<Intersection*> intersections_to_delete;
for(TIntersectionSet::const_iterator iit=intersections().begin(); iit != intersections().end(); iit++) {
  Intersection& is = *(*iit);
//  printf("\nIntersection %s:\n", is.name().c_str());
  uint32_t num_entries=0, num_exits=0;
  numEntriesAndExits(is, num_entries, num_exits);
  if(num_exits <=1 && num_entries <=1) {
    intersections_to_delete.push_back(&is);
  }
}
for(std::vector<Intersection*>::iterator isit=intersections_to_delete.begin();
    isit != intersections_to_delete.end(); isit++) {
  delIntersection(*isit);
}
cout << "Ok" << endl;
//cout << "Dumping intersection Map:\n\n";
//TIntersectionSet::const_iterator insit=intersections().begin(), insit_end=intersections().end();
//for(; insit!=insit_end; insit++) {
//  cout << "Intersection " << (*insit)->name() << ":" << endl;
//  TLaneSegmentSet::const_iterator lsit=(*insit)->laneSegments().begin(), lsit_end=(*insit)->laneSegments().end();
//  for(; lsit!=lsit_end; lsit++) {
//    cout << (*lsit)->name();
//    if((*lsit)->lane()->isVirtual()) {
//      WayPoint* entry, *exit;
//      entryAndExitPoints(*(*lsit), entry, exit);
//      cout << " (" << entry->name() << " - " << exit->name() << ")\n";
//    }
//    else {
//      cout << endl;
//    }
//  }
//}
}

bool RoadNetwork::testIntersection(LaneSegment& laneSeg1, LaneSegment& laneSeg2)
{
if (&laneSeg1 == &laneSeg2) {return false;}
if (!laneSeg1.fromWayPoint() || !laneSeg1.toWayPoint()) {return false;}
if (!laneSeg2.fromWayPoint() || !laneSeg2.toWayPoint()) {return false;}

//	cout << "  testing Lane "<< laneSeg1.lane()->name() << "   <-->   "<< laneSeg2.lane()->name() << endl;

// check for k-turn edges
if (laneSeg1.isKTurnEdge() || laneSeg2.isKTurnEdge()) {return false;}

// create points and segments
Point_2 l1ps = Point_2(laneSeg1.fromWayPoint()->x(), laneSeg1.fromWayPoint()->y());
Point_2 l1pe = Point_2(laneSeg1.toWayPoint()->x(), laneSeg1.toWayPoint()->y());
Point_2 l2ps = Point_2(laneSeg2.fromWayPoint()->x(), laneSeg2.fromWayPoint()->y());
Point_2 l2pe = Point_2(laneSeg2.toWayPoint()->x(), laneSeg2.toWayPoint()->y());
Segment_2 s1(l1ps, l1pe);
Segment_2 s2(l2ps, l2pe);
// check if start way points are identical
//	if ((l1ps - l2ps).squared_length() < 0.1) return false;
if ((l1pe - l2ps).squared_length() < 0.1) return false;
if ((l2pe - l1ps).squared_length() < 0.1) return false;

// check if lanes intersect
//  std::vector<Segment_2> segments;
//  segments.push_back(s1); segments.push_back(s2);
//	std::vector<Point_2> ipoints;
//	CGAL::compute_intersection_points(segments.begin(), segments.end(), back_inserter(ipoints));
double intx, inty;
bool end_point1, end_point2;
bool intersect = segmentIntersect(laneSeg1.fromWayPoint()->x(), laneSeg1.fromWayPoint()->y(),
    laneSeg1.toWayPoint()->x(), laneSeg1.toWayPoint()->y(),
    laneSeg2.fromWayPoint()->x(), laneSeg2.fromWayPoint()->y(),
    laneSeg2.toWayPoint()->x(), laneSeg2.toWayPoint()->y(), intx, inty, end_point1, end_point2);

if(!intersect) {return false;}
//	if(end_point1 || end_point2) {return false;}
//	  printf("intersecting @: %f, %f, ep1: %i, ep2: %i\n", intx, inty, (int)end_point1, (int)end_point2);

//	if(ipoints.empty()) return false;
//  printf("CGAL: intersecting @: %f, %f\n", ipoints[0].x(), ipoints[0].y());
//  if((l1ps - ipoints[0]).squared_length() < 0.1) return false;
//  if((l1pe - ipoints[0]).squared_length() < 0.1) return false;
//  if((l2ps - ipoints[0]).squared_length() < 0.1) return false;
//  if((l2pe - ipoints[0]).squared_length() < 0.1) return false;

//if (!do_intersect(s1, s2)) return false;		// && (l1pe - l2pe).squared_length() > 0.1
laneSeg1.crossingLaneSegments().insert(&laneSeg2);
laneSeg2.crossingLaneSegments().insert(&laneSeg1);

//	cout << "  -> intersect" << endl;

// add new intersection or merge with existing one and add corresponding lanes
addLaneSegmentToIntersection(laneSeg1, laneSeg2);

// add neighboring lanes
addAdjacentLanesToIntersection(laneSeg1, true);
addAdjacentLanesToIntersection(laneSeg1, false);
addAdjacentLanesToIntersection(laneSeg2, true);
addAdjacentLanesToIntersection(laneSeg2, false);

return true;
}

void RoadNetwork::addAdjacentLanesToIntersection(LaneSegment& ls, bool left_dir)
{
if (left_dir) {
  for (TLaneSegmentSet::iterator it = ls.leftLaneSegments().begin(); it != ls.leftLaneSegments().end(); ++it) {
    addLaneSegmentToIntersection(ls, **it);
    addAdjacentLanesToIntersection(**it, true);
  }
  for (TLaneSegmentSet::iterator it = ls.oncomingLaneSegments().begin(); it != ls.oncomingLaneSegments().end(); ++it) {
    addLaneSegmentToIntersection(ls, **it);
    addAdjacentLanesToIntersection(**it, false);
  }
}
else {
  for (TLaneSegmentSet::iterator it = ls.rightLaneSegments().begin(); it != ls.rightLaneSegments().end(); ++it) {
    addLaneSegmentToIntersection(ls, **it);
    addAdjacentLanesToIntersection(**it, false);
  }
}
}

void RoadNetwork::addTransitivInterectionLanes(TLaneSegmentVec& lsegs)
{
assert( lsegs.size() > 0 );
assert( lsegs[0]->intersection() );

// Nächstes LaneSegment ermitteln
LaneSegment* actSeg = *--lsegs.end();
Lane* actLane = actSeg->lane();
TLaneSegmentVec::iterator it = find(actLane->lane_segments_.begin(), actLane->lane_segments_.end(), actSeg);
assert( it != actLane->lane_segments_.end() );
if (++it == actLane->lane_segments_.end()) return;
LaneSegment* nextSeg = *it;

// Prüfen ob das letzte und erste untersuchte
if (nextSeg->intersection() == lsegs[0]->intersection()) {
  for (TLaneSegmentVec::iterator it=++lsegs.begin(); it != lsegs.end(); ++it) {
    addLaneSegmentToIntersection(*lsegs[0], **it);
  }
  return;
}

if (lsegs.size() < MAX_TRANSITIVE_EDGES_SEARCH_DEPTH && nextSeg->intersection() == NULL) {
  lsegs.push_back(nextSeg);
  addTransitivInterectionLanes(lsegs);
}
}

void RoadNetwork::addLaneSegmentToIntersection(LaneSegment& laneSeg1, LaneSegment& laneSeg2) {
Intersection* isec = NULL;
if (laneSeg1.intersection() != NULL) {
  isec = laneSeg1.intersection();
  Intersection* isec2 = laneSeg2.intersection();
  if (isec == isec2) return;
  if (isec2) {
    //            cout << "  -> fusing "<< (*isec) <<" and "<< (*isec2) << endl;
    TLaneSegmentSet& ilanes2 = *const_cast<TLaneSegmentSet*>(&isec2->laneSegments());
    for (TLaneSegmentSet::iterator it=ilanes2.begin(); it != ilanes2.end(); ++it) {
      LaneSegment* switch_lane = *it;
      isec->addLaneSegment(switch_lane);
      switch_lane->setIntersection(isec);
    }
    // delIntersection() would reset parent intersection in all lane segments that were
    // just transferred. Clear the list of ls for the intersection to be deleted to prevent
    // this.
    while(!ilanes2.empty()) {
      ilanes2.erase(ilanes2.begin());
    }
    delIntersection(isec2);
  }
  else {
    //			cout << "  -> adding lane_"<< laneSeg2.lane()->name() <<" to "<< *isec << endl;
    isec->addLaneSegment(&laneSeg2);
    laneSeg2.setIntersection(isec);
  }
}
else
if (laneSeg2.intersection()) {
  isec = laneSeg2.intersection();
  //		cout << "  -> adding lane_"<< laneSeg1.lane()->name() <<" to "<< *isec << endl;
  isec->addLaneSegment(&laneSeg1);
  laneSeg1.setIntersection(isec);
}
else {
  //		cout << "  -> Pommes ole" << endl;
  isec = addIntersection();
  //		cout << "  -> adding lane_"<< laneSeg1.lane()->name() <<" and lane_"<< laneSeg2.lane()->name() <<" to new "<< *isec << endl;
  isec->addLaneSegment(&laneSeg1);
  laneSeg1.setIntersection(isec);
  isec->addLaneSegment(&laneSeg2);
  laneSeg2.setIntersection(isec);
}
}

void RoadNetwork::addLaneSegemetTypes()
{
for (TStopMap::iterator it=stops_.begin(); it != stops_.end(); ++it) {
  Stop* s = it->second;
  assert(s);
  WayPoint* wp = s->wayPoint();
  assert(wp);
  Lane* l = wp->parentLane();
  assert(l);
  LaneSegment* lseg = l->laneSegmentWithToPoint(wp);

  markStopLaneSegments(*lseg, *lseg, lseg->intersection(), false);
  markStopLaneSegments(*lseg, *lseg, lseg->intersection(), true);
}
}

void RoadNetwork::markStopLaneSegments(LaneSegment& lseg, LaneSegment& start_seg, const Intersection* starting_isec, bool forward)
{
if (forward) {
  if (starting_isec != NULL && lseg.intersection() == NULL) return;
  if (&lseg != &start_seg && lseg.isStopLane()) return;
  lseg.stopLane() = true;
  if (lseg.lane()->isVirtual()) return;
  for (TLaneSegmentSet::iterator it=lseg.nextLaneSegments().begin(); it != lseg.nextLaneSegments().end(); ++it)
  markStopLaneSegments(**it, start_seg, (starting_isec ? starting_isec : lseg.intersection()), forward);
}
else {
  if (lseg.intersection() != NULL && (starting_isec == NULL || lseg.intersection()->id() != starting_isec->id())) return;
  if (lseg.isKTurnEdge()) return;
  if (&lseg != &start_seg && lseg.isStopLane()) return;
  lseg.stopLane() = true;
  for (TLaneSegmentSet::iterator it=lseg.prevLaneSegments().begin(); it != lseg.prevLaneSegments().end(); ++it)
  markStopLaneSegments(**it, start_seg, starting_isec, forward);
}
}

// ----------------------------------------------------------------------------


void RoadNetwork::dump()
{

cout << "Dumping road network " << name_ << "..." << endl;
cout << "Creation Date: " << creation_date_ << endl;
cout << "Format Version: " << format_version_ << endl;
cout << "# of Segments: " << segments_.size() << endl;
cout << "# of Lanes: " << lanes_.size() << endl;
cout << "# of WayPoints: " << waypoints_.size() << endl;
cout << "# of Checkpoints: " << checkpoints_.size() << endl;
cout << "# of Stops: " << stops_.size() << endl;
cout << "# of Exits: " << exits_.size() << endl;

cout << "--------- Network segments -------------" << endl;
TSegmentMap::iterator sit,sit_end;
for(sit = segments_.begin(),sit_end = segments_.end(); sit != sit_end; ++sit)
(*sit).second->dump();

cout << "----------- Network zones --------------" << endl;
TZoneMap::iterator zit,zit_end;
for(zit = zones_.begin(),zit_end = zones_.end(); zit != zit_end; ++zit)
(*zit).second->dump();
}

CheckPoint* RoadNetwork::checkPoint(const string& strName)
{
CheckPoint* pCheckPoint = NULL;

// check if node exists
TCheckPointMap::iterator it = checkpoints_.find(strName);
if (it != checkpoints_.end())
{
  pCheckPoint = (*it).second;
}
else
{
  setStatus("checkPoint: checkpoint with identifier " + strName + " not found.");
}
return(pCheckPoint);
}

Segment* RoadNetwork::segment(const string& strName)
{
Segment* pSegment = NULL;

// check if node exists
TSegmentMap::iterator it = segments_.find(strName);
if (it != segments_.end())
{
  pSegment = (*it).second;
}
else
{
  setStatus("segment: Segment with identifier " + strName + " not found.");
}
return(pSegment);
}

Perimeter* RoadNetwork::perimeter(const string& strName)
{
Perimeter* pPerimeter = NULL;

// check if node exists
TPerimeterMap::iterator it = perimeters_.find(strName);
if (it != perimeters_.end())
{
  pPerimeter = (*it).second;
}
else
{
  setStatus("perimeter: Perimeter with identifier " + strName + " not found.");
}
return(pPerimeter);
}

Zone* RoadNetwork::zone(const string& strName)
{
Zone* pZone = NULL;

// check if node exists
TZoneMap::iterator it = zones_.find(strName);
if (it != zones_.end())
{
  pZone = (*it).second;
}
else
{
  setStatus("zone: Zone with identifier " + strName + " not found.");
}
return(pZone);
}

uint32_t RoadNetwork::nextZoneId() const
{
return nextSegmentId();
}

std::string RoadNetwork::nextZoneStr() const
{
return nextSegmentStr();
}

Spot* RoadNetwork::getSpot(const string& strName)
{
Spot* pSpot = NULL;

// check if node exists
TSpotMap::iterator it = spots_.find(strName);
if (it != spots_.end())
{
  pSpot = (*it).second;
}
else
{
  setStatus("getSpot: Spot with identifier " + strName + " not found.");
}
return(pSpot);
}

WayPoint* RoadNetwork::wayPoint(const string& strName)
{
WayPoint* pWayPoint = NULL;

// check if node exists
TWayPointMap::iterator it = waypoints_.find(strName);
if (it != waypoints_.end())
{
  pWayPoint = (*it).second;
}
else
{
  setStatus("wayPoint: waypoint with identifier " + strName + " not found.");
}
return(pWayPoint);
}

PerimeterPoint* RoadNetwork::perimeterPoint(const string& strName)
{
PerimeterPoint* pPerimeterPoint = NULL;

// check if node exists
TPerimeterPointMap::iterator it = perimeter_points_.find(strName);
if (it != perimeter_points_.end())
{
  pPerimeterPoint = (*it).second;
}
else
{
  setStatus("getPerimiterPoint: perimiter point with identifier " + strName + " not found.");
}
return(pPerimeterPoint);
}

Lane* RoadNetwork::getLane(const string& strName) {
Lane* pLane = NULL;

// check if node exists
TLaneMap::iterator it = lanes_.find(strName);
if (it != lanes_.end()) {
  pLane = (*it).second;
}
else {
  setStatus("getLane: lane with identifier " + strName + " not found.");
}
return pLane;
}

void RoadNetwork::clear() {

return; // disabled for now, crashes....
while(!segments_.empty()) {
  segments_.begin()->second->destroy();
  segments_.erase(segments_.begin());
}

while(!lanes_.empty()) {
  lanes_.begin()->second->destroy();
  lanes_.erase(lanes_.begin());
}
while(!zones_.empty()) {
  zones_.begin()->second->destroy();
  zones_.erase(zones_.begin());
}

while(!perimeters_.empty()) {
  perimeters_.begin()->second->destroy();
  perimeters_.erase(perimeters_.begin());
}

while(!perimeter_points_.empty()) {
  perimeter_points_.begin()->second->destroy();
  perimeter_points_.erase(perimeter_points_.begin());
}

while(!spots_.empty()) {
  spots_.begin()->second->destroy();
  spots_.erase(spots_.begin());
}

while(!waypoints_.empty()) {
  waypoints_.begin()->second->destroy();
  waypoints_.erase(waypoints_.begin());
}

while(!checkpoints_.empty()) {
  checkpoints_.begin()->second->destroy();
  checkpoints_.erase(checkpoints_.begin());
}

while(!exits_.empty()) {
  exits_.begin()->second->destroy();
  exits_.erase(exits_.begin());
}

while(!traffic_lights_.empty()) {
  traffic_lights_.begin()->second->destroy();
  traffic_lights_.erase(traffic_lights_.begin());
}

while(!crosswalks_.empty()) {
  crosswalks_.begin()->second->destroy();
  crosswalks_.erase(crosswalks_.begin());
}
}

string RoadNetwork::section(istream& stream, const string& strStartToken,const string& strEndToken) {
vector<string> tokens;
string line;
string strData="";
while (true) {
  tokens.clear();
  if(!getline(stream,line)) break;
  if (!line.length()) continue;
  line = clearCComments(line);
  splitString(line,tokens,RNDF_DELIMITER);
  if (tokens.empty()) continue;
  if(tokens[0]==strStartToken) continue;
  if(tokens[0]==strEndToken) break;
  strData += line;
  strData += "\n";
}

return strData;
}

// calculate rndf center
coordinate_latlon_t RoadNetwork::center()
{
coordinate_latlon_t rndf_min, rndf_max, rndf_center;

rndf_min.lat = rndf_min.lon = 180.0;
rndf_max.lat = rndf_max.lon = -180.0;

// check all waypoints
rndf::WayPoint* w;
TWayPointMap::const_iterator it,it_end;
for(it=wayPoints().begin(),it_end=wayPoints().end();it!=it_end;++it) {
  w = it->second;
  assert(w);
  assert(w->lat()<180.0 && w->lat()>-180.0);
  assert(w->lon()<180.0 && w->lon()>-180.0);
  if(w->lat()<rndf_min.lat) rndf_min.lat = w->lat();
  if(w->lon()<rndf_min.lon) rndf_min.lon = w->lon();
  if(w->lat()>rndf_max.lat) rndf_max.lat = w->lat();
  if(w->lon()>rndf_max.lon) rndf_max.lon = w->lon();
}

// check all Perimeter points
rndf::PerimeterPoint* p;
TPerimeterPointMap::const_iterator itp,itp_end;
for(itp=perimeterPoints().begin(),itp_end=perimeterPoints().end();itp!=itp_end;++itp) {
  p = itp->second;
  assert(p);
  assert(p->lat()<180.0 && p->lat()>-180.0);
  assert(p->lon()<180.0 && p->lon()>-180.0);
  if(p->lat()<rndf_min.lat) rndf_min.lat = p->lat();
  if(p->lon()<rndf_min.lon) rndf_min.lon = p->lon();
  if(p->lat()>rndf_max.lat) rndf_max.lat = p->lat();
  if(p->lon()>rndf_max.lon) rndf_max.lon = p->lon();
}

rndf_center.lat =(rndf_min.lat+rndf_max.lat)/2.0;
rndf_center.lon =(rndf_min.lon+rndf_max.lon)/2.0;
return rndf_center;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const RoadNetwork& rn) {

os << std::setprecision(9);
os << RNDF_ROADNETWORK_NAME << " " << rn.name_ << endl;
os << RNDF_ROADNETWORK_NUM_SEGMENTS << " " << rn.segments_.size() << endl;
os << RNDF_ROADNETWORK_NUM_ZONES << " " << rn.zones_.size() << endl;
os << RNDF_ROADNETWORK_FORMAT_VERSION << " " << rn.format_version_ << endl;
os << RNDF_ROADNETWORK_CREATION_DATE << " " << rn.creation_date_ << endl;

TSegmentMap::const_iterator sit,sit_end;
for(sit = rn.segments_.begin(),sit_end = rn.segments_.end(); sit != sit_end; ++sit)
os << *sit->second;

TZoneMap::const_iterator zit,zit_end;
for(zit = rn.zones_.begin(),zit_end = rn.zones_.end(); zit != zit_end; ++zit)
os << *zit->second;

TCrosswalkMap::const_iterator cwit=rn.crosswalks_.begin(), cwit_end=rn.crosswalks_.end();
for(; cwit != cwit_end; ++cwit) {
  os << *cwit->second;
}

TTrafficLightMap::const_iterator tlit=rn.traffic_lights_.begin(), tlit_end=rn.traffic_lights_.end();
for(; tlit != tlit_end; ++tlit) {
  os << *tlit->second;
}

os << "end_file" << endl;

return os;
}

void RoadNetwork::draw(double center_x, double center_y, double blend, bool wp_labels) const {

if(!rndfgl_) {
  throw VLRException("Road network was created without GL support (default). "
      + std::string("GL support can be activated by setting the constructor parameter of the road network ")
      + std::string("create_visualization = true or calling createVisualization(). ")
      + std::string("Warning: The road network class has to be constructed after initializing the GL render context."));
}

bool draw_lane_backgrounds = false;

//Segment* s;
Lane* l;
WayPoint* w, *w_next;
Perimeter* p;
PerimeterPoint *pp;
Spot* s;

glLineWidth(2);

// draw the lanes
glColor4f(0, 0, 1, blend);
TSegmentMap::const_iterator itSegments, itSegments_end;
TLaneMap::const_iterator itLanes, itLanes_end;
TLaneSet::const_iterator itVLanes, itVLanes_end;
TPerimeterMap::const_iterator itPerimeters, itPerimeters_end;
TWayPointVec::const_iterator itWayPoints, itWayPoints_end, itWayPoints_next;
TPerimeterPointVec::const_iterator itPerimeterPoints, itPerimeterPoints_end, itPerimeterPoints_next;
TExitMap::const_iterator itExits, itExits_end;
TCheckPointMap::const_iterator cit;
TStopMap::const_iterator sit;
TSpotMap::const_iterator itSpots, itSpots_end;

// draw Lane arrow and background
for (itLanes = lanes().begin(), itLanes_end = lanes().end(); itLanes != itLanes_end; ++itLanes) {
  l = itLanes->second;
  if (l->isVirtual()) continue;
  // set lanewidth
  double width = (l->laneWidth() == 0. ? 1.0 : l->laneWidth());

  for (itWayPoints = l->wayPoints().begin(), itWayPoints_end = l->wayPoints().end(); itWayPoints
      != itWayPoints_end; ++itWayPoints) {
    w = *itWayPoints;
    itWayPoints_next = itWayPoints;
    itWayPoints_next++;
    if (itWayPoints_next != itWayPoints_end) {
      w_next = *itWayPoints_next;

      // draw lane background
      if (draw_lane_backgrounds) {
        glColor4f(0.7, 0.7, 0.7, blend*0.85);
        // glColor4f(0.7, 0.7, 0.7, 1.0);
        double dx = w_next->utmX() - w->utmX();
        double dy = w_next->utmY() - w->utmY();
        rndfgl_->draw_lane_background(w->utmX() + dx * 0.5 - center_x, w->utmY() + dy * 0.5 - center_y, atan2(dy, dx), width, sqrt(dx * dx + dy * dy));
      }
      // draw lane arrow
      glColor4f(0, 0, 1, blend);
      draw_arrow(w->utmX() - center_x, w->utmY() - center_y, w_next->utmX() - center_x,
          w_next->utmY() - center_y, 0.5, 2.0);
    }
  }
}

// draw lane boundaries
glColor4f(1, 1, 1, blend);
for (itLanes = lanes().begin(), itLanes_end = lanes().end(); itLanes != itLanes_end; ++itLanes) {
  l = itLanes->second;
  if (l->isVirtual()) continue;
  rndfgl_->draw_lane_boundary(l, center_x, center_y, 1, blend);
  rndfgl_->draw_lane_boundary(l, center_x, center_y, 0, blend);
}

// draw way points
glPointSize(5.0);
glPushMatrix();
glTranslatef(0, 0, -0.005);
for (itLanes = lanes().begin(), itLanes_end = lanes().end(); itLanes != itLanes_end; ++itLanes) {
  l = itLanes->second;
  if (l->isVirtual()) continue; // virtual lanes are drawn later
  for (itWayPoints = l->wayPoints().begin(), itWayPoints_end = l->wayPoints().end(); itWayPoints != itWayPoints_end; ++itWayPoints) {
    w = *itWayPoints;
    if(!w->isVirtual()) {
      glColor4f(0, 0, 1, blend);
      draw_circle(w->utmX() - center_x, w->utmY() - center_y, 0.5, 1);
      glColor4f(0, .4, 1, blend);
      render_stroke_text_centered_2D(w->utmX() - center_x - 1.5, w->utmY() - center_y - 1, GLUT_STROKE_ROMAN, 0.5, (char*) w->name().c_str());
    }
    else {
      glColor4f(.4, .4, 1, blend);
      draw_circle(w->utmX() - center_x, w->utmY() - center_y, 0.3, 1);
    }
  }
}
for (itSpots = spots().begin(), itSpots_end = spots().end(); itSpots != itSpots_end; ++itSpots) {
  s = itSpots->second;
  for (itWayPoints = s->wayPoints().begin(), itWayPoints_end = s->wayPoints().end(); itWayPoints
      != itWayPoints_end; ++itWayPoints) {
    w = *itWayPoints;
    glColor4f(0, 0, 1, blend);
    draw_circle(w->utmX() - center_x, w->utmY() - center_y, 0.5, 1);
    glColor4f(0, .4, 1, blend);
    render_stroke_text_centered_2D(w->utmX() - center_x - 1.5, w->utmY() - center_y - 1, GLUT_STROKE_ROMAN, 0.5,
        (char*) w->name().c_str());
  }
}
glPointSize(1.0);
glPopMatrix();

// draw the perimeter
glColor4f(1, 1, 0, blend);
for (itPerimeters = perimeters().begin(), itPerimeters_end = perimeters().end(); itPerimeters
    != itPerimeters_end; ++itPerimeters) {
  p = itPerimeters->second;
  glBegin(GL_LINE_LOOP);
  for (itPerimeterPoints = p->perimeterPoints().begin(), itPerimeterPoints_end = p->perimeterPoints().end(); itPerimeterPoints
      != itPerimeterPoints_end; ++itPerimeterPoints) {
    pp = *itPerimeterPoints;
    glVertex2f(pp->utmX() - center_x, pp->utmY() - center_y);
  }
  glEnd();
}

// perimeter points in magenta
glPointSize(5.0);
glPushMatrix();
glColor4f(1, 0, 1, blend);
glTranslatef(0, 0, -0.005);
for (itPerimeters = perimeters().begin(), itPerimeters_end = perimeters().end(); itPerimeters
    != itPerimeters_end; ++itPerimeters) {
  p = itPerimeters->second;
  glBegin(GL_POINTS);
  for (itPerimeterPoints = p->perimeterPoints().begin(), itPerimeterPoints_end = p->perimeterPoints().end(); itPerimeterPoints
      != itPerimeterPoints_end; ++itPerimeterPoints) {
    pp = *itPerimeterPoints;
    draw_circle(pp->utmX() - center_x, pp->utmY() - center_y, 0.5, 1);
    render_stroke_text_centered_2D(pp->utmX() - center_x - 1.5, pp->utmY() - center_y - 1, GLUT_STROKE_ROMAN, 0.5,
        (char*) pp->name().c_str());
  }
  glEnd();
}
glPointSize(1.0);
glPopMatrix();

//----------------------------------------------------------------------
// draw exits
Point_2 point_from;
Point_2 point_to;
Vector_2 dir_from;
Vector_2 dir_to;
double width;

// draw lane backgrounds for virtual lanes
if (draw_lane_backgrounds) {
  glColor4f(0.7, 0.7, 0.7, blend * 0.85);
  for (itExits = exits().begin(), itExits_end = exits().end(); itExits != itExits_end; ++itExits) {
    rndf::Exit* e = itExits->second;
    rndfgl_->get_exitpoint_params(e, point_from, point_to, dir_from, dir_to, width);
    if (e->exitType() == rndf::Exit::LaneToLane) {
      const vector<Point_2>& inter_points = rndfgl_->calc_intermediate_points(point_from, point_to, dir_from, dir_to);
      for (uint32_t i = 0; i < inter_points.size() - 1; ++i) {
        const Point_2& p1 = inter_points[i];
        const Point_2& p2 = inter_points[i + 1];
        rndfgl_->draw_lane_background(p1 - Vector_2(center_x, center_y), p2 - Vector_2(center_x, center_y), (i > 0 ? p1
                - inter_points[i - 1] : p2 - p1), (i < inter_points.size() - 2 ? inter_points[i + 2] - p2 : p2 - p1),
            width, width);
      } // i loop

    }
    else {

      double dx = point_to.x() - point_from.x();
      double dy = point_to.y() - point_from.y();
      rndfgl_->draw_lane_background(point_from.x() + dx * 0.5 - center_x, point_from.y() + dy * 0.5 - center_y, atan2(dy, dx),
          width, sqrt(dx * dx + dy * dy));

    }
  }
}

// draw lane dashed arrow
glColor4f(0, 0, 1, blend);
for (itExits = exits().begin(), itExits_end = exits().end(); itExits != itExits_end; ++itExits) {
  rndfgl_->get_exitpoint_params(itExits->second, point_from, point_to, dir_from, dir_to, width);
  const vector<Point_2>& inter_points = rndfgl_->calc_intermediate_points(point_from, point_to, dir_from, dir_to);
  for (uint i = 0; i < inter_points.size() - 1; ++i) {
    const Point_2& p1 = inter_points[i];
    const Point_2& p2 = inter_points[i + 1];
    draw_dashed_line(p1.x() - center_x, p1.y() - center_y, p2.x() - center_x, p2.y() - center_y, 1.0);
  } // i loop
  const Point_2& p1 = inter_points[inter_points.size() - 2];
  const Point_2& p2 = inter_points[inter_points.size() - 1];
  draw_arrowhead(p2.x() - center_x, p2.y() - center_y, atan2(p2.y() - p1.y(), p2.x() - p1.x()));
}

// draw entry and exit point
glColor4f(0.5, 0, 0, blend);
for (itExits = exits().begin(), itExits_end = exits().end(); itExits != itExits_end; ++itExits) {
  rndfgl_->get_exitpoint_params(itExits->second, point_from, point_to, dir_from, dir_to, width);
  draw_circle(point_from.x() - center_x, point_from.y() - center_y, 0.5, 1);
  draw_circle(point_to.x() - center_x, point_to.y() - center_y, 0.5, 1);
}
// ------------------------ end draw exits --------------------------------

// draw checkpoints
for (cit = checkPoints().begin(); cit != checkPoints().end(); ++cit) {
  int cpID = CStringTools::gnCInt((*cit).second->name().substr((*cit).second->name().find_last_of('.') + 1));
  rndfgl_->draw_checkpoint((*cit).second->wayPoint()->utmX() - center_x, (*cit).second->wayPoint()->utmY() - center_y,
      0.5, cpID, 0.8);
}

// draw stoppoints
for (sit = stops().begin(); sit != stops().end(); ++sit) {
  rndfgl_->draw_stoppoint(sit->second->wayPoint()->utmX() - center_x, sit->second->wayPoint()->utmY() - center_y, 0.7,
      0.8);
}

bool draw_lights = true, draw_crosswalks = true;

if (draw_lights) {
  rndfgl_->draw_rndf_lights(*this, 1, center_x, center_y, blend, true);
}

if (draw_crosswalks) {
  rndfgl_->draw_rndf_crosswalks(*this, 1, center_x, center_y, blend);
}

//
//    /* draw the stop signs */
//    for(i = 0; i < rndf->num_segments(); i++)
//      for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//        for(k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
//          w = rndf->segment(i)->Lane(j)->waypoint(k);
//          if(w->stop()) {
//            glPushMatrix();
//            glTranslatef(0, 0, -0.1);
//            if(w->prev() != NULL)
//              angle = atan2(w->utmY() - w->prev()->utmY(),
//                            w->utmX() - w->prev()->utmX());
//            else
//              angle = 0;
//            if(threeD_signs)
//              draw_stop_sign_3D(w->utmX() - origin_x, w->utmY() - origin_y,
//                                2.0, 0.5, angle, blend);
//            else
//              draw_stop_sign_2D(w->utmX() - origin_x, w->utmY() - origin_y,
//                                1.2, angle, blend);
//            glPopMatrix();
//          }
//        }
//

//
//    /* draw the zones */
//    for(i = 0; i < rndf->num_zones(); i++) {
//      glColor4f(0, 1, 0, blend);
//      glBegin(GL_LINE_LOOP);
//      for(j = 0; j < rndf->Zone(i)->num_perimeter_points(); j++) {
//        w = rndf->Zone(i)->perimeter(j);
//        glVertex2f(w->utmX() - origin_x,        w->utmY() - origin_y);
//      }
//      glEnd();
//
//      if(draw_numbers) {
//        glColor4f(1, 1, 1, blend);
//        for(j = 0; j < rndf->Zone(i)->num_perimeter_points(); j++) {
//          w = rndf->Zone(i)->perimeter(j);
//          sprintf(str, "%d.%d.%d", i + rndf->num_segments() + 1, 0, j + 1);
//          render_stroke_text_centered_2D(w->utmX() - origin_x + 2,
//                                         w->utmY() - origin_y + 2,
//                                         GLUT_STROKE_ROMAN, 1.0, str);
//        }
//      }
//    }
//
//    /* draw the Zone spots */
//    for(i = 0; i < rndf->num_zones(); i++) {
//      glBegin(GL_LINES);
//      glColor4f(0, 1, 0, blend);
//      for(j = 0; j < rndf->Zone(i)->num_spots(); j++) {
//        w = rndf->Zone(i)->Spot(j)->waypoint(0);
//        glVertex2f(w->utmX() - origin_x, w->utmY() - origin_y);
//        glVertex2f(w->next()->utmX() - origin_x, w->next()->utmY() - origin_y);
//      }
//      glEnd();
//
//      glColor4f(1, 1, 0, blend);
//      for(j = 0; j < rndf->Zone(i)->num_spots(); j++) {
//        w = rndf->Zone(i)->Spot(j)->waypoint(0);
//        draw_diamond(w->utmX() - origin_x, w->utmY() - origin_y, 1.0);
//        draw_diamond(w->next()->utmX() - origin_x,
//                     w->next()->utmY() - origin_y, 1.0);
//      }
//    }
//
//    /* draw the checkpoints */
//    for(i = 0; i < rndf->num_segments(); i++)
//      for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//        for(k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
//          w = rndf->segment(i)->Lane(j)->waypoint(k);
//          if(w->checkpoint())
//            draw_checkpoint(w->utmX() - origin_x, w->utmY() - origin_y, 3.0,
//                            w->checkpoint_id(), blend);
//        }
//    for(i = 0; i < rndf->num_zones(); i++)
//      for(j = 0; j < rndf->Zone(i)->num_spots(); j++)
//        for(k = 0; k < rndf->Zone(i)->Spot(j)->num_waypoints(); k++) {
//          w = rndf->Zone(i)->Spot(j)->waypoint(k);
//          if(w->checkpoint())
//            draw_checkpoint(w->utmX() - origin_x, w->utmY() - origin_y, 3.0,
//                            w->checkpoint_id(), blend);
//        }
//  }
}

void RoadNetwork::drawExtras(double center_x, double center_y, double blend) const {

if(!rndfgl_) {
  throw VLRException("Road network was created without GL support (default). "
      + std::string("GL support can be activated by setting the constructor parameter of the road network ")
      + std::string("create_visualization = true or calling createVisualization(). ")
      + std::string("Warning: The road network class has to be constructed after initializing the GL render context."));
}

// draw adjacent Lane connectors
TLaneMap::const_iterator itLanes, itLanes_end;
for (itLanes = lanes().begin(), itLanes_end = lanes().end(); itLanes != itLanes_end; ++itLanes) {
  rndfgl_->draw_lane_connections(*itLanes->second, center_x, center_y, blend);
}
//  for(itVLanes=virtualLanes().begin(), itVLanes_end=virtualLanes().end(); itVLanes!=itVLanes_end; ++itVLanes) {
//    draw_lane_connections( **itVLanes, center_x, center_y, blend );
//  }

// draw kturns
for (itLanes = lanes().begin(), itLanes_end = lanes().end(); itLanes != itLanes_end; ++itLanes) {
  Lane* l = itLanes->second;
  for (TLaneSegmentVec::const_iterator lseg_it = l->laneSegments().begin(); lseg_it != l->laneSegments().end(); ++lseg_it) {
    if ((*lseg_it)->isKTurnEdge()) rndfgl_->draw_kturns(*lseg_it, center_x, center_y, blend);
  }
}

// draw lane segment types
for (TLaneMap::const_iterator it = lanes().begin(); it != lanes().end(); ++it) {
  rndfgl_->draw_TypesLaneSegment(*it->second, center_x, center_y, blend);
}

// draw intersections
for (TIntersectionSet::const_iterator int_it = intersections().begin(); int_it != intersections().end(); ++int_it) {
  rndfgl_->draw_intersection(**int_it, center_x, center_y, blend);
}
}
void RoadNetwork::generateDisplayList(double blend, bool dynamic) {

if(!rndfgl_) {
  throw VLRException("Road network was created without GL support (default). "
      + std::string("GL support can be activated by setting the constructor parameter of the road network ")
      + std::string("create_visualization = true or calling createVisualization(). ")
      + std::string("Warning: The road network class has to be constructed after initializing the GL render context."));
}

rndfgl_->generate_rndf_display_list(*this, blend, dynamic);
}

} // namespace rndf

} // namespace vlr

