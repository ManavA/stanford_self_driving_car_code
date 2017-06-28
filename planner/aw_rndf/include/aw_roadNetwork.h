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


#ifndef AW_ROADNETWORK_H_
#define AW_ROADNETWORK_H_

#include <istream>
#include <stdint.h>

#ifdef RNDF_GL
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <lltransform.h>

#include <aw_RNDFTokens.h>
#include <aw_segment.h>
#include <aw_lane.h>
#include <aw_wayPoint.h>
#include <aw_exit.h>
#include <aw_checkPoint.h>
#include <aw_stop.h>
#include <aw_zone.h>
#include <aw_perimeter.h>
#include <aw_perimeterPoint.h>
#include <aw_spot.h>
#include <aw_intersection.h>
#include <aw_crosswalk.h>
#include <aw_trafficLight.h>

namespace vlr {

namespace rndf {

class RoadNetworkGL;

class RoadNetwork {
public:
    RoadNetwork(const std::string& strName="Road Network", bool activate_rendering = false);
    RoadNetwork(const RoadNetwork& other);
    virtual ~RoadNetwork(void);

  RoadNetwork& operator=(const RoadNetwork& other);
  RoadNetwork& copy(const RoadNetwork& other);
  virtual RoadNetwork* clone() const;


  void clear();
  bool empty() const { return segments_.empty(); };

  void setStatus(const std::string& strStatus)    { status_ = strStatus; }
  void appendStatus(std::string& strStatus) { status_ += strStatus + '\n'; }
  const std::string& status()            { return status_;              }

  void setName(std::string strName)         { name_ = strName;             }
  const std::string& name()              { return name_;                }

  // loading and storing a network
  bool loadRNDF(const std::string& strFileName);
  bool saveRNDF(const std::string& strFileName);

  // adding and removing segments
  Segment* addSegment();
  Segment* addSegment(const std::string& strName);
  Segment* addSegment(const std::string& strName, const std::string& strData);
  void delSegment(Segment* s);
  Segment* segment(const std::string& strName);
  int numSegments() { return (int)segments_.size(); }
  uint32_t nextSegmentId() const;
  std::string nextSegmentStr() const;

  // add Intersection
  Intersection* addIntersection();
  Intersection* addIntersection(const std::string& strName, const std::string& strData);
  void delIntersection(Intersection* intersec);
  size_t numIntersections() { return intersections_.size(); }
  uint32_t nextIntersectionId() const;
  std::string nextIntersectionStr() const;

  // adding and removing lanes
  Lane* addLane(Segment* pSegment);
  Lane* addLane(Segment* pSegment, const std::string& strName);
  Lane* addLane(Segment* pSegment, const std::string& strName, const std::string& strData);
  void delLane(Lane* l);
  Lane* getLane(const std::string& strName);
  int numLanes() { return (int)lanes_.size(); }

  // adding and removing WayPoints
  WayPoint* addWayPoint(Lane* pLane, const double& lat, const double& lon, bool is_virtual);
  WayPoint* addWayPoint(Lane* pLane, const double& lat, const double& lon, uint32_t insert_before, bool is_virtual);
  WayPoint* addWayPoint(Lane* pLane, const std::string& strName, const double& lat, const double& lon, bool is_virtual);
  WayPoint* addWayPoint(Lane* pLane, const std::string& strName, const double& lat, const double& lon, uint32_t insert_before, bool is_virtual);
  WayPoint* addWayPoint(Spot* pSpot, const double& lat, const double& lon);
  WayPoint* addWayPoint(Spot* pSpot, const std::string& strName, const double& lat, const double& lon);
  void delWayPoint(WayPoint* wp);
  WayPoint* wayPoint(const std::string& strName);
  int numWayPoints() { return (int)waypoints_.size(); }

  // exits
  Exit* addExit(std::string strName);
  Exit* addExit(WayPoint* pWayPointFrom, WayPoint* pWayPointTo);
  Exit* addExit(PerimeterPoint* pPerimeterPointTo, WayPoint* pWayPointTo);
  Exit* addExit(WayPoint* pWayPointFrom, PerimeterPoint* pPerimeterPointTo);
  Exit* addExit(PerimeterPoint* pPerimterPointFrom, PerimeterPoint* pPerimterPointTo);

  void delExit(Exit* e);
  int numExits() { return (int)exits_.size(); }

  // stops
  Stop* addStop(const std::string& strWayPoint);
  Stop* stop(const std::string& strName);
  void delStop(Stop* s);
  int numStops() { return (int)stops_.size(); }

  // CheckPoints
  CheckPoint* addCheckPoint(WayPoint* wp);
  CheckPoint* addCheckPoint(WayPoint* wp, const std::string& strName);
  CheckPoint* checkPoint(const std::string& strName);
  void delCheckPoint(CheckPoint* cp);
  int numCheckPoints() { return (int)checkpoints_.size(); }

  // zones
  Zone* addZone();
  Zone* addZone(const std::string& strName);
  Zone* addZone(const std::string& strName, const std::string& strData);
  Zone* zone(const std::string& strName);
  void delZone(Zone* z);
  int numZones() { return (int)zones_.size(); }
  uint32_t nextZoneId() const;
  std::string nextZoneStr() const;

  // Perimeter
  Perimeter* addPerimeter(Zone* Zone);
  Perimeter* addPerimeter(Zone* Zone, const std::string& strName);
  Perimeter* addPerimeter(Zone* Zone, const std::string& strName, const std::string& strData);
  Perimeter* perimeter(const std::string& strName);
  void delPerimeter(Perimeter* p);
  int numPerimeters() { return (int)perimeters_.size(); }

  // PerimeterPoints
//  PerimeterPoint* addPerimeterPoint(Perimeter* pPerimeter, const double& lat, const double& lon);
  PerimeterPoint* addPerimeterPoint(Perimeter* pPerimeter, const std::string& strName, const double& lat, const double& lon);
  PerimeterPoint* addPerimeterPoint(Perimeter* pPerimeter, const double& lat, const double& lon, uint32_t insert_before);
  PerimeterPoint* addPerimeterPoint(Perimeter* pPerimeter, const std::string& strName, const double& lat, const double& lon, uint32_t insert_before);
  PerimeterPoint* perimeterPoint(const std::string& strName);
  void delPerimeterPoint(PerimeterPoint* pp);
  int numPerimeterPoints() { return (int)perimeter_points_.size(); }

  // spots
  Spot* addSpot(Zone* pZone);
  Spot* addSpot(Zone* pZone, const std::string& strName);
  Spot* addSpot(Zone* pZone, const std::string& strName, const std::string& strData);
  Spot* getSpot(const std::string& strName);
  void delSpot(Spot* s);
  int numSpots() { return (int)spots_.size(); }

  // crosswalks
  Crosswalk* addCrosswalk(WayPoint* w);
  Crosswalk* addCrosswalk();
  Crosswalk* addCrosswalk(const std::string& strName);
  Crosswalk* addCrosswalk(const std::string& strName, const std::string& strData);
  Crosswalk* crosswalk(const std::string& strName);
  void delCrosswalk(Crosswalk* cw);
  uint32_t numCrosswalks() { return (uint32_t)crosswalks_.size(); }

  // traffic lights
  TrafficLight* addTrafficLight(WayPoint* w);
  TrafficLight* addTrafficLight(const std::string& strName);
  TrafficLight* addTrafficLight();
  TrafficLight* addTrafficLight(const std::string& strName, const std::string& strData);
  TrafficLight* trafficLight(const std::string& strName);
  void delTrafficLight(TrafficLight* tl);
  uint32_t numTrafficLights() { return (uint32_t)traffic_lights_.size(); }

  //! adds virtual lanes and connects adjacent lanes and creates intersections
  /*! \remark after relations where added the graph is not allowed to get modified */
  void addRelations();

  const TSegmentMap& segments() const  { return segments_; }
  const TIntersectionSet& intersections() const { return intersections_; }
  const TLaneMap& lanes() const { return lanes_; }
  const TLaneSet& virtualLanes() const { return m_virtualLanes; }
  const TWayPointMap& wayPoints() const { return waypoints_; }
  const TExitMap& exits() const { return exits_; }
  const TCheckPointMap& checkPoints() const { return checkpoints_; }
  const TStopMap& stops() const { return stops_; }
  const TZoneMap& zones() const { return zones_; }
  const TPerimeterMap& perimeters() const { return perimeters_; }
  const TPerimeterPointMap& perimeterPoints() const { return perimeter_points_; }
  const TSpotMap& spots() const { return spots_; }
  const TCrosswalkMap& crosswalks() const { return crosswalks_; }
  const TTrafficLightMap& trafficLights() const { return traffic_lights_; }

  //! returns the geographical center
  vlr::coordinate_latlon_t center();

  // debug
  void dump();

  // parsing methods
  static std::string section(std::istream& stream, const std::string& strStartToken,const std::string& strEndToken);

  bool hasVisualization() const {return rndfgl_ != NULL;}
  void createVisualization();
  void draw(double center_x, double center_y, double blend, bool wp_labels) const;
  void drawExtras(double center_x, double center_y, double blend) const;
  void generateDisplayList(double blend, bool dynamic = false);

private:
  //! adds virtual Lane objects for Exit-entry connections so that they can get connected
  void addVirtualLanes();

  //! adds waypoints so that the relations are especially beautiful ;)
  /*! doesnt work properly right know */
  void alignEdgeStarts();
  //! adds intermediate waypoints for the given lanes
  void alignEdgeStarts(Lane* l1, uint32_t l1wp_i, Lane* l2, uint32_t l2wp_i);
  //! adds intermediate waypoints to avoid long edges and get better neighbor relations
  void sampleEdges();
  //! add LaneSegments to the lanes according to their waypoints
  void addLaneSegments();

  //! finds all lanes that are adjacent and connects them
  void connectAdjacentLanes();
  //! finds all lanes that are adjacent and connects them
  bool connectAdjacentLanes(LaneSegment& laneSeg1, LaneSegment& laneSeg2);
  bool connectAdjacentLanes_orig(LaneSegment& laneSeg1, LaneSegment& laneSeg2);
  //! finds all lanes that form an Intersection and connects them accordingly

  //! finds all kturn edges and marks them
  void addKTurns();
  //! insert KTurn Edges at all positions where a KTurn is theoretical possible
  void addAdditionalKTurnEdges();
  // test if it is possible to do a KTurn
  void testKTurn(LaneSegment& laneSeg1, LaneSegment& laneSeg2);
  //! adds intersections to roadnet and connects the according lanes segments wih them
  void addIntersections();
  //! tests if the lanes \a laneSeg1 and \a laneSeg2 intersect and connects them to an Intersection
  bool testIntersection(LaneSegment& laneSeg1, LaneSegment& laneSeg2);
  //! adds adjacent lanes to intersections
  void addAdjacentLanesToIntersection(LaneSegment& ls, bool left_dir = true);
  //! adds two LaneSegments to a Intersection
  void addLaneSegmentToIntersection(LaneSegment& laneSeg1, LaneSegment& laneSeg2);
  //! adds transitive lanes to Intersection
  void addTransitivInterectionLanes(TLaneSegmentVec& lsegs);

  //! adds stoplane and priority flags to lanes segments
  void addLaneSegemetTypes();
  //! marks a lane Segment as a stoplane and explores recursivly in the given direction
  void markStopLaneSegments(LaneSegment& lseg, rndf::LaneSegment& start_seg, const Intersection* starting_isec, bool forward);

  void entryAndExitPoints(LaneSegment& ls, WayPoint*& entry, WayPoint*& exit);
  void numEntriesAndExits(Intersection& is, uint32_t& num_entries, uint32_t& num_exits);

  //! streaming operator
  friend std::ostream& operator<<(std::ostream& os, const RoadNetwork& rn);

  TSegmentMap segments_;
  TIntersectionSet intersections_;
  TLaneMap lanes_;
  TLaneSet m_virtualLanes;
  TWayPointMap waypoints_;
  TExitMap exits_;
  TCheckPointMap checkpoints_;
  TStopMap stops_;
  TZoneMap zones_;
  TPerimeterMap perimeters_;
  TPerimeterPointMap perimeter_points_;
  TSpotMap spots_;
  TCrosswalkMap crosswalks_;
  TTrafficLightMap traffic_lights_;

  std::string status_;
  std::string name_;
  float format_version_;
  std::string creation_date_;

  RoadNetworkGL* rndfgl_;
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const RoadNetwork& rn);

} // namespace rndf

} // namespace vlr

#endif
