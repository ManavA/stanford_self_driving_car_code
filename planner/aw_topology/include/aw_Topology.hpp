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


#ifndef TOPOLOGY_HPP_
#define TOPOLOGY_HPP_

#include <deque>
#include <boost/utility.hpp>
#include <boost/format.hpp>

#include <driving_common/Trajectory2D.h>

#include <aw_roadNetwork.h>
#include <aw_Mission.h>
#include <aw_VehicleManager.hpp>
#include <aw_BlockadeManager.hpp>
#include <aw_RndfGraphBuilder.h>
#include <aw_RndfGraph.h>
#include <aw_Route.h>
#include <aw_graph_tools.hpp>

using std::deque;

namespace vlr {

const unsigned int WHOLE_GRAPH_DT = 0;
const unsigned int MISSION_DT = 1;

#define TRIGGER_DIST_MAX_LOOKAHEAD				80.0
#define TRIGGER_DIST_MAX_LOOKBACK				30.0
#define TRIGGER_DIST_OFF_TRACK					5.0      // Distanz die das Fahrzeug vom Spurrand enfernt sein muss bevor replanned wird
#define TRIGGER_DIST_GET_BACK_ON_TRACK			5.0      // Distanz die das Fahrzeug vom Spurrand enfernt sein muss das Fahrzeug damit mit dem Navigator zurück zur Edge geplant wird

  // TODO: replace with passat_constants.h
// EGO VEHICLE
#define FRONT_BUMPER_DELTA						( 1.85 +1.93       +0.22)
#define BACK_BUMPER_DELTA						(-1.85 -1.93 +4.77 +0.22)
#define CENTER_DELTA                              1.85
const double EGO_VEHICLE_LENGTH                 = FRONT_BUMPER_DELTA + BACK_BUMPER_DELTA;
const double EGO_VEHICLE_WIDTH                  = 2.0;

#define OBSTACLE_SPEED_THRESHOLD				1.3 // CHECK: this seams to be a high value, depends on accuracy of tracker
#define VEH_DIST_FROM_STOPLINE_THRESHOLD		2.5 // [m]

const double MAX_INITIAL_SEARCH_DISTANCE          = 10.0;

const unsigned int TOPOLOGY_MESSAGE_QUEUE_SIZE    = 16;

class IntersectionManager;

/*
   collects some data structures describing road network and mission topology,
   as well as map matching functionality (jz)
 */

class Topology : private boost::noncopyable {
public:
  struct DistanceVisualisation {
    double x,y,r;
    double cr,cg,cb;
    DistanceVisualisation(double x, double y, double r, double cr=1.0, double cg=0.5, double cb=0.5) : x(x), y(y), r(r), cr(cr), cg(cg), cb(cb) {};
  };
  std::vector<DistanceVisualisation> debug_distances;

  Topology(const std::string& rndf_filename, const std::string& mdf_filename, const std::string name="", bool create_visualization = false, bool recover_from_file = false, bool save_mission_progress_to_file_ = false);

  void planMission(double initial_x, double initial_y, double initial_yaw);
	void updateEgoVehicle(double x, double y, double yaw, double speed); // update pos of ego vehicle and match it to graph and mission

	// get next obstacle on mission graph. puts found distance into distance
private:
	Vehicle* nextObstacle(double& distance, GraphTools::PlaceOnGraph& place, double max_scan_distance);

public:
	Vehicle* nextVehicle();

  // to paint the graphs
  void paint_complete_graph( double center_x, double center_y, double r, double g, double b );
  void paint_mission_graph( double center_x, double center_y, double r, double g, double b, bool annotations = true );
  void paint_complete_mission(double center_x, double center_y );
  void paint_annotations( double center_x, double center_y, AnnotatedRouteEdge* annoedge, double height=0.0);
  void draw_edge( double center_x, double center_y, const RoutePlanner::RndfEdge& edge, float r, float g, float b, float height=0.0 );
  void draw_arrow_at( double x, double y, double yaw, double scale, float r, float g, float b, float height=0.0 );
  void draw_dot_at(double x, double y, float r, float g, float b );

  const RoutePlanner::RndfGraph* completeGraph() const {return complete_graph;}
  RoutePlanner::RndfGraph* completeGraph() {return complete_graph;}

	// contains graph->edgeMap, this is a  std::map<int, RndfEdge*>, as below.
	// this contains the complete static scence graph
	RoutePlanner::RndfGraph* complete_graph;

	// Current Stats of the Ego Vehicle with its matching to the graph
	Vehicle ego_vehicle;

	// Route.route contains the mission graph. mission graph has
	// an other edge type, namely AnnotatedRouteEdge. AnnotatedRouteEdges have a RndfEdge* pointer
	// (edge) that points into complete_graph. Hence, route is an overlay
	// graph on complete_graph.
	// Contains only the actual relevant part of the complete route
	RoutePlanner::Route route;

  // a copy of the initial route.route - this list holds the whole mission all the time.
  Route::RouteEdgeList complete_mission_graph;

	// Iterator to the current annotated edge of the planned route
	RoutePlanner::Route::RouteEdgeList::iterator current_edge_it;

	// Iterator to the current annotated edge of the planned route (compl. mission graph)
	RoutePlanner::Route::RouteEdgeList::iterator current_edge_it_on_complete_mission_graph;

	bool route_is_finished() const { return current_edge_it == route.route.end(); };

	// indexes into route (RndfEdges, not AnnotatedRouteEdges), this is provided
	// for convenience,
	// NOTA: there is an issue about edges passed more then once
	// TODO: is this issue current?
	std::map<int, RoutePlanner::RndfEdge*> mission_graph_map;


	//! Copy of the checkpointlist of the mission (used for replanning)
	deque< RndfVertex* > checkpoints;

	//! Iterator to the next checkpoint (used for replanning)
	deque< RndfVertex* >::iterator next_check_point_it;

	//! stores the best alternative edge ( used for replanning) ( set by isOffTrack() )
	RndfEdge* best_alternative_edge;



	VehicleManager* vehicle_manager;

	//! link to the intersection manager
	/*! \remark the pointer is only set if the planner is in an intersectionstate otherwise NULL */
	IntersectionManager* intersection_manager;

	  // the road blockage manager takes care of road blockages
	  BlockadeManager* blockade_manager;

	// dist calc funcs
	double distToNextIntersection();
  double distToNextSoleStopLine(CurvePoint& stop_point);
  double distToNextStopLine(); // for visualization only
	double distToNextLaneChange();
	double distToNextTrafficLight();
	double distToNextTrafficLight(std::vector<std::string>* tl_names, CurvePoint* tl_pos);
	double distToNextCrosswalk();
	double distToNextCrosswalk(std::vector<std::string>* cw_names, CurvePoint* cw_pos);
	double distToNextKTurn();
	double distToNextMovingVehicle();
	double distToNextStandingVehicle();
	double distToPreviousVehicle();
	double distToNextZone();
	double distToNextZoneExit();
	double distToNextParkingSpot();
	double distToMissionEnd();
	double distToIntersection(RndfIntersection* isec);
	double distToStopLine(RndfIntersection* isec);
	double distToStopLine(RndfIntersection* isec, CurvePoint* cp);
  double distToTrafficLight(RndfIntersection* isec);
  double distToTrafficLight(RndfIntersection* isec, std::vector<std::string>* tl_names, CurvePoint* tl_pos);
	double dist_to_veh_ahead(Vehicle* veh);
	double dist_to_veh_behind(Vehicle* veh);
	Vehicle* get_next_vehicle();

	RndfIntersection* get_next_intersection();

  int nextTurnDirection();


  //! scans the current route for blockades and returns true if some thin is in the way
  bool isRouteBlocked(double max_scan_dist = 50., bool stop_scan_at_checkpoint = true);

  //! returns an iterator on the next checkpoint in mission
 // const deque< RndfVertex* >::const_iterator nextCheckPointIt() const;
  deque< RndfVertex* >::iterator nextCheckPointIt();

  //! returns the best edge for replanning. Considers blockades. NULL if none found
  /*! \remark sets member best_alternative_edge and next_check_point_it
   */
  RndfEdge* getBestReplanningEdge();

  //! returns true if the ego vehicle is of the track and needs replanning
  bool isOffTrack();


  bool isMissionPlanned() const { return mission_planned; }

  // messaging methods
  int addMessage(const std::string& message);
  int addMessage(const boost::format& frm);
  std::deque<std::string>& getMessages();

  // should be private ...
  void handle_ego_vehicle_update();
  void extract_relevant_part_of_route();

	double distToNextManeuver(maneuver_t maneuver);
	double distToNextManeuverStart(maneuver_t maneuver);
	double distToNextArea(area_type_t area);

	//! returns the next edge of the given maneuver type
  RoutePlanner::AnnotatedRouteEdge* next_edge_with_maneuver(maneuver_t maneuver, double max_scan_dist = TRIGGER_DIST_MAX_LOOKAHEAD );

  inline const rndf::RoadNetwork& roadNetwork() const {return rn;}
  inline const rndf::Mission& mission() const {return m;}

private:


	rndf::RoadNetwork rn;
	rndf::Mission m;
	bool mission_planned;
	std::string name; //!< debug name for simulation where we have multiple topologies

  // message queue
  std::deque<std::string> message_buffer;

  bool recover_from_file;
  bool save_mission_progress_to_file;
  // count reached cp
  int reached_cp_count;

  deque<RndfVertex*>::iterator old_checkpoint_iter;
};

} // namespace vlr

#endif
