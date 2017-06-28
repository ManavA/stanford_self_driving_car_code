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


#include <GL/glut.h>
#include <boost/lexical_cast.hpp>

#include <gl_support.h>
#include <obstacle_types.h>
#include <global.h>

#include <driving_common/TurnSignal.h>
#include <aw_Topology.hpp>
#include <aw_VehicleManager.hpp>
#include <aw_match_to_graph.hpp>
#include <aw_RndfIntersection.h>

using boost::lexical_cast;
using namespace dgc;

namespace vlr {

const double EDGE_BEHIND_BLOCKADE_PENALTY       = 50.;   // [m]

const double BACK_VIEW_DISTANCE = 100;
const double MIN_PREVIEW_LEN = 100;

const double GOAL_AFTERBURNER = 0.5; // [m] the distance we go over the last checkpoint so that we will definetly pass it

using namespace RoutePlanner;
using namespace rndf;
using namespace std;

#define TRACE(x) std::cout << "["<<name<<"] " << x << std::endl

Topology::Topology(const std::string& rndf_filename, const std::string& mdf_filename, const std::string name, bool create_visualization,
    bool recover_from_file_, bool save_mission_progress_to_file) :
  ego_vehicle(),
  route(),
  current_edge_it(route.route.end()),
  vehicle_manager( 0 ),
  intersection_manager ( NULL ),
  rn(),
  m(&rn),
  mission_planned(false),
  name(name),
  recover_from_file( recover_from_file_ ),
  save_mission_progress_to_file( save_mission_progress_to_file ),
  reached_cp_count( 0 )
{
  TRACE(" ** Topology constructor **");
  if ( rn.loadRNDF( rndf_filename) ) {
    TRACE( "Road network loaded successfully");
  }
  else {
    dgc_die("%s: Error loading road network : %s", name.c_str(), rn.status().c_str());
  }

//  rn.dump();

    // speed limits are in the MDF -> we have to load the MDF here before we build the graph
  if( m.loadMDF( mdf_filename ) ) {
    TRACE("Mission file loaded successfully");
  } else {
    dgc_die("%s: Error loading mission file: %s\n",name.c_str(), m.status().c_str());
  }

  //m.dump();
  TRACE("Mission status: " << m.status());

  RndfGraphBuilder graphBuilder(&rn);

  complete_graph = graphBuilder.buildGraph(); // speed limits are in the MDF -> we have to load the MDF here

//  graphBuilder.addLaneChangeEdges( complete_graph );
}

void Topology::planMission(double initial_x, double initial_y, double initial_yaw) {
  mission_planned = false;

  //  bool firstPoint = true;
  CheckpointList* cpl = m.checkpointList();
  if(!cpl) {
      throw VLRException("Failed to plan mission: check point list is invalid (empty?)");
  }

  if (recover_from_file) {
    std::cout << "RECOVERING MISSION FROM FILE";
    // throw away checkpoints that had already been reached in a previous run (for correct recover)
    std::ifstream recover_file;

    recover_file.open("recover_file");

    if (recover_file.good()) {
      recover_file >> reached_cp_count;
    }
    std::cout << "...throw away " << reached_cp_count << " checkpoints from start of mission" << std::endl;
    for (int i = 0; i < reached_cp_count; i++) {
      cpl->checkpoints().pop_front();
      if (cpl->checkpoints().size() == 1) {
        reached_cp_count--;
        break; // keep at least the last one!
      }
    }
  }

  assert(cpl->size()>0);
  if(cpl->empty()) {
      throw VLRException("Failed to plan mission: check point list is empty");
  }

  // -------------------------------------------------
  double d, x_matched, y_matched, offset_matched;
  TRACE("Matching egovehicle (" << initial_x << ", "<< initial_y << ", " << initial_yaw << ")");
  int sign;
  RndfEdge* startEdge = match_2_graph( initial_x, initial_y, initial_yaw, complete_graph->edgeMap, d, x_matched, y_matched, offset_matched, sign );
  TRACE("egovehicle initially matched on " << startEdge->name() << " distance: "<<d<<" offset:" <<offset_matched<<" x: "<<x_matched<<" y: "<<y_matched);



  /*
   * Find a valid mission start:
   * - delete the first checkpoint as long as we found a way between the first and second checkpoint
   * - search the initially matched edge in the first MAX_INITIAL_SEARCH_DISTANCE meter of this way
   * - if it is found no new mission start is added
   * - if it is not found:
   *   - starting with 0 meter fetch all surrounding edges of the initially matched one
   *   - from every surrounding edge search a path to the first checkpoint, take the best (lowest travel time)
   *   - if there is no at all, increase search distance for surrounding and start way search again
   */
  std::list < RndfVertex* > missionPoints;

  { // scope for auto_ptr
    bool found_valid_mission_start = false;
    while (!found_valid_mission_start) {
			//std::cout << "cpl" << cpl << std::endl;
			//std::cout << "(*cpl->begin())" << (*cpl->begin()) << std::endl;
			//std::cout << "(*cpl->begin())->name()" << (*cpl->begin())->name() << std::endl;
			if ((*cpl->begin()) == 0) {
				std::cerr << "ERROR: first checkpoint is nullpointer -> skip it" << std::endl;
	      cpl->checkpoints().pop_front();
				if (cpl->size() == 0) {
					dgc_die("All Checkpoints are invalid, cannot plan a mission\n");
				}
				assert(false); // TODO: remove for race
        continue; // while (!found_valid_mission_start)
			}
			TRACE("butter bei die fische " << cpl->size());
      RndfVertex* v1 = complete_graph->checkPoint(lexical_cast<int>((*cpl->begin())->name()));
      if (cpl->size()>1) {
        TRACE("searching initially matched edge in first mission part");
        // if startEdge is not inbetween cpl[0] - cpl[1] (including checkpoint 0, exclude checkpoint 1)
        // the mission is prepended with startEdge->fromVertex()
        RndfVertex* v2 = complete_graph->checkPoint(lexical_cast<int>((*(cpl->begin()+1))->name()));
        auto_ptr<RndfGraph::EdgeList> edgeList = auto_ptr<RndfGraph::EdgeList>(complete_graph->searchPath(v1, v2, true));
        if (&*edgeList == NULL) {
          TRACE("Could not plan a route from first to second checkpoint: "<<v1->name() << " to " << v2->name());
          cpl->checkpoints().pop_front();
          assert(false); // TODO: remove for race
          continue; // while (!found_valid_mission_start)
        }
        found_valid_mission_start =  true;
        bool foundEdge = false;
        double d = 0;
        for (RndfGraph::EdgeList::const_iterator iter = edgeList->begin(); iter != edgeList->end(); ++iter) {
          if (d > MAX_INITIAL_SEARCH_DISTANCE) break;
          if ((*iter) == startEdge) {
            foundEdge = true;
            break;
          }
          d += (*iter)->length();
        }
        edgeList.reset(0);
        if (!foundEdge) {
          TRACE("Initial matched position is too far from mission start -> add matched position as new starting checkpoint");
          // search the best starting edge in the surrounding of startEdge
          // increasing the surrounding as long as a path was found
          RndfVertex* bestVertex = 0;
          double search_distance = -1.0;
          map<RndfVertex*, bool> visited_vertexes;

          while (bestVertex == 0) { // if search_distance is big enough we will eventually include a direct edge of v1 so this while loop always has an exit condition

            TRACE("search possible start edges " << search_distance << "m from initial matched edge" );
            std::set<RndfEdge*> possibleStarts;
            if (search_distance < 0) {
              possibleStarts.insert(startEdge);
            } else {
              possibleStarts = GraphTools::getSurroundingEdges(startEdge, complete_graph, search_distance);
            }
            double bestCost = std::numeric_limits<double>::infinity();

            for (std::set<RndfEdge*>::const_iterator iter = possibleStarts.begin(); iter != possibleStarts.end(); ++iter) {
              //cout << **iter << endl;

              auto_ptr<RndfGraph::EdgeList> edges;

              if (!visited_vertexes[(*iter)->fromVertex()]) {
                visited_vertexes[(*iter)->fromVertex()] = true;
                edges.reset(complete_graph->searchPath((*iter)->fromVertex(), v1, true));
                if (&*edges != 0) { // found a way
                  double cost = 0;
                  if (edges->front() == *iter) {
                    for (RndfGraph::EdgeList::const_iterator iter2 = edges->begin(); iter2 != edges->end(); ++iter2) {
                      cost += (*iter2)->travelTime();
                    }
                    TRACE("found a way from " << (*iter)->fromVertex()->name() << " to " << v1->name()<<" costs = " << cost << ", bestCost = " << bestCost);
                    if (cost < bestCost) {
                      bestCost = cost;
                      bestVertex = (*iter)->fromVertex();
                      TRACE("set as best vertex");
                    }
                  } else {
                    TRACE("found a way from " << (*iter)->fromVertex()->name() << " to " << v1->name()<<" but it does not goes throu " << (*iter)->name());
                  }
                } else {
                  TRACE("did not found a way from " << (*iter)->fromVertex()->name() << " to " << v1->name());
                }
              }


              if (!visited_vertexes[(*iter)->toVertex()]) {
                visited_vertexes[(*iter)->toVertex()] = true;
                edges.reset(complete_graph->searchPath((*iter)->toVertex(), v1));
                if (&*edges != 0) { // found a way
                  double cost = 0;
                  for (RndfGraph::EdgeList::const_iterator iter2 = edges->begin(); iter2 != edges->end(); ++iter2) {
                    cost += (*iter2)->travelTime();
                  }
                  cost += (*iter)->length(); // we want to priorise fromVertexes
                  TRACE("found a way from " << (*iter)->toVertex()->name() << " to " << v1->name()<<" costs = " << cost << ", bestCost = " << bestCost);
                  if (cost < bestCost) {
                    bestCost = cost;
                    bestVertex = (*iter)->toVertex();
                    TRACE("set as best vertex");
                  }
                } else {
                  TRACE("did not found a way from " << (*iter)->toVertex()->name() << " to " << v1->name());
                }
              }
            } // for (...)
            search_distance += 5.0;
          } // while (bestVertex == 0)
          TRACE("Found a possible start position " << (search_distance-5.0) << "m from the initial matched edge");
          missionPoints.push_back(bestVertex);
        } else {
          TRACE("Initially matched position found, no need to add new starting checkpoint");
        }
      } else {
        TRACE("MISSION CONTAINS ONLY ONE CHECKPOINT");
        found_valid_mission_start = true;
        // if mission is only one checkpoint and we are not on the goal edge: plan to the goal
        if (startEdge->fromVertex() != v1 && startEdge->toVertex() != v1) {
          TRACE( "add matched position as new starting checkpoint");
          missionPoints.push_back(startEdge->fromVertex());
        }
      }

    } // while (!found_valid_mission_start)
  }
  // ----------------------------------------


  TRACE("Checkpoints ("<< cpl->size() <<"):");
  int i=0;
  for (TCheckpointVec::const_iterator it=cpl->begin(); it != cpl->end(); ++it, ++i) {
    cout <<  i <<". Checkpoint: "; (*it)->dump(); cout << endl;
    missionPoints.push_back(complete_graph->checkPoint(lexical_cast<int>((*it)->name())));
  }

  if (missionPoints.size() > 1) {
    std::list < RndfVertex* >::const_iterator it1, it2;
    it1 = missionPoints.begin();
    it2 = missionPoints.begin();
    ++it2;
    for ( ; it2 != missionPoints.end(); ++it1, ++it2) {
      int start = (*it1)->checkpointId();
      int goal = (*it2)->checkpointId();
//      cout << "planning " << start << "->" << goal << "... " << flush;
      auto_ptr<RndfGraph::EdgeList> edgeList = auto_ptr<RndfGraph::EdgeList>(complete_graph->searchPath(*it1, *it2, true));

      if (&*edgeList == NULL) {
        dgc_die("Could not plan a route from checkpoint %d --> %d\n",start,goal);
      }

      for (RndfGraph::EdgeList::const_iterator it = edgeList->begin(); it != edgeList->end(); ++it) {
        const RndfEdge * edge = *it;
        cout << edge->name() << " ";
      }
      cout << endl;
      route.addEdges( &*edgeList );
    }
  } else {
    // mission of only one checkpoint... -> we are directly at the goal
    route.addEdge(startEdge);
  }
  route.annotateRoute();

  complete_mission_graph = route.route; // copy complete route for later use. route.route will be modified as the vehicle makes progess in the mission
  std::cout << "complete mission graph map size == " << complete_mission_graph.size() << std::endl;

  extract_relevant_part_of_route();
  TRACE("initial mission graph map size == " << mission_graph_map.size());

  // update ego pose
  updateEgoVehicle(initial_x, initial_y, initial_yaw, 0);

  // set checkpoint list
  checkpoints.clear();
  const rndf::TCheckpointVec& rndf_cps = m.checkpointList()->checkpoints();
  for (rndf::TCheckpointVec::const_iterator it = rndf_cps.begin(); it != rndf_cps.end(); ++it) {
	RndfVertex* vertex = complete_graph->findVertex( (*it)->wayPoint()->name() );
	assert(vertex);
	checkpoints.push_back( vertex );
  }

  mission_planned = true;

  old_checkpoint_iter = nextCheckPointIt();
  //route.dump();
//  for (RndfGraph::EdgeMap::const_iterator iter = complete_graph->edgeMap.begin();
//  	iter != complete_graph->edgeMap.end(); ++iter) {
//  	if (iter->second->isZoneEdge()) {
//  		std::cout << *(iter->second) << " length="<<iter->second->length() << std::endl;
//  	}
//  }
  //::exit(1);
}

void Topology::updateEgoVehicle( const double x, const double y, const double yaw, const double speed )
{
  ego_vehicle.update( x, y, yaw, speed );
  vehicle_manager->robot.update( x, y, yaw, speed ); // TODO: we don't need our pose so many times
  handle_ego_vehicle_update();
}

// get next obstacle on mission graph. route.route.end() if none found.
Vehicle* Topology::nextObstacle( double& distance, GraphTools::PlaceOnGraph& place, double max_scan_distance  )
{
  if(!mission_planned) return NULL;

  distance = std::numeric_limits<double>::infinity();

  GraphTools::PlaceOnGraph ego_place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  //  assert( vehicle_manager );
  assert(*ego_place.edge_);

  return ego_place.next_obstacle( distance, place, max_scan_distance );
}

void Topology::handle_ego_vehicle_update()
{
  //std::cout << "handling ego vehicle...\n" << std::flush;
  ego_vehicle.match_to_graph( mission_graph_map, true );
//  TRACE("Rematch EgoVehicle: "<< ego_vehicle.edge->name());
  //std::cout << "mission_graph_map.size() == " << mission_graph_map.size() ;

  //  std::list<AnnotatedRouteEdge>::iterator edge_before_edge_before = current_edge_it;


  //std::cout << "route length " << route.route.size() << std::endl;

  double passed_distance = 0;
  int count = 0;


  // find current edge iterator and delete edges allready travelled (delete only in the overlay).
  // Keeps edge before current edge, and edge before that. (jz)

  for( current_edge_it = route.route.begin(); current_edge_it != route.route.end(); current_edge_it++ )
  {
    //      std::cout << "scanning..." << (*current_edge_it)->startPosition << "m " << std::endl;

    if( (*current_edge_it)->edge() == ego_vehicle.edge() )
    {

      passed_distance += ego_vehicle.distFromStart();
      break;
    }

    //      printf( "will erase edge @%x\n", (*current_edge_it)->edge() );
    // edge_before_edge_before = route.route.erase( edge_before_edge_before );
    passed_distance += (*current_edge_it)->edge()->length();
    count++;
  }
  if( mission_planned && !(checkpoints.size() == 0) )
    {
      if( !(*current_edge_it)->was_reached )
	{
	  (*current_edge_it)->was_reached = true;
	  if( save_mission_progress_to_file )
	    {
	      std::cout << "last cp " << (*nextCheckPointIt())->checkpointId() << " next cp " << (*old_checkpoint_iter)->checkpointId() << "\n";
	      bool count_it = false;
	      int id;
	      if( (*nextCheckPointIt())->checkpointId() != (*old_checkpoint_iter)->checkpointId() )
		{
		  count_it = true;
		  id = (*old_checkpoint_iter)->checkpointId();
		  old_checkpoint_iter = nextCheckPointIt();
		}
	      /*


	      if( (*current_edge_it)->edge()->isZoneEdge() )
	      {
	      if( (*current_edge_it)->isToCheckpointEdge() &&
	      (*current_edge_it)->edge()->toVertex()->isCheckpointVertex() &&
	      (*current_edge_it)->edge()->toVertex()->checkpointId() != -1 )
	      {
	      id = (*current_edge_it)->edge()->toVertex()->checkpointId();
	      count_it = true;
	      }
	      }
	      else
	      {
	      if( (*current_edge_it)->isFromCheckpointEdge() &&
	      (*current_edge_it)->edge()->fromVertex()->isCheckpointVertex() &&
	      (*current_edge_it)->edge()->fromVertex()->checkpointId() != -1 )
	      {
	      id = (*current_edge_it)->edge()->fromVertex()->checkpointId();
	      count_it = true;
	      }

	      }
	      */

	      if( count_it )
		{
		  reached_cp_count++;
		  std::ofstream recover_file( "recover_file" );
		  recover_file << reached_cp_count;
		  std::cout << "REACHED A CHECKPOINT (#" << reached_cp_count << ", Id " << id << "), WRITING THIS TO A FILE SINCE RECOVER TO FILE WAS REQUESTED.\n";
		}
	    }
	}
    }

  current_edge_it_on_complete_mission_graph = std::find( complete_mission_graph.begin(), complete_mission_graph.end(), *current_edge_it );

  RoutePlanner::Route::RouteEdgeList::iterator rit = current_edge_it;

  while( ( count > 2 ) && ( passed_distance > BACK_VIEW_DISTANCE ) )
  {
    //      std::cout << "count " << count << " " << passed_distance << std::endl;
    passed_distance -= route.route.front()->edge()->length();
    route.route.pop_front();
    count--;
  }

  // (jz) me must update mission edge map. At this point, a map seems
  // not like the most efficient data structure for what it is acutally used for
  // (dont know where indexing is used at all), but what the hell...
  extract_relevant_part_of_route();

  /*std::cout << "blub distToNextParkingSpot = " << distToNextParkingSpot()
    << " distToNextZone = " << distToNextZone()
    << " distToNextZoneExit = " << distToNextZoneExit()
    << std::endl;*/
}


void Topology::extract_relevant_part_of_route()
{
  // (jz) me must update mission edge map. At this point, a map seems
  // not like the most efficient data structure for what it is actually used for
  // (dont know where indexing is used at all), but what the hell...
  // translate route.route (std::list<RndfEdge*>) to std::map<int, RndfEdge*>, as required by distance transform
  unsigned int idx = 0;
  unsigned int c = 0;
  bool inFront = false;

  mission_graph_map.clear();

  debug_distances.clear();

  //bool hit_older_part_of_mission = false;

  double preview_len = 0;

  for( Route::RouteEdgeList::const_iterator it = route.route.begin(); it != route.route.end(); it++ )
  {
    if (it == current_edge_it) {
      inFront = true;
    }
    if (inFront) { // we need at least some edges in front of us
      ++c;
    }
    preview_len += (*it)->edge()->length();

    if( c>2 && ((*it)->edge()->visited() || ( preview_len > ( MIN_PREVIEW_LEN + BACK_VIEW_DISTANCE  ) )) )
    {
      break;
      //hit_older_part_of_mission = true;
    }
    mission_graph_map[idx] = (*it)->edge();
    (*it)->edge()->setVisited(true);
    idx++;
  }

  if ( mission_graph_map.empty() ) return;

  // reset visited predicate for next run
  for( Route::RouteEdgeList::const_iterator it = route.route.begin(); it != route.route.end(); it++ )
  {
    (*it)->edge()->setVisited(false);
  }
}

double Topology::distToNextManeuver(maneuver_t maneuver)
{
  if(!mission_planned) return std::numeric_limits<double>::infinity();

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  double d;
  RoutePlanner::AnnotatedRouteEdge* edge = place.next_manuever( d, place, TRIGGER_DIST_MAX_LOOKAHEAD, maneuver );
  if (edge) {
    return d - FRONT_BUMPER_DELTA;
  }
  return std::numeric_limits<double>::infinity();
}

double Topology::distToNextManeuverStart(maneuver_t maneuver)
{
  if(!mission_planned) return std::numeric_limits<double>::infinity();

  if ( (*current_edge_it)->hasAnnotation( maneuver ) )
	  return 0.;

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  double d;
  RoutePlanner::AnnotatedRouteEdge* edge = place.next_manuever( d, place, TRIGGER_DIST_MAX_LOOKAHEAD, maneuver );
  if (edge) {
    return d - FRONT_BUMPER_DELTA - edge->edge()->length();
  }
  return std::numeric_limits<double>::infinity();
}


double Topology::distToNextArea(area_type_t area)
{
  double d;

  if(!mission_planned) return std::numeric_limits<double>::infinity();

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  RoutePlanner::AnnotatedRouteEdge* edge = place.next_area( d, place, TRIGGER_DIST_MAX_LOOKAHEAD, area );
  if (edge) {
    return d - FRONT_BUMPER_DELTA;
  }
  return std::numeric_limits<double>::infinity();
}

RoutePlanner::AnnotatedRouteEdge* Topology::next_edge_with_maneuver(maneuver_t maneuver, double max_scan_dist)
{
  if(!mission_planned) return NULL;

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  double d;
  return  place.next_manuever( d, place, max_scan_dist, maneuver );
}


RndfIntersection* Topology::get_next_intersection()
{
  if(!mission_planned) return NULL;

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  double dist;
  vector<maneuver_t> maneuvers(3);
  maneuvers.push_back(UC_MANEUVER_INT_TURN_RIGHT);
  maneuvers.push_back(UC_MANEUVER_INT_TURN_LEFT);
  maneuvers.push_back(UC_MANEUVER_INT_STRAIGHT);

  RoutePlanner::AnnotatedRouteEdge* redge = place.next_manuever( dist, place, TRIGGER_DIST_MAX_LOOKAHEAD, maneuvers );

  if (redge && redge->edge()) return redge->edge()->intersection();
  return NULL;
}

double Topology::distToNextIntersection()
{
  double dist1 = distToNextManeuver(UC_MANEUVER_INT_TURN_RIGHT);
  double dist2 = distToNextManeuver(UC_MANEUVER_INT_TURN_LEFT);
  double dist3 = distToNextManeuver(UC_MANEUVER_INT_STRAIGHT);
  dist1 = min(dist1, dist2);
  dist1 = min(dist1, dist3);
  return dist1;
}

double Topology::distToNextLaneChange()
{
    if(!mission_planned) return numeric_limits<double>::infinity();

    double dist_1 = distToNextManeuverStart(UC_MANEUVER_LANECHANGE_LEFT);
    double dist_2 = distToNextManeuverStart(UC_MANEUVER_LANECHANGE_RIGHT);

    return min(dist_1, dist_2);
}

double Topology::distToNextCrosswalk() {
    return distToNextCrosswalk(NULL, NULL);
}

double Topology::distToNextCrosswalk(std::vector<std::string>* cw_names, CurvePoint* cw_pos)
{
  if(!mission_planned) return std::numeric_limits<double>::infinity();

  // check lookahead range for traffic lights
  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  double fwd_dist;
  RoutePlanner::AnnotatedRouteEdge* edge = place.go_fwd_to_crosswalk(fwd_dist, TRIGGER_DIST_MAX_LOOKAHEAD);
  if(!edge) {return std::numeric_limits<double>::infinity();}

    // if vertex is an exit, the crosswalk belongs to an intersection and
    // is handled there
    // TODO: Also check entries (though this should also be handled in intersections?!?)
  if(edge->edge()->toVertex()->isExitVertex()) {
    return std::numeric_limits<double>::infinity();
  }

  assert(fwd_dist >= -0.01);
  fwd_dist -= FRONT_BUMPER_DELTA;
  if (fwd_dist < 0.) {fwd_dist = 0.;}

  // TODO: check backward distance as well
//  RoutePlanner::AnnotatedRouteEdge* edge = place.go_bwd_to_crosswalk_light(bwd_dist, TRIGGER_DIST_MAX_LOOKAHEAD, isec );


    if(cw_names) {
      const std::set<std::string>& cw_name_set = edge->edge()->toVertex()->crosswalkNames();
      std::set<std::string>::const_iterator cwit=cw_name_set.begin(), cwit_end=cw_name_set.end();
      for(;cwit != cwit_end; cwit++) {
        cw_names->push_back((*cwit));
      }
    }
    if(cw_pos) {
      cw_pos->x = edge->edge()->toVertex()->x();
      cw_pos->y = edge->edge()->toVertex()->y();
     }
  //  return (fwd_dist <= bwd_dist ? fwd_dist : - bwd_dist);
  return fwd_dist;
//  if(!mission_planned) return std::numeric_limits<double>::infinity();
//
//  // check lookahead range for crosswalks
//  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
//  assert(*place.edge_);
//  double d;
//  RoutePlanner::AnnotatedRouteEdge* edge = place.next_manuever( d, place, TRIGGER_DIST_MAX_LOOKAHEAD, UC_MANEUVER_CROSSWALK );
//  if(!edge) {return std::numeric_limits<double>::infinity();}
//
//  if(cw_names) {
//    const std::set<std::string>& cw_name_set = edge->edge()->toVertex()->crosswalkNames();
//    std::set<std::string>::const_iterator cwit=cw_name_set.begin(), cwit_end=cw_name_set.end();
//    for(;cwit != cwit_end; cwit++) {
//      cw_names->push_back((*cwit));
//    }
//  }
//  if(cw_pos) {
//    cw_pos->x = edge->edge()->toVertex()->x();
//    cw_pos->y = edge->edge()->toVertex()->y();
//   }
//
//  return d - FRONT_BUMPER_DELTA - edge->edge()->length();
}

double Topology::distToNextTrafficLight() {
    return distToNextTrafficLight(NULL, NULL);
}

double Topology::distToNextTrafficLight(std::vector<std::string>* tl_names, CurvePoint* tl_pos)
{
  if(!mission_planned) return std::numeric_limits<double>::infinity();

  // check lookahead range for traffic lights
  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  double fwd_dist;
  RoutePlanner::AnnotatedRouteEdge* edge = place.go_fwd_to_traffic_light(fwd_dist, TRIGGER_DIST_MAX_LOOKAHEAD);
  if(!edge) {return std::numeric_limits<double>::infinity();}

  assert(fwd_dist >= -0.01);
  fwd_dist -= FRONT_BUMPER_DELTA;
  if (fwd_dist < 0.) fwd_dist = 0.;

  // TODO: check backward distance as well
//  RoutePlanner::AnnotatedRouteEdge* edge = place.go_bwd_to_traffic_light(bwd_dist, TRIGGER_DIST_MAX_LOOKAHEAD, isec );


    if(tl_names) {
      const std::set<std::string>& cw_name_set = edge->edge()->toVertex()->trafficLightNames();
      std::set<std::string>::const_iterator cwit=cw_name_set.begin(), cwit_end=cw_name_set.end();
      for(;cwit != cwit_end; cwit++) {
        tl_names->push_back((*cwit));
      }
    }
    if(tl_pos) {
      tl_pos->x = edge->edge()->toVertex()->x();
      tl_pos->y = edge->edge()->toVertex()->y();
     }
  //  return (fwd_dist <= bwd_dist ? fwd_dist : - bwd_dist);
  return fwd_dist;
}

// used for visualization only
double Topology::distToNextStopLine()
{
  double d;

  if(!mission_planned) return std::numeric_limits<double>::infinity();

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  GraphTools::PlaceOnGraph next_place;
  assert(*place.edge_);
  RoutePlanner::AnnotatedRouteEdge*  maneuver = place.next_manuever( d, next_place, TRIGGER_DIST_MAX_LOOKAHEAD, UC_MANEUVER_STOP_SIGN );
  double d_sum = 0;
  while (maneuver != NULL && !maneuver->edge()->toVertex()->hasIntersectionOutEdge()) {
    d_sum += d;
    place = next_place;
    maneuver = place.next_manuever( d, next_place, TRIGGER_DIST_MAX_LOOKAHEAD - d_sum, UC_MANEUVER_STOP_SIGN );
  }
  if (maneuver) {
    return d + d_sum - FRONT_BUMPER_DELTA;
  }
  return std::numeric_limits<double>::infinity();
}

double Topology::distToNextSoleStopLine(CurvePoint& stop_point)
{
  double d;

  if(!mission_planned) return std::numeric_limits<double>::infinity();

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  GraphTools::PlaceOnGraph next_place;
  assert(*place.edge_);
  RoutePlanner::AnnotatedRouteEdge*  maneuver = place.next_manuever( d, next_place, TRIGGER_DIST_MAX_LOOKAHEAD, UC_MANEUVER_STOP_SIGN );
  double d_sum = 0;
  while (maneuver != NULL && maneuver->edge()->toVertex()->hasIntersectionOutEdge()) {
    d_sum += d;
    place = next_place;
    maneuver = place.next_manuever( d, next_place, TRIGGER_DIST_MAX_LOOKAHEAD - d_sum, UC_MANEUVER_STOP_SIGN );
  }

  if(maneuver) {
    stop_point.x = (*next_place.edge_)->edge()->toVertex()->x();
    stop_point.y = (*next_place.edge_)->edge()->toVertex()->y();
    return d + d_sum - FRONT_BUMPER_DELTA;
  }
  return std::numeric_limits<double>::infinity();
}

double Topology::distToNextKTurn()
{
  return distToNextManeuver(UC_MANEUVER_U_TURN);
}

double Topology::distToNextZone()
{
  return std::max(0.0,std::min(distToNextArea(UC_ZONE), distToNextArea(UC_PARKING_ZONE)));
}
double Topology::distToNextZoneExit()
{
  double d;

  if(!mission_planned) return std::numeric_limits<double>::infinity();

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  RoutePlanner::AnnotatedRouteEdge* edge = place.next_zone_exit( d, place, TRIGGER_DIST_MAX_LOOKAHEAD );
  if (edge) {
    return d - FRONT_BUMPER_DELTA;
  }
  return std::numeric_limits<double>::infinity();
}
double Topology::distToNextParkingSpot()
{
  return distToNextManeuver(UC_MANEUVER_PARKING);
}

double Topology::distToNextMovingVehicle()
{
  double d;

  if(!mission_planned) return std::numeric_limits<double>::infinity();
  //if (route_is_finished())  return GOAL_AFTERBURNER;

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  GraphTools::PlaceOnGraph next_place;
  assert(*place.edge_);
  Vehicle* vehicle = place.next_obstacle( d, next_place, TRIGGER_DIST_MAX_LOOKAHEAD );
  double d_sum = 0;

  while (vehicle != NULL && std::abs(vehicle->speed()) <= OBSTACLE_SPEED_THRESHOLD) {
    //GraphTools::Point p = place.make_point();
    //debug_distances.push_back(DistanceVisualisation(p.x,p.y, d));
    d_sum += d;
    place = next_place;
    vehicle = place.next_obstacle( d, next_place, TRIGGER_DIST_MAX_LOOKAHEAD - d_sum );

  }
  if (vehicle) {
//    if(vehicle) {printf("%s: vehicle %i: d: %f, speed: %f\n", __FUNCTION__, vehicle->id(), d+d_sum - FRONT_BUMPER_DELTA - vehicle->length() / 2.0, vehicle->speed());}
    //double vl = sqrt(vehicle->length*vehicle->length + vehicle->width*vehicle->width); // just to be sure
    /*while (vehicle != 0 ) {
      GraphTools::Point p = place.make_point();
      debug_distances.push_back(DistanceVisualisation(p.x,p.y, d));
      place = next_place;
      vehicle = place.next_obstacle( d, next_place, TRIGGER_DIST_MAX_LOOKAHEAD );
    }*/
    // distance is measured from our rear axle to the mateched location of the vehcile (it's center point)
  	// to get the distance from car to car we have to subtract FRONT_BUMPER_DELTA and the half of vehcile length
    return d + d_sum - FRONT_BUMPER_DELTA - vehicle->length() / 2.0;
  }
  return std::numeric_limits<double>::infinity();
}

double Topology::distToNextStandingVehicle()
{
  double d;

  if(!mission_planned) return std::numeric_limits<double>::infinity();
  //if (route_is_finished())  return GOAL_AFTERBURNER;

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  Vehicle* vehicle = place.next_obstacle( d, place, TRIGGER_DIST_MAX_LOOKAHEAD );
  double d_sum = 0;
  while (vehicle != NULL && std::abs(vehicle->speed()) > OBSTACLE_SPEED_THRESHOLD) {
    d_sum += d;
    vehicle = place.next_obstacle( d, place, TRIGGER_DIST_MAX_LOOKAHEAD - d_sum );
//    if(vehicle) {printf("%s: vehicle %i: d: %f, speed: %f\n", __FUNCTION__, vehicle->id(), d+d_sum - FRONT_BUMPER_DELTA - vehicle->length() / 2.0, vehicle->speed());}
  }
//  if(vehicle) {printf("%s: vehicle %i: d: %f, speed: %f\n", __FUNCTION__, vehicle->id(), d+d_sum - FRONT_BUMPER_DELTA - vehicle->length() / 2.0, vehicle->speed());}
  if (vehicle) {
  	// distance is measured from our rear axle to the matched location of the vehicle (its center point)
  	// to get the distance from car to car we have to subtract FRONT_BUMPER_DELTA and the half of vehicle length
    return d + d_sum - FRONT_BUMPER_DELTA - vehicle->length() / 2.0;
  }
  return std::numeric_limits<double>::infinity();
}

double Topology::distToPreviousVehicle()
{
  double d;

  if(!mission_planned) return std::numeric_limits<double>::infinity();

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  Vehicle* vehicle = place.prev_obstacle( d, place, TRIGGER_DIST_MAX_LOOKAHEAD );
  if (vehicle) {
    return d - vehicle->length()/2.0;
  }
  return std::numeric_limits<double>::infinity();
}

double Topology::distToMissionEnd() {
  RoutePlanner::Route::RouteEdgeList::iterator curr_edge_it = this->current_edge_it;

  // Ende der Mission Ueberpruefen
  if (curr_edge_it == route.route.end())  return GOAL_AFTERBURNER - FRONT_BUMPER_DELTA;
  RndfEdge* edge = (*curr_edge_it)->edge();
  assert(edge);
  if (edge == NULL) return GOAL_AFTERBURNER - FRONT_BUMPER_DELTA;
  RndfEdge* start_edge = edge;

  double dist = ego_vehicle.distToEnd();
  while (dist < TRIGGER_DIST_MAX_LOOKAHEAD) {
    // Eine Kante weiter gehen
    if (edge != start_edge) dist += edge->length();
    ++curr_edge_it;
    if (curr_edge_it == route.route.end()) return dist+GOAL_AFTERBURNER - FRONT_BUMPER_DELTA;
    edge = (*curr_edge_it)->edge();
    assert(edge);
    if (edge == NULL) return dist;
  }

  return numeric_limits<double>::infinity();
}


double Topology::distToIntersection(RndfIntersection* isec)
{
  if(!mission_planned) return std::numeric_limits<double>::infinity();

  // Distanz voraus berechnen
//  std::cout << "Vehicle "<< id << ": Calc Fwd Dist to Intersection.. " << std::flush;
  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  double fwd_dist;
  place.go_fwd_to_intersection( fwd_dist, TRIGGER_DIST_MAX_LOOKAHEAD, isec );
  assert(fwd_dist >= -0.01);
  fwd_dist -= FRONT_BUMPER_DELTA;
  if (fwd_dist < 0.) fwd_dist = 0.;
//  std::cout << fwd_dist << std::endl;

  // Distanz rückwärts berechnen
//  std::cout << "Vehicle "<< id << ": Calc Bwd Dist to Intersection.. " << std::flush;
//  double bwd_dist = bwdDistToIntersection(intersection, *edge, -distToEnd());
//  assert(bwd_dist >= -0.01);
//  bwd_dist -= length/2.;
//  if (bwd_dist < 0.) bwd_dist = 0;
//  std::cout << bwd_dist << std::endl;

  // Vorzeichen richtig setzen
//  return (fwd_dist <= bwd_dist ? fwd_dist : - bwd_dist);
  return fwd_dist;
}

double Topology::distToStopLine(RndfIntersection* isec) {
  return distToStopLine(isec, NULL);
}

double Topology::distToStopLine(RndfIntersection* isec, CurvePoint* cp)
{
  if(!mission_planned) return std::numeric_limits<double>::infinity();

  // Distanz voraus berechnen
//  std::cout << "Vehicle "<< id << ": Calc Fwd Dist to Intersection.. " << std::flush;
  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  double fwd_dist;
  RoutePlanner::AnnotatedRouteEdge* edge = place.go_fwd_to_stopline(fwd_dist, TRIGGER_DIST_MAX_LOOKAHEAD, isec );
  assert(fwd_dist >= -0.01);
  fwd_dist -= FRONT_BUMPER_DELTA;
  if (fwd_dist < 0.) fwd_dist = 0.;

  // Distanz rückwärts berechnen
//  std::cout << "Vehicle "<< id << ": Calc Bwd Dist to Intersection.. " << std::flush;
//  double bwd_dist = bwdDistToIntersection(intersection, *edge, -distToEnd());
//  assert(bwd_dist >= -0.01);
//  bwd_dist -= length/2.;
//  if (bwd_dist < 0.) bwd_dist = 0;
//  std::cout << bwd_dist << std::endl;

  // Vorzeichen richtig setzen
//  return (fwd_dist <= bwd_dist ? fwd_dist : - bwd_dist);

  if(cp && edge) {
    cp->x = edge->edge()->toVertex()->x();
    cp->y = edge->edge()->toVertex()->y();
  }
  return fwd_dist;
}

double Topology::distToTrafficLight(RndfIntersection* isec) {
  return distToTrafficLight(isec, NULL, NULL);
}

double Topology::distToTrafficLight(RndfIntersection* isec, std::vector<std::string>* tl_names, CurvePoint* tl_pos)
{
  if(!mission_planned) return std::numeric_limits<double>::infinity();

  // check lookahead range for traffic lights
  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  double fwd_dist;
  RoutePlanner::AnnotatedRouteEdge* edge = place.go_fwd_to_traffic_light(fwd_dist, TRIGGER_DIST_MAX_LOOKAHEAD, isec );
  if(!edge) {return std::numeric_limits<double>::infinity();}

  assert(fwd_dist >= -0.01);
  fwd_dist -= FRONT_BUMPER_DELTA;
  if (fwd_dist < 0.) fwd_dist = 0.;

  // TODO: check backward distance as well
//  RoutePlanner::AnnotatedRouteEdge* edge = place.go_bwd_to_traffic_light(bwd_dist, TRIGGER_DIST_MAX_LOOKAHEAD, isec );

  if(tl_names) {
    const std::set<std::string>& tl_name_map = edge->edge()->toVertex()->trafficLightNames();
    std::set<std::string>::const_iterator tlit=tl_name_map.begin(), tlit_end=tl_name_map.end();
    for(;tlit != tlit_end; tlit++) {
      tl_names->push_back((*tlit));
    }
  }
  if(tl_pos && edge) {
    tl_pos->x = edge->edge()->toVertex()->x();
    tl_pos->y = edge->edge()->toVertex()->y();
   }
  //  return (fwd_dist <= bwd_dist ? fwd_dist : - bwd_dist);
  return fwd_dist;
}


int Topology::nextTurnDirection()
{
  RoutePlanner::Route::RouteEdgeList::iterator curr_edge_it = this->current_edge_it;

  // Ende der Mission Ueberpruefen
  if (curr_edge_it == route.route.end())  return 0;
  RndfEdge* edge = (*curr_edge_it)->edge();
  assert(edge);
  if (edge == NULL) return 0;
  RndfEdge* start_edge = edge;

  double dist = ego_vehicle.distToEnd() - FRONT_BUMPER_DELTA;
  while (dist < TRIGGER_DIST_MAX_LOOKAHEAD) {
  	// Check Annotations
    AnnotatedRouteEdge::AnnotationList annos = (*curr_edge_it)->annotations();
    for (AnnotatedRouteEdge::AnnotationList::iterator it=annos.begin(); it != annos.end(); ++it) {
      switch ((*it)->maneuver()) {
      case UC_MANEUVER_INT_TURN_RIGHT : return driving_common::TurnSignal::RIGHT;
      case UC_MANEUVER_INT_TURN_LEFT  : return driving_common::TurnSignal::LEFT;
      case UC_MANEUVER_INT_STRAIGHT : return driving_common::TurnSignal::NONE;
      default: break;
      }
    }
    // Eine Kante weiter gehen
    if (edge != start_edge) dist += edge->length();
    ++curr_edge_it;
    if (curr_edge_it == route.route.end()) return 0;
    edge = (*curr_edge_it)->edge();
    assert(edge);
    if (edge == NULL) return 0;
  }

  return 0;

}


bool Topology::isRouteBlocked(double max_scan_dist, bool stop_scan_at_checkpoint)
{
	double dist = - ego_vehicle.distFromStart();
	RoutePlanner::Route::RouteEdgeList::iterator edge_it = current_edge_it;
	nextCheckPointIt();

	while ( edge_it != route.route.end() && dist < max_scan_dist ) {
		RndfEdge* edge = (*edge_it)->edge();
		if ( ! edge ) continue;
		if ( edge->isBlockedEdge() || edge->fromVertex()->isBlockedVertex() || edge->toVertex()->isBlockedVertex() ) {
			TRACE("Route voraus ist blockiert -> Replanning");
			return true;
		}
		if ( stop_scan_at_checkpoint && (*edge_it)->edge()->toVertex() == (*next_check_point_it) ) return false;
		dist += edge->length();
		++edge_it;
	}
	return false;
}


deque< RndfVertex* >::iterator Topology::nextCheckPointIt()
{
  if(checkpoints.empty()) {
    throw VLRException("Checkpointlist is empty.");
  }

  if(complete_mission_graph.empty()) {
    throw VLRException("(Complete) mission graph is empty.");
  }
//	TRACE("Getting next checkpoint in mission");

//	TRACE("  Checkpoints:");
//	int i=0;
//	for (deque<RndfVertex*>::iterator it = checkpoints.begin(); it != checkpoints.end(); ++it, ++i)
//		TRACE("  "<< i << ".  "<< (*it)->name() );

  if(current_edge_it_on_complete_mission_graph == complete_mission_graph.end()) {
    throw VLRException("Current edge (matched to mission graph) is not on mission graph.");
  }

	RoutePlanner::Route::RouteEdgeList::iterator edge_it = current_edge_it_on_complete_mission_graph;
	deque<RndfVertex*>::iterator cp_it = checkpoints.begin();
	int i = 0;
	for (RoutePlanner::Route::RouteEdgeList::iterator it = complete_mission_graph.begin(); it != complete_mission_graph.end(); ++it, ++i) {
		assert( *it );
		assert( (*it)->edge() );

		// scan for the "To" Vertex in Zones! (jz)
		if( (*it)->edge()->isZoneEdge() )
		  {
		    if ( (*it)->edge()->toVertex() == *cp_it && cp_it == checkpoints.begin() )
		      ++cp_it;
		  }
		else
		  {
		    if ( (*it)->edge()->fromVertex() == *cp_it && cp_it == checkpoints.begin() )
		      ++cp_it;
		  }

		if ( *it == *edge_it ) break;
		if ( (*it)->edge()->toVertex() == *cp_it )
			++cp_it;

	}
	//assert( cp_it != checkpoints.end() );
	assert( edge_it != complete_mission_graph.end() ); // otherwise  current_edge_it_on_complete_mission_graph  is not valid

//	if ( cp_it != checkpoints.end() ) {
//		TRACE("Next Checkpoint: "<< i << ".  "<< (*cp_it)->name());
//	}

	next_check_point_it = cp_it;
	return next_check_point_it;
}


RndfEdge* Topology::getBestReplanningEdge()
{
	// Nächsten Checkpoint ermitteln
	nextCheckPointIt();
	if ( next_check_point_it == checkpoints.end() )
		return NULL;
	assert( next_check_point_it != checkpoints.end() );

	// Blockaden ermitteln die eine Rolle spielen
	vector< Point_2 > blocks;
	for (BlockadeManager::TBlockageMap::const_iterator it = blockade_manager->blockade_map.begin(); it != blockade_manager->blockade_map.end(); ++it) {
		const BlockadeManager::Blockade& block = it->second;
		if ( block.isPublished() && ego_vehicle.distToEdge( block.edge() ) < 50.) {
			RndfEdge* edge = block.edge();
			Point_2 pf = edge->fromVertexPoint();
			Point_2 pt = edge->toVertexPoint();
			blocks.push_back( pf + (pt-pf)*0.5 );
		}
	}

	// Fahrzeuge ermitteln die Blockaden darstellen
	for (map<int, Vehicle>::const_iterator it = vehicle_manager->vehicle_map.begin(); it != vehicle_manager->vehicle_map.end(); ++it) {
		const Vehicle& veh = it->second;
		if ( veh.isBlocking() && sqrt( (ego_vehicle.point() - veh.point()).squared_length() ) < 50.)
			blocks.push_back( veh.point() );
	}

	// Beste Matchkante im Graph ermitteln die erreichbar ist
	TRACE("Searching for best reachable edge for new route:");
	const map<double, RndfEdge*>& base_matchings = getBestMatchings( ego_vehicle, complete_graph->edgeMap );

	// Kanten hinter einer Blockade schlechter bewerten
	map<double, RndfEdge*> matching;
	TRACE("Looking for Edges behind Blockades:");
	for (map<double, RndfEdge*>::const_iterator match_it = base_matchings.begin(); match_it != base_matchings.end(); ++match_it)
	{
		RndfEdge* edge = match_it->second;
		double score = match_it->first;
		bool skip = false;
		for (vector< Point_2 >::iterator it = blocks.begin(); it != blocks.end(); ++it) {
			Point_2& block_point = *it;
			Vector_2 vec( ego_vehicle.point(), block_point );
			Line_2 line( block_point, vec.perpendicular( CGAL::LEFT_TURN ) );

			// Kanten nahe an der Blockade skippen
			if ( squared_distance( block_point, edge->segment() ) < sqr( 7. ) ) {
				skip = true;
				break;
			}

			// Kanten hinter der Blockade schlechter bewerten
			if ( ( line.has_on_negative_side( edge->fromVertexPoint() ) || line.has_on_negative_side( edge->toVertexPoint() ) ) &&
					squared_distance( block_point, edge->segment() ) < sqr( 50. ) )
			{
				score += EDGE_BEHIND_BLOCKADE_PENALTY;
				TRACE("  -> "<< edge->name() << "  ("<< score <<")");
			}
		}
		if ( skip ) continue;

		matching.insert( make_pair( score, edge ) );
	}

	best_alternative_edge = NULL;
	for (map<double, RndfEdge*>::const_iterator match_it = matching.begin(); match_it != matching.end(); ++match_it)
	{
		RndfEdge* edge = match_it->second;
		double score = match_it->first;
		TRACE("  "<< edge->name() <<" ("<< score <<"):");

		// SonderKanten skippen
		if ( edge->isBlockedEdge() ) {
			TRACE("  -> BlockedEdge (skip)");
			continue;
		}
		if ( edge->isLaneChangeEdge() ) {
			TRACE("  -> LaneChangeEdge (skip)");
			continue;
		}
		if(edge->fromVertex()->inEdges().size()==1) {
		  if((*edge->fromVertex()->inEdges().begin())->isUTurnEdge()) {
		    TRACE("  -> UTurnEdge (skip)");
		    continue;
		  }
		}
		if ( edge->isUTurnEdge() ||
			(edge->fromVertex()->numInEdges() == 1 &&
			(*edge->fromVertex()->inEdges().begin())->isUTurnEdge()) ) {
			TRACE("  -> UTurnEdge (skip)");
			continue;
		}
		if ( edge->isIntersectionEdge() ) {
			TRACE("  -> IntersectionEdge (skip)");
			continue;
		}

//		Eigentlich gute Funktion um zu testen ob eine Edge hinter der Blockade liegt führt
//		allerdings zu Problemen falls man selber auf einer blockierten Edge steht
//		(und auch in anderen Fällen)
//
//		// Pfadsuche durchführen um den Weg zwischen der jetzigen Edge und der neuen auf Blockaden zu testen
//		RndfGraph::EdgeList* edgeList = complete_graph->searchPath( ego_vehicle.edge->toVertex(), edge->fromVertex(), true );
//	//	assert( edgeList );
//		if ( ! edgeList )
//			TRACE("  -> Keinen Weg von der aktuellen Edge zur alternativen gefunden");
//		if ( edgeList && RoutePlanner::pathBlocked( *edgeList ) && calcRouteLength( *edgeList ) < 80. ) {
//			TRACE("  -> Weg zu dieser Edge geht durch Blockade (skip)");
//			if (edgeList) delete edgeList;
//			continue;
//		}
//		if (edgeList) delete edgeList;

		// Pfadsuche durchführen um den Weg zwischen best_edge und dem nächsten Checkpoint zu testen
		RndfGraph::EdgeList* edgeList = complete_graph->searchPath( edge->toVertex(), *next_check_point_it );
		// assert( edgeList );
		if ( ! edgeList ) {
			TRACE("  -> Keinen Weg von dieser Edge zum nächsten Checkpoint gefunden (skip)");
			continue;
		}
		if (edgeList) delete edgeList;

		TRACE("=> Beste Kante gefunden (Distanz: "<< ego_vehicle.distToEdge(edge) << " m, Score: "<< score <<")");
		best_alternative_edge = edge;
		break;
	}
	return best_alternative_edge;
}

bool Topology::isOffTrack()
{
	// Abstand zur Route bestimmen
	double dist_to_edge = ego_vehicle.distToMatchedEdge() - ego_vehicle.edge()->width();
	if ( dist_to_edge < TRIGGER_DIST_OFF_TRACK ) {
//		TRACE("*** Vehicle is ON the track ("<< dist_to_edge <<") ***");
		return false;
	}

	TRACE("*** Vehicle is OFF the track ("<< dist_to_edge <<") ***");

	// Testen ob sich Replanning lohnt
	getBestReplanningEdge();

	if (best_alternative_edge && best_alternative_edge != ego_vehicle.edge()) {
		TRACE("alternative_edge ("<< best_alternative_edge->name() <<")  !=  matched_edge ("<< ego_vehicle.edge()->name() <<")");
		TRACE("=> Replan");
		return true;
	}

	TRACE("*** No better route found ***");

	return false;
}

Vehicle* Topology::get_next_vehicle()
{
  double d;

  if(!mission_planned) return 0;

  GraphTools::PlaceOnGraph place( current_edge_it, ego_vehicle.distFromStart(), route.route );
  assert(*place.edge_);
  return place.next_obstacle( d, place, TRIGGER_DIST_MAX_LOOKAHEAD );
}

void Topology::draw_arrow_at( double x, double y, double yaw, double scale, float r, float g, float b, float height )
{
  glDisable( GL_DEPTH_TEST );

  yaw *= ( M_PI / 180. );
  double base1_x = cos( yaw );
  double base1_y = sin( yaw );
  double base2_x = -sin( yaw );
  double base2_y = cos( yaw );

  double x1, y1, x2, y2, x3, y3;
  x1 = x;
  y1 = y;
  x2 = x - scale * base1_x * 2.5 + scale * base2_x;
  y2 = y - scale * base1_y * 2.5 + scale * base2_y;
  x3 = x - scale * base1_x * 2.5  - scale * base2_x;
  y3 = y - scale * base1_y * 2.5  - scale * base2_y;


  glBegin( GL_TRIANGLES );

  glColor3f( r, g, b );
  glVertex3f( x1, y1, height );
  glVertex3f( x2, y2, height );
  glVertex3f( x3, y3, height );

  glEnd();
}


void Topology::draw_edge( double center_x, double center_y, const RoutePlanner::RndfEdge& edge, float r, float g, float b, float height)
{
  glDisable( GL_DEPTH_TEST );

  glBegin( GL_LINES );
  glLineWidth(3.0);

  double x1, y1, x2, y2;

  x1 = edge.fromVertex()->x() - center_x;
  y1 = edge.fromVertex()->y() - center_y;

  x2 = edge.toVertex()->x() - center_x;
  y2 = edge.toVertex()->y() - center_y;

//  printf("%f, %f - %f, %f\n", x1, y1, x2, y2);
  double angle = atan2( y2-y1, x2-x1 );
  angle *= 180./M_PI;

  glColor3f( r, g, b );
  glVertex3f( x1, y1, height );
  glVertex3f( x2, y2, height );

  glEnd();

  if (edge.intersection()) {
    glBegin( GL_LINES );
    glColor3f( 1.0, 0, 0 );
    double dx = (x2 - x1) / 2;
    double dy = (y2 - y1) / 2;
    glVertex3f( x1 +dx +dy, y1 +dy -dx, height );
    glVertex3f( x1 +dx -dy, y1 +dy +dx, height );
    glEnd();
  }

  double theta, dx, dy;
  if (edge.isBlockedEdge()) {
	  if ( ! edge.isLaneChangeEdge() ) {
		    theta = atan2(y2-y1, x2-x1);
		    dx = 2.0 * cos(theta + M_PI_2);
		    dy = 2.0 * sin(theta + M_PI_2);
		    glLineWidth(10);
		    glBegin( GL_LINES );
		    glColor3f( 1.0, 0, 0 );
		    glVertex3f( x1 + dx, y1 + dy, height );
		    glVertex3f( x2 - dx, y2 - dy, height );
		    glVertex3f( x1 - dx, y1 - dy, height );
		    glVertex3f( x2 + dx, y2 + dy, height );
		    glEnd();

	  } else {
	    glLineWidth(10);
	    glBegin( GL_LINES );
	    glColor3f( 1.0, 0, 0 );
	    glVertex3f( x1, y1, height );
	    glVertex3f( x2, y2, height );
	    glEnd();
	  }
  }

  draw_arrow_at( x2, y2, angle, 0.5, r, g, b, height);
}

void Topology::paint_complete_graph(double center_x, double center_y, double r, double g, double b)
{
  for(  std::map<int, RndfEdge*>::iterator it = complete_graph->edgeMap.begin(); it != complete_graph->edgeMap.end(); it++ ) {
    if(it->second->isOffroadEdge()) {
      r = 1; g = 0; b = 0;
    }
    else {
      r = 1; g = 1; b = 1;
    }
    draw_edge(center_x, center_y, *(it->second), r, g, b );
  }
}

void Topology::paint_mission_graph(double center_x, double center_y, double r, double g, double b, bool annotations )
{
  if( !annotations )
  {
    r=1;
    g=0;
    b=1;
  }

  double color_inc =  1./mission_graph_map.size();

  for(  std::map<int, RndfEdge*>::iterator it = mission_graph_map.begin(); it != mission_graph_map.end(); it++ )
  {
    if( !annotations )
    {
      r -= color_inc;
      draw_edge(center_x, center_y, *(it->second), r, g, b );
      continue;
    }
    /*RndfEdge* myedge=(*it).second;
    AnnotatedRouteEdge* annoedge=myedge->getAnnotatedEdge();

    if( annoedge )
    {
      paint_annotations(center_x, center_y, annoedge);
    }*/
  }
}

void Topology::paint_complete_mission( double center_x, double center_y )
{
  int counter = -1;
  int size = complete_mission_graph.size();
  for (Route::RouteEdgeList::iterator it = complete_mission_graph.begin(); it!=complete_mission_graph.end(); ++it){
    paint_annotations(center_x, center_y, *it, (double)(++counter)/size * 4.0 + 1.0);
  }
}


unsigned char pa_colorpalette[13][3] = {
  {239,230,0},
  {230,0,230},
  {0,230,230},
  {230,0,0},
  {128,51,128},
  {51,128,128},
  {255,51,51},
  {51,255,51},
  {51,51,255},
  {51,179,204},
  {128,255,51},
  {255,128,51},
  {51,128,255}
};

void Topology::paint_annotations( double center_x, double center_y, AnnotatedRouteEdge* annoedge, double height)
{
  AnnotatedRouteEdge::AnnotationList::const_iterator ait;
  double r=0.711;
  double g=0.711;
  double b=0.711;
  bool override = false;
  for( ait=annoedge->annotations().begin(); ait!=annoedge->annotations().end(); ++ait )
  {
#define SETCOLOR(_nr) if (!override) { r=pa_colorpalette[_nr][0]; g=pa_colorpalette[_nr][1]; b=pa_colorpalette[_nr][2];}
    switch((*ait)->maneuver())
    {
    case UC_MANEUVER_TRAVEL:              SETCOLOR(0); break;
    case UC_MANEUVER_STOP_SIGN:           SETCOLOR(1); break;
    case UC_MANEUVER_INT_TURN_RIGHT:      SETCOLOR(2); override = true; break;
    case UC_MANEUVER_INT_TURN_LEFT:       SETCOLOR(3); override = true; break;
    case UC_MANEUVER_INT_STRAIGHT:        SETCOLOR(4); override = true; break;
    case UC_MANEUVER_CURVE_RIGHT:         SETCOLOR(5); break;
    case UC_MANEUVER_CURVE_LEFT:          SETCOLOR(6); break;
    case UC_MANEUVER_U_TURN:              SETCOLOR(7); break;
    case UC_MANEUVER_NAVIGATE:            SETCOLOR(8); break;
//    case UC_MANEUVER_NAVIGATE:            r=1.0; g=0.0; b=0.0;break;
    case UC_MANEUVER_PARKING:             SETCOLOR(9); break;
    case UC_MANEUVER_CHECKPOINT:          SETCOLOR(10); break;
    case UC_MANEUVER_GOAL_REACHED:        SETCOLOR(11); break;
    case UC_MANEUVER_START_MISSION:       SETCOLOR(12); break;
    case UC_MANEUVER_ZONE_ENTRY:          SETCOLOR(5); break;
    case UC_MANEUVER_ZONE_EXIT:           SETCOLOR(6); break;
    case UC_MANEUVER_LANECHANGE_LEFT:     SETCOLOR(6); override = true; break;
    case UC_MANEUVER_LANECHANGE_RIGHT:    SETCOLOR(6); override = true; break;
    default:                              r=0.0; g=0.75; b=0.0;break;
    }
#undef SETCOLOR

//    switch((*ait)->getManeuver().area_type)
//    {
//    case UC_TRAVEL_AREA:                   r=255.0; g=0.0; b=0.0; break;
//    case UC_ZONE:                          r=0.0; g=255.0; b=0.0; break;
//    case UC_PARKING_ZONE:                  r=0.0; g=0.0; b=255.0; break;
//    }

  }
  draw_edge(center_x, center_y, *(annoedge->edge()), r/255.0, g/255.0, b/255.0, height );
}

void Topology::draw_dot_at(double x, double y, float r, float g, float b )
{
  glDisable( GL_DEPTH_TEST );
  glPointSize( 10. );
  glBegin( GL_POINTS );
  glColor3f( r, g, b );
  glVertex2f( x, y );
  glEnd();
}

int Topology::addMessage(const string& message) {
  message_buffer.push_back(message);
  while(message_buffer.size()>TOPOLOGY_MESSAGE_QUEUE_SIZE)
    message_buffer.pop_front();
  return message_buffer.size();
}
int Topology::addMessage(const boost::format& frm)
{
	return addMessage(frm.str());
}
std::deque<std::string>& Topology::getMessages() {
  return message_buffer;
}

} // namespace vlr
