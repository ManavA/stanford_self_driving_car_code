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


#include <assert.h>
#include <cmath>
#include <aw_roadNetwork.h>
#include <aw_Mission.h>
#include <global.h>
#include "aw_RndfGraphBuilder.h"

#define GRAPH_EDGE_SAMPLE_DISTANCE  5.0

using namespace std;

namespace vlr {

using namespace rndf;

#define TRACE(str) std::cout << "[RndfGraphBuilder] " << str << std::endl;

namespace RoutePlanner {

const double RndfGraphBuilder::max_default_edge_speed_ = dgc::dgc_mph2ms(25);  // maximum speed for edges with unspecified speed limit

RndfGraphBuilder::RndfGraphBuilder(RoadNetwork* rn) //, COORDINATE_TRAFO * transf)
:
	network(rn) /*, mission(m),*/ {
}


double RndfGraphBuilder::computeWeight(const RndfVertex* p1, const RndfVertex* p2, double speed) {
//	cout << "( "<< p1->x() <<", "<< p2->x() <<" )  ( "<< p1->y() <<", "<< p2->y() <<" )" << endl;
//	cout << "[Comute Weight]  "<< p1->name() << "  "<< p2->name() << endl;
	if(speed==0.0) {return DBL_MAX;}
	double length = hypot(p1->x() - p2->x(), p1->y() - p2->y());
	return length/speed;
}

void RndfGraphBuilder::addCheckpoints(RndfGraph& graph, const TCheckPointMap& checkpoints) {
	for (TCheckPointMap::const_iterator cpit = checkpoints.begin(); cpit != checkpoints.end(); ++cpit) {
		CheckPoint * cp = cpit->second;
		RndfVertex * vertex = graph.findVertex(cp->wayPoint()->name());
    if(!vertex) {
      throw VLRException("addCheckpoints: Could not find vertex associated with way point name " + cp->wayPoint()->name());
    }
		graph.setCheckpoint(atoi(cpit->first.c_str()), vertex);
	}
}

RndfVertex * RndfGraphBuilder::addVertex(RndfGraph * graph, double lat, double lon, double x, double y, const std::string& name) {
	return graph->addVertex(lat, lon, x, y, name, -1);
}

RndfVertex * RndfGraphBuilder::addVertex(RndfGraph * graph, WayPoint* point) {
	// x, y are NOT utm but local coordinates now
	return graph->addVertex(point->lat(), point->lon(), point->x(), point->y(), point->name(), -1);//point->id());
}

RndfVertex * RndfGraphBuilder::addVertex(RndfGraph * graph, PerimeterPoint* point) {
	// x, y are NOT utm but local coordinates now
	return graph->addVertex(point->lat(), point->lon(), point->x(), point->y(), point->name(), -1);//point->id());
}

void RndfGraphBuilder::addStops(RndfGraph& graph, const TStopMap& stops) {
  for (TStopMap::const_iterator stopit = stops.begin(); stopit != stops.end(); ++stopit) {
    Stop* mystop = stopit->second;
    RndfVertex* vertex = graph.findVertex(mystop->wayPoint()->name());
    if(!vertex) {
      throw VLRException("addStops: Could not find vertex associated with way point name.");
    }
    vertex->setStop();
  }
}

void RndfGraphBuilder::addTrafficLights(RndfGraph& graph, const TTrafficLightMap& tlmap) {
  for (TTrafficLightMap::const_iterator tlit =tlmap.begin(); tlit != tlmap.end(); ++tlit) {
    TrafficLight* tl = tlit->second;
    std::map<std::string, WayPoint*>::const_iterator wpit=tl->linkedWayPoints().begin(), wpit_end=tl->linkedWayPoints().end();
    for(; wpit!=wpit_end; wpit++) {
      RndfVertex* vertex = graph.findVertex((*wpit).second->name());
      if(!vertex) {
        throw VLRException("addTrafficLights: Could not find vertex associated with way point name.");
      }

      vertex->addTrafficLight(tl->name());
    }
  }
}

void RndfGraphBuilder::addCrosswalks(RndfGraph& graph, const TCrosswalkMap& cwmap) {
  for (TCrosswalkMap::const_iterator cwit=cwmap.begin(); cwit != cwmap.end(); ++cwit) {
    Crosswalk* cw = cwit->second;
    std::map<std::string, WayPoint*>::const_iterator wpit=cw->linkedWayPoints().begin(), wpit_end=cw->linkedWayPoints().end();
    for(; wpit!=wpit_end; wpit++) {
      RndfVertex* vertex = graph.findVertex((*wpit).second->name());
      if(!vertex) {
        throw VLRException("addCrosswalks: Could not find vertex associated with way point name.");
      }
      vertex->addCrosswalk(cw->name());
    }
  }
}

void RndfGraphBuilder::addPoints(RndfGraph * graph, const TWayPointMap* waypoints, bool perimeter_, bool spot_) {
	for (TWayPointMap::const_iterator pointit = waypoints->begin(); pointit != waypoints->end(); ++pointit) {
		WayPoint* point = (*pointit).second;
		RndfVertex * vertex = addVertex(graph, point);
		printf("Adding vertex for way point %s\n", point->name().c_str());
		if (perimeter_) {
			vertex->setPerimeterPoint();
		}
		if (spot_) {
			vertex->setParkingSpot();
		}
	}
}

void RndfGraphBuilder::addPoints(RndfGraph * graph, const TWayPointVec* waypoints, bool perimeter_, bool spot_) {
  for (TWayPointVec::const_iterator pointit = waypoints->begin(); pointit != waypoints->end(); ++pointit) {
    WayPoint* point = (*pointit);
    RndfVertex * vertex = addVertex(graph, point);
    printf("Adding vertex for way point %s\n", point->name().c_str());
    if (perimeter_) {
      vertex->setPerimeterPoint();
    }
    if (spot_) {
      vertex->setParkingSpot();
    }
  }
}

void RndfGraphBuilder::addPoints(RndfGraph * graph, const TPerimeterPointVec* perimeterpoints) {
	for (TPerimeterPointVec::const_iterator pointit = perimeterpoints->begin(); pointit
	!= perimeterpoints->end(); ++pointit) {
		PerimeterPoint* point = *pointit;
		RndfVertex * vertex = addVertex(graph, point);
		vertex->setPerimeterPoint();
	}
}

void RndfGraphBuilder::addExits(RndfGraph * graph, const TExitMap* exits, double min_speed, double max_speed)
{
//	printf("numExits: %i\n", exits->size());
	for (TExitMap::const_iterator exitit = exits->begin(); exitit != exits->end(); ++exitit) {
		rndf::Exit* myexit = exitit->second;

		switch (myexit->exitType()) {
			case rndf::Exit::LaneToLane: {
				WayPoint * fromPointL = myexit->getExitFromLane();
				WayPoint * toPointL = myexit->getExitToLane();
				bool isOffroad = toPointL->parentLane()->segment()->offroad();
				RndfEdge* edge = addEdge(graph, fromPointL, toPointL, "Exit_"+myexit->name(), false, min_speed, max_speed, isOffroad, -1);
//				std::cout << "Exit "<< fromPointL->id() << " "<< fromPointL->name() <<" "<< myexit->name() << std::endl;
				assert(graph->findEdge("Exit_"+myexit->name()) != NULL);
				graph->findVertex(fromPointL->name())->setExit();

				// determine corresponding roadnetwork lane und lanesegment
				Lane* vlane = network->getLane( fromPointL->name() + " -> " + toPointL->name() );
				assert(vlane);
				LaneSegment* lseg = vlane->laneSegment(0);
				assert(lseg);

				edge->setStopLane( lseg->isStopLane() );
				edge->setUTurn( lseg->isKTurnEdge() );

				assert( edge != NULL );
				if (lseg->intersection()) {
					IntersectionMap::iterator it_intersection = graph->intersectionMap.find( lseg->intersection()->id() );
					RndfIntersection* intersection = it_intersection->second;
					intersection->addEdge(edge);
					edge->intersection_ = intersection;
					//graph->findVertex( fromPointL->name() )->intersection() = intersection;
					//graph->findVertex( toPointL->name()   )->intersection() = intersection;
				}

				break;
			}
			case rndf::Exit::LaneToPerimeter: {
				WayPoint * fromPointL = myexit->getExitFromLane();
				PerimeterPoint * toPointP = myexit->getExitToPerimeter();
				bool isOffroad = toPointP->perimeter()->zone()->offroad();
				RndfEdge * edge = addEdge(graph, fromPointL, toPointP, "Exit_"+myexit->name(), false, min_speed, max_speed, isOffroad, -1);
//				std::cout << "Exit "<< fromPointL->id() << " "<< toPointP->name() <<" "<< myexit->name() << std::endl;
				assert(graph->findEdge("Exit_"+myexit->name()) != NULL);
				graph->findVertex(fromPointL->name())->setExit();
				edge->setZone();
				break;
			}
			case rndf::Exit::PerimeterToLane: {
				PerimeterPoint * fromPointP = myexit->getExitFromPerimeter();
				WayPoint * toPointL = myexit->getExitToLane();
				bool isOffroad = toPointL->parentLane()->segment()->offroad();
				RndfEdge * edge = addEdge(graph, fromPointP, toPointL, "Exit_"+myexit->name(), false, min_speed, max_speed, isOffroad, -1);//myexit->id());
//				std::cout << "Exit "<< fromPointP->id() << " "<< fromPointP->name() << std::endl;
				graph->findVertex(fromPointP->name())->setExit();
				edge->setZone();
				break;
			}
			default:
			  std::cout << "Exit connection (most probably PerimeterToPerimeter) nit implemented (yet)\n";
		}
	}
}


RndfEdge * RndfGraphBuilder::addEdge(RndfGraph * graph, const WayPoint * p1, const WayPoint * p2, string name,
		bool isLane, double min_speed, double max_speed, bool isOffroad, int id)
{
	RndfVertex * fromVertex = graph->findVertex(p1->name());
	assert(fromVertex != NULL);
	RndfVertex * toVertex = graph->findVertex(p2->name());
	assert(toVertex != NULL);
	return graph->addEdge(fromVertex, toVertex, name, isLane, false, min_speed, max_speed, isOffroad, computeWeight(fromVertex, toVertex,
			max_speed), id);
}

RndfEdge * RndfGraphBuilder::addEdge(RndfGraph * graph, const PerimeterPoint * p1, const WayPoint * p2, string name,
		bool isLane, double min_speed, double max_speed, bool isOffroad, int id)
{
	RndfVertex * fromVertex = graph->findVertex(p1->name());
	assert(fromVertex != NULL);
	RndfVertex * toVertex = graph->findVertex(p2->name());
	assert(toVertex != NULL);
	return graph->addEdge(fromVertex, toVertex, name, isLane, false, min_speed, max_speed, isOffroad, computeWeight(fromVertex, toVertex,
			max_speed), id);
}

RndfEdge * RndfGraphBuilder::addEdge(RndfGraph * graph, const WayPoint * p1, const PerimeterPoint * p2, string name,
		bool isLane, double min_speed, double max_speed, bool isOffroad, int id) {
	RndfVertex * fromVertex = graph->findVertex(p1->name());
	assert(fromVertex != NULL);
	RndfVertex * toVertex = graph->findVertex(p2->name());
	assert(toVertex != NULL);
	return graph->addEdge(fromVertex, toVertex, name, isLane, false, min_speed, max_speed, isOffroad, computeWeight(fromVertex, toVertex,
			max_speed), id);
}

RndfEdge * RndfGraphBuilder::addEdge(RndfGraph * graph, const PerimeterPoint * p1, const PerimeterPoint * p2,
		string name, bool isLane, double min_speed, double max_speed, bool isOffroad, int id) {
	RndfVertex * fromVertex = graph->findVertex(p1->name());
	assert(fromVertex != NULL);
	RndfVertex * toVertex = graph->findVertex(p2->name());
	assert(toVertex != NULL);
	return graph->addEdge(fromVertex, toVertex, name, isLane, false, min_speed, max_speed, isOffroad, computeWeight(fromVertex, toVertex,
			max_speed), id);
}

void RndfGraphBuilder::connectAll(RndfGraph * graph, const TWayPointMap* waypoints, string prefix,
		double min_speed, double max_speed, bool isOffroad, bool zone_, double width) {
	for (TWayPointMap::const_iterator it1 = waypoints->begin(); it1 != waypoints->end(); ++it1) {
		for (TWayPointMap::const_iterator it2 = waypoints->begin(); it2 != waypoints->end(); ++it2) {
			WayPoint * p1 = (*it1).second;
			WayPoint * p2 = (*it2).second;
			if (p1 == p2)
				continue;
			RndfEdge * edge = addEdge(graph, p1, p2, prefix+"_"+p1->name()+"_"+p2->name(), false, min_speed, max_speed, isOffroad, -1);
			if (zone_) {
				edge->setZone();
			}
			if (width > 0) {
				edge->setWidth(width);
			}
		}
	}
}

void RndfGraphBuilder::connectAll(RndfGraph * graph, const TWayPointMap* waypoints, const WayPoint * p2,
		string prefix, double min_speed, double max_speed, bool isOffroad, bool zone_) {
	for (TWayPointMap::const_iterator it1 = waypoints->begin(); it1 != waypoints->end(); ++it1) {
		WayPoint * p1 = (*it1).second;
		if (p1 == p2)
			continue;
		RndfEdge * edge12 = addEdge(graph, p1, p2, prefix+"_"+p1->name()+"_"+p2->name(), false, min_speed, max_speed,
		    isOffroad, -1);
		RndfEdge * edge21 = addEdge(graph, p2, p1, prefix+"_"+p2->name()+"_"+p1->name(), false, min_speed, max_speed,
		    isOffroad, -1);
		if (zone_) {
			edge12->setZone();
			edge21->setZone();
		}
	}
}

void RndfGraphBuilder::connectAll(RndfGraph * graph, const TWayPointVec* waypoints, string prefix,
    double min_speed, double max_speed, bool isOffroad, bool zone_, double width) {
  for (TWayPointVec::const_iterator it1 = waypoints->begin(); it1 != waypoints->end(); ++it1) {
    for (TWayPointVec::const_iterator it2 = waypoints->begin(); it2 != waypoints->end(); ++it2) {
      WayPoint * p1 = (*it1);
      WayPoint * p2 = (*it2);
      if (p1 == p2)
        continue;
      RndfEdge * edge = addEdge(graph, p1, p2, prefix+"_"+p1->name()+"_"+p2->name(), false, min_speed, max_speed,
          isOffroad, -1);
      if (zone_) {
        edge->setZone();
      }
      if (width > 0) {
        edge->setWidth(width);
      }
    }
  }
}

void RndfGraphBuilder::connectAll(RndfGraph * graph, const TWayPointVec* waypoints, const WayPoint * p2,
    string prefix, double min_speed, double max_speed, bool isOffroad, bool zone_) {
  for (TWayPointVec::const_iterator it1 = waypoints->begin(); it1 != waypoints->end(); ++it1) {
    WayPoint * p1 = (*it1);
    if (p1 == p2)
      continue;
    RndfEdge * edge12 = addEdge(graph, p1, p2, prefix+"_"+p1->name()+"_"+p2->name(), false, min_speed, max_speed, isOffroad, -1);
    RndfEdge * edge21 = addEdge(graph, p2, p1, prefix+"_"+p2->name()+"_"+p1->name(), false, min_speed, max_speed, isOffroad, -1);
    if (zone_) {
      edge12->setZone();
      edge21->setZone();
    }
  }
}
void RndfGraphBuilder::connectAll(RndfGraph * graph, const TPerimeterPointMap* perimeterpoints, string prefix,
		double min_speed, double max_speed, bool isOffroad) {
	for (TPerimeterPointMap::const_iterator it1 = perimeterpoints->begin(); it1 != perimeterpoints->end(); ++it1) {
		for (TPerimeterPointMap::const_iterator it2 = perimeterpoints->begin(); it2 != perimeterpoints->end(); ++it2) {
			PerimeterPoint * p1 = (*it1).second;
			PerimeterPoint * p2 = (*it2).second;
			if (p1 == p2)
				continue;
			RndfEdge * edge = addEdge(graph, p1, p2, prefix+"_"+p1->name()+"_"+p2->name(), false, min_speed, max_speed, isOffroad, -1);
			edge->setZone();
		}
	}
}

void RndfGraphBuilder::connectAll(RndfGraph * graph, const TPerimeterPointMap* perimeterpoints,
		const WayPoint * p2, string prefix, double min_speed, double max_speed, bool isOffroad) {
	for (TPerimeterPointMap::const_iterator it1 = perimeterpoints->begin(); it1 != perimeterpoints->end(); ++it1) {
		PerimeterPoint * p1 = (*it1).second;
		if ((WayPoint*)p1 == p2)
			continue;
		RndfEdge * edge12 = addEdge(graph, p1, p2, prefix+"_"+p1->name()+"_"+p2->name(), false, min_speed, max_speed, isOffroad, -1);
		RndfEdge * edge21 = addEdge(graph, p2, p1, prefix+"_"+p2->name()+"_"+p1->name(), false, min_speed, max_speed, isOffroad, -1);

		edge12->setZone();
		edge21->setZone();
	}
}

void dumpWay(RndfVertex* v, int depth)
{
	cout << "_";
	static set<RndfVertex*> visited;
	if (visited.find(v) != visited.end()) return;
	visited.insert(v);
	for (int i = 0; i<depth; ++i) cout << " ";
	cout << v->name() << endl;
	for (RndfGraph::EdgeIterator it = v->beginEdges(); it != v->endEdges(); ++it) {
		RndfVertex* t = (*it)->toVertex();
		assert(t);
		dumpWay(t, depth+1);
	}
}



RndfGraph* RndfGraphBuilder::buildGraph()
{
	if(!network) {
	    throw VLRException("Network not initialized.");
	}
    WayPoint* current_waypoint;
	WayPoint* next_waypoint;
	RndfVertex * current_vertex;
	RndfVertex * next_vertex;
	RndfEdge * edge;
	RndfGraph * graph = new RndfGraph();

	network->addRelations();


	// add intersections
	TIntersectionSet::const_iterator intit, intit_end;
	const TIntersectionSet& intersections = network->intersections();
	for (intit = intersections.begin(),intit_end = intersections.end(); intit != intit_end; ++intit) {
		Intersection* intersection = (*intit);
		graph->addIntersection(intersection->id());
	}

	// add lanes
	for (TLaneMap::const_iterator laneit = network->lanes().begin(); laneit != network->lanes().end(); ++laneit) {
		Lane * lane = laneit->second;
		if (lane->isVirtual()) continue;

		double max_speed = maxSpeed(lane);
		double min_speed = minSpeed(lane);
		bool isOffroad = lane->segment()->offroad();
		double width = lane->laneWidth(); // * FT2M_FACTOR;
		const rndf::TLaneSegmentVec& lane_segments = lane->laneSegments();
		rndf::TLaneSegmentVec::const_iterator its=lane_segments.begin();
		rndf::TLaneSegmentVec::const_iterator its_end=lane_segments.end();

		// empty Lane
		if(its==its_end) continue;

		// add first vertex
		current_waypoint = (*its)->fromWayPoint();
		current_vertex = addVertex(graph, current_waypoint);

		for (; its != its_end;++its)
		{
			next_waypoint = (*its)->toWayPoint();
			next_vertex = addVertex(graph, next_waypoint);

			// Edge erzeugen und Attribute setzen, intersection()(NULL)
//			cout << "  Com Weight: "<< "Lane_"+lane->name()+"_"+current_waypoint->name()+"_"+next_waypoint->name() << endl;
			edge = graph->addEdge(current_vertex, next_vertex, "Lane_"+lane->name()+"_"+current_waypoint->name()+"_"+next_waypoint->name(), true, false, min_speed, max_speed, isOffroad, computeWeight(
					current_vertex, next_vertex, max_speed), -1);
			edge->setWidth(width);
			edge->setLeftBoundary(lane->leftBoundaryType());
			edge->setRightBoundary(lane->rightBoundaryType());
			edge->setStopLane( (*its)->isStopLane() );

			// check if Lane segment belongs to an intersection
			if ((*its)->intersection()) {
				// add edge to intersection
				IntersectionMap::iterator it_intersection = graph->intersectionMap.find((*its)->intersection()->id());
				RndfIntersection* intersection = it_intersection->second;
				intersection->addEdge(edge);
				// store a intersection pointer at edge
				edge->intersection_ = intersection;
				//current_vertex->intersection_ = intersection;
				//next_vertex->intersection_ = intersection;
			}
			current_vertex = next_vertex;
			current_waypoint = next_waypoint;
		}
	}


  // add stops
  addStops(*graph, network->stops());

  // add traffic lights
  addTrafficLights(*graph, network->trafficLights());

  // add crosswalks
  addCrosswalks(*graph, network->crosswalks());

  addAdjacentLanes(*graph, network->lanes());

	// add zones
	for (TZoneMap::const_iterator zoneit = network->zones().begin(); zoneit != network->zones().end(); ++zoneit) {
		Zone* zone = zoneit->second;
    printf("Adding zone %s to graph\n", zone->name().c_str());
		double min_speed = minSpeed(zone);
		double max_speed = maxSpeed(zone);
		bool isOffroad = zone->offroad();

		// add all Perimeter points
    for (TPerimeterMap::const_iterator perimeterit = zone->perimeters().begin(); perimeterit != zone->perimeters().end(); ++perimeterit) {
      Perimeter* myperimeter = perimeterit->second;
      addPoints(graph, &myperimeter->perimeterPoints());
    }

    // add spots
    for (TSpotMap::const_iterator spotit = zone->spots().begin(); spotit != zone->spots().end(); ++spotit) {
      Spot* spot = spotit->second;
      printf("Adding spot %s to graph\n", spot->name().c_str());
      addPoints(graph, &spot->wayPoints(), false, true);
//      addCheckpoints(*graph, myspot->checkPoints());
      connectAll(graph, &spot->wayPoints(), "Spot_"+spot->name(), min_speed, max_speed, isOffroad, true, spot->getSpotWidth());
    }

		// interconnect all parking spots
		for (TSpotMap::const_iterator spotit1 = zone->spots().begin(); spotit1 != zone->spots().end(); ++spotit1) {
			for (TSpotMap::const_iterator spotit2 = zone->spots().begin(); spotit2 != zone->spots().end(); ++spotit2) {
				Spot * myspot1 = spotit1->second;
				Spot * myspot2 = spotit2->second;
				if (myspot1 == myspot2) continue;
				for (TWayPointVec::const_iterator pointit1 = myspot1->wayPoints().begin(); pointit1 != myspot1->wayPoints().end(); ++pointit1) {
					if ((*pointit1)->checkPoint()) {
						//assert(false);
						continue;
					}
					for (TWayPointVec::const_iterator pointit2 = myspot2->wayPoints().begin(); pointit2 != myspot2->wayPoints().end(); ++pointit2) {
						if ((*pointit2)->checkPoint()) {
							//assert(false);
							continue;
						}
						RndfEdge * edge = addEdge(graph, *pointit1, *pointit2, "Zone_"+(*pointit1)->name()+"_"+(*pointit2)->name(), false,
								min_speed, max_speed, isOffroad, -1);
						edge->setZone();
					}
				}
			}
		}

		// connect all parking spots with all entries and exits
		for (TSpotMap::const_iterator spotit1 = zone->spots().begin(); spotit1 != zone->spots().end(); ++spotit1) {
			Spot * myspot = spotit1->second;
			TWayPointVec::const_iterator spotwayit = myspot->wayPoints().begin();
			if ((*spotwayit)->checkPoint()) {
				++spotwayit;
				assert(spotwayit != myspot->wayPoints().end());
			}
			WayPoint * point1 = *spotwayit;
			for (TPerimeterMap::const_iterator perimeterit = zone->perimeters().begin(); perimeterit != zone->perimeters().end(); ++perimeterit) {
				Perimeter* myperimeter = perimeterit->second;
				for (TPerimeterPointVec::const_iterator ppointit = myperimeter->perimeterPoints().begin(); ppointit != myperimeter->perimeterPoints().end(); ++ppointit) {
					PerimeterPoint* mypoint = *ppointit;
					if (mypoint->exits().size()) {
						PerimeterPoint * point2 = mypoint;
						assert(point2); // TODO: connections to other zones?
						RndfEdge * edge = addEdge(graph, point1, point2, "Zone_p2ex_"+point2->name()+"_"+point1->name(), false, min_speed, max_speed, isOffroad, -1);
						edge->setZone();
					}
					if (mypoint->entries().size()) {
						PerimeterPoint * point2 = mypoint;
						assert(point2); // TODO: connections to other zones?
						RndfEdge * edge = addEdge(graph, point2, point1, "Zone_p2en_"+point1->name()+"_"+point2->name(), false, min_speed, max_speed, isOffroad, -1);
						edge->setZone();
					}
				}
			}
		}

		// connect all entries with all exits
		for (TPerimeterMap::const_iterator perimeterit = zone->perimeters().begin(); perimeterit != zone->perimeters().end(); ++perimeterit) {
			Perimeter* myperimeter = perimeterit->second;
			for (TPerimeterPointVec::const_iterator ppointit = myperimeter->perimeterPoints().begin(); ppointit != myperimeter->perimeterPoints().end(); ++ppointit) {
				PerimeterPoint* mypoint = *ppointit;
				if ( mypoint->entries().size()) {
					for (TPerimeterPointVec::const_iterator ppointit2 = myperimeter->perimeterPoints().begin(); ppointit2 != myperimeter->perimeterPoints().end(); ++ppointit2) {
						PerimeterPoint* mypoint2 = *ppointit2;
						if (mypoint == mypoint2) continue;
						if (mypoint2->exits().size()) {
							RndfEdge * edge = addEdge(graph, mypoint, mypoint2, "Zone_p2p_en_"+mypoint->name()+"_"+mypoint2->name(), false, min_speed, max_speed, isOffroad, -1);
							edge->setZone();
						}
					}
				}
			}
		}
	}

  // add checkpoints
  addCheckpoints(*graph, network->checkPoints());

	// add Lane exits
  for (TLaneMap::const_iterator laneit = network->lanes().begin(); laneit != network->lanes().end(); ++laneit) {
    Lane* mylane = laneit->second;
    addExits(graph, &mylane->exits(), minSpeed(mylane), maxSpeed(mylane));
  }

  // add Zone exits
  for (TPerimeterMap::const_iterator perimeterit = network->perimeters().begin(); perimeterit != network->perimeters().end(); ++perimeterit) {
    Perimeter* myperimeter = perimeterit->second;
    addExits(graph, &myperimeter->exits(), minSpeed(myperimeter->zone()), maxSpeed(myperimeter->zone()));
  }

// TODO: find were they are used...
      // add crossing lanes
  cout << "[RndfGraphBuilder] Adding crossing lanes ... "<< flush;
  for (TLaneMap::const_iterator laneit = network->lanes().begin(); laneit != network->lanes().end(); ++laneit) {
    Lane* mylane = laneit->second;
    if (mylane->isVirtual()) continue;
    for (rndf::TLaneSegmentVec::const_iterator it=mylane->laneSegments().begin(); it != mylane->laneSegments().end(); ++it) {
      rndf::LaneSegment* lseg = *it;

      string edge_name;
      if (mylane->isVirtual()) {
        edge_name = "Exit_"+mylane->name();
        string::size_type pos = edge_name.find(" -> ");
        assert(pos != string::npos);
        edge_name.replace(pos, 4, "_");
      } else
        edge_name = "Lane_"+mylane->name()+"_"+lseg->fromWayPoint()->name()+"_"+lseg->toWayPoint()->name();
      RndfEdge* edge = graph->findEdge( edge_name );
      assert(edge);

      for (rndf::TLaneSegmentSet::const_iterator c_it = lseg->crossingLaneSegments().begin(); c_it != lseg->crossingLaneSegments().end(); ++c_it) {
        rndf::LaneSegment* c_lseg = *c_it;
        rndf::Lane* c_lane = c_lseg->lane();

        string c_edge_name;
        if (c_lane->isVirtual()) {
          c_edge_name = "Exit_"+c_lane->name();
          string::size_type pos = c_edge_name.find(" -> ");
          assert(pos != string::npos);
          c_edge_name.replace(pos, 4, "_");
        } else
          c_edge_name = "Lane_"+c_lane->name()+"_"+c_lseg->fromWayPoint()->name()+"_"+c_lseg->toWayPoint()->name();
//        cout << "  "<< edge_name << "  <->  "<< c_edge_name << endl;
        RndfEdge* c_edge = graph->findEdge( c_edge_name );
        assert(c_edge);

        edge->addCrossingEdge(c_edge);
        c_edge->addCrossingEdge(edge);
      }
    }
  }
  cout << "Ok" << endl;;

	// add virtual lanes
	addVirtualLanes(graph);

  IntersectionMap::const_iterator intit2=graph->intersections().begin(), intit2_end=graph->intersections().end();
  for(; intit2 != intit2_end; intit2++) {
    (*intit2).second->updateRadius();
  }

	return graph;
}

void RndfGraphBuilder::addAdjacentLanes(RndfGraph& graph, const TLaneMap& lanes) {

  for (TLaneMap::const_iterator laneit = lanes.begin(); laneit != lanes.end(); ++laneit) {
      Lane* lane = laneit->second;
      if (lane->isVirtual()) {continue;}

      for (rndf::TLaneSegmentVec::const_iterator it=lane->laneSegments().begin(); it != lane->laneSegments().end(); ++it) {
          rndf::LaneSegment* lseg = *it;
          RndfEdge* edge = graph.findEdge("Lane_"+lane->name()+"_"+lseg->fromWayPoint()->name()+"_"+lseg->toWayPoint()->name());
          if(!edge) {
              throw VLRException("Invalid edge.");
          }

              // left adjacent lanes
          for (rndf::TLaneSegmentSet::iterator l_it=lseg->leftLaneSegments().begin(); l_it != lseg->leftLaneSegments().end(); ++l_it) {
              rndf::LaneSegment* n_lseg = *l_it;
              if (n_lseg->lane()->isVirtual()) {continue;}
              RndfEdge* n_edge = graph.findEdge("Lane_"+n_lseg->lane()->name()+"_"+n_lseg->fromWayPoint()->name()+"_"+n_lseg->toWayPoint()->name());
              if(!n_edge) {
                  throw VLRException("Invalid edge.");
              }
              edge->addLeftEdge(n_edge);
          }

              // right adjacent lanes
          for (rndf::TLaneSegmentSet::iterator l_it=lseg->rightLaneSegments().begin(); l_it != lseg->rightLaneSegments().end(); ++l_it) {
              rndf::LaneSegment* n_lseg = *l_it;
              if (n_lseg->lane()->isVirtual()) {continue;}
              RndfEdge* n_edge = graph.findEdge("Lane_"+n_lseg->lane()->name()+"_"+n_lseg->fromWayPoint()->name()+"_"+n_lseg->toWayPoint()->name());
              if(!n_edge) {
                  throw VLRException("Invalid edge.");
              }
              edge->addRightEdge(n_edge);
          }

              // left oncoming lanes
          for (rndf::TLaneSegmentSet::iterator l_it=lseg->oncomingLaneSegments().begin(); l_it != lseg->oncomingLaneSegments().end(); ++l_it) {
              rndf::LaneSegment* n_lseg = *l_it;
              if (n_lseg->lane()->isVirtual()) {continue;}
              RndfEdge* n_edge = graph.findEdge("Lane_"+n_lseg->lane()->name()+"_"+n_lseg->fromWayPoint()->name()+"_"+n_lseg->toWayPoint()->name());
              if(!n_edge) {
                  throw VLRException("Invalid edge.");
              }
              edge->addLeftOncomingEdge(n_edge);
          }
      }
  }
}

double RndfGraphBuilder::maxSpeed(Lane* lane) {
  const SpeedLimit* sl = lane->speedLimit();
  if(sl) {return sl->maxSpeed();}

  Segment* segment = lane->segment();
	if (!segment) {
		return max_default_edge_speed_;
	}
	sl = segment->speedLimit();
  if(sl) {return sl->maxSpeed();}

  return max_default_edge_speed_;
}

double RndfGraphBuilder::minSpeed(Lane* lane) {
  const SpeedLimit* sl = lane->speedLimit();
  if(sl) {return sl->minSpeed();}

  Segment* segment = lane->segment();
  if (!segment) {return 0.0;}
  sl = segment->speedLimit();
  if(sl) {return sl->minSpeed();}

  return 0.0;
}

double RndfGraphBuilder::maxSpeed(Zone* zone) {
	const SpeedLimit* sl = zone->speedLimit();
  if(sl) {return sl->maxSpeed();}
  return max_default_edge_speed_;
}

double RndfGraphBuilder::minSpeed(Zone* zone) {
  const SpeedLimit* sl = zone->speedLimit();
  if(sl) {return sl->minSpeed();}
  return 0.0;
}


void RndfGraphBuilder::transformCoordsGPStoUTM(double lat, double lon, double & x, double & y) {
	char utmzone[10];
	latLongToUtm(lat, lon, &x, &y, utmzone);
}


RndfEdge* RndfGraphBuilder::addLaneChangeEdge(RndfGraph * graph, rndf::WayPoint const * p1, rndf::WayPoint const * p2, double min_speed, double max_speed, bool isOffroad)
{
	if (p1 == NULL || p2 == NULL) {
		cout << "warning: undefined waypoints" << endl;
		return NULL;
	}
	RndfVertex * fromVertex = graph->findVertex(p1->name());
	RndfVertex * toVertex = graph->findVertex(p2->name());
	if (toVertex == NULL || fromVertex == NULL) {
		cout << "warning: undefined vertices" << endl;
		return NULL;
	}

	double length= sqrt( ( fromVertex->point() - toVertex->point() ).squared_length() );
	if (length < 10.) {
//		cout << "LaneChange Lane to short: "<< length <<"  (skipped)" << endl;
		return NULL;
	}
	if (length > 35.) {
//		cout << "LaneChange Lane to long: "<< length <<"  (skipped)" << endl;
		return NULL;
	}

	double weight = computeWeight(fromVertex, toVertex, max_speed) * LANECHANGE_EDGE_WEIGHT_FACTOR;  // Weight increased to avoid unnecessary Lane changes
	return graph->addEdge(fromVertex, toVertex, "LaneChange_"+p1->name()+"_"+p2->name(), false, true, min_speed, max_speed, isOffroad, weight, -1);
}

void RndfGraphBuilder::addLaneChangeEdges(RndfGraph * graph) {
  for (rndf::TLaneMap::const_iterator laneit = network->lanes().begin(); laneit != network->lanes().end(); ++laneit) {
    Lane* lane = laneit->second;
      // TODO: implement Lane change on virtual lanes
    if (lane->isVirtual()) continue;
    double min_speed = minSpeed(lane);
    double max_speed = maxSpeed(lane);
    bool isOffroad = lane->segment()->offroad();
    for (rndf::TLaneSegmentVec::const_iterator segit = lane->laneSegments().begin(); segit != lane->laneSegments().end(); ++segit) {
      LaneSegment* seg = *segit;
      RndfEdge* edge_1 = graph->findEdge("Lane_" + lane->name() + "_" + seg->fromWayPoint()->name() + "_" + seg->toWayPoint()->name());
      if (!edge_1) {
        throw VLRException("addLaneChangeEdges: Graph is inconsistent - cannot find edge Lane_" + lane->name() + "_" + seg->fromWayPoint()->name() + "_" + seg->toWayPoint()->name());
      }

        // don't create lane change edges around intersections
      bool skip=false;
      for (rndf::TLaneSegmentSet::const_iterator it = seg->nextLaneSegments().begin(); it != seg->nextLaneSegments().end(); ++it) {
        if ((*it)->intersection()) {skip=true;}
      }
      for (rndf::TLaneSegmentSet::const_iterator it = seg->prevLaneSegments().begin(); it != seg->prevLaneSegments().end(); ++it) {
        if ((*it)->intersection()) {skip=true;}
      }
      if(skip) {continue;}

      if (!seg->leftLaneSegments().empty()) {
        for (TLaneSegmentSet::iterator sit = seg->leftLaneSegments().begin(); sit != seg->leftLaneSegments().end(); ++sit) {
          LaneSegment * seg2 = *sit;
          if (seg2->lane()->isVirtual()) continue;
          if (lane->leftBoundaryType() == Lane::DoubleYellow) continue;
          if (lane->leftBoundaryType() == Lane::SolidWhite) continue;
          //					std::cout << seg->name() << " has left Lane " << seg2->name() << std::endl;
          RndfEdge* edge = addLaneChangeEdge(graph, seg->fromWayPoint(), seg2->toWayPoint(), min_speed, max_speed,
              isOffroad);
          if (edge) {
            edge->setLeftLaneChange();
            RndfEdge* edge_2 = graph->findEdge("Lane_" + seg2->lane()->name() + "_" + seg2->fromWayPoint()->name()
                + "_" + seg2->toWayPoint()->name());
            if (!edge_2) {
              throw VLRException("addLaneChangeEdges: Graph is inconsistent - cannot find edge Lane_"
                  + seg2->lane()->name() + "_" + seg2->fromWayPoint()->name() + "_" + seg2->toWayPoint()->name());
            }
            edge->addLeftEdge(edge_2);
            //edge_2->rightEdges.insert( edge );
            edge->addRightEdge(edge_1);
            //edge_1->leftEdges.insert( edge );
          }
        }
      }
      if (!seg->rightLaneSegments().empty()) {
        for (TLaneSegmentSet::iterator sit = seg->rightLaneSegments().begin(); sit != seg->rightLaneSegments().end(); ++sit) {
          LaneSegment * seg2 = *sit;
          if (seg2->lane()->isVirtual()) continue;
          if (lane->rightBoundaryType() == Lane::DoubleYellow) continue;
          if (lane->rightBoundaryType() == Lane::SolidWhite) continue;
          //					std::cout << seg->name() << " has right Lane " << seg2->name() << std::endl;
          RndfEdge* edge = addLaneChangeEdge(graph, seg->fromWayPoint(), seg2->toWayPoint(), min_speed, max_speed,
              isOffroad);
          if (edge) {
            edge->setRightLaneChange();
            RndfEdge* edge_2 = graph->findEdge("Lane_" + seg2->lane()->name() + "_" + seg2->fromWayPoint()->name()
                + "_" + seg2->toWayPoint()->name());
            assert(edge_2);
            edge->addRightEdge(edge_2);
            //edge_2->leftEdges.insert( edge );
            edge->addLeftEdge(edge_1);
            //edge_1->rightEdges.insert( edge );
          }
        }
      }
      if (!seg->oncomingLaneSegments().empty()) {
        //				for (TLaneSegmentSet::iterator sit = seg->oncomingLaneSegments().begin(); sit != seg->oncomingLaneSegments().end(); ++sit) {
        //					laneSegment * seg2 = *sit;
        //					if (seg2->lane()->isVirtual()) continue;
        //					if (seg2->lane()->leftBoundaryType() == Lane::DoubleYellow) continue;
        //					std::cout << seg->name() << " has opposite Lane " << (*(seg->oncomingLaneSegments().begin()))->name() << std::endl;
        //					RndfEdge* edge = addLaneChangeEdge(graph, seg->fromWayPoint(), seg2->fromWayPoint(), min_speed, max_speed);
        //					if (edge) edge->setLeftOppositeLaneChange();
        //				}
      }
    }
  }
  /*    for (RoadNetwork::TLaneSet::const_iterator laneit = network->virtualLanes().begin(); laneit != network->virtualLanes().end(); ++laneit) {
   Lane * mylane = *laneit;
   std::cout << mylane->name() << std::endl;
   if (mylane->getLeftLane() != NULL) {
   std::cout << mylane->name() << " has left Lane " << mylane->getLeftLane()->name() << std::endl;
   }
   if (mylane->getRightLane() != NULL) {
   std::cout << mylane->name() << " has right Lane " << mylane->getRightLane()->name() << std::endl;
   }
   if (mylane->getLeftOppositeLane() != NULL) {
   std::cout << mylane->name() << " has opposite Lane " << mylane->getLeftOppositeLane()->name() << std::endl;
   }
   }*/
}

void RndfGraphBuilder::addVirtualLanes(RndfGraph * graph)
{
	// for all lanes
  for (TLaneMap::const_iterator laneit = network->lanes().begin(); laneit != network->lanes().end(); ++laneit) {
		Lane * mylane = laneit->second;
		addVirtualLanes(graph, mylane, minSpeed(mylane), maxSpeed(mylane));
	}
  // for all perimeters
  for (TPerimeterMap::const_iterator perimeterit = network->perimeters().begin(); perimeterit != network->perimeters().end(); ++perimeterit) {
    Perimeter * myperimeter = perimeterit->second;
    addVirtualLanes(graph, myperimeter, minSpeed(myperimeter->zone()), maxSpeed(myperimeter->zone()));
  }

}

void RndfGraphBuilder::addVirtualLanes(RndfGraph* graph, Lane* mylane, double min_speed, double max_speed) {
  for (TExitMap::const_iterator exitit = mylane->exits().begin(); exitit != mylane->exits().end(); ++exitit) {
    RndfVertex* fromVertex;
    RndfVertex* toVertex;
    RndfVertex* firstVertex;
    RndfVertex* lastVertex;

    rndf::Exit* myexit = exitit->second;
    RndfEdge* oldEdge = graph->findEdge("Exit_" + myexit->name());
    assert(oldEdge != NULL);

    if (oldEdge->isZoneEdge()) {
      // TODO smooth Zone entries and exits
      oldEdge->setVirtual();
    }
    else {
      fromVertex = static_cast<RndfVertex*> (oldEdge->fromVertex());
      toVertex = static_cast<RndfVertex*> (oldEdge->toVertex());

      // TODO: Improve edge selection

      // determine previous edge
      RndfEdge * prevEdge = graph->findIncomingEdge(fromVertex);
      //      if (prevEdge == NULL || toVertex->getNumberOfEdges() != 1)
      if (prevEdge == NULL || toVertex->getNumberOfEdges() < 1) {
        std::cout << "warning (1): could not add virtual Lane for Exit " << myexit->name().c_str() << "\n";
        continue;
      }

      // determine following edge
      RndfEdge* followingEdge = *(toVertex->beginEdges());
      assert(followingEdge);

      //    if (!prevEdge->isLaneEdge() || !followingEdge->isLaneEdge())
      //      || prevEdge->isVirtualEdge() || followingEdge->isVirtualEdge())
      //    {
      //      std::cout << "warning (2): could not add virtual Lane for Exit "<< myexit->name() << std::endl;
      //      continue;
      //    }

      // additional points used  for tangent calculation
      firstVertex = prevEdge->fromVertex();
      lastVertex = followingEdge->toVertex();

      // TODO: Add this to zones as well..?!?
      double min_speed = std::min(prevEdge->minSpeed(), followingEdge->minSpeed());
      double max_speed = std::max(prevEdge->maxSpeed(), followingEdge->maxSpeed());
      // interpolate points on virtual edges (make them round and nice e.g. in intersections :-) )
      graph->addVirtualLane(oldEdge, firstVertex, fromVertex, toVertex, lastVertex, myexit->name(), min_speed, max_speed, this);
    }
  }
}

void RndfGraphBuilder::addVirtualLanes(RndfGraph * graph, Perimeter* myperimeter, double min_speed, double max_speed)
{
  for (TExitMap::const_iterator exitit = myperimeter->exits().begin(); exitit != myperimeter->exits().end(); ++exitit)
  {
    RndfVertex * fromVertex;
    RndfVertex * toVertex;
    RndfVertex * firstVertex;
    RndfVertex * lastVertex;

    rndf::Exit* myexit = exitit->second;
    RndfEdge * oldEdge = graph->findEdge("Exit_"+myexit->name());
    assert(oldEdge != NULL);

    if (oldEdge->isZoneEdge()) {
      // TODO smooth Zone entries and exits
      oldEdge->setVirtual();
    }
    else {
      fromVertex = (oldEdge->fromVertex());
      toVertex = (oldEdge->toVertex());

      // TODO Kantenauswahl Könnte bei Bedarf verbessert werden

      // Vorherige Kante ermitteln
      RndfEdge * prevEdge = NULL;
      for (set< RndfEdge* >::iterator it = fromVertex->inEdges().begin(); it != fromVertex->inEdges().end(); ++it) {
		if ( ! (*it)->isVirtualEdge() ) {
			prevEdge = *it;
			break;
		}
	}
//      graph->findIncomingEdge(fromVertex);

      // Nachfolgerkante ermitteln

      RndfEdge * followingEdge = NULL;
      for (set< RndfEdge* >::iterator it = toVertex->outEdges().begin(); it != toVertex->outEdges().end(); ++it) {
		if ( ! (*it)->isVirtualEdge() ) {
			followingEdge = *it;
			break;
		}
	}


      if ( prevEdge == NULL || followingEdge == NULL )
      {
    	  cerr << "warning (1): could not add virtual Lane for Exit "<< myexit->name() << endl;
    	  continue;
      }

//      if ( prevEdge == NULL || followingEdge == NULL )
//     {
//       dgc_die("warning (1): could not add virtual Lane for Exit %s\n", myexit->name().c_str());
//       continue;
//     }


      //    if (!prevEdge->isLaneEdge() || !followingEdge->isLaneEdge())
      //      || prevEdge->isVirtualEdge() || followingEdge->isVirtualEdge())
      //    {
      //      std::cout << "warning (2): could not add virtual lane for exit "<< myexit->name() << std::endl;
      //      continue;
      //    }

      // get additional points for tangents required by curve smoother
      firstVertex = (prevEdge->fromVertex());
      lastVertex = (followingEdge->toVertex());

      // Curve smoothen
      graph->addVirtualLane(oldEdge, firstVertex, fromVertex, toVertex, lastVertex, myexit->name(), min_speed, max_speed, this);
    }
  }
}

}

} // namespace vlr
