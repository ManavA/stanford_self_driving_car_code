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
#include <boost/lexical_cast.hpp>

#include <aw_CGAL.h>
#include <aw_AStarSearch.h>
#include <aw_RndfGraph.h>
#include <aw_RndfGraphBuilder.h>

using namespace std;

namespace vlr {

namespace RoutePlanner {

const uint32_t RndfGraph::max_bez_points_;

RndfGraph::RndfGraph() :
  Graph<RndfVertex, RndfEdge>(), maxSpeed(1) {
  xbez_ = new double[max_bez_points_];
  ybez_ = new double[max_bez_points_];
}

RndfGraph::~RndfGraph() {
  delete[] xbez_;
  delete[] ybez_;
}

RndfVertex * RndfGraph::addVertex(double lat, double lon, double x, double y, string name, int id) {
	if (id < 0) {
		id = vertex_id_count_.getNextId();
	}
//	cout << "  -> add vertex  LL( "<< lat <<", "<< lon <<" )  XY( "<< x <<", "<< y <<" )" << endl;
	RndfVertex * vertex = new RndfVertex(lat, lon, x, y, name, id);
	vertexMap.insert(make_pair(id, vertex));
	return vertex;
}

RndfEdge * RndfGraph::addEdge(RndfVertex * v1, RndfVertex * v2, string name, bool isLane, bool isLaneChange, double minSpeed,
		double maxSpeed, bool isOffroad, double weight, int id) {
	this->maxSpeed = max(this->maxSpeed, maxSpeed);
	if (id < 0) {
		id = edge_id_count_.getNextId();
	}
	RndfEdge * edge = new RndfEdge(v1, v2, name, isLane, false, isLaneChange, minSpeed, maxSpeed, isOffroad, weight, id);
	edgeMap.insert(make_pair(id, edge));
	v1->addEdge(edge);
	return edge;
}

RndfIntersection * RndfGraph::addIntersection(int id) {
	if (id < 0) {
		id = intersection_id_count_.getNextId();
	}
	RndfIntersection* intersection = new RndfIntersection(id);
	intersectionMap.insert(make_pair(id, intersection));
	return intersection;
}

void RndfGraph::setCheckpoint(int number, RndfVertex * vertex) {
	checkpointMap.insert(make_pair(number, vertex));
	vertex->setCheckpoint(number);
}

RndfVertex* RndfGraph::checkPoint(int id) {
	return checkpointMap[id];
}

RndfGraph::EdgeList * RndfGraph::searchPath(int fromCheckpoint, int toCheckpoint, bool ignore_blockades) {
	RndfVertex * fromVertex = checkpointMap[fromCheckpoint];
	assert(fromVertex != NULL);
	RndfVertex * toVertex = checkpointMap[toCheckpoint];
	assert(toVertex != NULL);
	return Graph<RndfVertex, RndfEdge>::searchPath(fromVertex, toVertex, ignore_blockades);
}
RndfGraph::EdgeList* RndfGraph::searchPath(RndfVertex* from, RndfVertex* to, bool ignore_blockades) {
//	assert(from != NULL);
//	assert(to != NULL);
  CheckpointMap::const_iterator cpit, cpit_end;
  if(checkpointMap.empty()) {
  printf("Checkpoint map is empty :-(\n\n");
  }
  for(cpit=checkpointMap.begin(), cpit_end=checkpointMap.end(); cpit!=cpit_end; cpit++) {
	printf("%i: %s\n", (*cpit).first, (*cpit).second->name().c_str());
	}
  return Graph<RndfVertex, RndfEdge>::searchPath(from, to, ignore_blockades);
}

double RndfGraph::searchHeuristic(RndfVertex const * vertex, RndfVertex const * goal) {
	return hypot(vertex->x() - goal->x(), vertex->y() - goal->y()) / maxSpeed;
}

RndfVertex * RndfGraph::findVertex(const string name) {
	//cout << "searching " << name << endl;
	for (VertexMap::iterator it = vertexMap.begin(); it != vertexMap.end(); ++it) {
		RndfVertex * vertex = it->second;
		//cout << vertex->name() << endl;
		if (vertex->name() == name) {
			return vertex;
		}
	}
	return NULL;
}

RndfEdge * RndfGraph::findEdge(const string name) {
	//cout << "searching " << name << endl;
	for (EdgeMap::iterator it = edgeMap.begin(); it != edgeMap.end(); ++it) {
		RndfEdge * edge = it->second;
		//cout << edge->name() << endl;
		if (edge->name() == name) {
			return edge;
		}
	}
	return NULL;
}

void RndfGraph::addVirtualLane(RndfEdge * oldEdge, RndfVertex * firstVertex, RndfVertex * fromVertex,
		RndfVertex * toVertex, RndfVertex * lastVertex, string exitName, double minSpeed, double maxSpeed,
		RndfGraphBuilder * graphBuilder)
{
  const uint32_t num_bez_points = 3;
  const uint32_t vl_vertex_sample_dist = 1;

//	oldEdge->setVirtual(); return; // TODO: REMOVE!
	int degree=3, id;
	double xcoord[4], ycoord[4];
	double dir1x, dir1y, dir2x, dir2y, diffx, diffy, dist;
	double len1, len2, ca;
	double distance_fromPoint_to_toLine, distance_toPoint_to_fromLine;
	bool isOffroad = oldEdge->isOffroadEdge();

	RndfVertex* prevVertex=NULL, *nextVertex=NULL;
	RndfEdge* vedge=NULL;

	dir1x=fromVertex->x()-firstVertex->x();
	dir1y=fromVertex->y()-firstVertex->y();
	dir2x=lastVertex->x()-toVertex->x();
	dir2y=lastVertex->y()-toVertex->y();

	len1=sqrt(dir1x*dir1x+dir1y*dir1y);
	len2=sqrt(dir2x*dir2x+dir2y*dir2y);

	dir1x/=len1;
	dir1y/=len1;
	dir2x/=len2;
	dir2y/=len2;

	if (len1==0.0 || len2==0.0) {
		return;
	}

	ca=(dir1x*dir2x+dir1y*dir2y)/(len1*len2);

    // k-turn / u-turn special case ?
	if (oldEdge->isUTurnEdge()) {
		//cout<<"ca : "<<ca<< " from : "<<fromVertex->name()<<" to "<<toVertex->name()<<endl;
		nextVertex = addVertex(-1, -1, 0.5*(toVertex->x()+fromVertex->x()), 0.5*(toVertex->y()
				+fromVertex->y()), exitName+"_kturn");
		nextVertex->setVirtual();

		id = edge_id_count_.getNextId();
		vedge = new RndfEdge(fromVertex, nextVertex, exitName+"_kturn_vedge_0", true, true, false, minSpeed, maxSpeed, isOffroad, graphBuilder->computeWeight(fromVertex, nextVertex, maxSpeed) * KTURN_EDGE_WEIGHT_FACTOR, id);
		edgeMap.insert(make_pair(vedge->getId(), vedge));
		vedge->setUTurn(true);
		fromVertex->addEdge(vedge);

		id = edge_id_count_.getNextId();
		vedge = new RndfEdge(nextVertex, toVertex, exitName+"_vedge_1", true, true, false, minSpeed, maxSpeed, isOffroad, graphBuilder->computeWeight(nextVertex, toVertex, maxSpeed) * KTURN_EDGE_WEIGHT_FACTOR, id);
		edgeMap.insert(make_pair(vedge->getId(), vedge));
//		vedge->setUTurn(true);
		nextVertex->addEdge(vedge);
	} else {
		diffx=toVertex->x()-fromVertex->x();
		diffy=toVertex->y()-fromVertex->y();

      // virtual edge too short for interpolation
		dist=sqrt(diffx*diffx+diffy*diffy);
    if(dist < 1.5*vl_vertex_sample_dist) {
      oldEdge->setVirtual();
      return;
    }


		// control polygon of lane center (hermite form -> Bezier)
		xcoord[0]=fromVertex->x();
		ycoord[0]=fromVertex->y();

		xcoord[3]=toVertex->x();
		ycoord[3]=toVertex->y();

		distance_fromPoint_to_toLine=fabs(diffx*dir2y-diffy*dir2x);
		distance_toPoint_to_fromLine=fabs(diffx*dir1y-diffy*dir1x);

    xcoord[1]=xcoord[0]+dir1x * 0.5* distance_fromPoint_to_toLine;
		ycoord[1]=ycoord[0]+dir1y * 0.5* distance_fromPoint_to_toLine;

		xcoord[2]=xcoord[3]-dir2x * 0.5* distance_toPoint_to_fromLine;
		ycoord[2]=ycoord[3]-dir2y * 0.5* distance_toPoint_to_fromLine;

		// convert virtual lane to spline
		cs_.bez_to_points(degree, num_bez_points, xcoord, xbez_);
		cs_.bez_to_points(degree, num_bez_points, ycoord, ybez_);

		  // we want the points to be approx. equidistant => estimate length of virtual lane
		  // and regenerate smoothed point set with correct number of points
		double len=0, sum_delta_x0=0, sum_delta_y0=0;
		for(uint32_t i=1; i<num_bez_points; i++) {
      double delta_x0 = std::abs(xbez_[i] - xbez_[i-1]);
      double delta_y0 = std::abs(ybez_[i] - ybez_[i-1]);
      sum_delta_x0 += delta_x0;
      sum_delta_y0 += delta_y0;
      len += hypot(delta_x0, delta_y0);
		}


    uint32_t num_points = num_bez_points;

    prevVertex = fromVertex;
		nextVertex = fromVertex;

#ifndef APPROX_MISSION
		num_points=2;
    xbez_[0]=xcoord[0]; ybez_[0]=ycoord[0];
    xbez_[1]=xcoord[3]; ybez_[1]=ycoord[3];
#endif

		for (uint32_t i=1; i<num_points; i++) {
			// Vertex erzeugen
			if (i != num_points-1)
				nextVertex = addVertex(-1, -1, xbez_[i], ybez_[i], exitName+"_int_"+boost::lexical_cast<std::string>(i-1) );
			else
				nextVertex = toVertex;

			// Vertex Attribute setzen
			nextVertex->setVirtual();
			//nextVertex->m_intersection = oldEdge->m_intersection;

			// Edge erzeugen
			id = edge_id_count_.getNextId();
			vedge = new RndfEdge(prevVertex, nextVertex, exitName+(oldEdge->intersection_ ? "_int_" : "_vedge_")+boost::lexical_cast<std::string>(i-1), true, true, false, minSpeed, maxSpeed, isOffroad, graphBuilder->computeWeight(prevVertex, nextVertex, maxSpeed), id);
			edgeMap.insert(make_pair(vedge->getId(), vedge));
			prevVertex->addEdge(vedge);
			prevVertex=nextVertex;

			vedge->copy_attributes_from( *oldEdge );
			vedge->isVirtual = true;
			// Edge Attribute setzen
			if (oldEdge->isStopLaneEdge()) vedge->setStopLane();
			if (oldEdge->intersection_) {
				vedge->intersection_ = oldEdge->intersection_;
				vedge->intersection_->addEdge(vedge);
			}

			// link crossing edges
			using namespace CGAL_Geometry;
			for (TRndfEdgeSet::const_iterator it = oldEdge->crossingEdges().begin(); it != oldEdge->crossingEdges().end(); ++it) {
				RndfEdge* oedge = *it;
				Point_2 p1f(vedge->fromVertex()->x(), vedge->fromVertex()->y());
				Point_2 p1t(vedge->toVertex()->x(),   vedge->toVertex()->y());
				Point_2 p2f(oedge->fromVertex()->x(), oedge->fromVertex()->y());
				Point_2 p2t(oedge->toVertex()->x(),   oedge->toVertex()->y());
				if ( ! CGAL::do_intersect( Segment_2( p1f, p1t ), Segment_2( p2f, p2t ) ) ) continue;
 				(*it)->addCrossingEdge(vedge);
 				vedge->addCrossingEdge(*it);
//		        cout << "  Cross  "<< vedge->name() << "  <->  "<< (*it)->name() << endl;
			}
		}
	}

	removeEdge(oldEdge);
	//oldEdge->setVirtual();
	//cout << endl;
}

void RndfGraph::clearBlockades()
{
	  for (EdgeMap::iterator it = edgeMap.begin(); it != edgeMap.end(); ++it)
		  it->second->setBlocked(false);
	  for (VertexMap::iterator it = vertexMap.begin(); it != vertexMap.end(); ++it)
		  it->second->setBlocked(false);
}


void RndfGraph::dumpEdges() {
	for (EdgeMap::iterator it = edgeMap.begin(); it != edgeMap.end(); ++it) {
		RndfEdge * edge = it->second;
		cout << edge->name() << endl;
	}
}

void RndfGraph::dumpVertices() {
	for (VertexMap::iterator it = vertexMap.begin(); it != vertexMap.end(); ++it) {
		RndfVertex * vertex = it->second;
		cout << vertex->name() << endl;
	}
}

RndfEdge* RndfGraph::findClosestEdge(double x, double y) {
  double dist;
  double min_dist = numeric_limits<double>::infinity();
  Point_2 pos( x, y );
  map<double, RndfEdge*> result;
  RndfEdge* best_edge = NULL;

  for (EdgeMap::const_iterator it = edgeMap.begin(); it != edgeMap.end(); ++it) {
    RndfEdge* edge = it->second;
    RndfVertex* v1 = edge->fromVertex();
    RndfVertex* v2 = edge->toVertex();
    Segment_2 seg( Point_2( v1->x(), v1->y() ), Point_2( v2->x(), v2->y() ) );
    dist = std::sqrt( squared_distance( pos, seg) );
    if (dist < min_dist) {
      best_edge = edge;
      min_dist = dist;
    }
  }
  return best_edge;
}

}

} // namespace vlr
