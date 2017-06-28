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


#ifndef AW_RNDF_GRAPH_H
#define AW_RNDF_GRAPH_H

#include <string>
#include <curveSmoother.h>
#include <aw_Graph.h>
#include <aw_RndfEdge.h>
#include <aw_RndfVertex.h>
#include <aw_RndfIntersection.h>

namespace vlr {

struct Vehicle;

namespace RoutePlanner {

typedef std::map<int, RndfIntersection* > IntersectionMap;

const double KTURN_EDGE_WEIGHT_FACTOR        = 15.; // HACK: was 500.
const double LANECHANGE_EDGE_WEIGHT_FACTOR   = 2.;


class RndfGraphBuilder;

class RndfParentElement {
  // UNIMPLEMENTED!
  // represents segments and zones, used e.g. to determine number of neighboring lanes
protected:
  bool segment;
  bool zone;
  int id;
  int numberOfLanes;
  int numberOfSpots;
};

class RndfGraph : public Graph<RndfVertex, RndfEdge> {
public:
  friend class RndfGraphBuilder;

  typedef Graph<RndfVertex, RndfEdge>			BaseGraph;
  typedef Graph<RndfVertex, RndfEdge>::VertexMap	VertexMap;
  typedef Graph<RndfVertex, RndfEdge>::EdgeMap	EdgeMap;
  typedef Graph<RndfVertex, RndfEdge>::EdgeList	EdgeList;
  typedef Graph<RndfVertex, RndfEdge>::EdgeSet	EdgeSet;
  typedef EdgeList::const_iterator	EdgeIterator;

  RndfGraph();
  virtual ~RndfGraph();

  RndfVertex * addVertex(double lat, double lon, double x, double y, std::string name, int id = -1);
  RndfEdge * addEdge(RndfVertex * v1, RndfVertex * v2, std::string name, bool isLane, bool isLaneChange, double minSpeed, double maxSpeed, bool isOffroad, double weight, int id);
//  RndfEdge * addEdge(int v1, int v2, std::string name, bool isLane, double weight, int id);
  RndfIntersection * addIntersection(int id);
  const IntersectionMap& intersections() const {return intersectionMap;}


  void setCheckpoint(int number, RndfVertex * vertex);

  void addVirtualLane(RndfEdge * oldEdge, RndfVertex * firstVertex, RndfVertex * fromVertex, RndfVertex * toVertex, RndfVertex * lastVertex,
      std::string exitName, double minSpeed, double maxSpeed, RndfGraphBuilder * graphBuilder);

  RndfEdge * findEdge(const std::string name);
  RndfVertex * findVertex(const std::string name);

  RndfEdge * findClosestEdge(double x, double y);

  EdgeList * searchPath(int fromCheckpoint, int toCheckpoint, bool ignore_blockades = false);
  EdgeList * searchPath(RndfVertex * from, RndfVertex * to, bool ignore_blockades = false);

  RndfVertex* checkPoint(int id);

  void clearBlockades();


  void dumpEdges();
  void dumpVertices();

protected:
  virtual double searchHeuristic(RndfVertex const * vertex, RndfVertex const * goal);

  typedef std::map<int, RndfVertex *> CheckpointMap;
  CheckpointMap checkpointMap;
 public:
  IntersectionMap intersectionMap;
 protected:
  double maxSpeed;
  rndf::Counter intersection_id_count_;
  CurveSmoother cs_;
  double* xbez_, *ybez_;
  static const uint32_t max_bez_points_ = 1000;
};


inline bool pathBlocked(const RndfGraph::EdgeList& edges)
{
	if (&edges == NULL) return false;
	for (RndfGraph::EdgeList::const_iterator it = edges.begin(); it != edges.end(); ++it) {
		RndfEdge* edge = *it;
		if ( edge->isBlockedEdge() || edge->fromVertex()->isBlockedVertex() || edge->toVertex()->isBlockedVertex() )
			return true;
	}
	return false;
}

inline double calcRouteLength(const RndfGraph::EdgeList& edges)
{
	if (&edges == NULL) return false;
	double res = 0.;
	for (RndfGraph::EdgeList::const_iterator it = edges.begin(); it != edges.end(); ++it)
		res += (*it)->length();
	return res;
}


} // namespace RoutePlanner

} // namespace vlr

#endif
