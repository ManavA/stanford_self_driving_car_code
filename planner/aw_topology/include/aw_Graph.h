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


#ifndef AW_GRAPH_H
#define AW_GRAPH_H

#include <list>
#include <map>
#include <set>
#include <vector>
#include <assert.h>
#include <vlrException.h>
#include <aw_Counter.h>

namespace vlr {

namespace RoutePlanner {

class AnnotatedRouteEdge;

template<class EdgeT> class Vertex {
public:
	typedef std::list<EdgeT *> EdgeList;
	typedef typename EdgeList::const_iterator	EdgeIterator;
	typedef std::set<EdgeT*> TEdgeSet;

	Vertex(int id = -1) : id(id) {}
	virtual ~Vertex() {}

	// deprecated interface
	void addEdge(EdgeT* edge) { assert(edge); assert(edge > (EdgeT*)0x040000); edgeList.push_back(edge); }
	void removeEdge(EdgeT* edge) { edgeList.remove(edge); }
	typename EdgeList::iterator beginEdges() { return edgeList.begin(); }
	typename EdgeList::iterator endEdges() { return edgeList.end();	}
	EdgeIterator beginEdges() const { return edgeList.begin(); }
	EdgeIterator endEdges() const {	return edgeList.end(); }
	int getNumberOfEdges() const { return edgeList.size(); }

	// new interface
	void addInEdge(EdgeT* edge)					{ assert(edge); assert(edge > (EdgeT *)0x040000); m_inEdges.insert(edge); }
	void removeInEdge(EdgeT* edge)				{ assert(edge); m_inEdges.erase(edge); }
	bool hasInEdge(EdgeT* edge) const		{ return m_inEdges.find(edge) != m_inEdges.end(); }
	size_t numInEdges() const				{ return m_inEdges.size(); }
	const TEdgeSet& inEdges() const			{ return m_inEdges; }

	void addOutEdge(EdgeT* edge)				{ assert(edge); assert(edge > (EdgeT *)0x040000); m_outEdges.insert(edge); }
	void removeOutEdge(EdgeT* edge)				{ assert(edge); m_outEdges.erase(edge); }
	bool hasOutEdge(EdgeT* edge) const			{ return m_outEdges.find(edge) != m_outEdges.end(); }
	size_t numOutEdges() const			{ return m_outEdges.size(); }
	const TEdgeSet& outEdges() const			{ return m_outEdges; }


	int getId() const {	return id; }

protected:
	EdgeList edgeList;
	TEdgeSet m_outEdges;
	TEdgeSet m_inEdges;

	int id;
};

template<class VertexT> class Edge {
public:
	Edge(VertexT * from, VertexT * to, double weight = 1., int id = -1)
	: visited_( false ), from(from), to(to), id(id), weight(weight)/*, annotatedEdge(0)*/
	{
		assert(weight > 0.); // has to be greater than 0. otherwise the AStar Route Planner fails
	}
	virtual ~Edge() { }

	void deleteVertices() {
		delete to;
		to = 0;

		delete from;
		from = 0;
	}

	virtual VertexT * fromVertex() {
		return from;
	}
	virtual VertexT * toVertex() {
		return to;
	}
	virtual VertexT const * fromVertex() const {
		return from;
	}
	virtual VertexT const * toVertex() const {
		return to;
	}
	int getId() const {
		return id;
	}
	double getWeight() const {
		return weight;
	}

 bool visited() const {return visited_;}
  void setVisited(bool visited = true) {visited_ = visited;}

	/* this is evil :) RndfEdge <-> AnnotetedRouteEdge is no 1:1 relation
	// get link to annotation if available (if edge is in route graph)
	AnnotatedRouteEdge* getAnnotatedEdge() {
		return annotatedEdge;
	}
	const AnnotatedRouteEdge* getAnnotatedEdge() const {
		return annotatedEdge;
	}
	void setAnnotatedEdge(AnnotatedRouteEdge* annoEdge) {
		annotatedEdge=annoEdge;
	}
	*/



protected:
	bool visited_;
	VertexT* from;
	VertexT* to;
	int id;
	double weight;
//	AnnotatedRouteEdge* annotatedEdge;
};

template<class VertexT, class EdgeT> class Graph {
public:
	typedef std::map<int, VertexT*> VertexMap;
	typedef std::map<int, EdgeT*> EdgeMap;
	typedef std::list<EdgeT*> EdgeList;
	typedef std::set<EdgeT*> EdgeSet;
  typedef std::vector<EdgeT*> EdgeId2EdgeMap;
  typedef std::vector<VertexT*> VertexId2EdgeMap; // TODO: verify

	Graph() {}
	virtual ~Graph();

	VertexT * addVertex();
	VertexT * addVertex(int id);
	EdgeT * connect(VertexT * v1, VertexT * v2, double weight = 1.);
	EdgeT * connect(VertexT * v1, VertexT * v2, double weight, int edgeId);
	void removeEdge(EdgeT * edge);

	VertexT * getVertex(int id);
	EdgeT * edge(int id);

	EdgeT * findIncomingEdge(VertexT * to);
	int countIncomingEdge(VertexT * to);

	EdgeList * searchPath(VertexT * from, VertexT * to, bool ignore_blockades = false);

	// returns a set of edges that are according to the graph
	EdgeSet edges();
//	const EdgeMap& edges() const {return edgeMap;}

protected:
	virtual double searchHeuristic(const VertexT* /*vertex*/, const VertexT* /*goal*/) {
		return 0.;
	}

public:
	EdgeMap edgeMap;
	EdgeId2EdgeMap edges_;
protected:
	VertexMap vertexMap;
	VertexId2EdgeMap vertices_;
	rndf::Counter vertex_id_count_, edge_id_count_;
};

template<class VertexT, class EdgeT>
VertexT * Graph<VertexT, EdgeT>::addVertex() {
	return addVertex(vertex_id_count_.getNextId());
}

template<class VertexT, class EdgeT> VertexT * Graph<VertexT, EdgeT>::addVertex(int id) {
	VertexT * v = new VertexT(id);
	vertexMap.insert(make_pair(id, v));
	return v;
}

template<class VertexT, class EdgeT>
EdgeT* Graph<VertexT, EdgeT>::connect(VertexT * v1, VertexT * v2, double weight) {
	return connect(v1, v2, weight, edge_id_count_.getNextId());
}

template<class VertexT, class EdgeT>
EdgeT* Graph<VertexT, EdgeT>::connect(VertexT * v1, VertexT * v2, double weight, int edgeId)
{
	assert( edgeMap.find(edgeId) == edgeMap.end() );
	assert(v1 && v2);

	EdgeT * edge = new EdgeT(v1, v2, weight, edgeId);
	v1->addEdge(edge);
	v1->addOutEdge(edge);
	v2->addInEdge(edge);
	edgeMap.insert(make_pair(edgeId, edge));

	return edge;
}

template<class VertexT, class EdgeT> VertexT * Graph<VertexT, EdgeT>::getVertex(int id) {
	typename VertexMap::iterator it = vertexMap.find(id);
	if (it == vertexMap.end()) {
		return NULL;
	} else {
		return it->second;
	}
}

template<class VertexT, class EdgeT> EdgeT * Graph<VertexT, EdgeT>::edge(int id) {
	typename EdgeMap::iterator it = edgeMap.find(id);
	if (it == edgeMap.end()) {
		return NULL;
	} else {
		return it->second;
	}
}

template<class VertexT, class EdgeT> EdgeT * Graph<VertexT, EdgeT>::findIncomingEdge(VertexT * to) {
	// #warning this looks sick. consider adapting vertex and edge types to allow backward traversal
	for (typename EdgeMap::iterator it = edgeMap.begin(); it != edgeMap.end(); ++it) {
		if (it->second->toVertex() == to) {
			return it->second;
		}
	}
	return NULL;
}

template<class VertexT, class EdgeT> int Graph<VertexT, EdgeT>::countIncomingEdge(VertexT * to) {
	int count=0;
	for (typename EdgeMap::iterator it = edgeMap.begin(); it != edgeMap.end(); ++it) {
		if (it->second->toVertex() == to) {
			++count;
		}
	}
	return count;
}

template<class VertexT, class EdgeT> Graph<VertexT, EdgeT>::~Graph() {
	for (typename VertexMap::iterator vit = vertexMap.begin(); vit != vertexMap.end(); ++vit) {
		delete vit->second;
	}
	for (typename EdgeMap::iterator eit = edgeMap.begin(); eit != edgeMap.end(); ++eit) {
		delete eit->second;
	}
}

template<class VertexT, class EdgeT> void Graph<VertexT, EdgeT>::removeEdge(EdgeT * edge) {
	assert(edge->fromVertex());
	assert(edge->toVertex());
	edge->fromVertex()->removeEdge(edge);
	edge->fromVertex()->removeOutEdge(edge);
	edge->toVertex()->removeInEdge(edge);
	//typename EdgeMap::iterator it = find(edgeMap.begin(), edgeMap.end(), edge->getId());
	typename EdgeMap::iterator it = edgeMap.find(edge->getId());
	assert(it != edgeMap.end());
	edgeMap.erase(it);
	delete edge;
}

template<class VertexT, class EdgeT> typename Graph<VertexT, EdgeT>::EdgeSet Graph<VertexT, EdgeT>::edges() {
	EdgeSet edges;
	for (typename EdgeMap::iterator it = edgeMap.begin(); it != edgeMap.end(); ++it)
		edges.insert(it->second);
	return edges;
}

}

} // namespace vlr

#endif
