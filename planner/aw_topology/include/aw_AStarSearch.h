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


#ifndef AW_A_STAR_SEARCH_H
#define AW_A_STAR_SEARCH_H

#include <queue>
#include <list>
#include <iostream>
#include <algorithm>
#include <functional>
#include <assert.h>

#include <aw_Graph.h>
#include <aw_RndfGraph.h>

namespace vlr {

namespace RoutePlanner {

template <class VertexT, class EdgeT>
struct AStarInfo {
	AStarInfo(VertexT * v, double pathLength, double heuristicValue, EdgeT * previousEdge, AStarInfo<VertexT, EdgeT> * previous)
	: pathLength(pathLength), heuristicValue(heuristicValue), totalScore(0), vertex(v), incomingEdge(previousEdge), previous(previous)
	{
		totalScore = pathLength + heuristicValue;
	}

	double pathLength, heuristicValue, totalScore;
	VertexT * vertex;
	EdgeT * incomingEdge;
	AStarInfo<VertexT, EdgeT> * previous;
};

template <class VertexT, class EdgeT>
void cleanupAStarInfo(std::list<AStarInfo<VertexT, EdgeT> *> & list) {
	for (typename std::list<AStarInfo<VertexT, EdgeT> *>::iterator it = list.begin(); it != list.end(); ++it) {
		delete *it;
	}
}

template <class VertexT, class EdgeT>
struct AStarInfoComparator : public std::binary_function<AStarInfo<VertexT, EdgeT> *, AStarInfo<VertexT, EdgeT> *, bool> {
	bool operator()(const AStarInfo<VertexT, EdgeT> * node1, const AStarInfo<VertexT, EdgeT> * node2) {
		//std::cout << "comp(" << node1->totalScore << ", " << node2->totalScore << ")";
		return node1->totalScore > node2->totalScore;
	}
};

template < class T, class Comparator >
class Heap {
public:
	T front() {
		return heap.front();
	}
	int size() {
		return heap.size();
	}
	bool empty() {
		return heap.empty();
	}
	bool isHeap() {
		int size = heap.size();
		//std::cout << std::endl;
		//for (int i=1; i<=size; ++i) {
		//std::cout << heap[i-1]->totalScore << " " << std::flush;
		//}
		for (int i = 1; i < size; ++i) {
			int j = 2*i;
			if (j < size && comparator(heap[i-1], heap[j])) {
				return false;
			}
			--j;
			if (j < size && comparator(heap[i-1], heap[j])) {
				return false;
			}
		}
		return true;
	}
	void pop_front() {
		//assert(isHeap());
		int size = heap.size() - 1;
		heap[0] = heap[size];
		heap.pop_back();
		int i = 1;
		while (i < size) {
			int j = 2*i;
			bool c1 = (j-1 < size) && comparator(heap[i-1], heap[j-1]);
			if (c1) {
				bool c2 = (j < size) && comparator(heap[j-1], heap[j]);
				if (c2) {
					T tmp = heap[i-1];
					heap[i-1] = heap[j];
					heap[j] = tmp;
					i = j+1;
				} else {
					T tmp = heap[i-1];
					heap[i-1] = heap[j-1];
					heap[j-1] = tmp;
					i = j;
				}
			} else {
				bool c3 = (j < size) && comparator(heap[i-1], heap[j]);
				if (c3) {
					T tmp = heap[i-1];
					heap[i-1] = heap[j];
					heap[j] = tmp;
					i = j+1;
				} else {
					assert(isHeap());
					return;
				}
			}
		}
		//assert(isHeap());
	}
	void push_back(T element) {
		//assert(isHeap());
		heap.push_back(element);
		int i = heap.size();
		while (i > 1) {
			int j = i/2;
			if (!comparator(heap[i-1], heap[j-1])) {
				T tmp = heap[i-1];
				heap[i-1] = heap[j-1];
				heap[j-1] = tmp;
			} else {
				assert(isHeap());
				return;
			}
			i = j;
		}
		//assert(isHeap());
	}
private:
	std::vector<T> heap;
	Comparator comparator;
};

template <class VertexT, class EdgeT>
typename Graph<VertexT, EdgeT>::EdgeList * Graph<VertexT, EdgeT>::searchPath(VertexT * from, VertexT * to, bool ignore_blockades) {
	//    AStarInfoComparator comparator;
	typedef AStarInfo<VertexT, EdgeT> AStarInfo;
	typedef AStarInfoComparator<VertexT, EdgeT> AStarInfoComparator;
	typedef std::map<const VertexT *, double>	TVisitedMap;

	Heap< AStarInfo *,  AStarInfoComparator > heap;
	std::list<AStarInfo *> old;
	TVisitedMap visited;
	heap.push_back(new AStarInfo(from, 0., searchHeuristic(from, to), NULL, NULL));

	while (!heap.empty()) {
		AStarInfo * current = heap.front();
//		 std::cout << "[AStar] p:"<< current->pathLength <<" h:"<< current->heuristicValue <<" t: "<< current->totalScore << "  "<< current->vertex->name() << "  "<< (current->incomingEdge ? current->incomingEdge->name() : "") << std::endl;

		old.push_back(current);
		heap.pop_front();

		// check if vertex was already reached on better path
		typename TVisitedMap::iterator it = visited.find( current->vertex );
		if ( current->vertex != to && it != visited.end() && it->second < current->pathLength ) continue;
		visited[current->vertex] = current->pathLength;

		if (current->vertex == to && current->pathLength > 0) {
			// goal reached, backtracing
			EdgeList * edgeList = new EdgeList();
			while (current->incomingEdge != NULL) {
				edgeList->push_front(static_cast<EdgeT *>(current->incomingEdge));
				current = current->previous;
			}
			cleanupAStarInfo(old);
			return edgeList;
		}

		// expand node
		for (typename VertexT::EdgeList::iterator it = current->vertex->beginEdges(); it != current->vertex->endEdges(); ++it) {
			EdgeT * edge = *it;

			assert(edge);

			// don't allow lane changes in intersection
			if ( current->incomingEdge && current->incomingEdge->intersection() && edge->isLaneChangeEdge() ) continue;

			// forbid perimeter points if not in zone
			if ( current->incomingEdge && current->incomingEdge->isZoneEdge() && current->incomingEdge->isVirtualEdge() && edge->isZoneEdge() && edge->isVirtualEdge() ) {
//				std::cout << "[AStar]   -> Skipped "<< edge->name() << "  coming from "<< current->incomingEdge->name() << std::endl;
				continue;
			}

			// don't allow blocked edges
			if ( edge->isBlockedEdge() && ! ignore_blockades ) continue;

			VertexT * vertex = edge->toVertex();
			assert(vertex != NULL);

			// don't expand blocked vertices
			if ( vertex != to && vertex->isBlockedVertex() && ! ignore_blockades ) continue;

//			if (visited[vertex]) continue;
			double heuristic = searchHeuristic(static_cast<const VertexT *>(vertex), to);
			heap.push_back(new AStarInfo(vertex, current->pathLength + edge->getWeight(), heuristic, edge, current));
//			 std::cout << "[AStar]   -> p:"<< current->pathLength + edge->getWeight() <<" h:"<< heuristic <<" t: "<< (current->pathLength + edge->getWeight() + heuristic) << "  "<< vertex->name() << "  "<< (edge ? edge->name() : "") << std::endl;
		}
	}
	cleanupAStarInfo(old);
	return NULL;
}

}

} // namespace vlr

#endif
