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

#include <aw_Graph.h>

using namespace std;

namespace vlr {

namespace RoutePlanner {

/*
  Edge::Edge(Vertex * from, Vertex * to, double weight, int id)
    : visited( false ), from(from), to(to), id(id), weight(weight), annotatedEdge(0)
  {
  }

  Vertex * Graph::addVertex() {
    return addVertex(idCount.getNextId());
  }

  Vertex * Graph::addVertex(int id) {
    Vertex * v = new Vertex(id);
    vertexMap.insert(make_pair(id, v));
    return v;
  }

  Edge * Graph::connect(Vertex * v1, Vertex * v2, double weight) {
    return connect(v1, v2, weight, idCount.getNextId());
  }

  Edge * Graph::connect(Vertex * v1, Vertex * v2, double weight, int edgeId) {
    Edge * edge = new Edge(v1, v2, weight, edgeId);
    v1->addEdge(edge);
    edgeMap.insert(make_pair(edgeId, edge));
    return edge;
  }

  Vertex * Graph::getVertex(int id) {
    return vertexMap.find(id)->second;
  }

  Edge * Graph::edge(int id) {
    return edgeMap.find(id)->second;
  }

  Graph::~Graph() {
    for (VertexMap::iterator vit = vertexMap.begin(); vit != vertexMap.end(); ++vit) {
      delete vit->second;
    }
    for (EdgeMap::iterator eit = edgeMap.begin(); eit != edgeMap.end(); ++eit) {
      delete eit->second;
    }
  }
  */

}

} // namespace vlr
