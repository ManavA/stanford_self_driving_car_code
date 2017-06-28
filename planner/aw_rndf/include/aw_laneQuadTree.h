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


#ifndef LANEQUADTREE_H_
#define LANEQUADTREE_H_

#include <vector>
#include <stack>
#include <sstream>
#include <limits>
#include <Eigen/StdVector>
#include <Eigen/Dense>

#include <aw_roadNetwork.h>

namespace vlr {

namespace rndf {

//class Segment;
//class Lane;
//class WayPoint;
//class RoadNetwork;

class LaneQuadTree {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
   struct Element {
    Element(Segment* new_s, Lane* new_l, WayPoint* new_w1, WayPoint* new_w2) : s(new_s), l(new_l), w1(new_w1), w2(new_w2) {}
    Segment* s;
    Lane* l;
    WayPoint* w1;
    WayPoint* w2;
  };
  typedef std::vector<Element*> ElementList;

  LaneQuadTree(const RoadNetwork* rndf, double min_radius = 0, int min_elements = 1);

  LaneQuadTree(const RoadNetwork* rndf, Eigen::Vector2d& center, double radius, const ElementList& elements, double min_radius = 0, int min_elements = 1,
      int parent_index = 0, LaneQuadTree* parent_tree = NULL);

  LaneQuadTree(const LaneQuadTree& lqt, LaneQuadTree* parent_tree=NULL);

  ~LaneQuadTree();

  bool search_nearest_neighbor(const Eigen::Vector2d& p, Element*& element, double& distance, Eigen::Vector2d& closest_p);

  bool is_leaf() {
    return is_leaf_;
  }
  unsigned long long tile_id() {
    return tile_id_;
  }

  void get_tiles(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& points, std::vector<int> &edges, std::vector<unsigned long long>& tile_ids);
  void get_elements(ElementList& elements);
private:
  void initialize(const ElementList& elements, double min_radius, int min_elements);
  bool closer_to_element(const Eigen::Vector2d& p, Element* element, double& distance, Eigen::Vector2d& closest_point);
  bool insideQuad(const Eigen::Vector2d& center, double radius, Element* element);

private:
  const RoadNetwork* rndf_;
  Eigen::Vector2d center_;
  double radius_;
  int parent_index_;
  LaneQuadTree* parent_tree_;
  LaneQuadTree* child_[4];
  bool is_leaf_;
  ElementList elements_;
  unsigned long long tile_id_;
};

}
;

} // namespace vlr

#endif // LANEQUADTREE_H_
