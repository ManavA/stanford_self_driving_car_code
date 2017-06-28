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


#define DEBUG_LEVEL 0

#include <float.h>
#include <aw_geometry.h>
#include <aw_laneQuadTree.h>
#include <aw_roadNetwork.h>

using namespace std;
using namespace Eigen;

namespace vlr {

namespace rndf {

LaneQuadTree::LaneQuadTree(const RoadNetwork* rndf, double min_radius, int min_elements) :
  rndf_(rndf), tile_id_(1) {
  Vector2d bb_min(DBL_MAX, DBL_MAX);
  Vector2d bb_max(-DBL_MAX, -DBL_MAX);
  ElementList elements;
  Element* element;

  // calculate bounding box and create elements
  int k = 0;
  Exit* e;
  TSegmentMap::iterator sit, sit_end;
  TLaneSet::const_iterator lit, lit_end;
  TWayPointVec::iterator wit, wit2, wit_end;
  TExitMap::iterator eit, eit_end;

  TSegmentMap segments = rndf_->segments();
  for (sit = segments.begin(), sit_end = segments.end(); sit != sit_end; ++sit) {
    const TLaneSet& lanes = sit->second->getLanes();
    for (lit = lanes.begin(), lit_end = lanes.end(); lit != lit_end; ++lit) {
      TWayPointVec waypoints = (*lit)->wayPoints();
      int num_waypoints = (int) waypoints.size();
      for (wit = waypoints.begin(), wit_end = waypoints.end(), k = 0; wit != wit_end; ++wit, ++k) {
        // calculate bounding box
        WayPoint* w = *wit;
        bb_min.x() = min(bb_min.x(), w->utmX());
        bb_max.x() = max(bb_max.x(), w->utmX());
        bb_min.y() = min(bb_min.y(), w->utmY());
        bb_max.y() = max(bb_max.y(), w->utmY());
        // create elements
        if (k < num_waypoints - 1) {
          wit2 = wit;
          wit2++;
          element = new Element(sit->second, *lit, *wit, *wit2);
          elements.push_back(element);
        } // k<num_waypoints-1

        // add exiting waypoints as new segments
        TExitMap exits = (*wit)->exits();
        for (eit = exits.begin(), eit_end = exits.end(); eit != eit_end; ++eit) {
          e = eit->second;
          if (e->exitType() != Exit::LaneToLane) continue;
          element = new Element(sit->second, *lit, *wit, e->getExitToLane());
          elements.push_back(element);
        } // exits
      } // waypoints
    } // lanes
  } // segments

  // set members
  center_ = bb_min + (bb_max - bb_min) / 2.0;
  Vector2d bb_size = bb_max - bb_min;
  radius_ = std::max(bb_size.x(), bb_size.y());
  parent_index_ = 0;
  parent_tree_ = NULL;

  // initialize quadtree
  initialize(elements, min_radius, min_elements);
}

LaneQuadTree::LaneQuadTree(const RoadNetwork* rndf, Vector2d& center, double radius, const ElementList& elements, double min_radius, int min_elements,
    int parent_index, LaneQuadTree* parent_tree) :
  rndf_(rndf), center_(center), radius_(radius), parent_index_(parent_index), parent_tree_(parent_tree) {
  if (parent_tree_) tile_id_ = parent_tree_->tile_id() * 10 + parent_index;
  else tile_id_ = 1;

  initialize(elements, min_radius, min_elements);
}

LaneQuadTree::LaneQuadTree(const LaneQuadTree& lqt, LaneQuadTree* parent_tree):
		center_ (lqt.center_) {
  rndf_ = lqt.rndf_;

  radius_ = lqt.radius_;
  parent_index_ = lqt.parent_index_;
  parent_tree_ = parent_tree;
  is_leaf_ = lqt.is_leaf_;
  tile_id_ = lqt.tile_id_;

  if (lqt.is_leaf_) {
      ElementList::const_iterator it, it_end;
      for (it = lqt.elements_.begin(), it_end = lqt.elements_.end(); it != it_end; ++it) {
        Element* element = *it;
        if (element) {
          elements_.push_back(new Element(*element));
        }
      }
    }
  else {
    for(int32_t i=0; i<4; i++) {
      if(lqt.child_[i]) {
        child_[i] = new LaneQuadTree(*lqt.child_[i], this);
      }
      else {
        child_[i] = NULL;
      }
    }
  }
}

LaneQuadTree::~LaneQuadTree() {
  if (is_leaf_) {
    ElementList::const_iterator it, it_end;
    for (it = elements_.begin(), it_end = elements_.end(); it != it_end; ++it) {
      Element* element = *it;
      if (element) {
        // TODO: keep a list of elements in all trees. This is easier to to free
        //       because this doesn't work!
        //        delete(element);
        //        element=NULL;
      }
    }
  }
  else {
    for (int i = 0; i < 4; i++) {
      if (child_[i]) {
        delete child_[i]; child_[i]=NULL;
      }
    }
  }
}

void LaneQuadTree::initialize(const ElementList& elements, double min_radius, int min_elements) {
  bool subdivided = false;
  double element_x[2];
  double element_y[2];
  double center_x = center_(0);
  double center_y = center_(1);

  child_[0] = child_[1] = child_[2] = child_[3] = NULL;

  elements_.clear();
  is_leaf_ = false;

  // check whether we need to subdivide more
  if ((radius_ < 2.0 * min_radius) || elements.size() == 1) {
    is_leaf_ = true;
    elements_ = elements;
    child_[0] = child_[1] = child_[2] = child_[3] = NULL;
    return;
  }

  // if so, try for triangle and each child whether they intersect
  double half_radius = radius_ * .5;
  Vector2d child_center;

  ElementList xp, xm; // x plus, x minus
  ElementList yp, ym; // y plus, y minus
  ElementList uk; // unknown (elements going across splits)
  xp.reserve(elements.size());
  xm.reserve(elements.size());
  uk.reserve(elements.size());

  // split about x axis
  int cnt;
  ElementList::const_iterator it, it_end;
  for (it = elements.begin(), it_end = elements.end(); it != it_end; ++it) {
    element_x[0] = (*it)->w1->utmX();
    element_x[1] = (*it)->w2->utmX();

    cnt = ((element_x[0] >= center_x) + (element_x[1] >= center_x));
    if (cnt == 2) {
      xp.push_back(*it);
    }
    else if (cnt == 0) {
      xm.push_back(*it);
    }
    else {
      uk.push_back(*it);
    }
  }

  // split about y axis
  ElementList &xv = xm;
  for (int i = 0; i < 2; i++) {
    if (i) {xv = xp;} else {xv = xm;}
    yp.clear();
    yp.reserve(xv.size());
    ym.clear();
    ym.reserve(xv.size());

    for (it = xv.begin(), it_end = xv.end(); it != it_end; ++it) {
      element_y[0] = (*it)->w1->utmY();
      element_y[1] = (*it)->w2->utmY();

      cnt = ((element_y[0] >= center_y) + (element_y[1] >= center_y));
      if (cnt == 2) {
        yp.push_back(*it);
      }
      else if (cnt == 0) {
        ym.push_back(*it);
      }
      else {
        uk.push_back(*it);
      }
    }

    // check the unknowns
    child_center(0) = center_(0) + (i ? half_radius : -half_radius);
    child_center(1) = center_(1) - half_radius;
    ElementList::const_iterator it1, it1_end;
    for (it = uk.begin(), it_end = uk.end(); it != it_end; ++it) {
      if (insideQuad(child_center, half_radius, *it)) {ym.push_back(*it);}
    }

    // create the children, if needed
    int k = i;
    if (ym.size() > 0) {
      child_[k] = new LaneQuadTree(rndf_, child_center, half_radius, ym, min_radius, min_elements, k, this);
      subdivided = true;
    }
    else {
      child_[k] = NULL;
    }

    // create the children, if needed
    child_center(1) = center_(1) + half_radius;
    k += 2;
    for (it = uk.begin(), it_end = uk.end(); it != it_end; ++it) {
      if (insideQuad(child_center, half_radius, *it)) yp.push_back(*it);
    }

    if (yp.size() > 0) {
      child_[k] = new LaneQuadTree(rndf_, child_center, half_radius, yp, min_radius, min_elements, k, this);
      subdivided = true;
    }
    else {
      child_[k] = NULL;
    }
  }

  // no subdivision made
  if (!subdivided) {
    elements_ = elements;
    is_leaf_ = true;
    child_[0] = child_[1] = child_[2] = child_[3] = NULL;
  }
}

bool LaneQuadTree::insideQuad(const Vector2d& center, double radius, Element* element) {
  Vector2d t1(element->w2->utmX(), element->w2->utmY());
  Vector2d t2(element->w1->utmX(), element->w1->utmY());
  bool outside = line_outside_of_rect(center, radius * 2.0, t1, t2);
  return (!outside);
}

bool LaneQuadTree::closer_to_element(const Vector2d& p, Element* element, double& distance, Vector2d& cp) {
  Vector2d a(element->w1->utmX(), element->w1->utmY());
  Vector2d b(element->w2->utmX(), element->w2->utmY());
  return closer_on_line(p, a, b, distance, cp);
}

bool LaneQuadTree::search_nearest_neighbor(const Vector2d& p, Element*& element, double& distance, Vector2d& closest_point) {
  // is this a leaf node?
  if (is_leaf_) {
    // look for a new closest distance
    ElementList::const_iterator it, it_end;
    for (it = elements_.begin(), it_end = elements_.end(); it != it_end; ++it) {
      // check if distance to this element is smaller
      if (closer_to_element(p, *it, distance, closest_point)) element = *it;
    }
    return circle_within_bounds(p, sqrt(distance), center_, radius_);
  }

  // which child contains p?
  int iChild = 2 * (center_(1) < p(1)) + (center_(0) < p(0));
  // check that child first
  if (child_[iChild] && child_[iChild]->search_nearest_neighbor(p, element, distance, closest_point)) return true;

  // now see if the other children need to be checked
  for (int i = 0; i < 4; i++) {
    if (i == iChild) continue;
    if (child_[i]) {
      if (bounds_overlap_circle(p, sqrt(distance), child_[i]->center_, child_[i]->radius_)) {
        if (child_[i]->search_nearest_neighbor(p, element, distance, closest_point)) return true;
      }
    }
  }
  return circle_within_bounds(p, sqrt(distance), center_, radius_);
}

void LaneQuadTree::get_tiles(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& points, std::vector<int>& edges, std::vector<unsigned long long> &tile_ids) {
  if (is_leaf_) {
    int s = points.size();
    points.reserve( points.size()+4 );
    points.push_back(center_ + Vector2d(-radius_, -radius_));//0
    points.push_back(center_ + Vector2d(radius_, -radius_));//1
    points.push_back(center_ + Vector2d(radius_, radius_));//2
    points.push_back(center_ + Vector2d(-radius_, radius_));//3

    edges.reserve( edges.size()+8 );
    edges.push_back(s + 0);
    edges.push_back(s + 1);
    edges.push_back(s + 1);
    edges.push_back(s + 2);
    edges.push_back(s + 2);
    edges.push_back(s + 3);
    edges.push_back(s + 3);
    edges.push_back(s + 0);

    tile_ids.push_back(tile_id_);
    return;
  }
  for (int i = 0; i < 4; i++) {
    if (child_[i]) child_[i]->get_tiles(points, edges, tile_ids);
  }
}

void LaneQuadTree::get_elements(ElementList& elements) {
  if (is_leaf_) {
    ElementList::const_iterator it, it_end;
    for (it = elements_.begin(), it_end = elements_.end(); it != it_end; ++it)
      elements.push_back(*it);
    return;
  }
  for (int i = 0; i < 4; i++) {
    if (child_[i]) child_[i]->get_elements(elements);
  }
}

} // namespace rndf

} // namespace vlr
