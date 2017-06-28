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


#include <laser_segment.h>
#include <utils.h>

using namespace std;

namespace drc = driving_common;

namespace perception {

inline int encoder_dist(int x, int y) {
  int de = x - y;
  if (de < -18000) de += 36000;
  if (de > 18000) de -= 36000;
  return de;
}

inline bool encoder_left(int x, int y) {
  return (encoder_dist(x,y) < 0);
}

inline bool points_close(laser_point_p prev, laser_point_p point) {
//  if (prev->obstacle != point->obstacle)
//    return false;

//  if (abs(prev->range - point->range) > 50)
//    return false;

  int dx = prev->point->x - point->point->x;
  int dy = prev->point->y - point->point->y;
  return ((dx*dx + dy*dy) < (1200)); // within 20 cm (20*20 = 400)

//    return false;

//  int de = abs(encoder_dist(point->encoder, prev->encoder));
//  if (de > 36) { // 18 is typical value for consecutive scans
//    printf("encoder (%d - %d = %d or %d)\n", point->encoder, prev->encoder, point->encoder - prev->encoder, de);
//    return false;
//  }

//  return true;
}

scan_segment_p segment_find(scan_segment_p x)
{
  if (x->parent == NULL)
    return x;

  x->parent = segment_find(x->parent);
  return x->parent;
}

void print_segment(scan_segment_p x) {
  printf("parent: %p\trank: %d\n", x->parent, x->rank);
  unsigned int n = x->num_points;
  for (unsigned int i=0; i < n; i++) {
    laser_point_p p = x->points[i];
    printf("  e:%d\tr:%d\tx:%d\ty:%d\tz:%d\n", p->encoder, p->range, p->point->x, p->point->y, p->point->z);
  }
}

void segment_union(scan_segment_p x, scan_segment_p y)
{
  scan_segment_p xRoot = segment_find(x);
  scan_segment_p yRoot = segment_find(y);
  if (xRoot->rank > yRoot->rank) {
    yRoot->parent = xRoot;
  }
  else if (xRoot->rank < yRoot->rank) {
    xRoot->parent = yRoot;
  }
  else if (xRoot != yRoot) {
    yRoot->parent = xRoot;
    xRoot->rank = xRoot->rank + 1;
  }
}

inline bool segments_overlap_obstacle(scan_segment_p x, scan_segment_p y) {
  return (x->front()->obstacle == y->front()->obstacle);
}

inline bool segments_overlap_angle(scan_segment_p x, scan_segment_p y)
{
  return (encoder_left(x->front()->encoder, y->back()->encoder) && encoder_left(y->front()->encoder, x->back()->encoder));
}

bool segments_overlap_range(scan_segment_p x, scan_segment_p y)
{
  // TODO: optimize this to take advantage of points ordering
//  const unsigned short max_delta_range = 25;
  unsigned int y_n = y->num_points;
  unsigned int x_n = x->num_points;
  for (unsigned int i = 0; i < y_n; i++) {
    for (unsigned int j = 0; j < x_n; j++) {
      if (points_close(y->points[i], x->points[j]))
        return true;
    }
  }

//  for (scan_segment::iterator y_it = y->begin(), y_end = y->end(); y_it != y_end; y_it++) {
//    for (scan_segment::iterator x_it = x->begin(), x_end = x->end(); x_it != x_end; x_it++) {
//      if (points_close(*x_it, *y_it))
//        return true;
//    }
//  }
  return false;


//  scan_segment::iterator y_it = y->begin();
//  scan_segment::iterator x_it = x->begin();
//  laser_point_p x_prev = NULL;
//  while (y_it != y->end()) {
//    while ( ( x_it != x->end() ) && encoder_left((*x_it)->encoder, (*y_it)->encoder)) {
//      x_prev = *x_it;
//      x_it++;
//    }
//    if ( x_it == x->end() )
//      break;
//
//    if ((points_close(*x_it, *y_it)) ||
//        (x_prev && points_close(x_prev, *y_it)))
//      return true;
//
//    y_it++;
//  }
//  return false;

//  const unsigned short max_delta_range = 25;
//  scan_segment::iterator y_it = y->begin();
//  scan_segment::iterator x_it = x->begin();
//
//  laser_point_p x_prev = NULL;
//  while (y_it != y->end()) {
//    while ( ( x_it != x->end() ) && encoder_left((*x_it)->encoder, (*y_it)->encoder)) {
//      x_prev = *x_it;
//      x_it++;
//    }
//    if ( x_it == x->end() )
//      break;
//
//    if ((points_close(*x_it, *y_it)) ||
//        (x_prev && points_close(x_prev, *y_it)))
//      return true;
//
//    y_it++;
//  }
//  return false;
//
////  while ((x_it != x->end()) && ((*x_it)->encoder < (*y_it)->encoder)) {
//  while ( ( x_it != x->end() ) && encoder_left((*x_it)->encoder, (*y_it)->encoder)) {
//    x_it++;
//  }
//
////  while ( (y_it != y->end()) &&
////          (encoder_dist((*y_it)->encoder, (*x_it)->encoder) < -18)) {
////    y_it++;
////  }
//
//  while ((y_it != y->end()) && (x_it != x->end()) && ( abs(encoder_dist((*x_it)->encoder, (*y_it)->encoder))) < 40) {
////    if (points_close(*x_it, *y_it))
////      return true;
//    unsigned short delta_range = fabs((*x_it)->range - (*y_it)->range);
//    if (delta_range < max_delta_range) {
//      return true;
//    }
//    x_it++;
//    y_it++;
//  }
//  return false;
}

// return true if x is far left of y
inline bool segment_far_left(scan_segment_p x, scan_segment_p y) {
  return (encoder_dist(x->back()->encoder, y->front()->encoder) < -500);
}

// return true if x is far right of y
inline bool segment_far_right(scan_segment_p x, scan_segment_p y) {
  return (encoder_dist(x->front()->encoder, y->back()->encoder) > 500);
}

inline bool segments_overlap(scan_segment_p x, scan_segment_p y) {
//  return (segments_overlap_obstacle(x,y) && segments_overlap_angle(x,y) && segments_overlap_range(x,y));
  bool angle = segments_overlap_angle(x,y);
  if (angle)
    return segments_overlap_range(x,y);
  else
    return (points_close(x->front(), y->back()) || points_close(y->front(), x->back()));
}

LaserSegmenter::LaserSegmenter(VelodyneRings* rings, Perception* perception) : Segmenter(perception),
  rings_(rings)
{

}

LaserSegmenter::~LaserSegmenter() {

}

void LaserSegmenter::combineSegments() {
  int comparisons = 0;

  #pragma omp parallel for
  for(int l = 1; l<NUM_LASER_BEAMS; l++) {
    int idx = rings_->beamToIndex(l); // ring[l].idx;
    int l_idx = rings_->beamToIndex(l-1); // ring[l-1].idx;
    bool u = false;

    unsigned int first_segment = 0;
    unsigned int s1 = segments[idx].num_segments;
    unsigned int s2 = segments[l_idx].num_segments;
    for (unsigned int i = 0; i < s1; i++) {
      scan_segment_p x = segments[idx][i];
      for (unsigned int j = first_segment; j < s2; j++) {
        comparisons++;
        scan_segment_p y = segments[l_idx][j];
        if (segment_far_right(x,y))
          first_segment = j;
        if (segment_far_left(x,y))
          break;
        if (segments_overlap(x, y)) {
          segment_union(x, y);
          u = true;
        }
      }
    }

    // TODO: do we need this check anymore?
//    if (!u && (l > 1)) {
//      int l_idx = rings_->beamToIndex(l-2); // ring[l-2].idx;
//      for (unsigned int i = 0; i<segments[idx].size(); i++) {
//        scan_segment_p x = segments[idx][i];
//        for (unsigned int j=0; j<segments[l_idx].size(); j++) {
//          scan_segment_p y = segments[l_idx][j];
//          if (segments_overlap(x, y)) {
//            segment_union(x, y);
//          }
//        }
//      }
//    }
  }
//  printf("comparisons: %d\n", comparisons);

//  int object_count = 0;
//  int segment_count = 0;
//  for(int l = 1; l<NUM_LASER_BEAMS; l++) {
//    for (int i = 0; i<segments_[l].size(); i++) {
//      scan_segment_p x = segments_[l][i];
//      segment_count ++;
//      if (x->parent == NULL)
//        object_count++;
//    }
//  }
//
//  printf("%d segments_, %d objects\n", segment_count, object_count);
}

void LaserSegmenter::clear_segments() {
  for (int l=0; l<NUM_LASER_BEAMS; l++) {
//    unsigned int s = segments[l].size();
//    for (unsigned int i=0; i < s; i++) {
//      delete segments[l].at(i);
//    }
    segments[l].clear();
  }
}

void LaserSegmenter::segmentBeams()
{
  int l;
  clear_segments();

  int segment_count = 0;
  #pragma omp parallel for
  for(l=0; l<NUM_LASER_BEAMS; l++) {

    laser_point_p prev_point = NULL;
    laser_point_p point = NULL;
    scan_segment_p segment = segments[l].new_segment();

    // find first point
    int num_points = lscan_[l].num_points;
    int i;
    for (i=0; i<num_points; i++) {
      point = &lscan_[l].laser_point[i];
      if (point->obstacle) {
        segment->push_back(point);
//        printf("adding %d %d (%d %d)\n", l, i, point->point->x, point->point->y);
        prev_point = point;
        break;
      }
    }

    for(i++; i<lscan_[l].num_points; i++) {
      point = &lscan_[l].laser_point[i];
      if (point->obstacle) {
        if (points_close(prev_point, point)) {
          segment->push_back(point);
        } else {
          segment_count++;

          if (segment->num_points > 3)
            segment = segments[l].new_segment();
          else
            segment->clear();

          segment->push_back(point);
        }
        prev_point = point;
      }
    }

    if (segment->num_points < 4)
      segments[l].pop_segment();
  }

//  printf("segments: %d\n", segment_count);
}

void LaserSegmenter::collectObstacles(std::vector< boost::shared_ptr<Obstacle> >* segmented_obstacles) {
  int l;

  static int observation_id = 0;

  for(l = 0; l<NUM_LASER_BEAMS; l++) {
    unsigned int s = segments[l].num_segments;
    for (unsigned int i = 0; i < s; i++) {
      scan_segment_p x = segments[l][i];
      if (x->parent != NULL) {
        scan_segment_p y = segment_find(x);
        y->add(x);
        x->clear();
      }
    }
  }

  #pragma omp parallel for
  for(l = 0; l<NUM_LASER_BEAMS; l++) {
    unsigned int s = segments[l].num_segments;
    for (unsigned int i = 0; i < s; i++) {
      scan_segment_p x = segments[l][i];
      if (x->parent == NULL) {
        int count = x->num_points;
        if ((count > 20)) {
          boost::shared_ptr<LaserObstacle>* current_obstacle = new boost::shared_ptr<LaserObstacle>(new LaserObstacle(observation_id, perception_));
          observation_id++;
//          current_obstacle->get()->timestamp = x->points[0]->scan->timestamp;
          for (int j=0; j < count; j++) {
            current_obstacle->get()->addPoint(x->points[j]);
//            printf("adding point %d %d\n", x->points[j]->point->x, x->points[j]->point->y);
          }

          #pragma omp critical
          {
            segmented_obstacles->push_back(*current_obstacle);
          }
        }
      }
    }
  }
}

void LaserSegmenter::segmentVelo(laser_scan_p lscan, std::vector< boost::shared_ptr<Obstacle> >* segmented_obstacles)
{
  lscan_ = lscan;

  segmented_obstacles->clear();

  double time = drc::Time::current();

  segmentBeams();
  display_time("Segment Beams", time); time = drc::Time::current();

  combineSegments();
  display_time("Combine Beams", time); time = drc::Time::current();

  collectObstacles(segmented_obstacles);
  display_time("Collect Obst", time);

//  printf("collected %d obstacles\n", segmented_obstacles->size());
}


} // namespace perception
