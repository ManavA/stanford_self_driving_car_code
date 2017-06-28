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


#ifndef LASER_SEGMENT_H_
#define LASER_SEGMENT_H_

#include <vector>

#include <perception_types.h>
#include <segment.h>
#include <velodyne_rings.h>

#define MAX_SEGMENT_POINTS 1000

namespace perception {

class Perception;

class scan_segment {
private:
  unsigned int allocated_points;

public:
  scan_segment() : allocated_points(100), num_points(0), parent(NULL), rank(0) {
    points = (laser_point_p*)malloc(sizeof(laser_point_p) * allocated_points);
  }

  ~scan_segment() {
    free(points);
  }

  void clear() {
    num_points = 0;
    parent = NULL;
    rank = 0;
  }

  void grow() {
    if (allocated_points == 100)
      allocated_points = 1000;
    else
      allocated_points *= 2;

    points = (laser_point_p*)realloc(points, allocated_points * sizeof(laser_point_p));
  }

  void push_back(laser_point_p p) {
    if (num_points == allocated_points)
      grow();
    points[num_points] = p;
    num_points++;
  }

  laser_point_p front() {
    return points[0];
  }

  laser_point_p back() {
    return points[num_points - 1];
  }

  void add(scan_segment* segment) {
    unsigned int total_points = num_points + segment->num_points;
    if (total_points > allocated_points) {
      allocated_points = total_points;
      points = (laser_point_p*)realloc(points, allocated_points * sizeof(laser_point_p));
    }

    memcpy(points + num_points, segment->points, segment->num_points * sizeof(laser_point_p));
    num_points += segment->num_points;
  }

  unsigned int num_points;
  laser_point_p  * points;

  scan_segment* parent;
  int rank;
};
typedef scan_segment* scan_segment_p;

class scan_segments {
private:
  unsigned int allocated_segments;

public:
  scan_segment_p * segments;
  unsigned int     num_segments;

  scan_segments() : allocated_segments(500), num_segments(0) {
    segments = (scan_segment_p*)malloc(sizeof(scan_segment_p) * allocated_segments);
    for (unsigned int i = 0; i < allocated_segments; i++)
      segments[i] = NULL;
  }

  ~scan_segments() {
    for (unsigned int i=0; i < num_segments; i++) {
      free(segments[i]);
    }
    free(segments);
  }

  void clear() {
    num_segments = 0;
  }

  void grow() {
    unsigned old_size = allocated_segments;
    allocated_segments *= 2;
    segments = (scan_segment_p*)realloc(segments, allocated_segments * sizeof(scan_segment_p));
    for (unsigned int i = old_size; i < allocated_segments; i++)
      segments[i] = NULL;
  }

  scan_segment_p operator[] (unsigned int i) {
    return segments[i];
  }

  scan_segment_p new_segment() {
    num_segments ++;
    if (num_segments > allocated_segments) {
      printf("*** *** Growing from %d\n", allocated_segments);
      grow();
    }

    scan_segment_p segment = segments[num_segments - 1];
    if (segment == NULL) {
 //     printf("adding new segment\n");
      segment = new scan_segment();
      segments[num_segments - 1] = segment;
    } else {
      segment->clear();
    }
    return segment;
  }

  scan_segment_p pop_segment() {
    num_segments--;
    return segments[num_segments];
  }

};

class LaserSegmenter { //: public Segmenter {
/*private:
  laser_scan_p                lscan_;

//  std::vector<scan_segment_p> segments[NUM_LASER_BEAMS];
  scan_segments               segments[NUM_LASER_BEAMS];

  VelodyneRings*              rings_;
  Perception* perception_;

  void clear_segments();
  void combineSegments();
  void segmentBeams();
  void collectObstacles(std::vector< boost::shared_ptr<Obstacle> >* segmented_obstacles);
public:
  LaserSegmenter(VelodyneRings* rings, Perception* perception);
  virtual ~LaserSegmenter();

  void segmentVelo(laser_scan_p lscan, std::vector< boost::shared_ptr<Obstacle> >* segments);
*/};

} // namespace perception

#endif // LASER_SEGMENT_H_
