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


#ifndef SEGMENTER_H_
#define SEGMENTER_H_

#include <grid.h>

#include <perception.h>
#include <obstacle.h>
#include <tracked_obstacle.h>

namespace perception {
class Perception;

class Segmenter {
public:
  Segmenter(Perception& perception) : perception_(perception) {}
  virtual ~Segmenter() {}
  void segmentVelodyne(const std::vector<velodyne::Block>& blocks);

private:
  struct PerceptionLineSegment {
    int object_id;
    int num_noise_points;
    int is_ground;
    double last_range;
    double min_angle, max_angle;
    double mean_range, mean_z, mean_dist;
    std::vector<velodyne::ScanPoint*>  points;

    // for union-find
    PerceptionLineSegment* parent;
    int rank;
  };

protected:
  Perception& perception_;

private:
  static const int32_t num_beams_ = 64;
  static const uint8_t colors[19][3];

  PerceptionLineSegment* segments[num_beams_];
  int num_segments[num_beams_];

  void drawSegmentation();
  void segmentScan(const velodyne::Block& block);

};

class GridSegmenter : public Segmenter {

public:
  GridSegmenter(const segmentation_settings_t& segmentation_settings, Perception& perception);
  virtual ~GridSegmenter();

  void segmentGrid(dgc_perception_map_cells_t* obstacles, std::vector< boost::shared_ptr<Obstacle> >* regions, std::vector< boost::shared_ptr<TrackedObstacle> > tracks, double timestamp);

private:
  std::vector<boost::shared_ptr<Obstacle> >* regions_;
  std::vector<boost::shared_ptr<TrackedObstacle> > tracks_;
  const Grid<PerceptionCell>& grid_;
  segmentation_settings_t segmentation_settings_;
  double timestamp_;

  void clearRegions();
  void regionsToObstacles(std::vector<Obstacle*>& out);
  void labelNeighbors(const PerceptionCell& cell, GridObstacle* current_region);
  void findRegions(dgc_perception_map_cells_t* obstacles);
};

} // namespace perception

#endif // SEGMENTER_H_
