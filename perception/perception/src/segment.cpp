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


#include <segment.h>
#include <utils.h>

#include <tr1/memory>

using namespace std;

namespace perception {

GridSegmenter::GridSegmenter(const segmentation_settings_t& segmentation_settings, Perception& perception) : Segmenter(perception), grid_(perception.grid()), segmentation_settings_(segmentation_settings) {

}

GridSegmenter::~GridSegmenter() {

}

void GridSegmenter::clearRegions() {
  regions_->clear();
}

void GridSegmenter::findRegions(dgc_perception_map_cells_t* obstacles) {
  clearRegions();

//  printf("min points: %d, max points: %d, num obstacles: %i\n", segmentation_settings_.min_points, segmentation_settings_.max_points, obstacles->num);
  unsigned short region_id = 1;

  boost::shared_ptr<GridObstacle>* current_region = new boost::shared_ptr<GridObstacle>(new GridObstacle(region_id, grid_, perception_));

  // clear region labels
  for (int i=0; i < obstacles->num; i++) {
    obstacles->cell[i]->region = 0;
  }

  for (int i=0; i < obstacles->num; i++) {
    PerceptionCell* cell = obstacles->cell[i];
    if ((cell->region == 0) /*&& (cell_eval(cell) != PERCEPTION_MAP_OBSTACLE_FREE) */) {
      cell->region = region_id;

//      current_region->get()->timestamp = timestamp_;
      current_region->get()->addCell(*cell);
      labelNeighbors(*cell, current_region->get());

      int size = current_region->get()->getSize();
      if (size > segmentation_settings_.min_points &&
          size < segmentation_settings_.max_points &&
          (*current_region)->getPoints().size() != 0) {
        (*current_region)->time_ = timestamp_;
        regions_->push_back(*current_region);
        delete current_region;
        current_region = new boost::shared_ptr<GridObstacle>(new GridObstacle(++region_id, grid_, perception_));
      } else {
        current_region->get()->clear();
        current_region->get()->id = ++region_id;
      }
    }
  }
  delete current_region;
}

void GridSegmenter::labelNeighbors(const PerceptionCell& cell, GridObstacle* current_region)
{
  int16_t row, col;
  grid_.cellToRCLocal(&cell, &row, &col);
  static int d = segmentation_settings_.kernel_size / 2;

  for (int16_t r = row - d; r <= row + d; ++r) {
    if (r < 0 || r > grid_.rows_ - 1) {continue;}

    for (int16_t c = col - d; c <= col + d; ++c) {
      if (c < 0 || c > grid_.cols_ - 1) {continue;}

      PerceptionCell* neighbor = grid_.getRCLocalUnsafe(r, c);

      // check cell
      if( neighbor->hits > 0 && /*(cell_eval(neighbor) != PERCEPTION_MAP_OBSTACLE_FREE) &&*/ neighbor->region == 0) {
        neighbor->region = current_region->id;
        current_region->addCell(*neighbor);
        labelNeighbors(*neighbor, current_region);
      }
    }
  }
}


void GridSegmenter::segmentGrid(dgc_perception_map_cells_t* obstacles, vector< boost::shared_ptr<Obstacle> >* regions, vector< boost::shared_ptr<TrackedObstacle> > tracks, double timestamp) {
  regions_ = regions;
  timestamp_ = timestamp;
  tracks_ = tracks;

  findRegions(obstacles);
}

} // namespace perception



