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


#include <perception.h>
#include <segment.h>
#include <assert.h>

#define  MAX_SEGMENTS_PER_BEAM 1024

using namespace std;

namespace perception {

Segmenter::Segmenter() {

}

Segmenter::~Segmenter() {

}

void Segmenter::drawSegmentation() {

  /* setup GLS header */
  gls->coordinates = GLS_LOCAL_COORDINATES;
  gls->origin_x = 0;
  gls->origin_y = 0;
  gls->origin_z = 0; //applanix_current_pose()->smooth_z;
  gls.color3f(1.0, 0.0, 0.0 );

  gls.pushMatrix();
//  glsScalef(gls, CM_TO_METER_FACTOR, CM_TO_METER_FACTOR, CM_TO_METER_FACTOR);
//  glsTranslatef(gls, , , CM_TO_METER_FACTOR);
  gls.begin(GLS_POINTS);
  for (int l=0; l < num_beams_; l++) {
    for (int i = 0; i < lscan[l].num_points; i++) {
      if ( lscan[l].laser_point[i].valid) {
        if (lscan[l].laser_point[i].obstacle) //{
//
//        } else {
          gls.color3f(1.0, 0.0, 0.0 );
        else
          gls.color3f(0.7, 0.7, 0.7 );

          gls.vertex3f(lscan[l].laser_point[i].point->x * .01 + lscan[l].laser_point[i].scan->robot.x,
                       lscan[l].laser_point[i].point->y * .01 + lscan[l].laser_point[i].scan->robot.y,
                       lscan[l].laser_point[i].point->z * .01);
//        }
      }
    }
  }
  gls.end();

  double x1,y1,x2,y2;
  gls.color3f(0.7, 0.7, 0.7);
  gls.lineWidth(1.0);
  gls.begin(GLS_LINES);
  grid_rc_to_xy(grid, 0, 0, &x1, &y1);
  grid_rc_to_xy(grid, grid->rows, grid->cols, &x2, &y2);
  gls.vertex3f(x1, y1, 0);
  gls.vertex3f(x1, y2, 0);

  gls.vertex3f(x1, y2, 0);
  gls.vertex3f(x2, y2, 0);

  gls.vertex3f(x2, y2, 0);
  gls.vertex3f(x2, y1, 0);

  gls.vertex3f(x2, y1, 0);
  gls.vertex3f(x1, y1, 0);
  gls.end();

  gls.popMatrix();
}


//void  segmentFrame(ObstacleList* obstacles) {
//  obstacles->obstacles.clear();
//
////  for (int l=0; l < NUM_BEAMS; l++) {
////    for (int i = 0; i < lscan[l].num_points; i++) {
////      if (lscan[l].laser_point[i].valid) {
////        lscan[l].laser_point[i].point->
////      }
////    }
////  }
//
//
//  if (settings_.segmentation_settings_.gls_output) {
//    drawSegmentation();
//  }
//
//  // collect obstacles
////  perception_collect_obstacles(regions, obstacles);
//
////  printf("segmented obstacles: %d\n", obstacles->obstacles.size());
//}

void Segmenter::segmentVelodyne(const std::vector<velodyne::Block>& blocks) {
  for (size_t i = 0; i < blocks.size(); i++) {
    segmentScan(blocks[i]);
  }
}

// the points come in order, but there are sometimes large sections missing
void Segmenter::segmentScan(const velodyne::Block& block) {
  PerceptionLineSegment* current_segment;

//  if(DEBUG) printf("scan_num_points: %d\n", scan_num_points);

  int block_offset = block.block * 32;
  for (int i = 0; i < 32; i++) {
    int l = i + block_offset;
    if (block.point[i].range < 200) continue;
    double delta_range = 1.0;
    double delta_angle = 0.01;
    int num_segment = num_segments[l];
    if (num_segment >= max_segments_per_laser) {
      printf("Warning: maxed out maximum segments to track!\n");
      continue;
    }

    current_segment = &segments[l][num_segment];
    int num_points = current_segment->points.size();

    if (num_points == 0) {
      current_segment->last_range = block.laser[i].distance;
      current_segment->min_angle = current_segment->max_angle = point->angle;
    }
    double delta_range = abs(point->range - current_segment->last_range);
    double delta_angle = abs(point->angle - current_segment->min_angle);

    // is this scan finished?
    if ( (delta_range > max_delta_range) ||
         (delta_angle > MAX_DELTA_ANGLE)) {
      if ( (num_points >= min_segment_points) &&
           (num_segments[l] < MAX_SEGMENTS_PER_BEAM)) {
        num_segments[l]++;
//        current_segment++;
        current_segment = &segments[l][num_segment+1];
      }
      clear_segment(current_segment);
    }
    // add current point to segment
    else if ((current_point->label != LABEL_NOISE) && (current_point->label != LABEL_OUTSIDE_RNDF)) {
      current_segment->points.push_back(current_point);
      current_segment->last_range = current_point->range;
    }
  }
}

static uint8_t Segmenter::colors[19][3] = {
  {255,0,0},
  {0,255,0},
  {0,0,255},
  {255,255,0},
  {0,255,255},
  {255,0,255},
  {239,230,0},
  {230,0,230},
  {0,230,230},
  {230,0,0},
  {128,51,128},
  {51,128,128},
  {255,51,51},
  {51,255,51},
  {51,51,255},
  {51,179,204},
  {128,255,51},
  {255,128,51},
  {51,128,255}
};

} // namespace perception
