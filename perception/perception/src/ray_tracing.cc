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
#include <utils.h>
#include <perception/VirtualScan.h>

using namespace dgc;
using namespace driving_common;

namespace drc = driving_common;

#define MAX_NUM_LINE_GRID_POINTS   1000

double round(double x);

namespace perception {

void Perception::freeSpaceRayTracing(uint16_t counter) {
  static int firsttime = 1;
  static perception::VirtualScan vscan;
  static grid_line_t line;

  double vscan_res = dgc_d2r(PERCEPTION_RAY_TRACING_RES);
  drc::GlobalPose current_pose = pose(drc::Time::current());
  PerceptionCell* cell = NULL;

  if (firsttime) {
    vscan.p.reserve((int) ceil(2 * M_PI / vscan_res));
    line.max = MAX_NUM_LINE_GRID_POINTS;
    line.numgrids = 0;
    line.grid = (ivec2_p) malloc(line.max * sizeof(ivec2_t));
    firsttime = 0;

  }

  if (obstacles_s->num == 0) {
    return;
  }

  vscan.origin_x = current_pose.x();
  vscan.origin_y = current_pose.y();

  int32_t num_vscans = (int32_t)vscan.p.size();
  for (int32_t i = 0; i < num_vscans; i++) {
    vscan.p[i].dist = FLT_MAX;
    vscan.p[i].angle = i * vscan_res;
    vscan.p[i].x = vscan.origin_x + PERCEPTION_RAY_TRACING_MAX * cos(vscan.p[i].angle);
    vscan.p[i].y = vscan.origin_y + PERCEPTION_RAY_TRACING_MAX * sin(vscan.p[i].angle);
  }

  double ll_x = (grid_->map_c0_) * grid_->resolution_;
  double ll_y = (grid_->map_r0_) * grid_->resolution_;

  for (int i = 0; i < obstacles_s->num; i++) {
    int16_t px, py;
    grid_->cellToRCLocal(obstacles_s->cell[i], &py, &px);
    double x = ll_x + (px + 0.5) * grid_->resolution_;
    double y = ll_y + (py + 0.5) * grid_->resolution_;
    float dist = hypot(x - vscan.origin_x, y - vscan.origin_y);
    float angle = atan2(y - vscan.origin_y, x - vscan.origin_x);
    size_t b = (num_vscans + (int) (angle / vscan_res)) % num_vscans;
    if (dist < vscan.p[b].dist) {
      vscan.p[b].dist = dist;
      vscan.p[b].x = x;
      vscan.p[b].y = y;
    }
  }

  if (settings_.gls_output && settings_.show_ray_tracing) {

      // setup GLS header
    gls.coordinates = GLSOverlay::SMOOTH_COORDINATES;
    gls.origin_x = 0.0;
    gls.origin_y = 0.0;
    gls.origin_z = current_pose.z() + 0.6 - DGC_PASSAT_HEIGHT;

      // draw the candidates
    gls.pushMatrix();
    gls.lineWidth(1.0);
    gls.color3f(0.0, 1.0, 0.0);
    gls.begin(GLSOverlay::LINES);
  }

  ivec2_t s, e;
  for (int32_t b = 0; b < num_vscans; b++) {

    double x = vscan.origin_x + cos(vscan.p[b].angle) * PERCEPTION_RAY_TRACING_MIN_DIST;
    double y = vscan.origin_y + sin(vscan.p[b].angle) * PERCEPTION_RAY_TRACING_MIN_DIST;

    if (settings_.gls_output && settings_.show_ray_tracing) {
      gls.vertex3f(x, y, 0.0);
    }

    s.x = (int) (x / grid_->resolution_);
    s.y = (int) (y / grid_->resolution_);

    if (vscan.p[b].dist < PERCEPTION_RAY_TRACING_MAX) {
      x = vscan.origin_x + cos(vscan.p[b].angle) * (vscan.p[b].dist - PERCEPTION_RAY_TRACING_SHORTEN);
      y = vscan.origin_y + sin(vscan.p[b].angle) * (vscan.p[b].dist - PERCEPTION_RAY_TRACING_SHORTEN);
    }
    else {
      x = vscan.origin_x + cos(vscan.p[b].angle) * PERCEPTION_RAY_TRACING_FREESPACE;
      y = vscan.origin_y + sin(vscan.p[b].angle) * PERCEPTION_RAY_TRACING_FREESPACE;
    }
    e.x = (int) (x / grid_->resolution_);
    e.y = (int) (y / grid_->resolution_);

    if (settings_.gls_output && settings_.show_ray_tracing) {
      gls.vertex3f(vscan.p[b].x, vscan.p[b].y, 0.0);
    }

    grid_line(s, e, &line);
    for (int32_t l = 0; l < line.numgrids; l++) {
      x = (line.grid[l].x + 0.5) * grid_->resolution_;
      y = (line.grid[l].y + 0.5) * grid_->resolution_;
      cell = grid_->getXY(x, y);
      if (cell) {
        if (cell->hits >= 1) {
          cell->hits -= 1;
        }
      }
    }

  }

  if (settings_.gls_output && settings_.show_ray_tracing) {
    gls.end();
    gls.lineWidth(1.0);
    gls.popMatrix();
  }

}

} // namespace perception
