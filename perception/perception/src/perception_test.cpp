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


#include <roadrunner.h>
#include <vector>

#include "perception.h"
#include "box.h"

#include <gtest/gtest.h>

using namespace dgc;
using std::vector;

/* variables */

dgc_perception_map_cells_p     obstacles_s;
dgc_perception_map_cells_p     map_s;

std::vector<std::tr1::shared_ptr<TrackedObstacle> > obstacles_predicted;
std::vector<std::tr1::shared_ptr<TrackedObstacle> > obstacles_tracked;
std::vector<std::tr1::shared_ptr<Obstacle> >    obstacles_segmented;

char *imagery_root;
char *cal_filename = NULL;
GlsOverlay                    * gls = NULL;

/* tracker stuff */
grid_stat_t                     grid_stat;
dgc_grid_p                      grid;
dgc_grid_p                      terrain_grid;

PerceptionCell*       default_map_cell     = NULL;
PerceptionCell*       default_terrain_cell = NULL;

LocalizePose                    localize_pose = {0, 0.0, 0.0, "", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "" };

char                          * rndf_filename = NULL;
dgc_global_pose_t               global = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "10S"};
perception_settings_t           settings;

TEST(BoundingBox, Square) {
  vector<point3d_t> points;
  point3d_t pt;

  pt.x=1; pt.y=1;
  points.push_back(pt);

  pt.x=1; pt.y=2;
  points.push_back(pt);

  pt.x=2; pt.y=1;
  points.push_back(pt);
  pt.x=2; pt.y=2;
  points.push_back(pt);

  double x,y,yaw,width,length;
  bounding_box(&points, 0.0f, &x, &y, &yaw, &width, &length, 0.0f, 0.0f);
  EXPECT_DOUBLE_EQ(1.5, x);
  EXPECT_DOUBLE_EQ(1.5, y);
  EXPECT_DOUBLE_EQ(0.0, yaw);
  EXPECT_DOUBLE_EQ(1.0, width);
  EXPECT_DOUBLE_EQ(1.0, length);
}

TEST(BoundingBox, Rectangle) {
  vector<point3d_t> points;
  point3d_t pt;

  pt.x=1; pt.y=1;
  points.push_back(pt);

  pt.x=2; pt.y=1;
  points.push_back(pt);

  pt.x=1; pt.y=3;
  points.push_back(pt);

  pt.x=2; pt.y=3;
  points.push_back(pt);

  double x,y,yaw,width,length;
  bounding_box(&points, 0.0f, &x, &y, &yaw, &width, &length, 0.0f, 0.0f);

  EXPECT_DOUBLE_EQ(1.5, x);
  EXPECT_DOUBLE_EQ(2.0, y);
  EXPECT_DOUBLE_EQ(0.0, yaw);
  EXPECT_DOUBLE_EQ(2.0, width);
  EXPECT_DOUBLE_EQ(1.0, length);
}

TEST(BoundingBox, L) {
  vector<point3d_t> points;
  point3d_t pt;

  pt.x=1; pt.y=1;
  points.push_back(pt);

  pt.x=2; pt.y=1;
  points.push_back(pt);

  pt.x=1; pt.y=3;
  points.push_back(pt);

  double x,y,yaw,width,length;
  bounding_box(&points, 0.0f, &x, &y, &yaw, &width, &length, 0.0f, 0.0f);

  EXPECT_DOUBLE_EQ(1.5, x);
  EXPECT_DOUBLE_EQ(2.0, y);
  EXPECT_DOUBLE_EQ(0.0, yaw);
  EXPECT_DOUBLE_EQ(2.0, width);
  EXPECT_DOUBLE_EQ(1.0, length);
}

TEST(BoundingBox, NoisyL) {
  vector<point3d_t> points;
  point3d_t pt;

  pt.x=1; pt.y=1;
  points.push_back(pt);
  pt.x=1; pt.y=1.1;
  points.push_back(pt);
  pt.x=1.1; pt.y=1.1;
  points.push_back(pt);

  pt.x=2; pt.y=1;
  points.push_back(pt);
  pt.x=1.87; pt.y=1.01;
  points.push_back(pt);
  pt.x=2; pt.y=1;
  points.push_back(pt);

  pt.x=1; pt.y=3;
  points.push_back(pt);
  pt.x=1; pt.y=1.5;
  points.push_back(pt);
  pt.x=1; pt.y=2.5;
  points.push_back(pt);

  double x,y,yaw,width,length;
  bounding_box(&points, 0.0f, &x, &y, &yaw, &width, &length, 0.0f, 0.0f);

  EXPECT_DOUBLE_EQ(1.5, x);
  EXPECT_DOUBLE_EQ(2.0, y);
  EXPECT_DOUBLE_EQ(0.0, yaw);
  EXPECT_DOUBLE_EQ(2.0, width);
  EXPECT_DOUBLE_EQ(1.0, length);
}
