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


#include <global.h>
#include <vector>

#include <perception.h>
#include <box.h>

#include <gtest/gtest.h>

using namespace dgc;
using std::vector;

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

TEST(AlignPoints, Lines) {
  vector<point3d_t> points;
  point3d_t pt;

  double err = 0.001;

  for (int i=0; i<5; i++) {
    pt.x = i;
    pt.y = i;
    points.push_back(pt);
  }
  double yaw = align_points(points);
  EXPECT_NEAR(M_PI_4, yaw, err);

  points.clear();

  for (int i=0; i<5; i++) {
    pt.x = -i;
    pt.y = i;
    points.push_back(pt);
  }
  yaw = align_points(points);
  EXPECT_NEAR(M_PI_4, yaw, err);

  points.clear();
  for (int i=0; i<5; i++) {
    pt.x = i;
    pt.y = -i;
    points.push_back(pt);
  }
  yaw = align_points(points);
  EXPECT_NEAR(M_PI_4, yaw, err);

  points.clear();
  for (int i=0; i<5; i++) {
    pt.x = -i;
    pt.y = -i;
    points.push_back(pt);
  }
  yaw = align_points(points);
  EXPECT_NEAR(M_PI_4, yaw, err);
}

TEST(AlignPoints, LongLine) {
  vector<point3d_t> points;
  point3d_t pt;

  double err = 0.001;

  for (int i=0; i<5000; i++) {
    pt.x = i;
    pt.y = i;
    points.push_back(pt);
  }
  double yaw = align_points(points);
  EXPECT_NEAR(M_PI_4, yaw, err);
}

TEST(AlignPoints, L) {
  vector<point3d_t> points;
  point3d_t pt;

  double err = M_PI_2 / 12.0 / 2.0;

  for (int i=0; i<5; i++) {
    pt.x = i;
    pt.y = 0;
    points.push_back(pt);
  }
  for (int i=0; i<8; i++) {
    pt.x = 0;
    pt.y = i;
    points.push_back(pt);
  }
  double yaw = align_points(points);
  EXPECT_NEAR(0, yaw, err);


  points.clear();
  for (int i=0; i<8; i++) {
    pt.x = i;
    pt.y = 0;
    points.push_back(pt);
  }
  for (int i=0; i<5; i++) {
    pt.x = 0;
    pt.y = i;
    points.push_back(pt);
  }
  yaw = align_points(points);
  EXPECT_NEAR(0, yaw, err);
}

double rand_uniform(double x) {
  return x * (double)rand() / RAND_MAX;
}

TEST(AlignPoints, RandomLine) {
  dgc_transform_t t;
  dgc_transform_identity(t);
  double yaw = rand_uniform(M_PI_2);
  dgc_transform_rotate_z(t, yaw);

  vector<point3d_t> points;
  point3d_t pt;

  double x,y,z;
  for (int i=0; i<5; i++) {
    x = i;
    y = 0;
    z = 0;
    dgc_transform_point(&x, &y, &z, t);

    pt.x = x;
    pt.y = y;
    points.push_back(pt);
  }

  double aligned_yaw = align_points(points);
  EXPECT_NEAR(aligned_yaw, yaw, 0.001);
}

TEST(AlignPoints, RandomNoisyLines) {
  dgc_transform_t t;
  double yaw;
  vector<point3d_t> points;
  point3d_t pt;

  for (int k = 0; k < 5; k++) {
    yaw = rand_uniform(M_2_PI);
    dgc_transform_identity(t);
    dgc_transform_rotate_z(t, yaw);
    points.clear();

    double x,y,z;
    for (int i=0; i<15; i++) {
      x = i + rand_uniform(0.1);
      y = rand_uniform(0.1);
      z = 0;
      dgc_transform_point(&x, &y, &z, t);

      pt.x = x;
      pt.y = y;
      points.push_back(pt);
    }

    double estimated_yaw = align_points(points);
    double aligned_yaw = yaw;
    while (aligned_yaw > M_PI_2)
      aligned_yaw -= M_PI_2;
    EXPECT_NEAR(estimated_yaw, aligned_yaw, 0.1);
  }
}
