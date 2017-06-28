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
#include "obstacle.h"
#include "tracked_obstacle.h"
#include "kalman_tracker.h"
#include "utils.h"

#include <algorithm>
#include <gtest/gtest.h>

using namespace dgc;
using std::tr1::shared_ptr;
using std::vector;

TEST(Tracker, Sanity) {
  TrackerKF tracker;

  vector< shared_ptr<GridObstacle> > obstacles;

  tracker.transitionUpdate(0.0);
  tracker.observationUpdate(0.0, obstacles);
}

GridObstacle* buildObstacle(double x, double y) {
  GridObstacle* obstacle = new GridObstacle(1, NULL);

  vector<point3d_t>& points = obstacle->getPoints();
  point3d_t pt;
  pt.x = x;
  pt.y = y;
  pt.z = 0.0;
  points.push_back(pt);

  return obstacle;
}

GridObstacle* buildNoisyObstacle(double x, double y) {
  GridObstacle* obstacle = new GridObstacle(1, NULL);

  vector<point3d_t>& points = obstacle->getPoints();

  for (int i=0; i<100; i++) {
    point3d_t pt;
    pt.x = x + sample(0.1);
    pt.y = y + sample(0.1);
    pt.z = 0.0;
    points.push_back(pt);
  }

  return obstacle;
}

TEST(Tracker, TwoObservations) {
  TrackerKF tracker;

  vector< shared_ptr<GridObstacle> > obstacles;

  shared_ptr<GridObstacle> obstacle (buildObstacle(0.0, 0));
  obstacles.push_back(obstacle);

  tracker.transitionUpdate(0.0);
  tracker.observationUpdate(0.0, obstacles);

  obstacles.clear();
  shared_ptr<GridObstacle> obstacle2 (buildObstacle(0.1, 0));
  obstacles.push_back(obstacle2);

  tracker.transitionUpdate(0.1);
  tracker.observationUpdate(0.1, obstacles);

  tracker.transitionUpdate(10.0);
  vector <shared_ptr<TrackedObstacle> > tracks;
  tracker.getDynamicObstacles(tracks);

  EXPECT_EQ(1, tracks.size());

  shared_ptr<TrackedObstacle> track = tracks[0];
  EXPECT_EQ(1.0, track->getXVel());
  EXPECT_EQ(0.0, track->getYVel());
  EXPECT_EQ(10.0, track->pose.x);
  EXPECT_EQ(0.0, track->pose.y);
}

TEST(Tracker, SingleObstacle) {
  TrackerKF tracker;
  double dt = 0.1;

  vector< shared_ptr<GridObstacle> > obstacles;
  tracker.transitionUpdate(0.0);
  tracker.observationUpdate(0.0, obstacles);

  for (int i=1; i < 101; i++) {
    shared_ptr<GridObstacle> obstacle (buildObstacle(i * dt * 2.0, 0));
    obstacles.push_back(obstacle);

    tracker.transitionUpdate(dt * i);
    tracker.observationUpdate(dt * i, obstacles);

    obstacles.clear();
  }

  vector <shared_ptr<TrackedObstacle> > tracks;
  tracker.getDynamicObstacles(tracks, 3.0);

  EXPECT_EQ(1, tracks.size());

  shared_ptr<TrackedObstacle> track = tracks[0];

  EXPECT_EQ(2.0, track->getXVel());
  EXPECT_EQ(0.0, track->getYVel());
  EXPECT_EQ(20.0, track->pose.x);
  EXPECT_EQ(0.0, track->pose.y);
}

bool compareObstaclesPoseX(const shared_ptr<TrackedObstacle> o1, const shared_ptr<TrackedObstacle> o2)  {
  return (o1->pose.x < o2->pose.x);
}

TEST(Tracker, TwoObstacles) {
  TrackerKF tracker;
  double dt = 0.1;

  vector< shared_ptr<GridObstacle> > obstacles;
  tracker.transitionUpdate(0.0);
  tracker.observationUpdate(0.0, obstacles);

  for (int i=1; i < 101; i++) {
    shared_ptr<GridObstacle> obstacle (buildObstacle(i * dt * 2.0, 0));
    shared_ptr<GridObstacle> obstacle2 (buildObstacle(5.0 + i * dt * 2.0, 5.0));

    obstacles.push_back(obstacle);
    obstacles.push_back(obstacle2);

    tracker.transitionUpdate(dt * i);
    tracker.observationUpdate(dt * i, obstacles);

    obstacles.clear();
  }

  vector <shared_ptr<TrackedObstacle> > tracks;
  tracker.getDynamicObstacles(tracks, 3.0);

  sort(tracks.begin(), tracks.end(), compareObstaclesPoseX);

  EXPECT_EQ(2, tracks.size());

  shared_ptr<TrackedObstacle> track = tracks[0];
  EXPECT_DOUBLE_EQ(2.0, track->getXVel());
  EXPECT_DOUBLE_EQ(0.0, track->getYVel());
  EXPECT_DOUBLE_EQ(20.0, track->pose.x);
  EXPECT_DOUBLE_EQ(0.0, track->pose.y);

  track = tracks[1];
  EXPECT_NEAR(2.0, track->getXVel(), 0.0001);
  EXPECT_DOUBLE_EQ(0.0, track->getYVel());
  EXPECT_DOUBLE_EQ(25.0, track->pose.x);
  EXPECT_DOUBLE_EQ(5.0, track->pose.y);
}

TEST(Tracker, TwoNoisyObstacles) {
  TrackerKF tracker;
  double dt = 0.1;

  vector <shared_ptr<TrackedObstacle> > tracks;

  vector< shared_ptr<GridObstacle> > obstacles;
  tracker.transitionUpdate(0.0);
  tracker.observationUpdate(0.0, obstacles);

  for (int i=1; i < 101; i++) {
    shared_ptr<GridObstacle> obstacle (buildNoisyObstacle(i * dt * 2.0, 0.0));
    shared_ptr<GridObstacle> obstacle2 (buildNoisyObstacle(5.0 + i * dt * 2.0, 5.0));

    obstacles.push_back(obstacle);
    obstacles.push_back(obstacle2);

    tracker.transitionUpdate(dt * i);
    tracker.observationUpdate(dt * i, obstacles);

    obstacles.clear();
  }

  tracker.getDynamicObstacles(tracks, 3.0);

  EXPECT_EQ(2, tracks.size());

  sort(tracks.begin(), tracks.end(), compareObstaclesPoseX);

  shared_ptr<TrackedObstacle> track = tracks[0];
  EXPECT_LT( fabs(2.0 - track->getXVel()), 0.015);
  EXPECT_LT( fabs(track->getYVel()), 0.015);
  EXPECT_LT( fabs(20.0 - track->pose.x), 0.015);
  EXPECT_LT( fabs(track->pose.y), 0.015);

  track = tracks[1];
  EXPECT_LT( fabs(2.0 - track->getXVel()), 0.015);
  EXPECT_LT( fabs(track->getYVel()), 0.015);
  EXPECT_LT( fabs(25.0 - track->pose.x), 0.015);
  EXPECT_LT( fabs(5.0 - track->pose.y), 0.015);
}

/*
 * Tests to see if tracker will successfully handle when one obstacle generates
 * a series of single observations, a series of two observations, and then a series
 * of single observations
 */
TEST(Tracker, ExtraObstacle) {
  TrackerKF tracker;
  double dt = 0.1;

  vector <shared_ptr<TrackedObstacle> > tracks;
  vector< shared_ptr<GridObstacle> > obstacles;
  tracker.transitionUpdate(0.0);
  tracker.observationUpdate(0.0, obstacles);

  int i;
  // one obstacle
  for (i=1; i < 11; i++) {
    shared_ptr<GridObstacle> obstacle (buildObstacle(i * dt * 2.0, 0.0));

    obstacles.push_back(obstacle);

    tracker.transitionUpdate(dt * i);
    tracker.observationUpdate(dt * i, obstacles);

    obstacles.clear();
  }

  // two obstacles
  for (i=11; i < 21; i++) {
    shared_ptr<GridObstacle> obstacle (buildObstacle(i * dt * 2.0, 0.0));
    shared_ptr<GridObstacle> obstacle2 (buildObstacle(i * dt * 2.0 + 0.5, 0.0));

    obstacles.push_back(obstacle);
    obstacles.push_back(obstacle2);

    tracker.transitionUpdate(dt * i);
    tracker.observationUpdate(dt * i, obstacles);

    obstacles.clear();
  }

  // one obstacle
  for (i=21; i < 31; i++) {
    shared_ptr<GridObstacle> obstacle (buildObstacle(i * dt * 2.0, 0.0));

    obstacles.push_back(obstacle);

    tracker.transitionUpdate(dt * i);
    tracker.observationUpdate(dt * i, obstacles);

    obstacles.clear();
  }

  tracker.getDynamicObstacles(tracks, 3.0);

  EXPECT_EQ(1, tracks.size());

  shared_ptr<TrackedObstacle> track = tracks[0];
  EXPECT_LT( fabs(2.0 - track->getXVel()), 0.01);
  EXPECT_LT( fabs(track->getYVel()), 0.01);
  EXPECT_LT( fabs(6.0 - track->pose.x), 0.01);
  EXPECT_LT( fabs(track->pose.y), 0.01);
}


TEST(Tracker, VelodyneOcclusionTest)
{
  TrackerKF tracker;

  ApplanixPose pose;
  pose.roll = 0.0;
  pose.pitch = 0.0;
  pose.yaw = 0.0;

  pose.smooth_x = 0.0;
  pose.smooth_y = 0.0;
  pose.smooth_z = 0.0;

  applanix_history_init(10);
  applanix_history_add(&pose);

  dgc_transform_identity(velodyne_offset);


  bool occluded = tracker.boolObstacleOccluded(SENSOR_VELODYNE, 10.0, 10.0);
  EXPECT_EQ(false, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_VELODYNE, 200.0, 10.0);
  EXPECT_EQ(true, occluded);
}

TEST(Tracker, RadarOcclusionTest)
{
  TrackerKF tracker;

  ApplanixPose pose;
  pose.roll = 0.0;
  pose.pitch = 0.0;
  pose.yaw = 0.0;

  pose.smooth_x = 0.0;
  pose.smooth_y = 0.0;
  pose.smooth_z = 0.0;

  applanix_history_init(10);
  applanix_history_add(&pose);

  dgc_transform_identity(radar_offset[0]);

  bool occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 10.0, 0.0);
  EXPECT_EQ(false, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 300.0, 0.0);
  EXPECT_EQ(true, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 8.0, 1.0);
  EXPECT_EQ(false, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 7.0, 1.0);
  EXPECT_EQ(true, occluded);

  dgc_transform_rotate_z( radar_offset[0], dgc_d2r(90) );
  dgc_transform_translate( radar_offset[0], 1.0, 1.0, 0.0 );

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 1.0, 11.0);
  EXPECT_EQ(false, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 1.0, 301.0);
  EXPECT_EQ(true, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 2.0, 9.0);
  EXPECT_EQ(false, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 2.0, 7.0);
  EXPECT_EQ(true, occluded);
}

TEST(Tracker, RadarOcclusionSmoothTest)
{
  TrackerKF tracker;

  ApplanixPose pose;
  pose.roll = 0.0;
  pose.pitch = 0.0;
  pose.yaw = 0.0;

  pose.smooth_x = 1.0;
  pose.smooth_y = 0.0;
  pose.smooth_z = 0.0;

  applanix_history_init(10);
  applanix_history_add(&pose);

  dgc_transform_identity(radar_offset[0]);

  bool occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 11.0, 0.0);
  EXPECT_EQ(false, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 301.0, 0.0);
  EXPECT_EQ(true, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 9.0, 1.0);
  EXPECT_EQ(false, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 8.0, 1.0);
  EXPECT_EQ(true, occluded);

  dgc_transform_rotate_z( radar_offset[0], dgc_d2r(90) );
  dgc_transform_translate( radar_offset[0], 1.0, 1.0, 0.0 );

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 1.0, 12.0);
  EXPECT_EQ(false, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 1.0, 302.0);
  EXPECT_EQ(true, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 2.0, 10.0);
  EXPECT_EQ(false, occluded);

  occluded = tracker.boolObstacleOccluded(SENSOR_RADAR1, 2.0, 8.0);
  EXPECT_EQ(true, occluded);
}
