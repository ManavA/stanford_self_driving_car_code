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


#include <time.h>
#include <stdio.h>

#include <stdint.h>
#include <iostream>
#include <vector>

#include <global.h>
#include <lltransform.h>

#include <baseDemo.h>

using namespace std;

namespace drc = driving_common;

namespace vlr {

BaseDemo::BaseDemo(const std::string& rndf_name, const std::string& mdf_name, const double start_lat, const double start_lon, const double start_yaw) :
                       rndf_name_(rndf_name), mdf_name_(mdf_name), start_lat_(start_lat), start_lon_(start_lon), start_yaw_(start_yaw),
                       smooth_x_start_(0), smooth_y_start_(0), current_offset_x_(0), current_offset_y_(0) {

  current_timestamp_ = drc::Time::current();

  latLongToUtm(start_lat_, start_lon_, &start_x_,  &start_y_, utm_zone_);

//  fake_tracker_ = new FakeObstacleTracker(car_states_);
  memset(&obstacle_msg_, 0, sizeof(obstacle_msg_));
//  obstacle_msg_.num_dynamic_obstacles = num_fast_cars_ + num_slow_cars_ + num_trucks_;
//  obstacle_msg_.dynamic_obstacle = new PerceptionDynamicObstacle[obstacle_msg_.num_dynamic_obstacles];
//  strcpy(obstacle_msg_.host, dgc_hostname());
}


BaseDemo::~BaseDemo() {

}

void BaseDemo::updatePoses(bool init, const driving_common::TrajectoryPoint2D& current_trajectory_point,
                             applanix::ApplanixPose& applanix_pose, localize::LocalizePose& localize_pose) {

  memset(&applanix_pose, 0, sizeof(applanix_pose));
  memset(&localize_pose, 0, sizeof(localize_pose));

  localize_pose.utmzone = utm_zone_;

  applanix_pose.accel_x = 0;
  applanix_pose.accel_y = 0;
  applanix_pose.accel_z = 0;

  if(init) {
    applanix_pose.rate_yaw = 0;
    applanix_pose.speed = 0;
    applanix_pose.altitude = 0;
    applanix_pose.yaw = start_yaw_;
    applanix_pose.pitch = 0;
    applanix_pose.roll = 0;
    smooth_x_start_ = current_trajectory_point.x;
    smooth_y_start_ = current_trajectory_point.y;
  }
  else {
    applanix_pose.rate_yaw  = current_trajectory_point.kappa * current_trajectory_point.v;
    applanix_pose.speed = current_trajectory_point.v;
    applanix_pose.altitude = 0;
    applanix_pose.yaw = current_trajectory_point.theta;
    applanix_pose.pitch = 0;
    applanix_pose.roll = 0;
  }

  applanix_pose.smooth_x = current_trajectory_point.x - smooth_x_start_;
  applanix_pose.smooth_y = current_trajectory_point.y - smooth_y_start_;

  localize_pose.x_offset = current_trajectory_point.x - applanix_pose.smooth_x;
  localize_pose.y_offset = current_trajectory_point.y - applanix_pose.smooth_y;

  current_offset_x_ = localize_pose.x_offset;
  current_offset_y_ = localize_pose.y_offset;
  current_timestamp_ = current_trajectory_point.t;

  utmToLatLong(applanix_pose.smooth_x+localize_pose.x_offset, applanix_pose.smooth_y+localize_pose.y_offset, utm_zone_, &applanix_pose.latitude, &applanix_pose.longitude);

  applanix_pose.timestamp = current_timestamp_;
  localize_pose.timestamp = current_timestamp_;
//  printf("smx: %f, smy: %f, offx: %f, offy: %f, lat: %f, long: %f\n",applanix_pose.smooth_x, applanix_pose.smooth_y,
//         localize_pose.x_offset, localize_pose.y_offset,
//         applanix_pose.latitude, applanix_pose.longitude);
//    return;
}

} // namespace vlr
