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


#include <sstream>
#include <iomanip>

#include <global.h>
#include <lltransform.h>
#include <applanix/ApplanixPose.h>
#include <localize/LocalizePose.h>
#include <fake_localize.h>

using namespace dgc;

vlr::FakeLocalizer* falo=NULL;

namespace vlr {


FakeLocalizer::FakeLocalizer(double x_shift, double y_shift) : nh_("/driving"), x_shift_(x_shift_), y_shift_(y_shift), msg_count_(0) {
  applanix_sub_ = nh_.subscribe("ApplanixPose", 5, &FakeLocalizer::applanixPoseHandler, this);
  localize_pose_pub_ = nh_.advertise<localize::LocalizePose> ("LocalizePose", 5);
}

FakeLocalizer::~FakeLocalizer() {
}

void FakeLocalizer::run() {
  ros::spin();
}

void FakeLocalizer::applanixPoseHandler(const applanix::ApplanixPose& applanix_pose) {
  msg_count_++;
  if(msg_count_ != 20) {return;}
  msg_count_ = 0;

  std::string utmzone;
  double x, y;
  vlr::latLongToUtm(applanix_pose.latitude, applanix_pose.longitude, &x, &y, utmzone);
  //  x -= ((int)floor(dgc_get_time()) % 2) * 1;

  localize::LocalizePose pose;
  pose.x_offset = x - applanix_pose.smooth_x + x_shift_;
  pose.y_offset = y - applanix_pose.smooth_y + y_shift_;
  pose.timestamp = driving_common::Time::current();
  pose.std_x = pose.std_y = pose.std_s = 0;
  localize_pose_pub_.publish(pose);
}

} // namespace vlr


int32_t main(int argc, char** argv) {

  ros::init(argc, argv, "localize");
  double x_shift=0, y_shift=0;
  if(argc >=3) {
    x_shift = atof(argv[1]);
    y_shift = atof(argv[2]);
  }

  try {
    vlr::FakeLocalizer falo(x_shift, y_shift);
    falo.run();
  } catch( vlr::Ex<>& e ) {
    std::cout << e.what() << std::endl;
  }

  return 0;
}
