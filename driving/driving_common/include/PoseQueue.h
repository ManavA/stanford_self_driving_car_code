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


#ifndef DRIVING_COMMON_GLOBAL_POSE_QUEUE_H_
#define DRIVING_COMMON_GLOBAL_POSE_QUEUE_H_

#include <map>
#include <stdint.h>

#include <GlobalPose.h>

namespace driving_common {

class PoseQueue {
 public:
  PoseQueue(uint32_t queue_size, bool extrapolate_data = false, double extrapolation_time_threshold = 0.0);
  virtual ~PoseQueue();

  bool empty() const {return queue_.empty();}
  void clear() {queue_.clear();}
  size_t size() const {return queue_.size();}
  size_t maxSize() const {return queue_size_;}
  double extrapolationTimeTreshold() {return extrapolation_time_threshold_;}
  void extrapolationTimeTreshold(double extrapolation_time_threshold) {extrapolation_time_threshold_ = extrapolation_time_threshold;}
  void push(const GlobalPose& pose, double timestamp);

  void xy(double timestamp, double& x, double& y);
  void xyAndOffsets(double timestamp, double& x, double& y, double& offset_x, double& offset_y);
  void offsets(double timestamp, double& offset_x, double& offset_y);
  void utmXY(double timestamp, double& utm_x, double& utm_y);
  void xyAndUtmXY(double timestamp, double& x, double& y, double& utm_x, double& utm_y);
  GlobalPose pose(double timestamp);
  GlobalPose latestPose();
  void latestPose(GlobalPose& latest_pose, double& latest_timestamp);
  double latestPoseTimestamp();

 private:
  int bracketPose(double timestamp,
        std::map<double, GlobalPose>::const_iterator& before, std::map<double, GlobalPose>::const_iterator& after);
  double interpolateYaw(double head1, double head2, double fraction);

  std::map <double, GlobalPose> queue_; // ok, it's a map, so what? ...
  size_t queue_size_;
  bool extrapolate_data_;
  double extrapolation_time_threshold_;
};

} // namespace driving_common
#endif
