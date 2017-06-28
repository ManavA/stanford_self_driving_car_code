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

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

#include <vlrException.h>
#include <PoseQueue.h>

namespace driving_common {

PoseQueue::PoseQueue(uint32_t queue_size, bool extrapolate_data, double extrapolation_time_threshold) :
  extrapolate_data_(extrapolate_data), extrapolation_time_threshold_(extrapolation_time_threshold) {

  if (queue_size == 0) {
    throw VLRException("Invalid queue size (zero) requested.");
  }
  queue_size_ = queue_size;
}

PoseQueue::~PoseQueue() {
}

void PoseQueue::push(const GlobalPose& pose, double timestamp) {
  if (queue_.size() > queue_size_) {
    queue_.erase(queue_.begin());
  }
  std::pair<std::map<double, GlobalPose>::iterator, bool> rqp = queue_.insert(std::make_pair(timestamp, pose));
}

int PoseQueue::bracketPose(double timestamp, std::map<double, GlobalPose>::const_iterator& before, std::map<double, GlobalPose>::const_iterator& after) {

  std::map<double, GlobalPose>::const_iterator pit = queue_.lower_bound(timestamp);

  if (pit == queue_.end()) {
    std::stringstream s;
    if (queue_.empty()) {
      throw VLRException("GlobalPose queue is empty.");
    }

    // requested timestamp is newer than newest entry in pose queue
    before = --queue_.end(); // TODO: implement proper extrapolation
    after = before;
    after--;
    return 1;
  }

  after = pit; // btw...who named lower_bound ?!?

  if (pit == queue_.begin()) {
    if (pit->first == timestamp) {
      before = pit;
      return -1;
    }

    // requested timestamp is older than oldest entry in pose queue
    before = queue_.begin();
    after = queue_.begin();
    after++;
    return -1;
  }

  // requested timestamp is somewhere between entries in pose queue
  before = --pit;
  return 0;
}

void PoseQueue::xy(double timestamp, double& x, double& y) {

  // empty case is handled in bracketPose
  if (queue_.size() == 1) {
    const GlobalPose& pose = queue_.begin()->second;
    x = pose.x();
    y = pose.y();
    return;
  }

  std::map<double, GlobalPose>::const_iterator before, after;
  int polation_mode = bracketPose(timestamp, before, after);
  const GlobalPose& pose1 = before->second;
  const GlobalPose& pose2 = after->second;

  if (polation_mode != 0) {
    if (!extrapolate_data_ || before->first == after->first) {
      // pose1 is always the one closest to requested ts
      x = pose1.x();
      y = pose1.y();
      return;
    }

    double ts1 = before->first;
    double ts2 = after->first;
    double dt = ts1 - ts2;

    if (std::abs(timestamp - ts1) > extrapolation_time_threshold_) {
      std::stringstream s;
      s << "Cannot extrapolate for requested timestamp " << timestamp << "since it's too far off (dt=" << timestamp - ts1 << ")";
      throw VLRException(s.str());
    }

    x = pose1.x() + (pose1.x() - pose2.x()) / dt * (timestamp - ts1);
    y = pose1.y() + (pose1.y() - pose2.y()) / dt * (timestamp - ts1);
    return;
  }

  double frac = (timestamp - before->first) / (after->first - before->first);

  x = pose1.x() + frac * (pose2.x() - pose1.x());
  y = pose1.y() + frac * (pose2.y() - pose1.y());
}

void PoseQueue::offsets(double timestamp, double& offset_x, double& offset_y) {
  // empty case is handled in bracketPose
  if (queue_.size() == 1) {
    const GlobalPose& pose = queue_.begin()->second;
    offset_x = pose.offsetX();
    offset_y = pose.offsetY();
    return;
  }

  std::map<double, GlobalPose>::const_iterator before, after;
  int polation_mode = bracketPose(timestamp, before, after);
  const GlobalPose& pose1 = before->second;
  const GlobalPose& pose2 = after->second;

  if (polation_mode != 0) {
    if (!extrapolate_data_ || before->first == after->first) {
      // pose1 is always the one closest to requested ts
      offset_x = pose1.offsetX();
      offset_y = pose1.offsetY();
      return;
    }
    double ts1 = before->first;
    double ts2 = after->first;
    double dt = ts1 - ts2;

    if (std::abs(timestamp - ts1) > extrapolation_time_threshold_) {
      std::stringstream s;
      s << "Cannot determine extrapolate for requested timestamp " << timestamp << "since it's too far off (dt=" << timestamp - ts1 << ")";
      throw VLRException(s.str());
    }

    offset_x = pose1.offsetX() + (pose1.offsetX() - pose2.offsetX()) / dt * (timestamp - ts1);
    offset_y = pose1.offsetY() + (pose1.offsetY() - pose2.offsetY()) / dt * (timestamp - ts1);
    return;
  }

  double frac = (timestamp - before->first) / (after->first - before->first);

  offset_x = pose1.offsetX() + frac * (pose2.offsetX() - pose1.offsetX());
  offset_y = pose1.offsetY() + frac * (pose2.offsetY() - pose1.offsetY());
}

void PoseQueue::xyAndOffsets(double timestamp, double& x, double& y, double& offset_x, double& offset_y) {
  // empty case is handled in bracketPose
  if (queue_.size() == 1) {
    const GlobalPose& pose = queue_.begin()->second;
    x = pose.x();
    y = pose.y();
    offset_x = pose.offsetX();
    offset_y = pose.offsetY();
    return;
  }

  std::map<double, GlobalPose>::const_iterator before, after;
  int polation_mode = bracketPose(timestamp, before, after);
  const GlobalPose& pose1 = before->second;
  const GlobalPose& pose2 = after->second;

  if (polation_mode != 0) {
    if (!extrapolate_data_ || before->first == after->first) {
      // pose1 is always the one closest to requested ts
      x = pose1.x();
      y = pose1.y();
      offset_x = pose1.offsetX();
      offset_y = pose1.offsetY();
      return;
    }
    double ts1 = before->first;
    double ts2 = after->first;
    double dt = ts1 - ts2;

    if (std::abs(timestamp - ts1) > extrapolation_time_threshold_) {
      std::stringstream s;
      s << "Cannot determine smooth coordinates and offsets for requested timestamp " << timestamp << "since it's too far off (dt=" << timestamp - ts1 << ")";
      throw VLRException(s.str());
    }

    x = pose1.x() + (pose1.x() - pose2.x()) / dt * (timestamp - ts1);
    y = pose1.y() + (pose1.y() - pose2.y()) / dt * (timestamp - ts1);
    offset_x = pose1.offsetX() + (pose1.offsetX() - pose2.offsetX()) / dt * (timestamp - ts1);
    offset_y = pose1.offsetY() + (pose1.offsetY() - pose2.offsetY()) / dt * (timestamp - ts1);
    return;
  }

  double frac = (timestamp - before->first) / (after->first - before->first);

  x = pose1.x() + frac * (pose2.x() - pose1.x());
  y = pose1.y() + frac * (pose2.y() - pose1.y());
  offset_x = pose1.offsetX() + frac * (pose2.offsetX() - pose1.offsetX());
  offset_y = pose1.offsetY() + frac * (pose2.offsetY() - pose1.offsetY());
}

void PoseQueue::utmXY(double timestamp, double& utm_x, double& utm_y) {
  double x, y, offset_x, offset_y;
  xyAndOffsets(timestamp, x, y, offset_x, offset_y);
  utm_x = x + offset_x;
  utm_y = y + offset_y;
}

void PoseQueue::xyAndUtmXY(double timestamp, double& x, double& y, double& utm_x, double& utm_y) {
  // utm_x and utm_y are also used to temporarily store offsets
  xyAndOffsets(timestamp, x, y, utm_x, utm_y);
  utm_x += x;
  utm_y += y;
}

void PoseQueue::latestPose(GlobalPose& latest_pose, double& latest_timestamp) {
  if (queue_.empty()) {
    throw VLRException("GlobalPose queue is empty");
  }
  std::map<double, GlobalPose>::const_reverse_iterator latest = queue_.rbegin();
  latest_pose = latest->second;
  latest_timestamp = latest->first;
}

double PoseQueue::latestPoseTimestamp() {
  if (queue_.empty()) {
    throw VLRException("GlobalPose queue is empty");
  }

  return queue_.rbegin()->first;
}

GlobalPose PoseQueue::latestPose() {
  if (queue_.empty()) {
    throw VLRException("GlobalPose queue is empty");
  }
  const GlobalPose& tpose = queue_.rbegin()->second;
  return tpose;
}

GlobalPose PoseQueue::pose(double timestamp) {
  // empty case is handled in bracketPose
  if (queue_.size() == 1) {
    return queue_.begin()->second;
  }

  std::map<double, GlobalPose>::const_iterator before, after;
  int polation_mode = bracketPose(timestamp, before, after);
  const GlobalPose& pose1 = before->second;
  const GlobalPose& pose2 = after->second;

  double ts1 = before->first;
  double ts2 = after->first;
  if (polation_mode != 0) {
    if (!extrapolate_data_ || before->first == after->first) {
      // pose1 is always the one closest to requested ts
      return pose1;
    }
    if (std::abs(timestamp - ts1) > extrapolation_time_threshold_) {
      std::stringstream s;
      s << "Cannot determine pose for requested timestamp " << timestamp << "since it's too far off (dt=" << timestamp - ts1 << ")";
      throw VLRException(s.str());
    }
    GlobalPose begin = queue_.begin()->second;
    GlobalPose end = queue_.rbegin()->second;
    double dt = ts1 - ts2;

    GlobalPose res_pose;
    res_pose.x() = pose1.x() + (pose1.x() - pose2.x()) / dt * (timestamp - ts1);
    res_pose.y() = pose1.y() + (pose1.y() - pose2.y()) / dt * (timestamp - ts1);
    res_pose.z() = pose1.z() + (pose1.z() - pose2.z()) / dt * (timestamp - ts1);
    res_pose.offsetX() = pose1.offsetX() + (pose1.offsetX() - pose2.offsetX()) / dt * (timestamp - ts1);
    res_pose.offsetY() = pose1.offsetY() + (pose1.offsetY() - pose2.offsetY()) / dt * (timestamp - ts1);
    res_pose.offsetZ() = pose1.offsetZ() + (pose1.offsetZ() - pose2.offsetZ()) / dt * (timestamp - ts1);

    // TODO: implement proper angle extrapolation
    std::cout << "Warning: angle extrapolation not implemented yet.\n";
    res_pose.yaw() = pose1.yaw();// + (pose1.yaw() - pose2.yaw()) / dt * (timestamp - ts1);
    res_pose.pitch() = pose1.pitch() + (pose1.pitch() - pose2.pitch()) / dt * (timestamp - ts1);
    res_pose.roll() = pose1.roll() + (pose1.roll() - pose2.roll()) / dt * (timestamp - ts1);
    return res_pose;
  }

  double frac = (timestamp - ts1) / (ts2 - ts1);

  GlobalPose res_pose;
  res_pose.x() = pose1.x() + frac * (pose2.x() - pose1.x());
  res_pose.y() = pose1.y() + frac * (pose2.y() - pose1.y());
  res_pose.z() = pose1.z() + frac * (pose2.z() - pose1.z());
  res_pose.offsetX() = pose1.offsetX() + frac * (pose2.offsetX() - pose1.offsetX());
  res_pose.offsetY() = pose1.offsetY() + frac * (pose2.offsetY() - pose1.offsetY());
  res_pose.offsetZ() = pose1.offsetZ() + frac * (pose2.offsetZ() - pose1.offsetZ());

  res_pose.yaw() = interpolateYaw(pose1.yaw(), pose2.yaw(), frac);
  res_pose.pitch() = pose1.pitch() + frac * (pose2.pitch() - pose1.pitch());
  res_pose.roll() = pose1.roll() + frac * (pose2.roll() - pose1.roll());

  res_pose.v() = pose1.v() + frac * (pose2.v() - pose1.v());
  res_pose.vLateral() = pose1.vLateral() + frac * (pose2.vLateral() - pose1.vLateral());
  res_pose.vZ() = pose1.vZ() + frac * (pose2.vZ() - pose1.vZ());

  res_pose.a() = pose1.a() + frac * (pose2.a() - pose1.a());
  res_pose.aLateral() = pose1.aLateral() + frac * (pose2.aLateral() - pose1.aLateral());
  res_pose.aZ() = pose1.aZ() + frac * (pose2.aZ() - pose1.aZ());
  return res_pose;
}

// TODO: verify this works...
double PoseQueue::interpolateYaw(double head1, double head2, double fraction) {
  double result;

  if (head1 > 0 && head2 < 0 && head1 - head2 > M_PI) {
    head2 += 2 * M_PI;
    result = head1 + fraction * (head2 - head1);
    if (result > M_PI) {
      result -= 2 * M_PI;
    }
    return result;
  }
  else if (head1 < 0 && head2 > 0 && head2 - head1 > M_PI) {
    head1 += 2 * M_PI;
    result = head1 + fraction * (head2 - head1);
    if (result > M_PI) {
      result -= 2 * M_PI;
    }
    return result;
  }

  return head1 + fraction * (head2 - head1);
}

} // namespace driving_common
