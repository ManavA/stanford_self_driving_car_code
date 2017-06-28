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


#ifndef CIRCLE_DEMO_H_
#define CIRCLE_DEMO_H_

#include <string>
#include <applanix/ApplanixPose.h>
#include <localize/LocalizePose.h>
#include <perception/PerceptionObstacles.h>

#include <baseDemo.h>
#include <fakeObstacleTracker.h>

namespace vlr {

class CircleDemo : public BaseDemo {
public:
  CircleDemo(const std::string& rndf_name, const std::string& mdf_name, const double start_lat, const double start_lon, const double base_r);
  virtual ~CircleDemo();

  inline void updateObstaclePredictions(double t) {
    // obstacle_msg_.num_dynamic_obstacles = num_fast_cars_ + num_slow_cars_ + num_trucks_;
  }

  inline void setFakeTrackerParams(double checked_horizon, double deltaT_sampling) {
    fake_tracker_->setParams(checked_horizon, deltaT_sampling);
  }

  inline std::vector<CircleDemoCarState>& getCarStates() {return car_states_;}

private:
  bool createMultiCircleRNDF();
  bool createMultiCircleMDF(std::vector<uint32_t>& checkpoints);
  void generateTraffic(uint32_t num_fast_cars, uint32_t num_slow_cars, uint32_t num_trucks);

private:
  FakeObstacleTracker* fake_tracker_;
  std::vector<CircleDemoCarState> car_states_;
  double base_r_;
  double c_x_, c_y_;
  uint32_t num_lanes_;
  double lane_width_;
  uint32_t num_fast_cars_;
  uint32_t num_slow_cars_;
  uint32_t num_trucks_;

};

} // namespace vlr

#endif // CIRCLE_DEMO_H_
