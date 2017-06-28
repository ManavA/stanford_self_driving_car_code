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


#ifndef BASE_DEMO_H_
#define BASE_DEMO_H_

#include <string>
#include <vlrException.h>
#include <applanix/ApplanixPose.h>
#include <localize/LocalizePose.h>
#include <perception/PerceptionObstacles.h>
#include <fakeObstacleTracker.h>

namespace vlr {

class BaseDemo {
public:
  BaseDemo(const std::string& rndf_name, const std::string& mdf_name, const double start_lat, const double start_lon, const double start_yaw);
  virtual ~BaseDemo();

  void updatePoses(bool init, const driving_common::TrajectoryPoint2D& current_trajectory_point,
                   applanix::ApplanixPose& applanix_pose, localize::LocalizePose& localize_pose);

  virtual void updateObstaclePredictions(double t) = 0;

  //virtual void setFakeTrackerParams(double checked_horizon, double deltaT_sampling) = 0;

//  inline std::vector<BaseDemoCarState>& getCarStates() {return car_states_;}
  inline const perception::PerceptionObstacles& getObstacleMessage() {return obstacle_msg_;}

protected:
    static const double meters2feet_ = 3.28083989501312;
//    FakeObstacleTracker* fake_tracker_;
    perception::PerceptionObstacles obstacle_msg_;
    std::string rndf_name_;
    std::string mdf_name_;
    double start_lat_, start_lon_;
    double start_x_, start_y_, start_yaw_;
    std::string utm_zone_;
    double smooth_x_start_, smooth_y_start_;
    double current_offset_x_, current_offset_y_;
    double current_timestamp_;
};

} // namespace vlr

#endif // BASE_DEMO_H_
