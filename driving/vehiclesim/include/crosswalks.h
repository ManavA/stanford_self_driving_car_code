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


#ifndef CROSSWALK_SIM_H_
#define CROSSWALK_SIM_H_

#include <vector>
#include <ros/ros.h>
#include <aw_roadNetwork.h>
#include <perception/PerceptionObstacles.h>
#include <obstacle_types.h>

namespace vlr {

struct SimCrosswalkPedestrian
{
    bool active;
    double speed;
    double utm_x, utm_y;
    double theta;
    double costheta;
    double sintheta;

    double current_x;
    double max_x;
    double next_time;
    char id[20];
};

class CrosswalkSimulator {

  struct CrosswalkParams {
      int generate_pedestrians;
      double max_wait_time;
      double min_speed;
      double max_speed;
    };

public:
  CrosswalkSimulator(rndf::RoadNetwork& rn, ros::NodeHandle& nh);
  ~CrosswalkSimulator();

  void update (double time);

  inline int maxObstacles() {
    return pedestrians_.size();
  };

  inline int maxObstaclePoints() {
    return pedestrians_.size() * 16;
  };


  inline const perception::PerceptionObstacles& getObstacles() {
    return obstacles_;
  };

private:

  void readParameters();

  ros::NodeHandle& nh_;

  CrosswalkParams params_;

  std::vector <SimCrosswalkPedestrian> pedestrians_;
//  vlr::rndf::RoadNetwork& rn_;

  double lastTime_;
  perception::PerceptionObstacles obstacles_;

};

} // namespace vlr

#endif
