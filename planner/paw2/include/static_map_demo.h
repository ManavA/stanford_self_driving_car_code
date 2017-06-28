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


#ifndef STATIC_MAP_DEMO_H_
#define STATIC_MAP_DEMO_H_

#include <string>
#include <applanix/ApplanixPose.h>
#include <localize/LocalizePose.h>
#include <perception/PerceptionObstacles.h>

#include <baseDemo.h>
//#include "fakeObstacleTracker.h"

namespace vlr {
class StaticMapDemo : public BaseDemo{
public:
  StaticMapDemo(const std::string& rndf_name, const std::string& mdf_name, const std::string& map_name,
             const double start_lat, const double start_lon, const double start_yaw);
  virtual ~StaticMapDemo();

  void updateObstaclePredictions(double t);
  void readObstacleMap(const std::string& map_name);

private:
  int32_t static_obstacle_map_size_x_;
  int32_t static_obstacle_map_size_y_;
  uint8_t* map_data_;
  double static_obstacle_map_resolution_;
  bool new_map_read_;
};

} // namespace vlr

#endif // STATIC_MAP_DEMO_H_
