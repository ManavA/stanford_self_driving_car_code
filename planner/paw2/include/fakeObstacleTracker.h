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


#ifndef FAKEOBSTACLEPERCEPTION_H_
#define FAKEOBSTACLEPERCEPTION_H_

#include <vector>
#include <driving_common/Trajectory2D.h>
#include <perception/PerceptionObstacles.h>
#include <obstaclePrediction.h>

namespace vlr {

struct CircleDemoCarState {
      double phi;
      double v;
      double r;
      double x;	// center of circle
      double y;	// center of circle
      double width;
      double length;
      double ref_offset;
};

class FakeObstacleTracker {

public:
	FakeObstacleTracker(std::vector<CircleDemoCarState>& init_states, double t_horizon=.1, double deltaT_sampling=3);
	virtual ~FakeObstacleTracker();

	inline void setParams(double checked_horizon, double deltaT_sampling) {
	  t_horizon_ = checked_horizon;
	  deltaT_sampling_ = deltaT_sampling;
	}

	void predictObstacles(double t, double offset_x, double offset_y, std::vector<ObstaclePrediction>& obstacle_predictions, perception::PerceptionObstacles& obstacle_msg);

private:
 std::vector<CircleDemoCarState>& car_states_;
 double t_horizon_;
 double deltaT_sampling_;
};

} // namespace vlr

#endif // FAKEOBSTACLEPERCEPTION_H_
