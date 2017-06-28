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


#ifndef COLLISIONCHECK_H_
#define COLLISIONCHECK_H_

#include <stdint.h>
#include <vector>
#include <obstaclePrediction.h>
#include <polyTraj2D.h>

namespace vlr {
typedef enum {
  OSM_FREE=0, OSM_MAYBE_BLOCKED=128, OSM_BLOCKED=255
} ObstacleMapOccupancyState_t;

typedef enum {
  TRJ_FREE, TRJ_BLOCKED, TRJ_MAYBE_BLOCKED, TRJ_UNAVAILABLE
} TrjOccupancyState_t;

bool checkCollisionOfTrajectories(const double& width_safety, const double& width_no_safety,
    const double& length_safety, const double& length_no_safety, const double& pull_away_time, const double& offset,
    const std::vector<driving_common::TrajectoryPoint2D>& trajectory, const std::vector<ObstaclePrediction>& obstacle_predictions,
    double& collision_time);
// width/length_safety		.. includes desired distance to other obstacles
// width/length_no_safety 	.. describes car with a fairly small safety margin in order be robust to noise of obstacle position
// pull_away_time			.. time to pull away from a car that got too close

bool checkaCollisionCircles(const Vehicle::circle& c1, const Vehicle::circle& c2);
// returns true if circles coincide

bool checkCollisionVehicles(const double& x1, const double& y1, const double& psi1, const ObstaclePrediction& par1,
    const double& x2, const double& y2, const double& psi2, const ObstaclePrediction& par2);
// returns true if circular vehicle approximations coincide

} // namespace vlr

#endif
