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


#include <perception.h>
#include <utils.h>


using namespace dgc;
using namespace vlr;
using namespace std;
using namespace std::tr1;

namespace perception {

dgc::dgc_transform_t  radar_offset[NUM_RADARS];
double radar_ts[NUM_RADARS] = {0,0,0,0,0,0};

/*void perception_add_radar(vector< shared_ptr<RadarObservation> >& obstacles, double timestamp) {
  ApplanixPose* current = applanix_current_pose();
  dgc_transform_t robot_to_smooth;
  dgc_transform_identity(robot_to_smooth);

  dgc_transform_rotate_x( robot_to_smooth, current->roll );
  dgc_transform_rotate_y( robot_to_smooth, current->pitch );
  dgc_transform_rotate_z( robot_to_smooth, current->yaw );
  dgc_transform_translate( robot_to_smooth, current->smooth_x, current->smooth_y, current->smooth_z );

  float applanix_velocity = applanix_current_pose()->speed;

  // front radar
  dgc_transform_t radar_to_smooth;
  dgc_transform_copy(radar_to_smooth, radar_offset[2]);
  dgc_transform_left_multiply(radar_to_smooth, robot_to_smooth);
  for (int i = 0; i < radar_lrr3[0].num_targets; i++) {
    RadarLRR3Target* target = &(radar_lrr3[0].target[i]);
    double x = target->long_distance;
    double y = target->lateral_distance;
    double z = 0.0;

    double x_vel = target->long_relative_velocity;
    double y_vel = target->lateral_relative_velocity;

    dgc_transform_point(&x, &y, &z, radar_to_smooth);
    dgc_transform_point(&x_vel, &y_vel, &z, radar_to_smooth);

    shared_ptr<RadarObservation> obstacle(new RadarObservation());
    obstacle->x = x;
    obstacle->y = y;
    obstacle->x_vel = x_vel;
    obstacle->y_vel = y_vel;
    obstacles.push_back(obstacle);
  }

  if (radar_lrr3[0].timestamp > radar_ts[2]) {
    radar_ts[2] = radar_lrr3[0].timestamp;
  }

}*/

} // namespace perception
