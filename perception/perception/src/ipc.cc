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


#include "perception.h"
#include "utils.h"
#include <limits.h>
#include <IL/il.h>

using namespace dgc;
using namespace vlr;
using std::tr1::shared_ptr;
using std::vector;

// ***********************************************************************
// *
// *
// *
// ***********************************************************************

#define   DYNAMIC_EXTRA_WIDTH_SPACE     4.0
#define   DYNAMIC_EXTRA_LENGTH_SPACE    4.0

int     send_map_reset     = 1;

void 
perception_register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages[] = {
    PerceptionObstaclesID,  PerceptionStopZonesID, HeartbeatID
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

// ***********************************************************************

int
point_inside_car( float px, float py, float box_x, float box_y,
		  float w, float l, float theta)
{
  double ctheta, stheta;
  double x1, y1, x2, y2;
  double perp_dist, par_dist, x_match, y_match;
  
  /* compute center axis vector */
  ctheta = cos(theta);
  stheta = sin(theta);
  x1  = box_x - (w / 2.0) * stheta;
  y1  = box_y + (w / 2.0) * ctheta;
  x2  = box_x - (-w / 2.0) * stheta;
  y2  = box_y + (-w / 2.0) * ctheta;
  dgc_point_to_line_distance(px, py, x1, y1, x2, y2, &par_dist,
			     &perp_dist, &x_match, &y_match);
  if(par_dist >= 0.0 && par_dist <= 1.0 && perp_dist <= l / 2.0)
    return 1;
  return 0;
}

void 
publishObstacles( IpcInterface *ipc, dgc_grid_p grid,
			      dgc_perception_map_cells_p points, vector< shared_ptr<TrackedObstacle> > obstacles,
			      unsigned short counter )
{
  static double last_time;

  int i, err;
  int num_obstacles = 0;

//    printf("%s %fx%f @ %f,%f\n", obstacle_type2str(obstacles[i]->type),
//        msg.dynamic_obstacle[i].x, msg.dynamic_obstacle[i].y,
//        msg.dynamic_obstacle[i].length, msg.dynamic_obstacle[i].width);

  num_obstacles = msg.num_dynamic_obstacles;
  printf("Really publishing %d obstacles\n", num_obstacles);
  ApplanixPose* pose;
  pose = applanix_current_pose();
  dgc_transform_t t;
  dgc_transform_t* radar;
  double time = dgc_get_time();
  if(last_time != 0.0)
  {
    printf("Interval: %f\n", time - last_time);
  }
  last_time = time;
  for (int r = 0; r < NUM_LRR3_RADARS; r++) {
    switch(r) {
      case 0: 
        radar = &radar_offset[2]; 
        pthread_mutex_lock(&radar_mutex[2]);
        break;
      case 1: 
        radar = &radar_offset[5]; 
        pthread_mutex_lock(&radar_mutex[5]);
        break;
    }

    dgc_transform_copy(t, *radar);
    dgc_transform_rotate_x( t, pose->roll );
    dgc_transform_rotate_y( t, pose->pitch );
    dgc_transform_rotate_z( t, pose->yaw );
    dgc_transform_translate( t, pose->smooth_x, pose->smooth_y, pose->smooth_z );
    /*
    for (i = 0; i < radar_lrr3[r].num_targets; i++) {
        
        RadarLRR3Target* target = &radar_lrr3[r].target[i];
        if (target->prob_exist < 0.9 || target->prob_obstacle < 0.4)
          continue;

        double x = target->long_distance + 2;
        double y = target->lateral_distance;
        double z = 0.0;
        dgc_transform_point(&x, &y, &z, t);

        double x_vel = target->long_relative_velocity + pose->speed;
        double y_vel = target->lateral_relative_velocity;
        double z_vel = 0.0;

        msg.dynamic_obstacle[num_obstacles].id = target->id;
        msg.dynamic_obstacle[num_obstacles].obstacleType = OBSTACLE_CAR;
        msg.dynamic_obstacle[num_obstacles].x = x;
        msg.dynamic_obstacle[num_obstacles].y = y;

        msg.dynamic_obstacle[num_obstacles].velocity = sqrt(x_vel * x_vel + y_vel * y_vel);
        msg.dynamic_obstacle[num_obstacles].direction = atan2(y_vel, x_vel) + pose->yaw;

        msg.dynamic_obstacle[num_obstacles].length = 4.0;
        msg.dynamic_obstacle[num_obstacles].width = 1.75;

        x = target->long_distance_std;
        y = target->lateral_distance_std;

        z = 0.0;
        dgc_transform_point(&x, &y, &z, t);
        msg.dynamic_obstacle[num_obstacles].confidence = 30.0;
        msg.dynamic_obstacle[num_obstacles].x_var = x;
        msg.dynamic_obstacle[num_obstacles].y_var = y;
        msg.dynamic_obstacle[num_obstacles].xy_cov = 0;
        num_obstacles++;
    }*/
    switch(r) {
      case 0: 
        pthread_mutex_unlock(&radar_mutex[2]);
        break;
      case 1: 
        pthread_mutex_unlock(&radar_mutex[5]);
        break;
    }
  }

  msg.num_dynamic_obstacles = num_obstacles;
  printf("%d obstacles with radar\n", num_obstacles);

  if (settings.gls_output) {
    // setup GLS header //
    gls->coordinates = GLS_SMOOTH_COORDINATES;
    gls->origin_x = 0;
    gls->origin_y = 0;
    gls->origin_z = applanix_current_pose()->smooth_z;
    glsColor3f( gls, 1.0, 0.0, 0.0 );
    glsLineWidth(gls, 2.0);
    for (int i=0; i < num_obstacles; i++) {
      glsPushMatrix(gls);
      PerceptionDynamicObstacle* obstacle = &msg.dynamic_obstacle[i];
      glsTranslatef(gls, obstacle->x, obstacle->y, 0);
      glsRotatef(gls, dgc_r2d(obstacle->direction), 0, 0, 1);

      float l = obstacle->length;
      float w = obstacle->width;
      glsBegin(gls, GLS_LINE_LOOP);
      glsVertex3f(gls, l / 2, w / 2, 0);
      glsVertex3f(gls, l / 2, -w / 2, 0);
      glsVertex3f(gls, -l / 2, -w / 2, 0);
      glsVertex3f(gls, -l / 2, w / 2, 0);
      glsVertex3f(gls, l / 2, 0, 0);
      glsVertex3f(gls, l / 2 + obstacle->velocity, 0, 0);
      glsVertex3f(gls, l / 2, 0, 0);
      glsVertex3f(gls, -l / 2, -w / 2, 0);
      glsVertex3f(gls, -l / 2, w / 2, 0);
      glsEnd(gls);

      glsPopMatrix(gls);
    }
  }

  err = ipc->Publish(PerceptionObstaclesID, &msg);
  TestIpcExit(err, "Could not publish", PerceptionObstaclesID);
}
