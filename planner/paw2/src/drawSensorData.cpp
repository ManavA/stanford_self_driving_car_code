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

#include <velodyne.h>
#include <paw2Velodyne.h>
#include <paw2Camera.h>
#include <drawSensorData.h>

//#include "view.h"
//#include "trafficlights.h"
//#include <obstacle_types.h>
//#define MAX_SENSOR_TIME_DELAY    0.5
//
//using namespace std;
//using namespace vlr::rndf;

namespace drc = driving_common;

namespace vlr {

void Paw2Subscriber::unsubscribe() {
  if (subscribed_) {
    sub_.shutdown(); // TODO: verify
    subscribed_ = false;
  }
}

void Paw2Can::handler(const driving_common::CanStatus& status) {
  boost::unique_lock < boost::mutex > lock(mutex_);
  status_ = status;
  QString qs;
  qs.sprintf("%.2f", status_.throttle_position);
  gui_.ui.canStatusThrottlePos->setText(qs);

  qs.sprintf("%.2f", status_.steering_angle);
  gui_.ui.canStatusSteeringAngle->setText(qs);

  qs.sprintf("%.2f", status_.steering_rate);
  gui_.ui.canStatusSteeringRate->setText(qs);

  qs.sprintf("%.2f", status_.engine_rpm);
  gui_.ui.canStatusEngineRpm->setText(qs);

  qs.sprintf("%s", (status_.parking_brake ? "on" : "off"));
  gui_.ui.canStatusParkingBrake->setText(qs);

  qs.sprintf("%c", status_.target_gear);
  gui_.ui.canStatusTargetGear->setText(qs);

  qs.sprintf("%c", status_.gear_position);
  gui_.ui.canStatusGearPosition->setText(qs);
}

void Paw2ApplanixPose::handler(const applanix::ApplanixPose& pose) {
  boost::unique_lock<boost::mutex> lock(mutex_);
  pose_ = pose;
  QString qs;

  // Generic
  qs.sprintf("%.2f", applanix_hz_);
  gui_.ui.applanixStatusRate->setText(qs);

  qs.sprintf("%.2f", dgc::dgc_ms2mph(pose.speed));
  gui_.ui.applanixStatusSpeedMph->setText(qs);

  qs.sprintf("%.2f", dgc::dgc_ms2kph(pose.speed));
  gui_.ui.applanixStatusSpeedKph->setText(qs);

  qs.sprintf("%.2f", pose.speed);
  gui_.ui.applanixStatusSpeedMs->setText(qs);

  // Pose
  qs.sprintf("%.2f", pose.latitude);
  gui_.ui.applanixStatusLatitude->setText(qs);

  qs.sprintf("%.2f", pose.longitude);
  gui_.ui.applanixStatusLongitude->setText(qs);

  qs.sprintf("%.2f", pose.altitude);
  gui_.ui.applanixStatusAltitude->setText(qs);

  qs.sprintf("%.2f", pose.smooth_x);
  gui_.ui.applanixStatusPosX->setText(qs);

  qs.sprintf("%.2f", pose.smooth_y);
  gui_.ui.applanixStatusPosY->setText(qs);

  qs.sprintf("%.2f", pose.smooth_z);
  gui_.ui.applanixStatusPosZ->setText(qs);

  qs.sprintf("%.2f", pose.yaw);
  gui_.ui.applanixStatusYaw->setText(qs);

  qs.sprintf("%.2f", pose.pitch);
  gui_.ui.applanixStatusPitch->setText(qs);

  qs.sprintf("%.2f", pose.roll);
  gui_.ui.applanixStatusRoll->setText(qs);

  qs.sprintf("%.2f", pose.vel_east);
  gui_.ui.applanixStatusVelX->setText(qs);

  qs.sprintf("%.2f", pose.vel_north);
  gui_.ui.applanixStatusVelY->setText(qs);

  qs.sprintf("%.2f", pose.vel_up);
  gui_.ui.applanixStatusVelZ->setText(qs);

  qs.sprintf("%.2f", pose.rate_yaw);
  gui_.ui.applanixStatusYawRate->setText(qs);

  qs.sprintf("%.2f", pose.rate_pitch);
  gui_.ui.applanixStatusPitchRate->setText(qs);

  qs.sprintf("%.2f", pose.rate_roll);
  gui_.ui.applanixStatusRollRate->setText(qs);

  qs.sprintf("%.2f", pose.accel_x);
  gui_.ui.applanixStatusAccelX->setText(qs);

  qs.sprintf("%.2f", pose.accel_y);
  gui_.ui.applanixStatusAccelY->setText(qs);

  qs.sprintf("%.2f", pose.accel_z);
  gui_.ui.applanixStatusAccelZ->setText(qs);
}

void Paw2ApplanixRMS::handler(const applanix::ApplanixRMS& rms) {
  boost::unique_lock<boost::mutex> lock(mutex_);
  rms_ = rms;
  QString qs;

  qs.sprintf("%.5f", rms.rms_east);
  gui_.ui.applanixStatusErrPosX->setText(qs);

  qs.sprintf("%.5f", rms.rms_north);
  gui_.ui.applanixStatusErrPosY->setText(qs);

  qs.sprintf("%.5f", rms.rms_up);
  gui_.ui.applanixStatusErrPosZ->setText(qs);

  qs.sprintf("%.5f", rms.rms_yaw);
  gui_.ui.applanixStatusErrYaw->setText(qs);

  qs.sprintf("%.5f", rms.rms_pitch);
  gui_.ui.applanixStatusErrPitch->setText(qs);

  qs.sprintf("%.5f", rms.rms_roll);
  gui_.ui.applanixStatusErrRoll->setText(qs);

  qs.sprintf("%.5f", rms.rms_v_east);
  gui_.ui.applanixStatusErrVelX->setText(qs);

  qs.sprintf("%.5f", rms.rms_v_north);
  gui_.ui.applanixStatusErrVelY->setText(qs);

  qs.sprintf("%.5f", rms.rms_v_up);
  gui_.ui.applanixStatusErrVelZ->setText(qs);

  qs.sprintf("%.5f", rms.semi_major);
  gui_.ui.applanixStatusErrEllMax->setText(qs);

  qs.sprintf("%.5f", rms.semi_minor);
  gui_.ui.applanixStatusErrEllMin->setText(qs);

  qs.sprintf("%.5f", dgc::dgc_r2d(rms.orientation));
  gui_.ui.applanixStatusErrEllRot->setText(qs);
}

void Paw2ApplanixDMI::handler(const applanix::ApplanixDMI& dmi) {
  boost::unique_lock<boost::mutex> lock(mutex_);
  dmi_ = dmi;
  QString qs;

  qs.sprintf("%.2f", dmi.signed_odometer);
  gui_.ui.applanixStatusDMI->setText(qs);
}

std::vector<std::string> Paw2ApplanixGPS::gams_solution_;

void Paw2ApplanixGPS::handler(const applanix::ApplanixGPS& gps) {
  boost::unique_lock<boost::mutex> lock(mutex_);
  gps_ = gps;
  QString qs;

  qs.sprintf("%02i satellites", gps.primary_sats);
  gui_.ui.applanixStatusPrimarySatellites->setText(qs);

  qs.sprintf("%02i satellites", gps.secondary_sats);
  gui_.ui.applanixStatusSecondarySatellites->setText(qs);

  //int32_t code;
  qs.sprintf("%s", gams_solution_[gps.gams_solution_code].c_str());
  gui_.ui.applanixStatusGAMSSolution->setText(qs);
}

void Paw2GlView::drawSensorData(const drc::GlobalPose& pose) {
  if (gui_->show_velodyne_) {
    gui_->velodyne_->draw(pose);
  }

  //  // drawSensorData is called for planner display only...
  //  if(gui_->show_camera_lf_) {
  //    gui_->camera_lf_->draw(pose);
  //  }

  //  if(gui_->show_camera_rf_) {
  //    gui_->camera_rf_->draw(pose);
  //  }
}

//void
//display( void )
//{
//
//  char                          zone[5]           = "10S ";
//  static float                  marker_size = 0.0;
//  static float                  marker_size_move = 0.04;
//  static dgc_hsv_t              run_color = {0.0,1.0,1.0};
//  static float                  run_color_move = -0.075;
//  dgc_transform_t               t;
//  static double                 last_can_ts = 0;
//  static double                 last_gcar_ts = 0;
//  static double                 wheel_angle = 0;
//  static double                 velodyne_angle = 0;
//  char                          str[4096];
//  char                          str0[4096], str1[4096];
//  double                        theta, px, py, pz;
//  double                        sx = 0, sy = 0;
//  double                        scene_x = 0.0;
//  double                        scene_y = 0.0;
//  double                        dist, dangle, time, delta, lat, lon;
//  ApplanixPose                * pose, car_pose;
//  int                           fs, i, n, r, w, max_width, week;
//  dgc_rgb_t                     rgb1={0.0,0.0,0.0}, rgb2={0.0,0.0,0.0};
//  WayPoint*                     wp=NULL;
//  double                        current_time = dgc_get_time();
//  static double                 last_time = 0;
//  static double             radar_lrr2_py = LRR2_MAX_DIST * tan(dgc_d2r(LRR2_OPENING_HALF_ANGLE));
//  static double             radar_lrr3_py = LRR3_MAX_DIST * tan(dgc_d2r(LRR3_OPENING_HALF_ANGLE));
//  static int              radar_num_lrr2[NUM_LRR2_RADARS] = {1, 2, 4, 5};
//  static int              radar_num_lrr3[NUM_LRR3_RADARS] = {3, 6};
//
//
//  pthread_mutex_lock(&gls_mutex);
//  for(i = 0; i < (signed)gls_cache.size(); i++) {
//    if (current_time-gls_cache[i]->timestamp>GLS_REMOVE_TIME) {
//      free(gls_cache[i]->byte);
//      gls_cache.erase(gls_cache.begin()+i);
//    }
//  }
//  pthread_mutex_unlock(&gls_mutex);
//
//  marker_size += marker_size_move;
//  if (marker_size<0.0 || marker_size>1.0)
//    marker_size_move *= -1;
//
//  glEnable(GL_BLEND);
//  glEnable(GL_LINE_SMOOTH);
//  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//
//  // *******************************************************************
//  // *    Draw ipc grid
//  // *******************************************************************
//  if (show_ipc_grid_map) {
//    pthread_mutex_lock( &map_mutex );
//    draw_grid_map( ipc_grid, &car_pose, (char *)ipc_grid->cell, 0 );
//    pthread_mutex_unlock( &map_mutex );
//  }
//
//  // *******************************************************************
//  // *    Draw grid
//  // *******************************************************************
//  glPushMatrix();
//  {
//    if(show_grid) {
//      glDisable(GL_DEPTH_TEST);
//      if (!plain_mode)
//        glTranslatef(0, 0, 0.01-DGC_PASSAT_HEIGHT);
//      draw_grid(&car_pose, 0, 0);
//      glEnable(GL_DEPTH_TEST);
//    }
//    if(show_distances) {
//      glDisable(GL_DEPTH_TEST);
//      if (!plain_mode)
//        glTranslatef(0, 0, 0.01-DGC_PASSAT_HEIGHT);
//      draw_distances(&car_pose);
//      glEnable(GL_DEPTH_TEST);
//    }
//  }
//  glPopMatrix();
//
//
//  // *******************************************************************
//  // *    Draw the trajectory
//  // *******************************************************************
//  if(show_trajectory) {
//    glColor3f( 0, 0.8, 0);
//    glLineWidth(3.0);
//    pthread_mutex_lock( &traj_mutex );
//    glPushMatrix();
//    {
//      if (!plain_mode)
//        glTranslatef(0, 0, 0.15-DGC_PASSAT_HEIGHT);
//      glBegin(GL_LINE_STRIP);
//      for(i = 0; i < planner_trajectory.num_waypoints-1; i++)
//        myglVertex3f(planner_trajectory.waypoint[i].x - car_pose.smooth_x,
//            planner_trajectory.waypoint[i].y - car_pose.smooth_y,
//            0.2);
//      glEnd();
//      glLineWidth(1.0);
//      n = planner_trajectory.num_waypoints-2;
//      if (n>=0) {
//        glPushMatrix();
//        {
//          theta = atan2( planner_trajectory.waypoint[n].y -
//              planner_trajectory.waypoint[n-1].y,
//              planner_trajectory.waypoint[n].x -
//              planner_trajectory.waypoint[n-1].x );
//          glTranslatef( planner_trajectory.waypoint[n].x - car_pose.smooth_x,
//              planner_trajectory.waypoint[n].y - car_pose.smooth_y,
//              0.0 );
//          glRotatef(dgc_r2d(theta),   0, 0, 1);
//          draw_narrow_arrow( 0.0, 0.0, 0.2, 1.5 );
//        }
//        glPopMatrix();
//
//      }
//    }
//    glPopMatrix();
//    pthread_mutex_unlock( &traj_mutex );
//
//    pthread_mutex_lock(&traj2D_mutex);
//    glPushMatrix();
//    {
//      if (!plain_mode)
//        glTranslatef(0, 0, 0.15 - DGC_PASSAT_HEIGHT);
//
//      glColor3f( .8, 0., 0.);
//      glBegin(GL_LINE_STRIP);
//      for (i = 0; i < trajectory_points.num_points; i++)
//        myglVertex3f(trajectory_points.points[i].x - car_pose.smooth_x,
//            trajectory_points.points[i].y - car_pose.smooth_y, 0.2);
//      glEnd();
//      glColor3f( .8, .8, 0.);
//      glLineWidth(1.0);
//      for (i = 0; i < trajectory_points.num_points; i++)
//      {
//        glPushMatrix();
//        glTranslatef(trajectory_points.points[i].x - car_pose.smooth_x,
//            trajectory_points.points[i].y - car_pose.smooth_y, 0.0);
//        glRotatef(dgc_r2d(trajectory_points.points[i].theta), 0, 0, 1);
//        draw_narrow_arrow(0.0, 0.0, 0.2, 1.+trajectory_points.points[i].a/2.);
//
//        glPopMatrix();
//      }
//    }
//    glPopMatrix();
//    pthread_mutex_unlock(&traj2D_mutex);
//
//  }
//
//  if (show_planner_goal &&
//      goal_place_active != GOAL_NOT_SET) {
//
//    glPushMatrix();
//    {
//      if (!plain_mode)
//        glTranslatef(0, 0, 0.15-DGC_PASSAT_HEIGHT);
//
//      gui3D_pick_point( mouse_pos_x, mouse_pos_y, &scene_x, &scene_y );
//      latLongToUtm( goal_place.lat, goal_place.lon, &sx, &sy, zone );
//      sx -= global_x;
//      sy -= global_y;
//      switch (goal_place_active) {
//      case GOAL_DO_SET:
//        utmToLatLong( global_x + scene_x, global_y + scene_y, utmzone,
//            &goal_place.lat, &goal_place.lon );
//        sx = scene_x;
//        sy = scene_y;
//        draw_cross_hair( sx, sy, 0.15, 1.25 );
//        break;
//      case GOAL_XY_SET:
//        goal_place.theta = atan2(scene_y-sy,scene_x-sx);
//        glTranslatef(sx, sy, 0.15);
//        glRotatef(dgc_r2d(goal_place.theta),   0, 0, 1);
//        glColor4f(1, 0, 0, 0.4);
//        glTranslatef(1.65, 0.0, 0.0);
//        draw_arrow( 0.0, 0.0, 0.05, 2.0 );
//        glColor4f(1, 0.5, 0.5, 0.2);
//        fprintf( stderr, "=> %f %f\n", scene_x, scene_y );
//        draw_car_box( DGC_PASSAT_LENGTH, DGC_PASSAT_WIDTH );
//        break;
//      case GOAL_ALL_SET:
//        if (show_inverse) {
//          glColor4f(0, 0, 1, 0.4);
//        } else {
//          glColor4f(1, 1, 0, 0.4);
//        }
//        glTranslatef(sx, sy, 0.15);
//        glRotatef(dgc_r2d(goal_place.theta),   0, 0, 1);
//        glTranslatef(1.65, 0.0, 0.0);
//        if (show_inverse) {
//          glColor4f(1, 1, 0, 0.4);
//        } else {
//          glColor4f(0, 0, 1, 0.4);
//        }
//        draw_arrow( 0.0, 0.0, 0.05, 2.0 );
//        if (show_inverse) {
//          glColor4f(0, 0, 1, 0.4);
//        } else {
//          glColor4f(1, 1, 0, 0.4);
//        }
//        draw_car_box( DGC_PASSAT_LENGTH, DGC_PASSAT_WIDTH );
//        break;
//      default:
//        break;
//      }
//    }
//    glPopMatrix();
//  }
//
//  if (show_coordinates) {
//    glPushMatrix();
//    {
//      if (!plain_mode)
//        glTranslatef(0, 0, 0.15-DGC_PASSAT_HEIGHT);
//      gui3D_pick_point( mouse_pos_x, mouse_pos_y, &scene_x, &scene_y );
//      utmToLatLong( global_x+scene_x, global_y+scene_y, utmzone, &lat, &lon );
//      draw_cross_hair( scene_x, scene_y, 0.15, 1.25 );
//      draw_msg( "sm-x: %.2f  sm-y: %.2f  lat: %f  lon: %f",
//          scene_x+car_pose.smooth_x,
//          scene_y+car_pose.smooth_y,
//          lat, lon );
//    }
//    glPopMatrix();
//  }
//
//  if ( show_planner_goal &&
//      planner_goal.goal_set) {
//
//    glPushMatrix();
//    {
//      if (!plain_mode) {
//        glTranslatef(planner_goal.utm_x - global_x,
//            planner_goal.utm_y - global_y,
//            0.20-DGC_PASSAT_HEIGHT );
//      } else {
//        glTranslatef(planner_goal.utm_x - global_x,
//            planner_goal.utm_y - global_y,
//            0.20 );
//      }
//      glRotatef(dgc_r2d(planner_goal.theta), 0, 0, 1);
//      glTranslatef(1.65, 0.0, 0.0);
//      glColor3f( 1.0, 0.5, 0 );
//      draw_arrow( 0.0, 0.0, 0.1, 2.0 );
//      glLineWidth(3.0);
//      if (show_inverse) {
//        glColor3f( 0.0, 0.0, 0.0 );
//      } else {
//        glColor3f( 0.5, 0.5, 0.5 );
//      }
//      draw_empty_box( DGC_PASSAT_LENGTH, DGC_PASSAT_WIDTH );
//    }
//    glPopMatrix();
//  }
//
//  // *******************************************************************
//  // *    Draw goal
//  // *******************************************************************
//  if(show_rndf && show_goal) {
//    pthread_mutex_lock( &goal_mutex );
//    if (planner_mdf_goal.goal_s>=0 &&
//        planner_mdf_goal.goal_l>=0 &&
//        planner_mdf_goal.goal_w>=0) {
//      //      w = rndf->lookup_waypoint(planner_mdf_goal.goal_s,  // TODO: Fix goal visualization
//      //               planner_mdf_goal.goal_l,
//      //               planner_mdf_goal.goal_w);
//      if(wp != NULL) {
//
//        GLUquadricObj   * quad = gluNewQuadric();
//        GLfloat           material[]  = {0.0, 0.0, 1.0, 0.0};
//
//        gluQuadricNormals(quad, GLU_SMOOTH);
//        gluQuadricTexture(quad, GL_TRUE);
//        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, material);
//
//        glPushMatrix();
//        {
//          if (!plain_mode) {
//            glTranslatef(wp->utm_x() - global_x, wp->utm_y() - global_y, 0.0 - DGC_PASSAT_HEIGHT);
//          }
//          else {
//            glTranslatef(wp->utm_x() - global_x, wp->utm_y() - global_y, 0.0);
//          }
//          gluCylinder(quad, marker_size * 2.0f, 0.0f, 0.25f + marker_size * 0.25f, 32, 32);
//
//          //    glRotatef(dgc_r2d(w->heading()) - 90, 0, 0, 1);  // TODO: Fix waypoint orientation..maybe..
//          glTranslatef(-1.0, 0.0, 0.21);
//          sprintf(str, "%d of %d", planner_mdf_goal.current_goal_num, planner_mdf_goal.num_goals);
//          render_stroke_text_2D(0, 0, GLUT_STROKE_ROMAN, 0.4, str);
//        }
//        glPopMatrix();
//
//      }
//
//    }
//    pthread_mutex_unlock( &goal_mutex );
//    glLineWidth(1.0);
//  }
//
//  // *******************************************************************
//  // *    Draw the VELO scan
//  // *******************************************************************
//
//  glPointSize(show_point_size);
//  if (show_point_size>1) {
//    glLineWidth(3.0);
//  } else {
//    glPointSize(1.0);
//  }
//
//  if (show_inverse) {
//    glColor3f( 0, 0, 0);
//  } else {
//    glColor3f( crgb[color_num].r,
//        crgb[color_num].g,
//        crgb[color_num].b );
//  }
//
//  if (show_velodyne) {
//    draw_velodyne( num_scans, scans, &car_pose );
//  }
//
//  // *******************************************************************
//  // * IPC obstacles message
//  // *******************************************************************
//
//  if (show_obstacles && obstacles.num_points>0) {
//
//    /* Draw point obstacles */
//    glPushMatrix();
//    {
//      glColor3f( 1.0, 0.0, 0.0 );
//
//      pthread_mutex_lock( &obstacles_mutex );
//
//      rgb1.r = 0.0;
//      rgb1.g = 0.0;
//      rgb1.b = 0.0;
//      for(i = 0; i < obstacles.num_points; i++) {
//        if (show_dynamic && obstacles.point[i].type!=0) {
//          if (show_inverse) {
//            rgb2.r = 0.0;
//            rgb2.g = 0.0;
//            rgb2.b = 1.0;
//          } else {
//            rgb2.r = 1.0;
//            rgb2.g = 1.0;
//            rgb2.b = 0.0;
//          }
//        } else {
//          rgb2.r = 1.0;
//          rgb2.g = 0.0;
//          rgb2.b = 0.0;
//        }
//        if (show_dynamic && obstacles.point[i].type!=0) {
//          draw_cube( obstacles.point[i].x-car_pose.smooth_x,
//              obstacles.point[i].y-car_pose.smooth_y,
//              obstacles.point[i].z_min-car_pose.smooth_z,
//              obstacles.point[i].z_max-car_pose.smooth_z,
//              0.1, rgb1, rgb2, 0.8 );
//        } else {
//          draw_cube( obstacles.point[i].x-car_pose.smooth_x,
//              obstacles.point[i].y-car_pose.smooth_y,
//              obstacles.point[i].z_min-car_pose.smooth_z,
//              obstacles.point[i].z_max-car_pose.smooth_z,
//              0.1, rgb1, rgb2, 0.8 );
//        }
//      }
//
//      pthread_mutex_unlock( &obstacles_mutex );
//    }
//    glPopMatrix();
//  }
//
//
//
//
//  if(show_track_classifications && obstacles.num_points>0) {
//
//    /* Draw dynamic obstacles (i.e. pedestrians, etc.) */
//    glPushMatrix();
//    {
//      pthread_mutex_lock( &obstacles_mutex );
//      for (i = 0; i < obstacles.num_dynamic_obstacles; i++){
//  switch (obstacles.dynamic_obstacle[i].obstacleType){
//  case (OBSTACLE_CAR):
//    rgb2.r = 1.0;  /* car color setting */
//    rgb2.g = 0.0;
//    rgb2.b = 0.0;
//    draw_obstacle_frame( obstacles.dynamic_obstacle[i].x-car_pose.smooth_x,
//             obstacles.dynamic_obstacle[i].y-car_pose.smooth_y,
//             0,
//             obstacles.dynamic_obstacle[i].length,
//             obstacles.dynamic_obstacle[i].width,
//             obstacles.dynamic_obstacle[i].direction,
//             obstacles.dynamic_obstacle[i].velocity,
//             rgb2);
//    break;
//
//  case (OBSTACLE_PEDESTRIAN):
//    rgb2.r = 0.98;  /* skin color setting */
//    rgb2.g = 0.76;
//    rgb2.b = 0.44;
//    draw_pedestrian( obstacles.dynamic_obstacle[i].x-car_pose.smooth_x,
//         obstacles.dynamic_obstacle[i].y-car_pose.smooth_y,
//         -DGC_PASSAT_HEIGHT+1.5, dgc_feet2meters(6.0), 0.35,
//         obstacles.dynamic_obstacle[i].direction,
//         obstacles.dynamic_obstacle[i].velocity, rgb2, 0.8);
//    break;
//
//  case (OBSTACLE_BICYCLIST):
//    rgb2.r = 0.1;  /* skin color setting */
//    rgb2.g = 0.9;
//    rgb2.b = 0.1;
//    draw_bicyclist(obstacles.dynamic_obstacle[i].x-car_pose.smooth_x,
//       obstacles.dynamic_obstacle[i].y-car_pose.smooth_y,
//       -DGC_PASSAT_HEIGHT+1.5, dgc_feet2meters(6.0),
//       obstacles.dynamic_obstacle[i].direction,
//       obstacles.dynamic_obstacle[i].velocity, rgb2, 0.8);
//    break;
//
//  case (OBSTACLE_UNKNOWN):
//  default:
//    rgb2.r = 1.0;  /* unknown color setting */
//    rgb2.g = 1.0;
//    rgb2.b = 0.0;
//    draw_obstacle_frame( obstacles.dynamic_obstacle[i].x-car_pose.smooth_x,
//             obstacles.dynamic_obstacle[i].y-car_pose.smooth_y,
//             0,
//             obstacles.dynamic_obstacle[i].length,
//             obstacles.dynamic_obstacle[i].width,
//             obstacles.dynamic_obstacle[i].direction,
//             obstacles.dynamic_obstacle[i].velocity,
//             rgb2);
//    break;
//  }
//      }
//
//      pthread_mutex_unlock( &obstacles_mutex );
//    }
//    glPopMatrix();
//  }
//
//    // -- Draw small icon for frame classification.
//  if(show_frame_classifications && obstacles.num_points>0) {
//    glPushMatrix();
//    {
//      pthread_mutex_lock( &obstacles_mutex );
//
//      for(i = 0; i < obstacles.num_dynamic_obstacles; i++){
//
//  cout << i << ": classified this frame: " << obstacles.dynamic_obstacle[i].classifiedThisFrame << endl;
//  if(obstacles.dynamic_obstacle[i].classifiedThisFrame == 0) {
//    rgb2.r = 0.5;
//    rgb2.g = 0.5;
//    rgb2.b = 0.5;
//    draw_obstacle_frame( obstacles.dynamic_obstacle[i].x-car_pose.smooth_x+0.5,
//                                 obstacles.dynamic_obstacle[i].y-car_pose.smooth_y+0.5,
//                                 -DGC_PASSAT_HEIGHT+3.5,
//                                 obstacles.dynamic_obstacle[i].length / 4.0,
//                                 obstacles.dynamic_obstacle[i].width / 4.0,
//                                 obstacles.dynamic_obstacle[i].direction,
//                                 obstacles.dynamic_obstacle[i].velocity / 5.0,
//                                 rgb2);
//    continue;
//  }
//
//  switch (obstacles.dynamic_obstacle[i].obstacleTypeThisFrame){
//          case (OBSTACLE_UNKNOWN):
//            rgb2.r = 1.0;
//            rgb2.g = 1.0;
//            rgb2.b = 0.0;
//            draw_obstacle_frame( obstacles.dynamic_obstacle[i].x-car_pose.smooth_x+0.5,
//                                 obstacles.dynamic_obstacle[i].y-car_pose.smooth_y+0.5,
//                                 -DGC_PASSAT_HEIGHT+3.5,
//                                 obstacles.dynamic_obstacle[i].length / 4.0,
//                                 obstacles.dynamic_obstacle[i].width / 4.0,
//                                 obstacles.dynamic_obstacle[i].direction,
//                                 obstacles.dynamic_obstacle[i].velocity / 5.0,
//                                 rgb2);
//            break;
//
//
//  case (OBSTACLE_CAR):
//            rgb2.r = 1.0;  /* car color setting */
//            rgb2.g = 0.0;
//            rgb2.b = 0.0;
//            draw_obstacle_frame( obstacles.dynamic_obstacle[i].x-car_pose.smooth_x+0.5,
//                                 obstacles.dynamic_obstacle[i].y-car_pose.smooth_y+0.5,
//                                 -DGC_PASSAT_HEIGHT+3.5,
//                                 obstacles.dynamic_obstacle[i].length / 4.0,
//                                 obstacles.dynamic_obstacle[i].width / 4.0,
//                                 obstacles.dynamic_obstacle[i].direction,
//                                 obstacles.dynamic_obstacle[i].velocity / 5.0,
//                                 rgb2);
//            break;
//
//          case (OBSTACLE_PEDESTRIAN):
//            rgb2.r = 0.98;  /* skin color setting */
//            rgb2.g = 0.76;
//            rgb2.b = 0.44;
//            draw_pedestrian( obstacles.dynamic_obstacle[i].x-car_pose.smooth_x+0.5,
//                             obstacles.dynamic_obstacle[i].y-car_pose.smooth_y+0.5,
//                             -DGC_PASSAT_HEIGHT+3.5, 0.3, 0.15,
//                             obstacles.dynamic_obstacle[i].direction,
//                             obstacles.dynamic_obstacle[i].velocity / 5.0, rgb2, 0.8);
//            break;
//
//  case (OBSTACLE_BICYCLIST):
//            rgb2.r = 0.1;  /* skin color setting */
//            rgb2.g = 0.9;
//            rgb2.b = 0.1;
//      draw_bicyclist(obstacles.dynamic_obstacle[i].x-car_pose.smooth_x+0.5,
//         obstacles.dynamic_obstacle[i].y-car_pose.smooth_y+0.5,
//         -DGC_PASSAT_HEIGHT+3.5, dgc_feet2meters(2.0),
//         obstacles.dynamic_obstacle[i].direction,
//         obstacles.dynamic_obstacle[i].velocity / 5.0, rgb2, 0.8);
//            break;
//
//          default:
//            break;
//        }
//      }
//
//      pthread_mutex_unlock( &obstacles_mutex );
//    }
//    glPopMatrix();
//  }
//
//
//
//
//
//
//
//  // *******************************************************************
//  // *    Draw the RADAR scan
//  // *******************************************************************
//  if (show_radar) {
//
//    /* Process LRR2 radars */
//    for(r = 0; r < NUM_LRR2_RADARS; r++) {
//
//      pose = applanix_pose(radar_lrr2[r].timestamp);
//      dgc_transform_copy(t, radar_lrr2_offset[r]);
//      dgc_transform_rotate_x(t, pose->roll);
//      dgc_transform_rotate_y(t, pose->pitch);
//      dgc_transform_rotate_z(t, pose->yaw);
//
//      if (show_radar_cal) {
//
//        /* Draw radar visible area */
//        glPushMatrix();
//        {
//          glColor4f( 1.0, 0.0, 0.0, 0.3);
//
//          glLineWidth(2.0);
//          glBegin(GL_POLYGON);
//
//          px = 0.0; py = 0.0; pz = 0.0;
//          dgc_transform_point(&px, &py, &pz, t);
//          myglVertex3f( px, py, pz );
//
//          px = LRR2_MAX_DIST; py = -radar_lrr2_py; pz = 0.0;
//          dgc_transform_point(&px, &py, &pz, t);
//          myglVertex3f( px, py, pz );
//
//          px = LRR2_MAX_DIST; py = radar_lrr2_py; pz = 0.0;
//          dgc_transform_point(&px, &py, &pz, t);
//          myglVertex3f( px, py, pz );
//
//          glEnd();
//        }
//        glPopMatrix();
//
//        /* Draw radar centerline */
//        glPushMatrix();
//        {
//          glColor4f( 1.0, 0.0, 0.0, 0.8);
//
//          glLineWidth(2.0);
//          glBegin(GL_LINE);
//
//          px = 0.0; py = 0.0; pz = 0.0;
//          dgc_transform_point(&px, &py, &pz, t);
//          myglVertex3f( px, py, pz );
//
//          px = LRR2_MAX_DIST; py = 0.0; pz = 0.0;
//          dgc_transform_point(&px, &py, &pz, t);
//          myglVertex3f( px, py, pz );
//
//          glEnd();
//        }
//        glPopMatrix();
//      }
//
//      if (radar_lrr2[r].num_targets>0) {
//
//        glPushMatrix();
//        {
//          if (plain_mode)
//            dgc_transform_translate
//            (t,
//                pose->smooth_x-car_pose.smooth_x,
//                pose->smooth_y-car_pose.smooth_y,
//                0.0 );
//          else
//            dgc_transform_translate
//            (t,
//                pose->smooth_x-car_pose.smooth_x,
//                pose->smooth_y-car_pose.smooth_y,
//                pose->smooth_z-car_pose.smooth_z);
//
//          rgb1.r = 0.0;
//          rgb1.g = 0.0;
//          rgb1.b = 0.0;
//
//          pthread_mutex_lock( &radar_mutex );
//          for(i = 0; i < radar_lrr2[r].num_targets; i++) {
//            px = radar_lrr2[r].target[i].distance;
//            py = radar_lrr2[r].target[i].lateral_offset;
//            pz = 0;
//            dgc_transform_point(&px, &py, &pz, t);
//            if (show_radar_cal) {
//              draw_radar_cal( px, py, 0.3,
//                  4*RADAR_MARKER_SIZE,
//                  pose, radar_num_lrr2[r], radar_lrr2[r].target[i] );
//            } else {
//              if (show_all_radars ||
//                  (radar_lrr2[r].target[i].measured &&
//                      fabs(radar_lrr2[r].target[i].relative_velocity)>dgc_mph2ms(0.5) &&
//                      fabs(radar_lrr2[r].target[i].relative_velocity)<dgc_mph2ms(40.0) )) {
//                switch(r) {
//                case 0: case 4:
//                  rgb2.r = 0.5;
//                  rgb2.g = 1.0;
//                  rgb2.b = 0.0;
//                  break;
//                case 1: case 3:
//                  rgb2.r = 1.0;
//                  rgb2.g = 1.0;
//                  rgb2.b = 0.0;
//                  break;
//                case 2:
//                  rgb2.r = 1.0;
//                  rgb2.g = 0.5;
//                  rgb2.b = 0.0;
//                  break;
//                default:
//                  fprintf( stderr, "-> %d\n", r );
//                  rgb2.r = 1.0;
//                  rgb2.g = 1.0;
//                  rgb2.b = 0.0;
//                  break;
//                }
//                draw_radar_spot( px, py, 0.0,
//                    RADAR_MARKER_SIZE,
//                    0.2, rgb1, rgb2, 0.85 );
//              }
//            }
//          }
//          pthread_mutex_unlock( &radar_mutex );
//
//        } glPopMatrix();
//      }
//    }
//
//    /* Process LRR3 radars */
//    for(r = 0; r < NUM_LRR3_RADARS; r++) {
//
//      pose = applanix_pose(radar_lrr3[r].timestamp);
//      dgc_transform_copy(t, radar_lrr3_offset[r]);
//      dgc_transform_rotate_x(t, pose->roll);
//      dgc_transform_rotate_y(t, pose->pitch);
//      dgc_transform_rotate_z(t, pose->yaw);
//
//      if (show_radar_cal) {
//
//        /* Draw radar visible area */
//        glPushMatrix();
//        {
//          glColor4f( 0.0, 0.0, 1.0, 0.3);
//
//          glLineWidth(2.0);
//          glBegin(GL_POLYGON);
//
//          px = 0.0; py = 0.0; pz = 0.0;
//          dgc_transform_point(&px, &py, &pz, t);
//          myglVertex3f( px, py, pz );
//
//          px = LRR3_MAX_DIST; py = -radar_lrr3_py; pz = 0.0;
//          dgc_transform_point(&px, &py, &pz, t);
//          myglVertex3f( px, py, pz );
//
//          px = LRR3_MAX_DIST; py = radar_lrr3_py; pz = 0.0;
//          dgc_transform_point(&px, &py, &pz, t);
//          myglVertex3f( px, py, pz );
//
//          glEnd();
//        }
//        glPopMatrix();
//
//        /* Draw radar center line */
//        glPushMatrix();
//        {
//          glColor4f( 0.0, 0.0, 1.0, 0.8);
//
//          glLineWidth(2.0);
//          glBegin(GL_LINE);
//
//          px = 0.0; py = 0.0; pz = 0.0;
//          dgc_transform_point(&px, &py, &pz, t);
//          myglVertex3f( px, py, pz );
//
//          px = LRR3_MAX_DIST; py = 0.0; pz = 0.0;
//          dgc_transform_point(&px, &py, &pz, t);
//          myglVertex3f( px, py, pz );
//
//          glEnd();
//        }
//        glPopMatrix();
//      }
//
//      if (radar_lrr3[r].num_targets>0) {
//
//        glPushMatrix();
//        {
//          if (plain_mode)
//            dgc_transform_translate
//            (t,
//                pose->smooth_x-car_pose.smooth_x,
//                pose->smooth_y-car_pose.smooth_y,
//                0.0 );
//          else
//            dgc_transform_translate
//            (t,
//                pose->smooth_x-car_pose.smooth_x,
//                pose->smooth_y-car_pose.smooth_y,
//                pose->smooth_z-car_pose.smooth_z);
//
//          rgb1.r = 0.0;
//          rgb1.g = 0.0;
//          rgb1.b = 0.0;
//
//          pthread_mutex_lock( &radar_mutex );
//          for(i = 0; i < radar_lrr3[r].num_targets; i++) {
//            px = radar_lrr3[r].target[i].long_distance;
//            py = radar_lrr3[r].target[i].lateral_distance;
//            pz = 0;
//            dgc_transform_point(&px, &py, &pz, t);
//            if (show_radar_cal) {
//              draw_radar_lrr3_cal( px, py, 0.3,
//                  4*RADAR_MARKER_SIZE,
//                  pose, radar_num_lrr3[r], radar_lrr3[r].target[i] );
//            } else {
//              if (show_all_radars ||
//                  (radar_lrr3[r].target[i].measured &&
//                      fabs(radar_lrr3[r].target[i].long_relative_velocity)>dgc_mph2ms(0.5) &&
//                      fabs(radar_lrr3[r].target[i].long_relative_velocity)<dgc_mph2ms(40.0) )) {
//                switch(r) {
//                case 0: case 4:
//                  rgb2.r = 0.5;
//                  rgb2.g = 1.0;
//                  rgb2.b = 0.0;
//                  break;
//                case 1: case 3:
//                  rgb2.r = 1.0;
//                  rgb2.g = 1.0;
//                  rgb2.b = 0.0;
//                  break;
//                case 2:
//                  rgb2.r = 1.0;
//                  rgb2.g = 0.5;
//                  rgb2.b = 0.0;
//                  break;
//                default:
//                  fprintf( stderr, "-> %d\n", r );
//                  rgb2.r = 1.0;
//                  rgb2.g = 1.0;
//                  rgb2.b = 0.0;
//                  break;
//                }
//                draw_radar_spot( px, py, 0.0,
//                    RADAR_MARKER_SIZE,
//                    0.2, rgb1, rgb2, 0.85 );
//              }
//            }
//          }
//          pthread_mutex_unlock( &radar_mutex );
//
//        } glPopMatrix();
//      }
//    }
//  }
//
//  // *******************************************************************
//  // *    Draw the LDLRS1 scan
//  // *******************************************************************
//  if (show_ldlrs1 &&
//      fabs(ldlrs1.timestamp-
//          car_pose.timestamp)<MAX_SENSOR_TIME_DELAY) {
//    if (show_colors) {
//      glColor3f( crgb[cycle_colors(change_colors+2)].r,
//          crgb[cycle_colors(change_colors+2)].g,
//          crgb[cycle_colors(change_colors+2)].b );
//    } else {
//      glColor3f( 0, 0, 1);
//    }
//    draw_ldlrs( &ldlrs1, &car_pose, ldlrs1_offset, &ldlrs1_mutex );
//  }
//
//  // *******************************************************************
//  // *    Draw the LDLRS2 scan
//  // *******************************************************************
//  if (show_ldlrs2 &&
//      fabs(ldlrs2.timestamp-
//          car_pose.timestamp)<MAX_SENSOR_TIME_DELAY) {
//    if (show_colors) {
//      glColor3f( crgb[cycle_colors(change_colors+3)].r,
//          crgb[cycle_colors(change_colors+3)].g,
//          crgb[cycle_colors(change_colors+3)].b );
//    } else {
//      glColor3f( 1, 0, 1);
//    }
//    draw_ldlrs( &ldlrs2, &car_pose, ldlrs2_offset, &ldlrs2_mutex );
//  }
//
//  // *******************************************************************
//  // *    Draw the LMS1 scan
//  // *******************************************************************
//  if (show_lms1) {
//    if (show_colors) {
//      glColor3f( crgb[cycle_colors(change_colors+3)].r,
//          crgb[cycle_colors(change_colors+3)].g,
//          crgb[cycle_colors(change_colors+3)].b );
//    } else {
//      glColor3f( 0, 1, 1);
//    }
//    draw_lms( &lms1, &car_pose, lms1_offset, &lms1_mesh, &lms1_mutex );
//  }
//
//  // *******************************************************************
//  // *    Draw the LMS2 scan
//  // *******************************************************************
//  if (show_lms3) {
//    if (show_colors) {
//      glColor3f( crgb[cycle_colors(change_colors+4)].r,
//          crgb[cycle_colors(change_colors+4)].g,
//          crgb[cycle_colors(change_colors+4)].b );
//    } else {
//      glColor3f( 0, 1, 1);
//    }
//    draw_lms( &lms3, &car_pose, lms3_offset, &lms3_mesh, &lms3_mutex );
//  }
//
//  // *******************************************************************
//  // *    Draw GLS
//  // *******************************************************************
//  if(show_gls) {
//    pthread_mutex_lock(&gls_mutex);
//    n = (signed)gls_cache.size();
//    if (n>MAX_GLS_SOURCES)
//      n = MAX_GLS_SOURCES;
//    for(i=0; i<n; i++) {
//      if (gls_enable[i]) {
//        gls_draw(gls_cache[i],
//            global_x, global_y, 0,
//            car_pose.roll, car_pose.pitch, car_pose.yaw,
//            car_pose.smooth_x, car_pose.smooth_y, car_pose.smooth_z);
//      }
//    }
//    pthread_mutex_unlock(&gls_mutex);
//  }
//
//  // *******************************************************************
//  // *    Draw the vehicle
//  // *******************************************************************
//  if (!show_no_car) {
//    if (!show_simulator) {
//      if( last_can_ts != 0 ) {
//        double dt = can.timestamp - last_can_ts;
//        double speed_ms = dgc_kph2ms((can.wheel_speed_rl + can.wheel_speed_rr)/2.0);
//        dist = dt * speed_ms;
//        dangle = 2*M_PI * dist / ( M_PI * DGC_PASSAT_WHEEL_RADIUS);
//        wheel_angle = vlr::normalizeAngle( wheel_angle + dangle);
//      }
//      last_can_ts = can.timestamp;
//      glPushMatrix();
//      {
//        if (plain_mode)
//          glTranslatef(0, 0, DGC_PASSAT_HEIGHT);
//
//        glRotatef(dgc_r2d(car_pose.yaw),   0, 0, 1);
//        glRotatef(dgc_r2d(car_pose.pitch), 0, 1, 0);
//        glRotatef(dgc_r2d(car_pose.roll),  1, 0, 0);
//        glTranslatef(1.65, 0, -0.6);
//        glEnable(GL_LIGHTING);
//        if (show_car) {
//          velodyne_angle = fmod( velodyne_angle + (M_PI/1.7), 2*M_PI );
//          if (show_stickers) {
//            draw_stickered_passat( passat,
//                dgc_d2r(30.0)*(can.steering_angle/360.0),
//                wheel_angle, velodyne_angle, 1 );
//          } else {
//            passatwagonmodel_draw( passat,
//                dgc_d2r(30.0)*(can.steering_angle/360.0),
//                wheel_angle, velodyne_angle );
//          }
//        } else {
//          passatwagonwheels_draw( passat,
//              dgc_d2r(30.0)*(can.steering_angle/360.0),
//              wheel_angle );
//        }
//        glDisable(GL_LIGHTING);
//
//      }
//      glPopMatrix();
//
//      if (!show_no_car && !show_car) {
//        glPushMatrix();
//        {
//          glRotatef(dgc_r2d(car_pose.yaw),   0, 0, 1);
//          glRotatef(dgc_r2d(car_pose.pitch), 0, 1, 0);
//          glRotatef(dgc_r2d(car_pose.roll),  1, 0, 0);
//          if (!plain_mode)
//            glTranslatef(1.65, 0, -DGC_PASSAT_HEIGHT);
//          else
//            glTranslatef(1.65, 0.0, 0.0 );
//          draw_cage( DGC_PASSAT_LENGTH,
//              DGC_PASSAT_WIDTH,
//              DGC_PASSAT_HEIGHT );
//          if (show_cte) {
//            glPushMatrix();
//            {
//              glTranslatef(0.0, 0.0, 0.2);
//              draw_cte( controller_target.cross_track_error );
//            }
//            glPopMatrix();
//          }
//          glPopMatrix();
//        }
//      }
//
//      if (show_localize) {
//        glPushMatrix();
//        {
//          if (!plain_mode)
//            glTranslatef(gps_offset_x, gps_offset_y, 0.05-DGC_PASSAT_HEIGHT);
//          else
//            glTranslatef(gps_offset_x, gps_offset_y, 0.05);
//          glColor4f(1, 0, 0, 0.5);
//          glRotatef(dgc_r2d(car_pose.yaw),   0, 0, 1);
//          glTranslatef(1.65, 0.0, -0.15);
//          draw_car_box( DGC_PASSAT_LENGTH, DGC_PASSAT_WIDTH );
//        }
//        glPopMatrix();
//
//        glPushMatrix();
//        {
//          if (show_inverse || show_imagery) {
//            glColor3f(0, 0, 0 );
//          } else {
//            glColor3f(1, 1, 1 );
//          }
//          if (!plain_mode)
//            glTranslatef(gps_offset_x, gps_offset_y, -0.15-DGC_PASSAT_HEIGHT);
//          else
//            glTranslatef(gps_offset_x, gps_offset_y, -0.15);
//          glRotatef(dgc_r2d(car_pose.yaw)-90.0,   0, 0, 1);
//          glTranslatef(1.65-2.7, -1.2, 0.0);
//          sprintf(str, "loc-x: %s", _numf(gps_offset_x,"m") );
//          render_stroke_text_2D( 0, 0, GLUT_STROKE_ROMAN, 0.2, str );
//          sprintf(str, "loc-y: %s", _numf(gps_offset_y,"m") );
//          render_stroke_text_2D( 0, -0.3, GLUT_STROKE_ROMAN, 0.2, str );
//          sprintf(str, "loc-d: %s", _numf(hypot(gps_offset_x,gps_offset_y),"m") );
//          render_stroke_text_2D( 0, -0.6, GLUT_STROKE_ROMAN, 0.2, str );
//        }
//        glPopMatrix();
//
//      }
//
//    } else {
//      // *******************************************************************
//      // *    Draw simulator vehicles
//      // *******************************************************************
//
//      pthread_mutex_lock(&simulator_mutex);
//
//      for (i=0; i<simulator_groundtruth.num_vehicles;i++) {
//        if (show_car_marker) {
//          glColor4f(1, 0, 0, 0.75);
//          draw_marker(  simulator_groundtruth.vehicle[i].x-global_x,
//              simulator_groundtruth.vehicle[i].y-global_y,
//              0.10, marker_size/3.0 );
//        }
//
//        dist   =  simulator_groundtruth.vehicle[i].v;
//        dangle = 2*M_PI * dist / ( M_PI * DGC_PASSAT_WHEEL_RADIUS);
//        n = i%NUM_SIM_CAR_COLORS;
//        glPushMatrix();
//        {
//          glTranslatef( simulator_groundtruth.vehicle[i].x-global_x,
//              simulator_groundtruth.vehicle[i].y-global_y,
//              -0.6);
//          glRotatef(dgc_r2d(simulator_groundtruth.vehicle[i].theta), 0, 0, 1);
//
//          if(show_car) {
//
//            glPushMatrix();
//            {
//#ifdef SIMULATOR_FX
//              if (sim_car_status[i].model_nr == DGC_PASSAT_MODEL_ID) {
//                glEnable(GL_LIGHTING);
//                passatwagonmodel_draw( sim_passat[n],
//                    simulator_groundtruth.vehicle[i].alpha,
//                    dangle, velodyne_angle );
//                glDisable(GL_LIGHTING);
//              } else {
//                if (sim_car_status[i].model_nr==
//                    DGC_STICKERED_PASSAT_MODEL_ID) {
//                  glPushMatrix();
//                  {
//                    glRotatef(180.0, 0, 0, 1);
//                    glTranslatef(-0.15,0.0,0.0);
//                    glEnable(GL_LIGHTING);
//                    passatwagonwheels_draw( sim_passat[n],
//                        simulator_groundtruth.vehicle[i].alpha,
//                        dangle);
//                    glDisable(GL_LIGHTING);
//                  }
//                  glPopMatrix();
//                  glRotatef(180.0, 0, 0, 1);
//                  glScalef(5.0, 5.0, 5.0);
//                  glEnable(GL_LIGHTING);
//                  vehicle_gl_models.draw(sim_car_status[i].model_nr);
//                  glDisable(GL_LIGHTING);
//                } else {
//                  glRotatef(90.0, 0, 0, 1);
//                  glScalef(5.0, 5.0, 5.0);
//                  glEnable(GL_LIGHTING);
//                  vehicle_gl_models.draw(sim_car_status[i].model_nr);
//                  glDisable(GL_LIGHTING);
//                }
//              }
//#else
//              glEnable(GL_LIGHTING);
//              passatwagonmodel_draw( sim_passat[n],
//                  simulator_groundtruth.vehicle[i].alpha,
//                  dangle, velodyne_angle );
//              glDisable(GL_LIGHTING);
//#endif
//            }
//            glPopMatrix();
//
//          } else {
//
//            glPushMatrix();
//            {
//              glEnable(GL_LIGHTING);
//              passatwagonwheels_draw( sim_passat[n],
//                  simulator_groundtruth.vehicle[i].alpha,
//                  dangle);
//              glDisable(GL_LIGHTING);
//            }
//            glPopMatrix();
//
//            glPushMatrix();
//            {
//              glTranslatef(0, 0, 0.7);
//              draw_cage( DGC_PASSAT_LENGTH,
//                  DGC_PASSAT_WIDTH,
//                  DGC_PASSAT_HEIGHT );
//            }
//            glPopMatrix();
//
//          }
//
//          time = dgc_get_time();
//          if (sim_car_status[i].forward_accel_warning_time>time) {
//            draw_warning_sign_3D( 2.5, 0.0, 2.0, 2.0, 0.0, 0.75 );
//          }
//          if (sim_car_status[i].collision_warning_time>time) {
//            draw_accident_sign_3D( 2.5, 0.0, 2.1, 2.0, 0.0, 0.75 );
//          }
//          if (sim_car_status[i].lateral_accel_warning_time>time) {
//            draw_slippery_sign_3D( 2.5, 0.0, 2.2, 2.0, 0.0, 0.75 );
//          }
//          if (sim_car_status[i].plan_warning_time>time) {
//            draw_death_sign_3D( 2.5, 0.0, 2.3, 2.0, 0.0, 0.75 );
//          }
//          /* LABEL */
//          glRotatef(0.0,   0, 0, 1);
//          glTranslatef(-2, -0.2, 2.0);
//          render_stroke_text_2D( 0, 0.0, GLUT_STROKE_ROMAN, 0.5,
//              sim_car_status[i].name );
//
//        }
//        glPopMatrix();
//      }
//      pthread_mutex_unlock(&simulator_mutex);
//    }
//  }
//
//  if (show_car_marker) {
//
//    GLUquadricObj   * quad = gluNewQuadric();
//    GLfloat           material[]  = {1.0, 0.0, 0.0, 0.0};
//
//    gluQuadricNormals(quad, GLU_SMOOTH);
//    gluQuadricTexture(quad, GL_TRUE);
//    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, material);
//    gluCylinder(quad, marker_size * 2.0f, 0.0f,
//        0.25f + marker_size * 0.25f, 32, 32);
//  }
//
//  // *******************************************************************
//  // *    Draw ghost car
//  // *******************************************************************
//  if (show_ghost_car) {
//    if( last_gcar_ts != 0 ) {
//      double dt = ghost_car.timestamp - last_gcar_ts;
//      dist = dt * ghost_car.speed;
//      dangle = 2*M_PI * dist / ( M_PI * DGC_PASSAT_WHEEL_RADIUS);
//      wheel_angle = vlr::normalizeAngle( wheel_angle + dangle);
//      glPushMatrix();
//      {
//        glTranslatef( ghost_car_x-global_x, ghost_car_y-global_y, 0.0 );
//        glRotatef(dgc_r2d(ghost_car.yaw),   0, 0, 1);
//        glRotatef(dgc_r2d(ghost_car.pitch), 0, 1, 0);
//        glRotatef(dgc_r2d(ghost_car.roll),  1, 0, 0);
//        glTranslatef(0.0, 0, -0.60);
//        velodyne_angle = fmod( velodyne_angle + (M_PI/1.7), 2*M_PI );
//        glEnable(GL_LIGHTING);
//        passatwagonmodel_draw( ghost, ghost_car.wheel_angle,
//            wheel_angle, velodyne_angle );
//        glDisable(GL_LIGHTING);
//      }
//      glPopMatrix();
//
//    }
//    last_gcar_ts = ghost_car.timestamp;
//  }
//
//
//  glEnable(GL_BLEND);
//
//  // *******************************************************************
//  // *    Draw the 2D graphics */
//  // *******************************************************************
//  set_display_mode_2D(gui3D.window_width, gui3D.window_height);
//
//  if (show_actuators) {
//    if (show_canact) {
//      draw_actuators( can.steering_angle, can.throttle_position/100.0,
//          can.brake_pressure, passat_signal.signal,
//          (can.wheel_speed_fl+can.wheel_speed_fr)/(1.609344*2.0),
//          show_cte, controller_target.cross_track_error,
//          0, 0, 1.0);
//    } else {
//      draw_actuators( can.steering_angle, passat_act.throttle_fraction,
//          passat_act.brake_pressure, passat_signal.signal,
//          (can.wheel_speed_fl+can.wheel_speed_fr)/(1.609344*2.0),
//          show_cte, controller_target.cross_track_error,
//          0, 0, 1.0);
//    }
//    if (show_ghost_car && last_gcar_ts != 0 ) {
//      draw_actuators( dgc_r2d(ghost_car.wheel_angle)*DGC_PASSAT_STEERING_RATIO,
//          ghost_car.throttle,
//          ghost_car.brake, ' ', ghost_car.speed*2.23704,
//          0, 0.0, 140, 0, 0.5 );
//    }
//  }
//
//  if (show_gmeter) {
//    float size = 200.0;
//    draw_g_meter( gui3D.window_width-(size/2+15), size/2+37, size,
//        car_pose.a_x/9.81,
//        car_pose.a_y/9.81 );
//  }
//

//
//  if (show_info) {
//    if (!show_actuators) {
//      /* PERCEPTION HZ */
//      if (perception_hz>0) {
//        glColor4f(1, 1, 0, 0.7);
//        glBegin(GL_QUADS);
//        glVertex2f(gui3D.window_width-190,gui3D.window_height-5);
//        glVertex2f(gui3D.window_width-190,gui3D.window_height-30);
//        glVertex2f(gui3D.window_width-5,  gui3D.window_height-30);
//        glVertex2f(gui3D.window_width-5,  gui3D.window_height-5);
//        glEnd();
//        glColor3f(0, 0, 0);
//        sprintf(str, " %05d - %s ",
//            obstacles.num_points, numf(perception_hz,"hz") );
//        renderBitmapString(gui3D.window_width-170,
//            gui3D.window_height-23,
//            GLUT_BITMAP_HELVETICA_18, str);
//      }
//    }
//
//    /* MESSAGES */
//    pthread_mutex_lock(&message_buffer_mutex);
//
//    max_width = 0;
//    if (show_status) {
//      for(i = 0; i < MESSAGE_BUFFER_SIZE; i++) {
//        w = bitmapStringWidth(GLUT_BITMAP_HELVETICA_12,
//            message_buffer[i].string);
//        if(w > max_width)
//          max_width = w;
//      }
//    } else {
//      for(i = 0; i < MESSAGE_BUFFER_SIZE; i++) {
//        w = bitmapStringWidth(GLUT_BITMAP_HELVETICA_12,
//            error_buffer[i].string);
//        if(w+20 > max_width)
//          max_width = w+20;
//      }
//    }
//    if (max_width<400) max_width=400;
//
//    time = dgc_get_time();
//
//    fs = 16;
//    n = fs*MESSAGE_BUFFER_SIZE;
//
//    glColor4f(0.1f, 0.1f, 0.1f, 0.95f);
//    glBegin(GL_QUADS);
//    glVertex2f(0,           n+5);
//    glVertex2f(max_width,   n+5);
//    glVertex2f(max_width,   n);
//    glVertex2f(0,           n);
//    glEnd();
//
//    for(i = 0; i < MESSAGE_BUFFER_SIZE; i++) {
//
//      if (show_status) {
//        delta = time-message_buffer[i].timestamp;
//      } else {
//        delta = time-error_buffer[i].timestamp;
//      }
//      if (delta<3.0) {
//        glColor4f(0.6f, 0.0f, 0.0f, 0.85f);
//      } else {
//        glColor4f(0.1f, 0.1f, 0.1f, 0.85f);
//      }
//
//      glBegin(GL_QUADS);
//      glVertex2f(0,           n-(i+1)*fs);
//      glVertex2f(max_width,   n-(i+1)*fs);
//      glVertex2f(max_width,   n-(i)*fs);
//      glVertex2f(0,           n-(i)*fs);
//      glEnd();
//
//      glColor3f(1.0f, 1.0f, 1.0f);
//      if (show_status) {
//        renderBitmapString(5, n+2-(i+1)*15,
//            GLUT_BITMAP_HELVETICA_12,
//            message_buffer[i].string);
//      } else {
//        renderBitmapString(5, n+2-((i+1)*fs),
//            GLUT_BITMAP_HELVETICA_12,
//            error_buffer[i].string);
//      }
//    }
//
//    pthread_mutex_unlock(&message_buffer_mutex);
//
//
//  }
//
//  // *******************************************************************
//
//  draw_msg( NULL );
//
//  // *******************************************************************
//  // *    Show GLS select
//  // *******************************************************************
//  if (show_gls_select) {
//    snprintf( gls_select[0], MAX_GLS_NAME_LENGTH,
//        "GLS messages:    [%s]", numf(gls_hz,"hz") );
//    strncpy( gls_select[1], "        ",
//        MAX_GLS_NAME_LENGTH );
//    strncpy( gls_select[2], "'0': select NONE",
//        MAX_GLS_NAME_LENGTH );
//    strncpy( gls_select[3], "'1': select ALL            ",
//        MAX_GLS_NAME_LENGTH );
//    strncpy( gls_select[4], "        ",
//        MAX_GLS_NAME_LENGTH );
//
//    n = (signed)gls_cache.size();
//    if (n>MAX_GLS_SOURCES) {
//      n = MAX_GLS_SOURCES;
//    }
//    for (i=0; i<n; i++) {
//      strncpy( str0, numf(gls_size[i],"KB/s"), 4096 );
//      strncpy( str1, numf(gls_s_hz[i],"Hz"), 4096 );
//      snprintf( gls_select[i+5], MAX_GLS_NAME_LENGTH, "'%c': %s  [%s - %s]",
//          97+i, gls_cache[i]->name, str0, str1 );
//    }
//    strncpy( gls_select[n+5], "     ", MAX_GLS_NAME_LENGTH );
//    gui3D_help( 6+n, gls_select, GLUT_BITMAP_HELVETICA_18 );
//
//    // *******************************************************************
//    // *    Show Help
//    // *******************************************************************
//  } else if (show_help) {
//    draw_help();
//  } else if (show_sensor_info==1) {
//    draw_info_window(&car_pose);
//  } else if (show_sensor_info==2) {
//    draw_applanix_window(&car_pose);
//  }
//
//  pthread_mutex_unlock ( &pose_mutex );
//
//}

} // namespace vlr
