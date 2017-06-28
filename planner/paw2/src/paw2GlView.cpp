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

#include <vector>
#include <sstream>
#include <iostream>
#include <cmath>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#include <passatmodel.h>
#include <gl_support.h>
#include <textures.h>
#include <aw_roadNetwork.h>
#include <aw_ChsmPlanner.hpp>
#include <aw_StLaneChange.hpp>

#include <global.h>
#include <passatmodel.h>
#include <passat_constants.h>
#include <transform.h>
#include <imagery.h>

#include <tf/tf.h>
//#include <camera_shm_interface.h>

#include <paw2GlView.h>
#include <boost/signals.hpp>
#include <paw2_gui.h>
#include <graphics.h>
#include <vlrImaging.h>
#include <display.h>

#include <paw2InternalData.h>

using namespace dgc;
using std::cout;
using std::endl;
using std::flush;

namespace drc = driving_common;

extern GLint generateHenry();

namespace vlr {

Paw2GlView::Paw2GlView(QWidget* /*parent*/) :
  frame_count_(0), car_(NULL), configuration_space_texture_(0), obstacle_map_texture_(0), road_map_texture_(0), create_rndf_display_list_(true),
      last_raw_map_timestamp_(0), last_configuration_space_timestamp_(0), last_roadmap_timestamp_(0), smoothed_mission_points_buf_(NULL),
      smoothed_mission_points_buf_size_(0) {

  pthread_mutex_init(&gui_ready_mutex_, NULL);
  pthread_cond_init(&gui_ready_cv_, NULL);

  light_ambient_ = {0, 0, 0, 0};
  light_diffuse_ = {1, 1, 1, 1};
  light_specular_ = {1, 1, 1, 1};
  light_position_ = {0, 0, 100, 0};

  //setFrameRate(30.0);
      setInitialCameraPos(180.0, 89.99, 100.0, 0, 0, 0);
      setCameraParams(0.01, 0.3, 0.001, 0.009, 60, 0.4, 2000000);
    }

    Paw2GlView::~Paw2GlView() {
      pthread_mutex_destroy(&gui_ready_mutex_);
      pthread_cond_destroy(&gui_ready_cv_);
    }

    void Paw2GlView::signalGuiReady(Paw2Gui* gui) {
      //    pthread_mutex_lock(&gui_ready_mutex_);
      gui_=gui;
      //    pthread_cond_signal(&gui_ready_cv_);
      //    pthread_mutex_unlock(&gui_ready_mutex_);
    }

    void Paw2GlView::mousePressEvent(QMouseEvent* event)
    {
      switch (event->modifiers())
      {
        case Qt::ControlModifier:
        if (event->buttons() & Qt::LeftButton)
        {
          // use ctrl + left as replacement for right; useful for Mac Trackpad
          QMouseEvent mouse_event(event->type(), QPoint(event->x(), event->y()), Qt::RightButton, Qt::RightButton, Qt::NoModifier);
          GLWidget::mousePressEvent(&mouse_event);
        }
        else if (event->buttons() & Qt::RightButton)
        {
        }
        requestRedraw();
        break;

        default: // let base class handle other cases
        GLWidget::mousePressEvent(event);
        break;
      }
    }

    void Paw2GlView::mouseMoveEvent(QMouseEvent *event)
    {
      switch (event->modifiers())
      {
        case Qt::ControlModifier: // selected object is affected
        if (event->buttons() & Qt::LeftButton)
        {
          // use ctrl + left as replacement for right; useful for Mac Trackpad
          QMouseEvent mouse_event(event->type(), QPoint(event->x(), event->y()), Qt::RightButton, Qt::RightButton, Qt::NoModifier);
          GLWidget::mouseMoveEvent(&mouse_event);
        }
        else if (event->buttons() & Qt::RightButton)
        {
        }
        requestRedraw();
        break;

        case Qt::NoModifier:
        //		gui->last_mouse_x = event->x();
        //		gui->last_mouse_y = event->y();

        default: // let base class handle other cases
        GLWidget::mouseMoveEvent(event);
        break;
      }

      //gui->last_move_utm_x = utm_x;
      //gui->last_move_utm_y = utm_y;
    }

    // Parses keyboard commands
    void Paw2GlView::keyPressEvent(QKeyEvent* event) {
      //double x2, y2, utm_x, utm_y;

      unsigned int key=(unsigned int)(*(event->text().toAscii().constData()));

      std::cout << "keyboard: "<< key << " (#"<< (int) key <<") ";

      switch (key) {
        case 'v':
        gui_->on_plannerDisplayVehicle_stateChanged(!gui_->show_vehicle_);
        break;

        default:
        std::cout << "(no command)"<< std::endl;
        break;
      }

      requestRedraw();
    }

    void Paw2GlView::initializeGL() {

      glEnable(GL_DEPTH_TEST);
      glShadeModel(GL_SMOOTH);
      glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient_);
      glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse_);
      glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular_);
      glLightfv(GL_LIGHT0, GL_POSITION, light_position_);
      glEnable(GL_LIGHT0);
      glDisable(GL_LIGHTING);
      glEnable(GL_NORMALIZE);

      //glClearColor(0.0f, 0.0f, 0.0f, 0.5f);				// Black Background
      //glClearDepth(1.0f);									// Depth Buffer Setup
      //glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
      glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Really Nice Perspective Calculations

      sphere_ = gluNewQuadric();

      car_ = passatwagonmodel_load(0.0, 0.0, 0.5, 1.0);
      //  car_ = passatwagonmodel_load(0.5, 0.5, 0.5, 1.0);
      porsche_ = new VehiclePorscheModel;
      porsche_->Init();
      hummer_ = new VehicleHummerModel;
      hummer_->Init();
      elise_ = new VehicleEliseModel;
      elise_->Init();

      glGenTextures(1, &obstacle_map_texture_);
      glGenTextures(1, &configuration_space_texture_);
      glGenTextures(1, &road_map_texture_);

      const rndf::RoadNetwork& rn = gui_->awp_.chsm_planner_->topology_->roadNetwork();
      //rndf_display_list_ = glGenLists(1);
      //glNewList(rndf_display_list_, GL_COMPILE);
      //rndf_dl_origin_x_ = rn.PointMap().begin()->second->utm_x();
      //rndf_dl_origin_y_ = rn.PointMap().begin()->second->utm_y();
      //rn.draw(rndf_dl_origin_x_, rndf_dl_origin_y_, 1, true);
      //glEndList();

      pedestrian_display_list_ = generateHenry();

      try {
        const_cast<rndf::RoadNetwork*>(&gui_->awp_.chsm_planner_->topology_->roadNetwork())->createVisualization();
      }
      catch(vlr::Ex<>& e) {
        std::cout << "Failed to create rndf visualization :" << e.what() << std::endl;
      }

      double blend = 0.5;
      bool dynamic = false;
      const_cast<rndf::RoadNetwork*>(&rn)->generateDisplayList(blend, dynamic);

      try {
        imagery_ = new Imagery(gui_->imagery_folder_);
      }
      catch(vlr::Ex<>& e) {
        std::cout << "Failed to create/load imagery :" << e.what() << std::endl;
      }

      imagery_->setType(Imagery::LASER);

      connect(&timer, SIGNAL(timeout()), this, SLOT(redraw(void)));

      timer.start(DISPLAY_REFRESH_DELAY_MS);
      gl_initialized_ = true;
    }

    void Paw2GlView::paintGL() {
      //  static double last_t=0;
      //  double t = Time::current();
      //  double dt = t-last_t;
      //  last_t=t;
      //  printf("dt=%f\n", dt*1000);
      activate3DMode();

      // clear window
      glClearColor(0.13, 0.32, 0.17, 1.0);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if(!gui_->awp_.received_localize_pose_ || !gui_->awp_.received_applanix_pose_) {return;}

      drc::GlobalPose latest_pose;
      double latest_timestamp;
      gui_->awp_.chsm_planner_->latestPose(latest_pose, latest_timestamp);
      pthread_mutex_lock(&gui_->awp_.chsm_planner_->pose_mutex_);
      dgc_pose_t robot_pose_accuracy_copy = gui_->awp_.pose_accuracy_;
      pthread_mutex_unlock(&gui_->awp_.chsm_planner_->pose_mutex_);

      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_LINE_SMOOTH);

      // draw things in the robot coordinate system
      glDisable(GL_DEPTH_TEST);
      glPushMatrix();

      bool show_flat_imagery = true;
      double current_time = drc::Time::current();
      static double last_time = 0;

      // draw imagery
      if (gui_->show_imagery_) {
        if (current_time - last_time > 0.05) {
          try {
            imagery_->update();
          }
          catch(vlr::Ex<>& e) {
            std::cout << "Failed to update imagery :" << e.what() << std::endl;
          }

          last_time = current_time;
        }
        glPushMatrix();
        {
          glTranslatef(0, 0, -DGC_PASSAT_HEIGHT);
          try {
            imagery_->draw3D(cameraPose.distance, cameraPose.xOffset,
                cameraPose.yOffset, latest_pose.utmX(), latest_pose.utmY(), "10S", show_flat_imagery, 1.0, 1); // TODO: integrate zone in pose queue
          }
          catch(vlr::Ex<>& e) {
            std::cout << "Failed to draw imagery :" << e.what() << std::endl;
          }
        }
        glPopMatrix();
      }

      // draw road map
      if (gui_->show_road_map_) {
        boost::unique_lock<boost::mutex> lock(gui_->internal_data_->mutex());
        vlr::Image<uint8_t>* road_map;
        double timestamp;
        drc::GlobalPose pose;
        gui_->internal_data_->roadMap(road_map, pose, timestamp);
        if(road_map && road_map->data()) {
          last_roadmap_timestamp_=-1;
          updateRoadMapTexture(road_map->data(), timestamp);
          drawRoadMap(pose, latest_pose, 2, .7, .4);
        }
      }

      // draw convoluted obstacle map (configuration space)
      if (gui_->show_configuration_space_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->static_obstacle_map_mutex_);
        updateObstacleMapTexture(gui_->awp_.chsm_planner_->obstacleMapCS(), configuration_space_texture_, last_configuration_space_timestamp_);
        drawObstacleMap(configuration_space_texture_, gui_->awp_.chsm_planner_->obstacleMapCenterX(), gui_->awp_.chsm_planner_->obstacleMapCenterY(), latest_pose, 1, .8, 0, false);
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->static_obstacle_map_mutex_);
      }

      printf("Obstacle Map Position: %f %f %f %f \n",gui_->awp_.chsm_planner_->obstacleMapCenterX(), gui_->awp_.chsm_planner_->obstacleMapCenterY(), latest_pose.x(), latest_pose.y());
      // draw raw obstacle map
      if (gui_->show_obstacle_map_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->static_obstacle_map_mutex_);
        updateObstacleMapTexture(gui_->awp_.chsm_planner_->obstacleMapRaw(), obstacle_map_texture_, last_raw_map_timestamp_);
        drawObstacleMap(obstacle_map_texture_, gui_->awp_.chsm_planner_->obstacleMapCenterX(), gui_->awp_.chsm_planner_->obstacleMapCenterY(), latest_pose, 1, .2, 0, true);
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->static_obstacle_map_mutex_);
      }

      // draw rndf
      if (gui_->show_rndf_) {
        glPushMatrix();
        gui_->awp_.chsm_planner_->topology_->roadNetwork().draw(latest_pose.utmX(), latest_pose.utmY(), 1, true);
        //      draw_rndf_display_list(&rndf_display_list2_, 0, 1, 1, 1, 0, 0, latest_pose.utmX(), latest_pose.utmY());
        glPopMatrix();
        //glPushMatrix();
        //glTranslatef(rndf_dl_origin_x_ - latest_pose.utmX(), rndf_dl_origin_y_ - latest_pose.utmY(), 0);
        //glCallList(rndf_display_list_);
        //    if(draw_stops) {
        //      if(threeD_signs)
        //        glCallList(dl->threeD_stops_dl);
        //      else
        //        glCallList(dl->flat_stops_dl);
        //    }
        //    glPopMatrix();
      }

      // draw the trajectory
      if (gui_->show_pose_history_) {
        drawPoseHistory(latest_pose.utmX(), latest_pose.utmY());
        //    draw_corrected_trajectory(latest_pose.utmX(), latest_pose.utmY());
      }

      // draw ego vehicle distances
      if (gui_->show_ego_distances_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->topology_mutex_);
        drawEgoDistances(latest_pose.utmX(), latest_pose.utmY(), latest_pose.yaw(), gui_->awp_.chsm_planner_->topology_);
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->topology_mutex_);
      }

      // draw other vehicles distances
      if (gui_->show_veh_distances_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->dyn_obstacles_mutex_);
        drawVehDistances(latest_pose.utmX(), latest_pose.utmY(), gui_->awp_.chsm_planner_->topology_);
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->dyn_obstacles_mutex_);
      }

      // draw vehicle destination probabilities
      if (gui_->show_obstacle_destinations_) {
        // TODO: replace topology mutex
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->topology_mutex_);
        //    pthread_mutex_lock(&gui_->awp_.chsm_planner_->intersection_predictor_mutex_);
        drawDestinations(latest_pose.utmX(), latest_pose.utmY());
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->topology_mutex_);
        //    pthread_mutex_unlock(&gui_->awp_.chsm_planner_->intersection_predictor_mutex_);
      }

      // draw lane change merging points
      if (gui_->show_lanchange_merging_points_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->topology_mutex_);
        drawLaneChangeMergingPoints(latest_pose.utmX(), latest_pose.utmY(), gui_->awp_.chsm_planner_->topology_);
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->topology_mutex_);
      }

      // draw distance debug circles
      pthread_mutex_lock(&gui_->awp_.chsm_planner_->topology_mutex_);
      glPushMatrix();
      glTranslatef(-latest_pose.utmX(), -latest_pose.utmY(), 0);
      std::vector<Topology::DistanceVisualisation>::const_iterator iter = gui_->awp_.chsm_planner_->topology_->debug_distances.begin();
      for (; iter != gui_->awp_.chsm_planner_->topology_->debug_distances.end(); ++iter ) {
        glColor3f(iter->cr, iter->cg, iter->cb);
        drawDistanceCond(iter->x, iter->y , iter->r);
      }
      glPopMatrix();
      pthread_mutex_unlock(&gui_->awp_.chsm_planner_->topology_mutex_);

      if (gui_->show_intersection_merging_point_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->topology_mutex_);
        drawIntersectionMergingPoint(latest_pose.utmX(), latest_pose.utmY(), gui_->awp_.chsm_planner_->topology_);
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->topology_mutex_);
      }

      // draw the topology
      if (gui_->show_topology_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->topology_mutex_);
        drawTopology(latest_pose.utmX(), latest_pose.utmY());
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->topology_mutex_);
      }

      // draw radius rings
      if( gui_->show_radius_ ) {
        draw_distance_rings(0, 0, latest_pose.yaw(), 120, 10);
      }

      // draw latest_pose accuracy ellipse
      if (gui_->show_pose_accuracy_) {
        drawPoseAccuracy(robot_pose_accuracy_copy);
      }
      glEnable(GL_DEPTH_TEST);

      if (gui_->show_complete_center_line_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->center_line_mutex_);
        drawCompleteCenterLine( gui_->awp_.chsm_planner_->missionPointsBezier(), gui_->awp_.chsm_planner_->center_line_, latest_pose.utmX(), latest_pose.utmY());
        //      drawCompleteCenterLine( gui_->awp_.chsm_planner_->missionPointsBezier(), gui_->awp_.chsm_planner_->smoothed_mission_points_, latest_pose.utmX(), latest_pose.utmY());
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->center_line_mutex_);
      }

      if (gui_->show_center_line_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->center_line_mutex_);
        drawCenterline(gui_->awp_.chsm_planner_->center_line_, latest_pose.utmX(), latest_pose.utmY());
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->center_line_mutex_);
      }

      if (gui_->show_trajectories_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->trajectory_mutex_);
        drawTrajectories(latest_pose);
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->trajectory_mutex_);
      }

      if (gui_->show_best_trajectory_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->trajectory_mutex_);
        drawBestTrajectory(latest_pose);
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->trajectory_mutex_);
      }

      // draw obstacles
      if (gui_->show_dynamic_objects_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->dyn_obstacles_mutex_);
        printf("Drawing %u cars.\n", gui_->awp_.chsm_planner_->vehicle_manager->vehicles().size());
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->dyn_obstacles_mutex_);
        drawObstacles(latest_pose.utmX(), latest_pose.utmY());
      }

      if(gui_->show_obstacle_predictions_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->dyn_obstacles_mutex_);
        drawObstaclePredictions(latest_pose);
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->dyn_obstacles_mutex_);

        //draw pedestrians
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->pedestrian_prediction_mutex_);
        RGB rgb;
        rgb.r = 0.98; // skin color setting
        rgb.g = 0.76;
        rgb.b = 0.44;
//        printf("Drawing %u peds.\n", gui_->awp_.chsm_planner_->predicted_pedestrians_.size());
        for (uint32_t i = 0; i < gui_->awp_.chsm_planner_->predicted_pedestrians_.size(); i++) {
          for (int j = gui_->awp_.chsm_planner_->predicted_pedestrians_[i].predicted_traj_.size()-1; j>=0; j--) {
            MovingBox& box = gui_->awp_.chsm_planner_->predicted_pedestrians_[i].predicted_traj_[j];
            glPushMatrix();
            glTranslatef(box.x - latest_pose.utmX(), box.y - latest_pose.utmY(), 0.0);
            glTranslatef(-box.length/2+box.ref_offset, 0, 0.9);
            //        glRotatef(box.psi / M_PI * 180 + 90, 0, 0, 1);
            glRotatef(box.psi / M_PI * 180, 0, 0, 1);
            glEnable(GL_BLEND);
            glDisable(GL_DEPTH_TEST);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_LINE_SMOOTH);

            glLineWidth(2);
            float val = float(gui_->awp_.chsm_planner_->predicted_pedestrians_[i].predicted_traj_.size()-1-j)/float(gui_->awp_.chsm_planner_->predicted_pedestrians_[i].predicted_traj_.size()-1);
            float val2 = val*.4;
            if(j==0) {glColor4f(1, 0.7, .3, 0.7);}
            else {glColor4f(1-val, 1-val2, 1-val2, 0.2);}
            float l = box.length;
            float w = box.width;

            glBegin(GL_POLYGON);
            glVertex2f(l / 2, w / 2);
            glVertex2f(l / 2, -w / 2);
            glVertex2f(-l / 2, -w / 2);
            glVertex2f(-l / 2, w / 2);
            glEnd();

            glBegin(GL_LINE_LOOP);
            glVertex2f(l / 2, w / 2);
            glVertex2f(l / 2, -w / 2);
            glVertex2f(-l / 2, -w / 2);
            glVertex2f(-l / 2, w / 2);
            glEnd();

            glLineWidth(1);

            glDisable(GL_LINE_SMOOTH);
            glDisable(GL_BLEND);
            glEnable(GL_DEPTH_TEST);
            glPopMatrix();
          }
          MovingBox& box = gui_->awp_.chsm_planner_->predicted_pedestrians_[i].predicted_traj_[0];
          MovingBox& box_last = *(--gui_->awp_.chsm_planner_->predicted_pedestrians_[i].predicted_traj_.end());
          //    double dist = hypot(box_last.x-box.x, box_last.y-box.y);
          //    double v = dist/gui_->awp_.chsm_planner_->traj_eval_->params().checked_horizon;
          //    drawPedestrian(box.x-latest_pose.utmX(), box.y-latest_pose.utmY(), 0, 1.7, 0.5*(box.width+box.length), box.psi, v, rgb, 0.8);
          glPushMatrix();
          glTranslatef(box.x-latest_pose.utmX(), box.y-latest_pose.utmY(), .7);
          double psi = dgc_r2d(atan2(box_last.y-box.y, box_last.x-box.x));
          if( (psi>=0 && psi <90.0) || psi <=-90.0) {psi -= 180.0;} // TODO: why?!?
          glRotatef( 90-psi, 0., 0., 1. );
          glColor3f(.7, .2, .4);
          glEnable(GL_DEPTH_TEST);
          glEnable(GL_LIGHTING);
          glCallList(pedestrian_display_list_);
          glDisable(GL_LIGHTING);
          glDisable(GL_DEPTH_TEST);
          glPopMatrix();
        }
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->pedestrian_prediction_mutex_);
      }

      if(gui_->show_vehicle_) {
        drawVehicle(latest_pose);
      }

      glEnable(GL_DEPTH_TEST);

      glPopMatrix();

      glPushMatrix();
      drawSensorData(latest_pose);
      glPopMatrix();

      char str[100];
      activate2DMode();
      const int lower_panel_width = 350;
      const int lower_panel_height = 100;
      const int right_panel_width = 200;
      const int right_panel_height = 200;

      bool show_gmeter = true;
      if (show_gmeter) {
        float size = 0.1*windowWidth;
        drawGMeter(size/2, windowHeight-size/2, size, latest_pose.a()/9.81, latest_pose.aLateral()/9.81);
        //      draw_g_meter( gui3D.window_width-(size/2+15), size/2+37, size, car_pose.a_x/9.81, car_pose.a_y/9.81);
      }

      // draw the transparent panel background
      glEnable (GL_BLEND);
      glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glColor4f(0.0, 0.0, 0.0, 0.5);
      glBegin(GL_QUADS);
      glVertex2f(0, 0);
      glVertex2f(0, lower_panel_height);
      glVertex2f(lower_panel_width, lower_panel_height);
      glVertex2f(lower_panel_width, 0);

      glVertex2f(windowWidth-right_panel_width, windowHeight-right_panel_height);
      glVertex2f(windowWidth-right_panel_width, windowHeight);
      glVertex2f(windowWidth, windowHeight);
      glVertex2f(windowWidth, windowHeight-right_panel_height);

      glVertex2f(windowWidth-right_panel_width, 0);
      glVertex2f(windowWidth-right_panel_width, 150);
      glVertex2f(windowWidth, 150);
      glVertex2f(windowWidth, 0);
      glEnd();

      glDisable (GL_BLEND);

      // draw text
      glColor3f(1, 1, 1);
      sprintf(str, "x: %.4f y: %.4f yaw: %.4f", latest_pose.utmX(), latest_pose.utmY(), latest_pose.yaw());
      renderBitmapString(10, 70, GLUT_BITMAP_HELVETICA_12, str);
      sprintf(str, "lat: %.7f lon: %.7f", gui_->awp_.lat_,gui_->awp_.lon_);
      renderBitmapString(10, 50, GLUT_BITMAP_HELVETICA_12, str);
      sprintf(str, "speed: %.2f desired: %.2f follow: %.2f", dgc_ms2mph(latest_pose.v()), dgc_ms2mph(gui_->awp_.chsm_planner_->velocity_desired_), dgc_ms2mph(gui_->awp_.chsm_planner_->velocity_following_));
      renderBitmapString(10, 30, GLUT_BITMAP_HELVETICA_12, str);

      // draw message queue
      int n=0;
      std::deque<std::string>& messages = gui_->awp_.chsm_planner_->getMessages();
      std::deque<std::string>::reverse_iterator it, it_end;
      for(it=messages.rbegin(), it_end=messages.rend(),n=0;it!=it_end;++it,++n) {
        sprintf(str, "%s", it->c_str());
        renderBitmapString(windowWidth-right_panel_width+10, n*12+windowHeight-right_panel_height+10, GLUT_BITMAP_HELVETICA_10, str);
      }

      int y_delta = 11;
      int y=y_delta*11;
      pthread_mutex_lock(&gui_->awp_.chsm_planner_->mission_mutex_);
      sprintf(str, "mission end: %f", gui_->awp_.chsm_planner_->topology_->distToMissionEnd());
      pthread_mutex_unlock(&gui_->awp_.chsm_planner_->mission_mutex_);
      renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      y -=y_delta;
      //  double standing, moving;
      pthread_mutex_lock(&gui_->awp_.chsm_planner_->dyn_obstacles_mutex_);
      pthread_mutex_lock(&gui_->awp_.chsm_planner_->topology_mutex_);
      pthread_mutex_lock(&gui_->awp_.chsm_planner_->mission_mutex_);
      //  gui_->awp_.chsm_planner_->getVehicleDistances(standing, moving);
      //  sprintf(str, "mv: %f (%f)", gui_->awp_.chsm_planner_->topology_->distToNextMovingVehicle(), moving);
      //  renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      //  y -=y_delta;
      //  sprintf(str, "sv: %f (%f)", gui_->awp_.chsm_planner_->topology_->distToNextStandingVehicle(), standing);
      //  renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      //  y -=y_delta;
      //  sprintf(str, "speed: %f", gui_->awp_.chsm_planner_->topology_->speed_of_next_veh());
      //  renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      //  y -=y_delta;
      //  sprintf(str, "pv: %f", gui_->awp_.chsm_planner_->topology_->distToPreviousVehicle());
      //  renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      //  y -=y_delta;
      pthread_mutex_unlock(&gui_->awp_.chsm_planner_->mission_mutex_);
      pthread_mutex_unlock(&gui_->awp_.chsm_planner_->topology_mutex_);
      pthread_mutex_unlock(&gui_->awp_.chsm_planner_->dyn_obstacles_mutex_);

      pthread_mutex_lock(&gui_->awp_.chsm_planner_->topology_mutex_);
      sprintf(str, "stop: %f", gui_->awp_.chsm_planner_->topology_->distToNextStopLine());
      renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      y -=y_delta;
      sprintf(str, "lanechange: %f", gui_->awp_.chsm_planner_->topology_->distToNextLaneChange());
      renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      y -=y_delta;
      sprintf(str, "intersect: %f", gui_->awp_.chsm_planner_->topology_->distToNextIntersection());
      renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      y -=y_delta;
      sprintf(str, "kturn: %f", gui_->awp_.chsm_planner_->topology_->distToNextKTurn());
      renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      y -=y_delta;
      sprintf(str, "zone: %f", gui_->awp_.chsm_planner_->topology_->distToNextZone());
      renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      y -=y_delta;
      sprintf(str, "zone exit: %f", gui_->awp_.chsm_planner_->topology_->distToNextZoneExit());
      renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      y -=y_delta;
      sprintf(str, "next turn: %d", gui_->awp_.chsm_planner_->topology_->nextTurnDirection());
      renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      y -=y_delta;

      pthread_mutex_unlock(&gui_->awp_.chsm_planner_->topology_mutex_);

      pthread_mutex_lock(&gui_->awp_.chsm_planner_->trajectory_mutex_);
      TrajectoryEvaluator::traj_mode_t mode = gui_->awp_.chsm_planner_->traj_eval_->trajectoryMode();
      pthread_mutex_unlock(&gui_->awp_.chsm_planner_->trajectory_mutex_);
      switch (mode) {
        case TrajectoryEvaluator::TRAJ_MODE_FOLLOW:
        sprintf(str, "following");
        break;
        case TrajectoryEvaluator::TRAJ_MODE_VELOCITY:
        sprintf(str, "drive");
        break;
        case TrajectoryEvaluator::TRAJ_MODE_STOP:
        sprintf(str, "stop");
        break;
      }

      renderBitmapString(windowWidth-right_panel_width+10, y, GLUT_BITMAP_HELVETICA_10, str);
      y -=y_delta;

      //  printf("display time: %f\n", (Time::current() - last_t)*1000);


      bool record_images_=false;
      if (record_images_) {

        static uint8_t* img_buf = NULL;
        static uint32_t frame_num = 0;
        unsigned int img_id = 0;
        uint32_t width = 800, height = 600;

//        if (!img_buf) {
//          img_buf = new uint8_t[width * height * 3];
//          ilInit();
//          ilGenImages(1, &img_id);
//        }
//
//        char buf[100];
//        sprintf(buf, "paw2%04d.JPG", frame_num);
//        printf("%s\n", buf);
//        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, img_buf);
//        ilBindImage(img_id);
//        ilTexImage(width, height, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_buf);
//        ilSave(IL_JPG, buf);
//        //if(!png->writePNGImage(buf, *sns_img)) {
        //  printf("Couldn't write image %s\n",buf);
        //}
        frame_num++;
      }
    }

    void Paw2GlView::drawTrajectories(drc::GlobalPose& pose) {
      std::multiset<PolyTraj2D>::const_iterator ptit, ptit_end;
      uint32_t set_size;
      switch(gui_->awp_.chsm_planner_->traj_eval_->trajectoryMode()) {
        case TrajectoryEvaluator::TRAJ_MODE_VELOCITY:
        ptit = gui_->awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.begin();
        ptit_end = gui_->awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.end();
        set_size = gui_->awp_.chsm_planner_->traj_eval_->getLVSet2D().set_data_.size();
        break;

        case TrajectoryEvaluator::TRAJ_MODE_FOLLOW:
        ptit = gui_->awp_.chsm_planner_->traj_eval_->getFollowingSet2D().set_data_.begin();
        ptit_end = gui_->awp_.chsm_planner_->traj_eval_->getFollowingSet2D().set_data_.end();
        set_size = gui_->awp_.chsm_planner_->traj_eval_->getFollowingSet2D().set_data_.size();
        break;

        case TrajectoryEvaluator::TRAJ_MODE_STOP:
        ptit = gui_->awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.begin();
        ptit_end = gui_->awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.end();
        set_size = gui_->awp_.chsm_planner_->traj_eval_->stopSet2D().set_data_.size();
        break;

        default:
        return;
      }

      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      //  glTranslated(-pose.x, -pose.y, 0.); // Don't do that with utm coordinates !!
      glEnable(GL_BLEND);
      glEnable(GL_DEPTH_TEST);
      //  glDisable(GL_DEPTH_TEST);

      for (unsigned int j = 0; ptit != ptit_end; ptit++) {
        std::vector<driving_common::TrajectoryPoint2D> trj = (*ptit).trajectory2D_;

        int col_index = 255 - 255 * j / (double)(set_size-1);
        //      int col_index = (int(256*j/(double)lane_and_velocity_set.set_data_.size()+70))%256;
        glColor4f(cmap_rb2_red_[col_index], cmap_rb2_green_[col_index], cmap_rb2_blue_[col_index], .15);
        //      glColor4f(cmap_rb1_red_[col_index], cmap_rb1_green_[col_index], cmap_rb1_blue_[col_index], .65);
        glLineWidth(2);
        glBegin(GL_LINE_STRIP);
        for (std::vector<driving_common::TrajectoryPoint2D>::const_iterator tit=trj.begin(); tit != trj.end(); tit++) {
          //      glVertex3d((*tit).x, (*tit).y, (*tit).v);
          glVertex3f((*tit).x-pose.utmX(), (*tit).y-pose.utmY(), (*tit).v);
        }
        glEnd();
        j++;
      }

      glDisable(GL_BLEND);
      //glEnable(GL_DEPTH_TEST);

      glPopMatrix();
    }

    void Paw2GlView::drawBestTrajectory(drc::GlobalPose& pose) {
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glEnable(GL_BLEND);
      //  glDisable(GL_DEPTH_TEST);

      const std::vector<driving_common::TrajectoryPoint2D>& trj = gui_->awp_.chsm_planner_->traj_eval_->bestTrajectory();
      glLineWidth(2);
      glColor4f(0.2, 1, 0.3, 1);
      //  glColor4f(1, 1, 1, 1);
      glBegin(GL_LINE_STRIP);
      for (std::vector<driving_common::TrajectoryPoint2D>::const_iterator tit=trj.begin(); tit != trj.end(); tit++) {
        glVertex3f((*tit).x-pose.utmX(), (*tit).y-pose.utmY(), (*tit).v);
      }
      glEnd();

      //    for (std::vector<driving_common::TrajectoryPoint2D>::const_iterator tit=trj.begin(); tit != trj.end(); tit++) {
      //        glPushMatrix();
      //        glColor4f(0.2, .3, 1, 1);
      //        glTranslatef((*tit).x-pose.x, (*tit).y-pose.y, (*tit).v);
      //        gluSphere(sphere_, .2, 16, 16);
      //        glPopMatrix();
      //    }
      glDisable(GL_BLEND);
      //  glEnable(GL_DEPTH_TEST);

      //drawBestTrajectory_();
      glPopMatrix();
    }

    // TODO: fix cutting off of first/last points
    void Paw2GlView::drawCenterline(std::vector<CurvePoint>& center_line, double center_x, double center_y) {
      if(center_line.empty()) {return;}
      glDisable(GL_DEPTH_TEST);

      // Draw line between center line points
      glLineWidth(3.0);
      glEnable(GL_LINE_SMOOTH);
      glColor3f(0.6, 0.0, 0.0);
      glBegin(GL_LINE_STRIP);
      std::vector<CurvePoint>::const_iterator clit = center_line.begin();
      std::vector<CurvePoint>::const_iterator clit_end = --center_line.end();

      for(; clit != clit_end; clit++) {
        glVertex3f((*clit).x - center_x, (*clit).y - center_y, 0.0);
      }
      glEnd();
      glDisable(GL_LINE_SMOOTH);

      // Draw cones to show orientation, color represents curvature
      glPointSize(8.0);
      clit = center_line.begin();

      for(; clit != clit_end; clit++) {
        uint32_t col_index = std::min(255., 255*std::abs((*clit).kappa/0.2));
        glColor3f(cmap_rb2_red_[col_index], cmap_rb2_green_[col_index], cmap_rb2_blue_[col_index]);
        glPushMatrix();
        glTranslatef((*clit).x - center_x, (*clit).y - center_y, 0.0);
        glRotatef( 90.0 , 0., 1., 0.);
        glRotatef( -dgc_r2d((*clit).theta), 1., 0., 0. );
        glutSolidCone(0.4, 1.1, 6, 1);
        glPopMatrix();
      }

      glEnable(GL_DEPTH_TEST);
    }

    //void Paw2GlView::drawCompleteCenterLine(const std::vector<CurvePoint>& mission_line_bez, const std::vector<CurvePoint>& mission_line, double center_x, double center_y) {
    //  if(mission_line.empty()) {return;}
    //  glDisable(GL_DEPTH_TEST);
    //
    //  if(smoothed_mission_points_buf_size_<mission_line_bez.size()) {
    //    if(smoothed_mission_points_buf_) {delete[] smoothed_mission_points_buf_;}
    //    smoothed_mission_points_buf_ = new double[3*mission_line_bez.size()];
    //    smoothed_mission_points_buf_size_ = mission_line_bez.size();
    //  }
    //
    //
    //  for(uint32_t i=0, i3=0; i<mission_line_bez.size(); i++) {
    //    smoothed_mission_points_buf_[i3++]=mission_line_bez[i].x - center_x;
    //    smoothed_mission_points_buf_[i3++]=mission_line_bez[i].y - center_y;
    //    smoothed_mission_points_buf_[i3++]=0;
    //  }
    //
    //  glEnable(GL_MAP1_VERTEX_3);
    //
    //  // Draw line between center line points
    //  glLineWidth(3.0);
    //  glEnable(GL_LINE_SMOOTH);
    //  glColor3f(0.6, 0.0, 0.0);
    //  for (uint32_t segments = 0; segments < mission_line_bez.size()-1; segments += 3) {
    //    glMap1d(GL_MAP1_VERTEX_3, 0.0, 1.0, 3, 4, &smoothed_mission_points_buf_[3*segments]);
    //    glBegin(GL_LINE_STRIP);
    //    for (uint32_t i = 0; i <= 30; i++) {
    //      glEvalCoord1d((GLdouble) i / 30.0);
    //    }
    //    glEnd();
    //  }
    //  glDisable(GL_LINE_SMOOTH);
    //  glDisable(GL_MAP1_VERTEX_3);
    //
    //  // Draw cones to show orientation, color represents curvature
    //  glPointSize(8.0);
    //  std::vector<CurvePoint>::const_iterator clit = mission_line.begin();
    //  std::vector<CurvePoint>::const_iterator clit_end = --mission_line.end();
    //
    //  for(; clit != clit_end; clit++) {
    //    uint32_t col_index = std::min(255., 255*std::abs((*clit).kappa/0.2));
    //    glColor3f(cmap_rb2_red_[col_index], cmap_rb2_green_[col_index], cmap_rb2_blue_[col_index]);
    //    glPushMatrix();
    //    glTranslatef((*clit).x - center_x, (*clit).y - center_y, 0.0);
    //    glRotatef(  90.0 , 0., 1., 0.);
    //    glRotatef(  -dgc_r2d((*clit).theta), 1., 0., 0. );
    //    glutSolidCone(0.4, 1.1, 6, 1);
    //    glPopMatrix();
    //  }
    //
    //  glEnable(GL_DEPTH_TEST);
    //}

    ///////////////////////////////////////////////////////////////////////////////
    // GLU_TESS CALLBACKS
    ///////////////////////////////////////////////////////////////////////////////
    void tessBeginCB(GLenum which) {glBegin(which);}

    void tessEndCB() {glEnd();}

    void tessVertexCB(const GLvoid *data) {
      // cast back to double type
      const GLdouble *ptr = (const GLdouble*)data;

      glVertex3dv(ptr);
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Combine callback is used to create a new vertex where edges intersect.
    // In this function, copy the vertex data into local array and compute the
    // color of the vertex. And send it back to tessellator, so tessellator pass it
    // to vertex callback function.
    //
    // newVertex: the intersect point which tessellator creates for us
    // neighborVertex[4]: 4 neighbor vertices to cause intersection (given from 3rd param of gluTessVertex()
    // neighborWeight[4]: 4 interpolation weights of 4 neighbor vertices
    // outData: the vertex data to return to tessellator
    ///////////////////////////////////////////////////////////////////////////////
    void tessCombineCB(const GLdouble newVertex[3], const GLdouble *neighborVertex[4],
        const GLfloat neighborWeight[4], GLdouble **outData)
    {
      //      // copy new intersect vertex to local array
      //      // Because newVertex is temporal and cannot be hold by tessellator until next
      //      // vertex callback called, it must be copied to the safe place in the app.
      //      // Once gluTessEndPolygon() called, then you can safly deallocate the array.
      //      vertices[vertexIndex][0] = newVertex[0];
      //      vertices[vertexIndex][1] = newVertex[1];
      //      vertices[vertexIndex][2] = newVertex[2];
      //
      //      // compute vertex color with given weights and colors of 4 neighbors
      //      // the neighborVertex[4] must hold required info, in this case, color.
      //      // neighborVertex was actually the third param of gluTessVertex() and is
      //      // passed into here to compute the color of the intersect vertex.
      //      vertices[vertexIndex][3] = neighborWeight[0] * neighborVertex[0][3] +   // red
      //                                 neighborWeight[1] * neighborVertex[1][3] +
      //                                 neighborWeight[2] * neighborVertex[2][3] +
      //                                 neighborWeight[3] * neighborVertex[3][3];
      //      vertices[vertexIndex][4] = neighborWeight[0] * neighborVertex[0][4] +   // green
      //                                 neighborWeight[1] * neighborVertex[1][4] +
      //                                 neighborWeight[2] * neighborVertex[2][4] +
      //                                 neighborWeight[3] * neighborVertex[3][4];
      //      vertices[vertexIndex][5] = neighborWeight[0] * neighborVertex[0][5] +   // blue
      //                                 neighborWeight[1] * neighborVertex[1][5] +
      //                                 neighborWeight[2] * neighborVertex[2][5] +
      //                                 neighborWeight[3] * neighborVertex[3][5];
      //
      //
      //      // return output data (vertex coords and others)
      //      *outData = vertices[vertexIndex];   // assign the address of new intersect vertex
      //
      //      ++vertexIndex;  // increase index for next vertex
    }

    void tessErrorCB(GLenum errorCode) {
      const GLubyte *errorStr;

      errorStr = gluErrorString(errorCode);
      std::cout << "[ERROR]: " << errorStr << std::endl;
    }

    void Paw2GlView::drawCompleteCenterLine(const std::vector<CurvePoint>& mission_line_bez, const std::vector<CurvePoint>& center_line, double center_x, double center_y) {
      if(center_line.empty()) {return;}
      glDisable(GL_DEPTH_TEST);

      if(smoothed_mission_points_buf_size_<center_line.size()) {
        if(smoothed_mission_points_buf_) {delete[] smoothed_mission_points_buf_;}
        smoothed_mission_points_buf_ = new double[6*center_line.size()];
        smoothed_mission_points_buf_size_ = center_line.size();
      }

      uint32_t i3=0;
      uint32_t num_points=center_line.size();
      for(uint32_t i=0; i<num_points; i+=2) {
        //    for(uint32_t i=0; i<center_line.size(); i++) {
        double theta = center_line[i].theta;
        smoothed_mission_points_buf_[i3++]=center_line[i].x - center_x-3*sin(theta);
        smoothed_mission_points_buf_[i3++]=center_line[i].y - center_y+3*cos(theta);
        smoothed_mission_points_buf_[i3++]=0;
      }

      for(int32_t i=num_points-1; i>=0; i-=2) {
        //    for(int32_t i=center_line.size()-1; i>=0; i--) {
        double theta = center_line[i].theta;
        smoothed_mission_points_buf_[i3++]=center_line[i].x - center_x+3*sin(theta);
        smoothed_mission_points_buf_[i3++]=center_line[i].y - center_y-3*cos(theta);
        smoothed_mission_points_buf_[i3++]=0;
      }

      GLUtesselator * tess = gluNewTess(); // create a tessellator
      if(!tess) return;

      // register callback functions
      gluTessCallback(tess, GLU_TESS_BEGIN, (void (*)())tessBeginCB);
      gluTessCallback(tess, GLU_TESS_END, (void (*)())tessEndCB);
      gluTessCallback(tess, GLU_TESS_ERROR, (void (*)())tessErrorCB);
      gluTessCallback(tess, GLU_TESS_VERTEX, (void (*)())tessVertexCB);
      gluTessCallback(tess, GLU_TESS_COMBINE, (void (*)())tessCombineCB);

      // tessellate and compile a concave quad into display list
      glColor3f(1,.1,1);
      gluTessBeginPolygon(tess, 0); // with NULL data
      gluTessBeginContour(tess); // outer quad
      for (uint32_t i = 0; i < num_points; i++) {
        //        for (uint32_t i = 0; i < 2*center_line.size()-1; i++) {
        printf("tp: %f, %f, %f\n", smoothed_mission_points_buf_[3*i], smoothed_mission_points_buf_[3*i+1], smoothed_mission_points_buf_[3*i+2]);
        gluTessVertex(tess, &smoothed_mission_points_buf_[3*i], &smoothed_mission_points_buf_[3*i]);
      }
      gluTessEndContour(tess);
      gluTessEndPolygon(tess);

      gluDeleteTess(tess); // delete after tessellation
      glEnable(GL_DEPTH_TEST);
    }

    void Paw2GlView::drawTopology(double center_x, double center_y) {
      RoutePlanner::RndfEdge* edge = gui_->awp_.chsm_planner_->topology_->ego_vehicle.edge();
      if (edge==NULL)
      return;
      double cx = gui_->awp_.chsm_planner_->topology_->ego_vehicle.xMatched();
      double cy = gui_->awp_.chsm_planner_->topology_->ego_vehicle.yMatched();

      // paint the whole graph in white
      if (gui_->show_complete_graph_)
      gui_->awp_.chsm_planner_->topology_->paint_complete_graph(center_x, center_y, 1, 1, 1);

      // paint mission graph on top in pink
      if (gui_->show_mission_graph_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->mission_mutex_);
        gui_->awp_.chsm_planner_->topology_->paint_mission_graph(center_x, center_y, 1, 0, 1, false);
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->mission_mutex_);
      }

      if (gui_->show_complete_mission_graph_) {
        pthread_mutex_lock(&gui_->awp_.chsm_planner_->mission_mutex_);
        gui_->awp_.chsm_planner_->topology_->paint_complete_mission(center_x, center_y);
        pthread_mutex_unlock(&gui_->awp_.chsm_planner_->mission_mutex_);
      }

      // draw matched edge in green
      if (gui_->show_matched_edge_) {
        gui_->awp_.chsm_planner_->topology_->draw_edge(center_x, center_y, *edge, 0., 1., 0.);
      }
      // draw this point in yellow
      if (gui_->show_yellow_dot_) {
        gui_->awp_.chsm_planner_->topology_->draw_dot_at(cx-center_x, cy-center_y, 1., 1., 0.);
      }
      //glPopMatrix();
    }

    void Paw2GlView::drawObstacles(double center_x, double center_y) {
      if(!gui_->awp_.chsm_planner_) return;

      if ( !gui_->awp_.chsm_planner_->vehicle_manager) {
        std::cout << "Warning: drawObstacles(): VehicleManager == 0!\n";
        return;
      }

      pthread_mutex_lock(&gui_->awp_.chsm_planner_->dyn_obstacles_mutex_);
      pthread_mutex_lock(&gui_->awp_.chsm_planner_->topology_mutex_);
      glPushMatrix();

      int id_display = 0;

      //    pthread_mutex_lock(&awp_.chsm_planner_->intersection_predictor_mutex_);
      for (std::map<int, Vehicle>::iterator it = gui_->awp_.chsm_planner_->vehicle_manager->vehicle_map.begin(); it
          != gui_->awp_.chsm_planner_->vehicle_manager->vehicle_map.end(); it++) {
        const Vehicle& veh = it->second;

        double width = (veh.width() != 0.0 ? veh.width() : 2.5);
        double length = (veh.length() != 0.0 ? veh.length() : 4.5);

        // Turn signals
        double proba_none = 0.0;
        double proba_left = 0.0;
        double proba_right = 0.0;

        bool display_turn_signal = false;
        if (veh.id() == id_display) {
          display_turn_signal = true;
        }

        //     pthread_mutex_unlock(&gui_->awp_.chsm_planner_->intersection_predictor_mutex_);
        //cout << "speed=" << veh.speed() << endl;
        //    if ((gui_->awp_.chsm_planner_->topology_->intersection_manager != NULL) && (veh.speed() != -100.0)) {
        //if ((gui_->awp_.chsm_planner_->topology_->intersection_manager != NULL) && (gui_->awp_.chsm_planner_->topology_->intersection_manager->relevant_vehicles_.find(veh.id()) != gui_->awp_.chsm_planner_->topology_->intersection_manager->relevant_vehicles_.end())) {
        draw_observed_car(veh.xMatchedFrom()-center_x, veh.yMatchedFrom()-center_y, veh.yawMatchedFrom(), width, length, veh.id(),
            veh.speed(), 1, 1, 0, 0, 0, 0, true, cameraPose.pan, proba_none, proba_left, proba_right, display_turn_signal);
        //    }


        // Multi Edge Kanten malen
        for (std::map< RndfEdge*, double >::const_iterator edge_it = veh.edges().begin(); edge_it != veh.edges().end(); ++edge_it) {
          RndfEdge* edge = edge_it->first;
          Point_2 p1( edge->fromVertex()->x(), edge->fromVertex()->y() );
          Point_2 p2( edge->toVertex()->x(), edge->toVertex()->y() );
          Point_2 dot = p1 + ( p2 - p1 ) * edge_it->second / edge->length();
          gui_->awp_.chsm_planner_->topology_->draw_dot_at(dot.x()-center_x, dot.y()-center_y, 0.7, 0.2, 0);
          gui_->awp_.chsm_planner_->topology_->draw_edge(center_x,center_y, *edge, 0.7, 0, 0.7);
        }

        // draw main matched edge
        gui_->awp_.chsm_planner_->topology_->draw_dot_at(veh.xMatched()-center_x, veh.yMatched()-center_y, 0, .5, 0);
        gui_->awp_.chsm_planner_->topology_->draw_edge(center_x,center_y, *(veh.edge()), 1, 0, 1);
      }
      glColor3f(0.0, 0.8, 0.0);
      for (std::map<int, Vehicle*>::iterator it = gui_->awp_.chsm_planner_->vehicle_manager->moving_map.begin(); it
          != gui_->awp_.chsm_planner_->vehicle_manager->moving_map.end(); it++) {
        const Vehicle* veh = it->second;
        //      std::cout << "moving " << veh->dist_to_end << " "<< veh->xMatchedFrom() <<" "<< veh->yMatchedFrom()<<" "<< veh->length << std::endl;
        draw_dashed_circle(veh->xMatchedFrom()-center_x, veh->yMatchedFrom()-center_y, veh->length());
      }
      pthread_mutex_unlock(&gui_->awp_.chsm_planner_->topology_mutex_);

      glColor3f(0.8, 0.0, 0.0);
      for (std::map<int, Vehicle*>::iterator it = gui_->awp_.chsm_planner_->vehicle_manager->blockage_map.begin(); it
          != gui_->awp_.chsm_planner_->vehicle_manager->blockage_map.end(); it++) {
        const Vehicle* veh = it->second;
        //      std::cout << "standing " << veh->dist_to_end << " " <<veh->xMatchedFrom() <<" "<< veh->yMatchedFrom()<<" "<< veh->length << std::endl;
        draw_dashed_circle(veh->xMatchedFrom()-center_x, veh->yMatchedFrom()-center_y, veh->length());
      }
      glPopMatrix();
      pthread_mutex_unlock(&gui_->awp_.chsm_planner_->dyn_obstacles_mutex_);

      glPushMatrix();

      //  double d;
      //  GraphTools::PlaceOnGraph obstacle_place;
      //  Vehicle* next_obst = gui_->awp_.chsm_planner_->topology_->next_obstacle(d, obstacle_place, MAX_SCAN_DISTANCE_OBSTACLE );
      //
      //  if (next_obst ) {
      //    draw_observed_car(next_obst->xMatchedFrom()-center_x, next_obst->yMatchedFrom(), next_obst->yawMatchedFrom()-center_y, 2.5, 4.5, 0,
      //        0, 0, 1, 0, 0, 0, 0, false, gui3D.camera_pose.pan);
      //
      //    gui_->awp_.chsm_planner_->topology_->draw_dot_at(next_obst->xMatched()-center_x, next_obst->yMatched()-center_y, 1., 0.5, 0.0);
      //  }

      glPopMatrix();
    }

    void Paw2GlView::drawObstaclePredictions(drc::GlobalPose& pose) {
      const std::map<int, Vehicle>& vehicles = gui_->awp_.chsm_planner_->vehicle_manager->vehicles();
      std::map<int, Vehicle>::const_iterator vit=vehicles.begin(), vit_end=vehicles.end();
      for (; vit != vit_end; vit++) {
        for (int j = (*vit).second.predictedTrajectory().size()-1; j>=0; j--) {
          const MovingBox& box = (*vit).second.predictedTrajectory()[j];
          glPushMatrix();
          glTranslatef(box.x - pose.utmX(), box.y - pose.utmY(), 0.0);
          glTranslatef(-box.length/2+box.ref_offset, 0, 0.9);
          //        glRotatef(box.psi / M_PI * 180 + 90, 0, 0, 1);
          glRotatef(box.psi / M_PI * 180, 0, 0, 1);
          glEnable(GL_BLEND);
          glDisable(GL_DEPTH_TEST);
          glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
          glEnable(GL_LINE_SMOOTH);

          glLineWidth(2);
          float val = float((*vit).second.predictedTrajectory().size()-1-j)/float((*vit).second.predictedTrajectory().size()-1);
          float val2 = val*.4;
          if(j==0) {glColor4f(1, 0, 0, 0.7);}
          else {glColor4f(1-val, 1-val2, 1-val2, 0.2);}
          float l = box.length;
          float w = box.width;

          glBegin(GL_POLYGON);
          glVertex2f(l / 2, w / 2);
          glVertex2f(l / 2, -w / 2);
          glVertex2f(-l / 2, -w / 2);
          glVertex2f(-l / 2, w / 2);
          glEnd();

          glBegin(GL_LINE_LOOP);
          glVertex2f(l / 2, w / 2);
          glVertex2f(l / 2, -w / 2);
          glVertex2f(-l / 2, -w / 2);
          glVertex2f(-l / 2, w / 2);
          glEnd();

          glLineWidth(1);

          glDisable(GL_LINE_SMOOTH);
          glDisable(GL_BLEND);
          glEnable(GL_DEPTH_TEST);

          //        glEnable(GL_LIGHTING);
          //        glScalef(5.0, 5.0, 5.0);
          //        if(box.length > 5.5) {
          //          hummer_->draw();
          //        }
          //        else if(box.length < 5.2) {
          //          elise_->draw();
          //        }
          //        else {
          //          porsche_->draw();
          //        }
          //        glDisable(GL_LIGHTING);
          glPopMatrix();
        }

      }
    }

    void Paw2GlView::drawVehicle(const drc::GlobalPose& pose) {

      glPushMatrix();
      glRotatef(dgc_r2d(pose.yaw()), 0, 0, 1);
      glRotatef(dgc_r2d(pose.pitch()), 0, 1, 0);
      glRotatef(dgc_r2d(pose.roll()), 1, 0, 0);

      glTranslatef(1.25, 0, 0.9);
      glEnable(GL_DEPTH_TEST);

      bool show_car_model_=true;
      if(show_car_model_) {
        glEnable(GL_LIGHTING);
        static double last_timestamp_=0;
        static double wheel_angle_=0;
        double timestamp = drc::Time::current();
        double dt = timestamp - last_timestamp_;
        last_timestamp_ = timestamp;
        double dist = dt * pose.v();
        double dangle = 2 * M_PI * dist / (M_PI * DGC_PASSAT_WHEEL_RADIUS);
        wheel_angle_ = normalizeAngle(wheel_angle_ + dangle);
        passatwagonmodel_draw(car_, 0, wheel_angle_, 0);
        glDisable(GL_LIGHTING);
      }
      else {
        //      if (!plain_mode) glTranslatef(1.65, 0, -DGC_PASSAT_HEIGHT);
        //      else glTranslatef(1.65, 0.0, 0.0);
        //     draw_cage(DGC_PASSAT_LENGTH, DGC_PASSAT_WIDTH, DGC_PASSAT_HEIGHT );
        //      if (show_cte) {
        //        glPushMatrix();
        //          glTranslatef(0.0, 0.0, 0.2);
        //          draw_cte(controller_target.cross_track_error);
        //        glPopMatrix();
        //      }
      }
      glPopMatrix();

      //  // *******************************************************************
      //  // *    Draw the vehicle
      //  // *******************************************************************
      //    if (!show_simulator) {
      //    if (last_can_ts != 0) {
      //      double dt = can.timestamp - last_can_ts;
      //      double speed_ms = dgc_kph2ms((can.wheel_speed_rl + can.wheel_speed_rr) / 2.0);
      //      dist = dt * speed_ms;
      //      dangle = 2 * M_PI * dist / (M_PI * DGC_PASSAT_WHEEL_RADIUS);
      //      wheel_angle = vlr::normalizeAngle(wheel_angle + dangle);
      //    }
      //    last_can_ts = can.timestamp;
      //    glPushMatrix();
      //    {
      //      if (plain_mode) glTranslatef(0, 0, DGC_PASSAT_HEIGHT);
      //
      //      glRotatef(dgc_r2d(car_pose.yaw), 0, 0, 1);
      //      glRotatef(dgc_r2d(car_pose.pitch), 0, 1, 0);
      //      glRotatef(dgc_r2d(car_pose.roll), 1, 0, 0);
      //      glTranslatef(1.65, 0, -0.6);
      //      glEnable(GL_LIGHTING);
      //      if (show_car) {
      //        velodyne_angle = fmod(velodyne_angle + (M_PI / 1.7), 2 * M_PI );
      //        if (show_stickers) {
      //          draw_stickered_passat(passat, dgc_d2r(30.0) * (can.steering_angle / 360.0), wheel_angle, velodyne_angle, 1);
      //        }
      //        else {
      //          passatwagonmodel_draw(passat, dgc_d2r(30.0) * (can.steering_angle / 360.0), wheel_angle, velodyne_angle);
      //        }
      //      }
      //      else {
      //        passatwagonwheels_draw(passat, dgc_d2r(30.0) * (can.steering_angle / 360.0), wheel_angle);
      //      }
      //      glDisable(GL_LIGHTING);
      //
      //    }
      //    glPopMatrix();
      //    }
    }

    // this function is used to either update the raw obstacle map texture
    // or the configuration space texture
    void Paw2GlView::updateObstacleMapTexture(const uint8_t* map_data, GLuint& texture, double& last_timestamp) {

      if(!map_data) {return;}
      //   if(gui_->awp_.chsm_planner_->obstacleMapTimestamp() == last_timestamp) {return;}

      glEnable(GL_TEXTURE_2D);
      glEnable(GL_TEXTURE_RECTANGLE_ARB);

      glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture);

      //  glTexParameterf(texType, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      //  glTexParameterf(texType, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);


      glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_ALPHA, gui_->awp_.chsm_planner_->obstacleMapWidth(), gui_->awp_.chsm_planner_->obstacleMapHeight(), 0, GL_ALPHA, GL_UNSIGNED_BYTE, map_data);
      glDisable(GL_TEXTURE_2D);
      glDisable(GL_TEXTURE_RECTANGLE_ARB);

      last_timestamp = gui_->awp_.chsm_planner_->obstacleMapTimestamp();
    }

    void Paw2GlView::updateRoadMapTexture(const uint8_t* map_data, double timestamp) {

      if(!map_data) {return;}
      if(timestamp == last_roadmap_timestamp_) {return;}

      glEnable(GL_TEXTURE_2D);
      glEnable(GL_TEXTURE_RECTANGLE_ARB);

      glBindTexture(GL_TEXTURE_RECTANGLE_ARB, road_map_texture_);

      glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_ALPHA, gui_->internal_data_->width(), gui_->internal_data_->height(),
          0, GL_ALPHA, GL_UNSIGNED_BYTE, map_data);
      glDisable(GL_TEXTURE_2D);
      glDisable(GL_TEXTURE_RECTANGLE_ARB);

      last_roadmap_timestamp_ = timestamp;
    }

    void Paw2GlView::drawObstacleMap(GLuint texture, double smooth_x, double smooth_y, const drc::GlobalPose& latest_pose, float r, float g, float b, bool invert) {
      glEnable(GL_TEXTURE_2D);
      glEnable(GL_TEXTURE_RECTANGLE_ARB);

      glPushMatrix();
      int32_t width = (int32_t)gui_->awp_.chsm_planner_->obstacleMapWidth();
      int32_t height = (int32_t)gui_->awp_.chsm_planner_->obstacleMapHeight();
      double resolution = gui_->awp_.chsm_planner_->obstacleMapResolution();
      double trans_x = -width*resolution/2 + smooth_x - latest_pose.x();
      double trans_y = -height*resolution/2 + smooth_y - latest_pose.y();
      glTranslatef(trans_x, trans_y, 0);
      glEnable(GL_BLEND);
      if(invert) {glBlendFunc(GL_ONE_MINUS_SRC_ALPHA, GL_SRC_ALPHA);}
      else {glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);}
      glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
      glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture);

      glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

      float x1=0, y1=0;
      float x2 = width*resolution;
      float y2 = height*resolution;
      glColor3f(r, g, b);
      glBegin(GL_POLYGON);
      glTexCoord2f(0, 0); glVertex2f(x2, y2);
      glTexCoord2f(0, height); glVertex2f(x2, y1);
      glTexCoord2f(width, height); glVertex2f(x1, y1);
      glTexCoord2f(width, 0); glVertex2f(x1, y2);
      glEnd();
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glDisable(GL_TEXTURE_RECTANGLE_ARB);
      glDisable(GL_TEXTURE_2D);

      // draw map boundary
      glColor4f(1, 1, 0.2, 0.5);
      glBegin(GL_LINE_LOOP);
      glVertex2f(0, 0);
      glVertex2f(x2, 0);
      glVertex2f(x2, y2);
      glVertex2f(0, y2);
      glEnd();
      glPopMatrix();
    }

    void Paw2GlView::drawRoadMap(const drc::GlobalPose& pose, const drc::GlobalPose& latest_pose, float r, float g, float b) {
      glEnable(GL_TEXTURE_2D);
      glEnable(GL_TEXTURE_RECTANGLE_ARB);

      glPushMatrix();
      int32_t width = (int32_t)gui_->internal_data_->width();
      int32_t height = (int32_t)gui_->internal_data_->height();
      double resolution = gui_->internal_data_->resolution();
      double trans_x = -width*resolution/2 + pose.x() - latest_pose.x();
      double trans_y = -height*resolution/2 + pose.y() - latest_pose.y();
      glTranslatef(trans_x, trans_y, 0);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
      glBindTexture(GL_TEXTURE_RECTANGLE_ARB, road_map_texture_);

      glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

      float x1=0, y1=0;
      float x2 = width*resolution;
      float y2 = height*resolution;
      glColor3f(r, g, b);
      glBegin(GL_POLYGON);
      glTexCoord2f(0, 0); glVertex2f(x2, y2);
      glTexCoord2f(0, height); glVertex2f(x2, y1);
      glTexCoord2f(width, height); glVertex2f(x1, y1);
      glTexCoord2f(width, 0); glVertex2f(x1, y2);
      glEnd();
      glDisable(GL_TEXTURE_RECTANGLE_ARB);
      glDisable(GL_TEXTURE_2D);

      // draw map boundary
      glColor4f(1, 1, 0.2, 0.5);
      glBegin(GL_LINE_LOOP);
      glVertex2f(0, 0);
      glVertex2f(x2, 0);
      glVertex2f(x2, y2);
      glVertex2f(0, y2);
      glEnd();
      glPopMatrix();
    }

    void Paw2GlView::drawDestinations(const double center_x, const double center_y) {
      // Empty for now...
    }

    void Paw2GlView::drawPoseAccuracy(dgc_pose_t robot_pose_accuracy) {
      glPushMatrix();
      glColor4f(0.6, 0.6, 0, 0.5);
      draw_ellipse(0, 0, robot_pose_accuracy.x, robot_pose_accuracy.y,0);
      glPopMatrix();
    }

    void Paw2GlView::drawDistanceCond(const double x, const double y, const double r)
    {
      if (r < std::numeric_limits<double>::max()) {
        draw_circle(x, y, std::abs(r), 0);
        draw_circle(x, y, 0.15, 1);
      }
    }

    void Paw2GlView::drawEgoDistances(const double /*center_x*/, const double /*center_y*/, const double yaw, Topology* t)
    {
      if (!t->isMissionPlanned()) return;

      //  double x = FRONT_BUMPER_DELTA*cos(yaw) - center_x;
      //  double y = FRONT_BUMPER_DELTA*sin(yaw) - center_y;
      double x = FRONT_BUMPER_DELTA*cos(yaw);
      double y = FRONT_BUMPER_DELTA*sin(yaw);
      glColor3f(0.0, 0.0, 1.0);
      drawDistanceCond(x, y, t->distToNextStandingVehicle());
      glColor3f(0.0, 1.0, 0.0);
      drawDistanceCond(x, y, t->distToNextMovingVehicle());
      glColor3f(1.0, 0.0, 0.0);
      drawDistanceCond(x, y, t->distToNextStopLine());
      glColor3f(1.0, 0.0, 0.0);
      drawDistanceCond(x, y, t->distToNextTrafficLight());
      glColor3f(1.0, 0.0, 0.0);
      drawDistanceCond(x, y, t->distToNextCrosswalk());
      glColor3f(0.5, 0.0, 0.0);
      drawDistanceCond(x, y, t->distToNextIntersection());
      glColor3f(0.7, 0.7, 0.0);
      drawDistanceCond(x, y, t->distToNextKTurn());
      glColor3f(0.3, 1.0, 0.3);
      drawDistanceCond(x, y, t->distToNextZone());
      glColor3f(0.3, 0.7, 0.3);
      drawDistanceCond(x, y, t->distToNextZoneExit());
      glColor3f(0.3, 0.7, 0.3);
      drawDistanceCond(BACK_BUMPER_DELTA*-cos(yaw), BACK_BUMPER_DELTA*-sin(yaw), t->distToPreviousVehicle());

    }

    void Paw2GlView::drawVehDistances(const double center_x, const double center_y, Topology* t)
    {
      //  cout << "Draw Veh Distances" << endl;
      if (!t->vehicle_manager) return;

      // Nächste Kreuzung ermitteln
      RndfIntersection* isec = t->get_next_intersection();
      //  if (!isec) return;
      //  if (isec) cout << "  -> Intersec found ("<< isec->getId() <<")" << endl;

      glPushMatrix();
      glTranslatef(-center_x, -center_y, 0.);

      // Distanzen für die einzeln Fahrzuge ermitteln und zeichnen
      const std::map<int, Vehicle>& vehicle_map = t->vehicle_manager->vehicles();
      for (std::map<int, Vehicle>::const_iterator it=vehicle_map.begin(); it != vehicle_map.end(); ++it)
      {
        const Vehicle& veh = it->second;
        double x = veh.xMatchedFrom();
        double y = veh.yMatchedFrom();
        double fx = veh.xMatchedFrom() + (veh.length()/2.)*cos(veh.yawMatchedFrom());
        double fy = veh.yMatchedFrom() + (veh.length()/2.)*sin(veh.yawMatchedFrom());
        double bx = veh.xMatchedFrom() - (veh.length()/2.)*cos(veh.yawMatchedFrom());
        double by = veh.yMatchedFrom() - (veh.length()/2.)*sin(veh.yawMatchedFrom());

        //    // Distanz zur Stoplinine visualisieren
        //    double distToStopLine = veh.distToStopLine(isec);
        //    cout << "  -> Veh "<< veh.id <<" dist to stopline: "<< distToStopLine <<" m" << endl;
        //    if ( !isfinite(distToStopLine) ) continue;
        //
        //    if ( fabs(distToStopLine) <= VEH_DIST_FROM_STOPLINE_THRESHOLD) {
        //      glColor3f(0.0, 1.0, 0.0);
        //      drawDistanceCond(x, y, veh.length / 2);
        //    } else if ( distToStopLine >= 0. ) {
        //      glColor3f(0.0, 1.0, 0.0);
        //      drawDistanceCond(fx, fy, distToStopLine);
        //    } else {
        //      glColor3f(1.0, 0.0, 0.0);
        //      drawDistanceCond(fx, fy, -distToStopLine);
        //    }

        // Distanz zur Intersection visualisieren
        double distToIntersection = veh.distToIntersection(isec);
        //    cout << "  -> Veh "<< veh.id <<" dist to isec: "<< distToIntersection <<" m" << endl;
        if ( !std::isfinite(distToIntersection) ) {continue;}

        if ( std::abs(distToIntersection) == 0.) {
          glColor3f(0.0, 1.0, 0.0);
          drawDistanceCond(x, y, veh.length());
        }
        else if ( distToIntersection > 0.) {
          glColor3f(0.0, 0.0, 1.0);
          drawDistanceCond(fx, fy, distToIntersection);
        }
        else {
          glColor3f(1.0, 0.0, 0.0);
          drawDistanceCond(bx, by, -distToIntersection);
        }

      }

      glPopMatrix();
    }

    void Paw2GlView::drawIntersectionMergingPoint(const double center_x, const double center_y, Topology* t)
    {
      if (!t->vehicle_manager) return;

      RndfIntersection* isec = t->get_next_intersection();
      if (!isec) return;
      //if (isec) cout << "  -> Intersec found ("<< isec->getId() <<") center " << isec->center() << endl;

      CGAL_Geometry::Point_2 center = isec->center();
      glColor3f(1.0, 0.3, 0.3);
      draw_circle(center.x()-center_x, center.y()-center_y, 0.5, 1);
      draw_circle(center.x()-center_x, center.y()-center_y, isec->radius(), 0);

      // Speziellen MergingPoint ermiteln und malen
      glColor3f(1.0, 0.5, 0.1);
      GraphPlace ego_place( t->ego_vehicle.edge(), t->ego_vehicle.distFromStart() );
      for (std::map<int, Vehicle>::iterator it = t->vehicle_manager->vehicle_map.begin(); it != t->vehicle_manager->vehicle_map.end(); ++it)
      {
        GraphPlace veh_place( it->second.edge(), it->second.distFromStart() );
        GraphPlace cross_place = searchCrossingPoint(ego_place, veh_place, isec, 50.);
        if (cross_place.valid) {
          Point_2 cp = cross_place.point();
          //    std::cout << "[CrossPoint] Isec found: "<< cp << std::endl;
          draw_circle(cp.x()-center_x, cp.y()-center_y, 0.5, 1);
        }// else std::cout << "[CrossPoint] No Isec found"<< std::endl;
      }
    }

    void Paw2GlView::drawLaneChangeMergingPoints(const double center_x, const double center_y, Topology* t)
    {
      GraphPlace ego_place( t->ego_vehicle.edge(), t->ego_vehicle.distFromStart() );

      StLaneChange* lc = gui_->awp_.chsm_planner_->lane_change_data;
      if ( ! lc ) {
        //    cout << "** No Lanechange Data available **" << endl;
        return;
      }
      //  cout << "Lanechange Data available" << endl;

      // draw change point
      if ( lc->change_point.isValid() ) {
        Point_2 lcs_p = (*lc->change_point.edge_)->edge()->point( lc->merge_point.offset() );
        glColor3f(0.9, 0.3, 0.5);
        draw_circle(lcs_p.x()-center_x, lcs_p.y()-center_y, 0.5, 1);
      }

      // draw merge point
      if ( lc->merge_point.valid ) {
        glColor3f(0.9, 0.3, 0.5);
        draw_circle(lc->merge_point.point().x()-center_x, lc->merge_point.point().y()-center_y, 0.5, 1);
        if (lc->merge_allowed)
        glColor3f(0., 0.9, 0.);
        else
        glColor3f(0.9, 0., 0.);
        draw_circle(lc->merge_point.point().x()-center_x, lc->merge_point.point().y()-center_y, 0.9, 0);
      }

      // draw merge end point
      if ( lc->merge_end_point.valid ) {
        glColor3f(0.9, 0.3, 0.5);
        draw_circle(lc->merge_end_point.point().x()-center_x, lc->merge_end_point.point().y()-center_y, 0.5, 0);
        if (lc->merge_allowed)
        glColor3f(0., 0.9, 0.);
        else
        glColor3f(0.9, 0., 0.);
        draw_circle(lc->merge_end_point.point().x()-center_x, lc->merge_end_point.point().y()-center_y, 0.9, 0);
      }

      //
      //  // merge left
      //  GraphPlace merge_start = ego_place;
      //  GraphPlace merge_end;
      //  merge_start.goToLeftEdge();
      //  if ( ! merge_start.valid ) goto merge_right;
      //  glColor3f(0.9, 0.3, 0.5);
      //  draw_circle(merge_start.point().x()-center_x, merge_start.point().y()-center_y, 0.5, 1);
      //
      //  merge_end = searchDistOnLane(merge_start, GraphSearchTraits::FORWARD, 30. );
      //  if ( ! merge_end.valid ) goto merge_right;
      //  glColor3f(0.9, 0.3, 0.6);
      //  draw_circle(merge_end.point().x()-center_x, merge_end.point().y()-center_y, 0.8, 0);
      //
      //  merge_right:
      //
      //  // merge right
      //  merge_start = ego_place;
      //  merge_start.goToRightEdge();
      //  if ( ! merge_start.valid ) goto merge_left_opposite;
      //  glColor3f(0.9, 0.3, 0.5);
      //  draw_circle(merge_start.point().x()-center_x, merge_start.point().y()-center_y, 0.5, 1);
      //
      //  merge_end = searchDistOnLane(merge_start, GraphSearchTraits::FORWARD, 30. );
      //  if ( ! merge_end.valid ) goto merge_left_opposite;
      //  glColor3f(0.9, 0.3, 0.6);
      //  draw_circle(merge_end.point().x()-center_x, merge_end.point().y()-center_y, 0.8, 0);
      //
      //  merge_left_opposite:
      //
      //  // merge right
      //  merge_start = ego_place;
      //  merge_start.goToLeftOppositeEdge();
      //  if ( ! merge_start.valid ) return;
      //  glColor3f(0.9, 0.3, 0.5);
      //  draw_circle(merge_start.point().x()-center_x, merge_start.point().y()-center_y, 0.5, 1);
      //
      //  merge_end = searchDistOnLane(merge_start, GraphSearchTraits::BACKWARD, 30. );
      //  if ( ! merge_end.valid ) return;
      //  glColor3f(0.9, 0.3, 0.6);
      //  draw_circle(merge_end.point().x()-center_x, merge_end.point().y()-center_y, 0.8, 0);
    }

    void Paw2GlView::drawPoseHistory(double center_x, double center_y) {
      glLineWidth(4.0);

      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_LINE_SMOOTH);
      glColor3f(0.6, 0.0, 0.0);
      glBegin(GL_LINES);
      int num_poses = (int)gui_->awp_.pose_history_.size();
      for (int i=0; i<num_poses-1; ++i) {
        glVertex3f(gui_->awp_.pose_history_[i].x - center_x, gui_->awp_.pose_history_[i].y - center_y, 0);
        glVertex3f(gui_->awp_.pose_history_[i+1].x - center_x, gui_->awp_.pose_history_[i+1].y - center_y, 0);
      }
      glEnd();

      glDisable(GL_LINE_SMOOTH);
      glDisable(GL_BLEND);
    }

    const float Paw2GlView::cmap_rb1_red_[256] = {
      1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
      1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.988235, 0.964706, 0.941176, 0.917647, 0.894118, 0.870588, 0.847059, 0.823530, 0.800000, 0.776470, 0.752941, 0.729412, 0.705882, 0.682353, 0.658824, 0.635294, 0.611765, 0.588235, 0.564706, 0.541176, 0.517647,
      0.494118, 0.470588, 0.447059, 0.423529, 0.400000, 0.376470, 0.352941, 0.329412, 0.305882, 0.282353, 0.258823, 0.235294, 0.211765, 0.188235, 0.164706, 0.141176, 0.117647, 0.094118, 0.070588, 0.047059, 0.023529, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.023530, 0.047059, 0.070588, 0.094118, 0.117647, 0.141176, 0.164706, 0.188235, 0.211765, 0.235295, 0.258824, 0.282353, 0.305882, 0.329412, 0.352941, 0.376470, 0.400000, 0.423530, 0.447059, 0.470588, 0.494118,
      0.517647, 0.541176, 0.564706, 0.588235, 0.611765, 0.635294, 0.658824, 0.682353, 0.705882, 0.729412, 0.752941, 0.776470, 0.800000, 0.823530, 0.847059, 0.870588, 0.894118, 0.917647, 0.941176, 0.964706, 0.988236, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
      1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
    };

    const float Paw2GlView::cmap_rb1_green_[256] = {
      0.000000, 0.023529, 0.047059, 0.070588, 0.094118, 0.117647, 0.141176, 0.164706, 0.188235, 0.211765, 0.235294, 0.258824, 0.282353, 0.305882, 0.329412, 0.352941, 0.376471, 0.400000, 0.423529, 0.447059, 0.470588, 0.494118, 0.517647, 0.541176, 0.564706, 0.588235, 0.611765, 0.635294, 0.658824, 0.682353, 0.705882, 0.729412,
      0.752941, 0.776471, 0.800000, 0.823529, 0.847059, 0.870588, 0.894118, 0.917647, 0.941177, 0.964706, 0.988235, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
      1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
      1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
      0.988235, 0.964706, 0.941176, 0.917647, 0.894118, 0.870588, 0.847059, 0.823529, 0.800000, 0.776470, 0.752941, 0.729412, 0.705882, 0.682353, 0.658823, 0.635294, 0.611765, 0.588235, 0.564706, 0.541176, 0.517647, 0.494117, 0.470588, 0.447059, 0.423529, 0.400000, 0.376470, 0.352941, 0.329412, 0.305882, 0.282353, 0.258823,
      0.235294, 0.211765, 0.188235, 0.164706, 0.141176, 0.117647, 0.094117, 0.070588, 0.047059, 0.023529, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    };

    const float Paw2GlView::cmap_rb1_blue_[256] = {
      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.023529, 0.047059, 0.070588, 0.094118, 0.117647, 0.141177, 0.164706, 0.188235, 0.211765, 0.235294,
      0.258824, 0.282353, 0.305883, 0.329412, 0.352941, 0.376471, 0.400000, 0.423530, 0.447059, 0.470588, 0.494118, 0.517647, 0.541177, 0.564706, 0.588235, 0.611765, 0.635294, 0.658824, 0.682353, 0.705882, 0.729412, 0.752941, 0.776471, 0.800000, 0.823529, 0.847059, 0.870588, 0.894118, 0.917647, 0.941176, 0.964706, 0.988235,
      1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
      1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000,
      1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.988236, 0.964706, 0.941176, 0.917647, 0.894118, 0.870588, 0.847059, 0.823530, 0.800000, 0.776470, 0.752941,
      0.729412, 0.705882, 0.682353, 0.658824, 0.635294, 0.611765, 0.588235, 0.564706, 0.541176, 0.517647, 0.494118, 0.470588, 0.447059, 0.423530, 0.400000, 0.376470, 0.352941, 0.329412, 0.305882, 0.282353, 0.258824, 0.235294, 0.211765, 0.188235, 0.164706, 0.141176, 0.117647, 0.094118, 0.070588, 0.047059, 0.023530, 0.000000,
    };

    const float Paw2GlView::cmap_rb2_red_[256] = {
      0.0104, 0.0208, 0.0313, 0.0417, 0.0521, 0.0625, 0.0729, 0.0833, 0.0938, 0.1042, 0.1146, 0.125, 0.1354, 0.1458, 0.1563, 0.1667, 0.1771, 0.1875, 0.1979, 0.2083, 0.2188, 0.2292, 0.2396, 0.25, 0.2604, 0.2708, 0.2813, 0.2917, 0.3021, 0.3125, 0.3229, 0.3333, 0.3438, 0.3542, 0.3646, 0.375, 0.3854, 0.3958, 0.4063, 0.4167, 0.4271, 0.4375, 0.4479, 0.4583, 0.4688, 0.4792, 0.4896, 0.5, 0.5104, 0.5208, 0.5313, 0.5417, 0.5521, 0.5625, 0.5729, 0.5833, 0.5938, 0.6042, 0.6146, 0.625, 0.6354, 0.6458, 0.6563, 0.6667, 0.6771, 0.6875, 0.6979, 0.7083, 0.7188, 0.7292, 0.7396, 0.75, 0.7604, 0.7708, 0.7813, 0.7917, 0.8021, 0.8125, 0.8229, 0.8333, 0.8438, 0.8542, 0.8646, 0.875, 0.8854, 0.8958, 0.9063, 0.9167, 0.9271, 0.9375, 0.9479, 0.9583, 0.9688, 0.9792, 0.9896, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };

    const float Paw2GlView::cmap_rb2_green_[256] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0104, 0.0208, 0.0313, 0.0417, 0.0521, 0.0625, 0.0729, 0.0833, 0.0938, 0.1042, 0.1146, 0.125, 0.1354, 0.1458, 0.1563, 0.1667, 0.1771, 0.1875, 0.1979, 0.2083, 0.2188, 0.2292, 0.2396, 0.25, 0.2604, 0.2708, 0.2813, 0.2917, 0.3021, 0.3125, 0.3229, 0.3333, 0.3438, 0.3542, 0.3646, 0.375, 0.3854, 0.3958, 0.4063, 0.4167, 0.4271, 0.4375, 0.4479, 0.4583, 0.4688, 0.4792, 0.4896, 0.5, 0.5104, 0.5208, 0.5313, 0.5417, 0.5521, 0.5625, 0.5729, 0.5833, 0.5938, 0.6042, 0.6146, 0.625, 0.6354, 0.6458, 0.6563, 0.6667, 0.6771, 0.6875, 0.6979, 0.7083, 0.7188, 0.7292, 0.7396, 0.75, 0.7604, 0.7708, 0.7813, 0.7917, 0.8021, 0.8125, 0.8229, 0.8333, 0.8438, 0.8542, 0.8646, 0.875, 0.8854, 0.8958, 0.9063, 0.9167, 0.9271, 0.9375, 0.9479, 0.9583, 0.9688, 0.9792, 0.9896, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };

    const float Paw2GlView::cmap_rb2_blue_[256] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0156, 0.0313, 0.0469, 0.0625, 0.0781, 0.0938, 0.1094, 0.125, 0.1406, 0.1563, 0.1719, 0.1875, 0.2031, 0.2188, 0.2344, 0.25, 0.2656, 0.2813, 0.2969, 0.3125, 0.3281, 0.3438, 0.3594, 0.375, 0.3906, 0.4063, 0.4219, 0.4375, 0.4531, 0.4688, 0.4844, 0.5, 0.5156, 0.5313, 0.5469, 0.5625, 0.5781, 0.5938, 0.6094, 0.625, 0.6406, 0.6563, 0.6719, 0.6875, 0.7031, 0.7188, 0.7344, 0.75, 0.7656, 0.7813, 0.7969, 0.8125, 0.8281, 0.8438, 0.8594, 0.875, 0.8906, 0.9063, 0.9219, 0.9375, 0.9531, 0.9688, 0.9844, 1
    };

  } // namespace vlr
