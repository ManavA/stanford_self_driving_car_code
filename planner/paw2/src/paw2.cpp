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

#include <cmath>

#include <passat_fsm.h>

#include <paw2.h>

#include <lltransform.h>

// for gui only
#include <GL/glut.h>
#include <passatmodel.h>
#include <scaledTime.h>
#include <obstacle_types.h>

#include <bufferedStringBuf.h>
#include <paw2App.h>
#include <paw2_gui.h>
#include <paw2InternalData.h>

using namespace dgc;

namespace drc = driving_common;

vlr::AWRoadPlanner* awp = NULL;
vlr::Paw2Gui* gui = NULL;
vlr::Paw2App* qtapp = NULL;

namespace vlr {

// member initialization
const double AWRoadPlanner::circle_demo_r_ = 35;
const double AWRoadPlanner::circle_demo_start_lat_ = 37.4275144;
const double AWRoadPlanner::circle_demo_start_lon_ = -122.0769586;
const std::string AWRoadPlanner::circle_demo_rndf_filename_ = "./circle_demo_rndf.txt";
const std::string AWRoadPlanner::circle_demo_mdf_filename_ = "./circle_demo_mdf.txt";

const double AWRoadPlanner::static_map_demo_start_lat_ = 37.4305720025051; // start position
const double AWRoadPlanner::static_map_demo_start_lon_ = -122.1830539998572;
const double AWRoadPlanner::static_map_demo_start_yaw_ = 3;
const std::string AWRoadPlanner::static_map_demo_rndf_filename_ = "./static_map_demo_rndf.txt";
const std::string AWRoadPlanner::static_map_demo_mdf_filename_ = "./static_map_demo_mdf.txt";
const std::string AWRoadPlanner::static_map_demo_map_name_ = "./testmap.png";

AWRoadPlanner::AWRoadPlanner(int argc, char **argv) :
  last_estop_state_(driving_common::EStopStatus::ESTOP_DISABLE), show_gui_(true), received_applanix_pose_(false), received_localize_pose_(false),
      received_estop_(false), received_vehicle_state_(false), data_ready_to_publish_(false), run_planner_cycle_(true), wait_for_vehicle_(true), demo_mode_(
          false), show_static_map_demo_(false), show_fourway_stop1_demo_(false), show_circle_demo_(false), demo_(NULL), argc_(argc), argv_(argv),
      estop_run_requested_(false), estop_pause_requested_(false), caught_exception_(false), gui_thread_id_(0), quit_planner_(false), nh_("/driving") {

  pthread_attr_init(&def_thread_attr_);
  pthread_attr_setdetachstate(&def_thread_attr_, PTHREAD_CREATE_JOINABLE);

  pthread_mutex_init(&estop_mutex_, NULL);
  pthread_mutex_init(&traffic_light_mutex_, NULL);
  pthread_mutex_init(&vehicle_state_mutex_, NULL);

  pthread_mutex_init(&quit_planner_mutex_, NULL);
  pthread_cond_init(&quit_planner_cv_, NULL);

  pthread_mutex_init(&emergency_message_sent_mutex_, NULL);
  pthread_cond_init(&emergency_message_sent_cv_, NULL);

  pthread_mutex_init(&received_vehicle_state_mutex_, NULL);
  pthread_cond_init(&received_vehicle_state_cv_, NULL);

  memset(&pose_accuracy_, 0, sizeof(pose_accuracy_));

  // initialize estop command message
  estop_request_msg_.estop_code = driving_common::EStopRequest::ESTOP_DISABLE;

  try {
    // read parameters for on-road and off-road driving
    readParameters();

    if (argc > 1) {
      for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "nogui") == 0) {
          show_gui_ = false;
        }
        else if (strcmp(argv[i], "nowaitforcar") == 0) {
          wait_for_vehicle_ = false;
        }
        else if (strcmp(argv[i], "circledemo") == 0) {
          demo_mode_ = true;
          show_circle_demo_ = true;
          rndf_filename_ = circle_demo_rndf_filename_;
          mdf_filename_ = circle_demo_mdf_filename_;
          demo_ = new CircleDemo(rndf_filename_, mdf_filename_, circle_demo_start_lat_, circle_demo_start_lon_, circle_demo_r_);
        }
        else if (strcmp(argv[i], "staticdemo") == 0) {
          demo_mode_ = true;
          show_static_map_demo_ = true;
          rndf_filename_ = static_map_demo_rndf_filename_;
          mdf_filename_ = static_map_demo_mdf_filename_;
          demo_ = new StaticMapDemo(rndf_filename_, mdf_filename_, static_map_demo_map_name_, static_map_demo_start_lat_, static_map_demo_start_lon_,
              static_map_demo_start_yaw_);
        }
      }
    }

    // don't wait for vehicle to shift when we run a simulation
    if (demo_mode_) {
      wait_for_vehicle_ = false;
    }

    //initialize planner core
    chsm_planner_ = new ChsmPlanner(rndf_filename_, mdf_filename_, static_obstacle_map_size_x_, static_obstacle_map_size_y_, static_obstacle_map_resolution_);

    // set everything demo related with dependencies on planner here
    if (demo_mode_) {
      std::string zone;
      if (show_circle_demo_) {
        static_cast<CircleDemo*> (demo_)->setFakeTrackerParams(chsm_planner_->traj_eval_->params().checked_horizon,
            chsm_planner_->traj_eval_->params().time_sample_res);
        latLongToUtm(circle_demo_start_lat_, circle_demo_start_lon_, &demo_start_point_.x, &demo_start_point_.y, zone);
      }
      else if (show_static_map_demo_) {
        latLongToUtm(static_map_demo_start_lat_, static_map_demo_start_lon_, &demo_start_point_.x, &demo_start_point_.y, zone);
      }

      demo_start_point_.t = drc::Time::current();
      demo_->updatePoses(true, demo_start_point_, applanix_pose_msg_, localize_pose_msg_);
      applanixHandler(applanix_pose_msg_);
      localizePoseHandler(localize_pose_msg_);
      demo_->updateObstaclePredictions(drc::Time::current());
      perceptionHandler(*const_cast<perception::PerceptionObstacles*> (&demo_->getObstacleMessage()));
    }

    // if we are in demo mode we do not want messages
    if (!demo_mode_) {
      trajectory_pub_ = nh_.advertise<driving_common::Trajectory2D> ("Trajectory2D", 5);
      tf_request_pub_ = nh_.advertise<driving_common::TrafficLightStateRequest> ("TrafficLightStateRequest", 5);
      hci_pub_ = nh_.advertise<sound_play::SoundRequest> ("/robotsound", 10);
      turnsignal_pub_ = nh_.advertise<driving_common::TurnSignal> ("TurnSignal", 10);
      estop_request_pub_ = nh_.advertise<driving_common::EStopRequest> ("EStopRequest", 10);

      // poses
      applanix_sub_ = nh_.subscribe("ApplanixPose", 5, &AWRoadPlanner::applanixHandler, this);
      localize_sub_ = nh_.subscribe("LocalizePose", 5, &AWRoadPlanner::localizePoseHandler, this);

      // perception output
      perception_sub_ = nh_.subscribe("PerceptionObstacles", 1, &AWRoadPlanner::perceptionHandler, this);

      // estop
      estop_status_sub_ = nh_.subscribe("EStopStatus", 5, &AWRoadPlanner::estopHandler, this);

      // passat fsm state (to see if car is in right gear on activation)
      vehicle_state_sub_ = nh_.subscribe("PassatState", 5, &AWRoadPlanner::vehicleStateHandler, this);

      // traffic lights
      tf_states_sub_ = nh_.subscribe("TrafficLightStates", 5, &AWRoadPlanner::trafficLightHandler, this);
    }

    if (show_gui_) {
      pthread_create(&gui_thread_id_, &def_thread_attr_, threadCBWrapper<AWRoadPlanner, &AWRoadPlanner::guiThread> , this);
      while (!gui) {
        usleep(100000);
      }
      gui->registerMessageHandlers();
    }
  }
  catch (vlr::Ex<>& e) {
    caught_exception_ = true;
    caught_exception_text_ = e.what();
    std::cout << e.what() << std::endl;
    if (!demo_mode_) {
      pthread_mutex_lock(&emergency_message_sent_mutex_);
      pthread_cond_wait(&emergency_message_sent_cv_, &emergency_message_sent_mutex_);
      pthread_mutex_unlock(&emergency_message_sent_mutex_);
    }
    quitPlanner();
  }
}

AWRoadPlanner::~AWRoadPlanner() {

  // in case unhandled exception occured, notify other threads
  quitPlanner();

  // if we quit before gui was initialized properly we will get stuck in gui main loop
  while (gui_thread_id_ == 0 && show_gui_) {
    usleep(100000);
  }

  if (gui_thread_id_ > 0) {
    qtapp->quit();
    pthread_join(gui_thread_id_, NULL);
    std::cout << "GUI thread joined.\n";
  }

  if (planner_thread_id_ > 0) {
    run_planner_cycle_ = false;
    pthread_join(planner_thread_id_, NULL);
    std::cout << "Planner thread joined.\n";
  }

  if (chsm_planner_) {
    chsm_planner_->terminate(); // should not be necessary but doesn't hurt
    delete chsm_planner_;
  }

  pthread_attr_destroy(&def_thread_attr_);

  pthread_mutex_destroy(&estop_mutex_);
  pthread_mutex_destroy(&traffic_light_mutex_);
  pthread_mutex_destroy(&vehicle_state_mutex_);

  pthread_mutex_destroy(&quit_planner_mutex_);
  pthread_cond_destroy(&quit_planner_cv_);

  pthread_mutex_destroy(&emergency_message_sent_mutex_);
  pthread_cond_destroy(&emergency_message_sent_cv_);

  pthread_mutex_destroy(&received_vehicle_state_mutex_);
  pthread_cond_destroy(&received_vehicle_state_cv_);
}

void AWRoadPlanner::applanixHandler(const applanix::ApplanixPose& applanix_pose) {
  received_applanix_pose_ = true;
  //  printf("%s: %f, %f\n", __PRETTY_FUNCTION__, applanix_pose.latitude, applanix_pose.longitude);
  updatePose(applanix_pose, localize_pose_msg_);
}

void AWRoadPlanner::localizePoseHandler(const localize::LocalizePose& localize_pose) {
  received_localize_pose_ = true;
  //  printf("%s: %f, %f\n", __PRETTY_FUNCTION__, localize_pose.x_offset, localize_pose.y_offset);
  updatePose(applanix_pose_msg_, localize_pose);
}

void AWRoadPlanner::updateRoadMap(int32_t width, int32_t height, double res) {
  if (!gui) {
    return;
  }
  //  Pose pose;
  //  try {
  //    pose = chsm_planner_->currentPose();
  //  }
  //  catch(...) {
  //    return;
  //  }
  //  pthread_mutex_lock(&chsm_planner_->center_line_mutex_);
  //  gui->internal_data_->draw(chsm_planner_->center_line_, width, height, res, pose.utmX(), pose.utmY());
  //  pthread_mutex_unlock(&chsm_planner_->center_line_mutex_);
  //  if (gui->internal_data_->roadMap()) {
  //    chsm_planner_->setRoadMap(gui->internal_data_->roadMap()->data());
  //  }
}

void AWRoadPlanner::perceptionHandler(const perception::PerceptionObstacles& obstacles) {

  uint32_t num_peds=0, num_cars=0, num_bikes=0;
  for (uint32_t i = 0; i < obstacles.dynamic_obstacle.size(); i++) {
    if(obstacles.dynamic_obstacle[i].type == OBSTACLE_CAR) {num_cars++;}
    else if(obstacles.dynamic_obstacle[i].type == OBSTACLE_PEDESTRIAN) {num_peds++;}
    else if(obstacles.dynamic_obstacle[i].type == OBSTACLE_BICYCLIST) {num_bikes++;}
  }
  printf("Got %u dynamic obstacles (%u cars, %u peds, %u bikes) and %u statics\n", obstacles.dynamic_obstacle.size(), num_cars, num_peds, num_bikes, obstacles.static_point.size());

  if (!received_applanix_pose_ || !received_localize_pose_) {
    return;
  }
  if (obstacles.static_point.size() == 0 && obstacles.dynamic_obstacle.size() == 0) {
    std::cout << "Warning: empty perception message will be discarded.\n";
    return;
  }

  try {
    chsm_planner_->updateVehicles(obstacles.dynamic_obstacle, obstacles.timestamp);
    chsm_planner_->updateStaticObstacleMap(obstacles.static_point, obstacles.timestamp);
    if (gui) {
      updateRoadMap(chsm_planner_->obstacleMapWidth(), chsm_planner_->obstacleMapHeight(), chsm_planner_->obstacleMapResolution());
    }
  }
  catch (vlr::Ex<>& e) {
    if (!demo_mode_) {
      pthread_mutex_lock(&emergency_message_sent_mutex_);
      pthread_cond_wait(&emergency_message_sent_cv_, &emergency_message_sent_mutex_);
      pthread_mutex_unlock(&emergency_message_sent_mutex_);
    }
    quitPlanner();
  }

  if (gui) {
    gui->requestPlannerGlRedraw();
  }
}

void AWRoadPlanner::trafficLightHandler(const driving_common::TrafficLightStates& tls) {
  chsm_planner_->updateTrafficLightStates(tls);
}

void AWRoadPlanner::estopHandler(const driving_common::EStopStatus& estop_status) {
  pthread_mutex_lock(&estop_mutex_);
  received_estop_ = true;
  estop_status_msg_ = estop_status;
  std::string estop_cmd;
  if (estop_status_msg_.estop_code == driving_common::EStopStatus::ESTOP_RUN) {
    estop_cmd = "ESTOP_RUN";
  }
  else if (estop_status_msg_.estop_code == driving_common::EStopStatus::ESTOP_PAUSE) {
    estop_cmd = "ESTOP_PAUSE";
  }
  else if (estop_status_msg_.estop_code == driving_common::EStopStatus::ESTOP_DISABLE) {
    estop_cmd = "ESTOP_DISABLE";
  }

  //  printf("received estop command %s\n", estop_cmd.c_str());
  pthread_mutex_unlock(&estop_mutex_);
}

void AWRoadPlanner::vehicleStateHandler(const passat::PassatState& vehicle_state) {
  pthread_mutex_lock(&vehicle_state_mutex_);
  received_vehicle_state_ = true;
  //  pthread_mutex_lock(&received_vehicle_state_mutex_);
  //  pthread_cond_signal(&received_vehicle_state_cv_);
  //  pthread_mutex_unlock(&received_vehicle_state_mutex_);
  pthread_mutex_unlock(&vehicle_state_mutex_);
}

void AWRoadPlanner::run() {
  try {
    double t1, t2, extra;
    bool init = true; // for demos

    if (!chsm_planner_->poseAvailable()) {
      std::cout << "Waiting for first Applanix and localize message...";
      while (!chsm_planner_->poseAvailable()) {
        if (!ros::ok()) {
          quitPlanner();
          return;
        }
        if (!run_planner_cycle_) {
          return;
        }
        if (gui_thread_id_ != 0 && show_gui_) {
          qtapp->updateStatus(gui);
        }
        usleep(100000);
      }
      std::cout << "ok..position received.\n";
    }

    chsm_planner_->start();

    printf("CHSM planner started...\n");

    // awp loop
    while (run_planner_cycle_) {

      if (!ros::ok()) {
        run_planner_cycle_ = false;
      }

      //    std::cout << "pose is " << pose_.x << ", " << pose_.y << " / " << lat_ << ", " << lon_ << " (yaw: " << pose_.yaw << ")\n";
      if (demo_mode_) {
        driving_common::TrajectoryPoint2D current_trajectory_point;
        init = false;
        try {
          chsm_planner_->traj_eval_->getCurrentTrajectoryPoint(drc::Time::current(), current_trajectory_point);
        }
        catch (vlr::Ex<>& e) {
          //        std::cout << e.what() << "\n";
          demo_start_point_.t = drc::Time::current();
          current_trajectory_point = demo_start_point_;
          init = true;
        }

        demo_->updatePoses(init, current_trajectory_point, applanix_pose_msg_, localize_pose_msg_);

        applanixHandler(applanix_pose_msg_);
        localizePoseHandler(localize_pose_msg_);

        if (show_circle_demo_) {
          if (!init) {
            pthread_mutex_lock(&chsm_planner_->dyn_obstacles_mutex_);
            demo_->updateObstaclePredictions(current_trajectory_point.t);
            pthread_mutex_unlock(&chsm_planner_->dyn_obstacles_mutex_);

            perceptionHandler(*const_cast<perception::PerceptionObstacles*> (&demo_->getObstacleMessage()));

            pthread_mutex_unlock(&chsm_planner_->topology_mutex_);
            pthread_mutex_unlock(&chsm_planner_->dyn_obstacles_mutex_);
          }
        }
        else if (show_static_map_demo_) {
          pthread_mutex_lock(&chsm_planner_->dyn_obstacles_mutex_);
          demo_->updateObstaclePredictions(current_trajectory_point.t);
          pthread_mutex_unlock(&chsm_planner_->dyn_obstacles_mutex_);
          perceptionHandler(*const_cast<perception::PerceptionObstacles*> (&demo_->getObstacleMessage()));
        }
      }

      t1 = drc::Time::current();

      // copy estop data
      //todo: is it allowed to call activate() if there was no actual state change?
      pthread_mutex_lock(&estop_mutex_);
      if (received_estop_) {
        if (estop_status_msg_.estop_code == driving_common::EStopStatus::ESTOP_RUN && last_estop_state_ != driving_common::EStopStatus::ESTOP_RUN) {
          if (wait_for_vehicle_ && !demo_mode_) {
            pthread_mutex_unlock(&estop_mutex_);
            waitForVehicle();
            pthread_mutex_lock(&estop_mutex_);
          }
          chsm_planner_->activate();
        }
        else if (estop_status_msg_.estop_code == driving_common::EStopStatus::ESTOP_PAUSE && last_estop_state_ != driving_common::EStopStatus::ESTOP_PAUSE) {
          chsm_planner_->pause();
        }
        last_estop_state_ = estop_status_msg_.estop_code;
      }
      pthread_mutex_unlock(&estop_mutex_);

      if (demo_mode_ && estop_run_requested_) {
        chsm_planner_->activate();
        estop_run_requested_ = false;
      }
      else if (demo_mode_ && estop_pause_requested_) {
        chsm_planner_->pause();
        estop_pause_requested_ = false;
      }

      // update road map in case perception is not running...
      if (gui) {
        updateRoadMap(chsm_planner_->obstacleMapWidth(), chsm_planner_->obstacleMapHeight(), chsm_planner_->obstacleMapResolution());
      }

      if (gui_thread_id_ != 0 && show_gui_) {
        qtapp->updateStatus(gui);
      }

      // do one planner iteration
      pthread_mutex_lock(&chsm_planner_->dyn_obstacles_mutex_);
      pthread_mutex_lock(&chsm_planner_->topology_mutex_);
      chsm_planner_->process();
      pthread_mutex_unlock(&chsm_planner_->topology_mutex_);
      pthread_mutex_unlock(&chsm_planner_->dyn_obstacles_mutex_);

      if (!demo_mode_) {
        data_ready_to_publish_ = true;
      }

      if (chsm_planner_->emergencyStopInitiated()) {
        if (!demo_mode_) {
          pthread_mutex_lock(&emergency_message_sent_mutex_);
          pthread_cond_wait(&emergency_message_sent_cv_, &emergency_message_sent_mutex_);
          pthread_mutex_unlock(&emergency_message_sent_mutex_);
        }
        //        quitPlanner();
        //        break;
      }

      // ... and loop
      t2 = drc::Time::current();
      extra = 1.0 / planner_hz_ - (t2 - t1);
      if (extra > 0) {
        usleep((int) rint(extra * 1e6));
      }
      else if (extra < 0) {
        printf("TIMING ERROR - SYSTEM TOO SLOW: planner cycle time: %f\n", t2 - t1);
      }
    }
  }
  catch (vlr::Ex<>& e) {
    caught_exception_ = true;
    caught_exception_text_ = e.what();
    data_ready_to_publish_ = true;
    std::cout << e.what() << std::endl;
    if (!demo_mode_) {
      pthread_mutex_lock(&emergency_message_sent_mutex_);
      pthread_cond_wait(&emergency_message_sent_cv_, &emergency_message_sent_mutex_);
      pthread_mutex_unlock(&emergency_message_sent_mutex_);
    }
    std::cout << "Leaving main planner loop after emergency.\n";
    quitPlanner();
  }

  try {
    chsm_planner_->terminate();
  }
  catch (...) {
    // nothing left to do, just exit regularly
  }

  std::cout << "Leaving main planner loop.\n";

  quitPlanner(); // will signal other threads
}

void AWRoadPlanner::publishEstopRequest() {
  if (estop_run_requested_) {
    estop_run_requested_ = false;
    estop_request_msg_.estop_code = driving_common::EStopRequest::ESTOP_RUN;
  }
  else if (estop_pause_requested_) {
    estop_pause_requested_ = false;
    estop_request_msg_.estop_code = driving_common::EStopRequest::ESTOP_PAUSE;
  }
  else {
    return;
  }

  estop_request_pub_.publish(estop_request_msg_);

  //  if (err != IPC_OK) {
  //    if (err == IPC_Timeout) {
  //      printf("IPC Timeout occurred. Could not publish %s (error code %i)\n", EstopSoftstopID.name, IPC_errno);
  //    }
  //    else if (IPC_errno != IPC_No_Error) {
  //      printf("IPC ERROR occurred. Could not publish %s (error code %i)\n", EstopSoftstopID.name, IPC_errno);
  //    }
  //  }
}

void AWRoadPlanner::publishTurnSignalState() {
  // TODO: Mutex required for copying ?!?
  // pthread_mutex_lock(&chsm_planner_->turn_signal_mutex_);
  turn_signal_msg_.signal = chsm_planner_->turn_signal_.signal;
  // pthread_mutex_lock(&chsm_planner_->turn_signal_mutex_);
  turn_signal_msg_.timestamp = drc::Time::current();
  turnsignal_pub_.publish(turn_signal_msg_);
}

void AWRoadPlanner::publish() {

  publishEstopRequest();
  publishTurnSignalState();
  publishTrajectory(chsm_planner_->trajectory_points_);

  if (chsm_planner_->publish_traffic_lights_) {
    publishTrafficLightRequest();
  }

  if (chsm_planner_->emergencyStopInitiated()) {
    publishEmergencyMessage("Emergency stop initiated.");
    pthread_mutex_lock(&emergency_message_sent_mutex_);
    pthread_cond_signal(&emergency_message_sent_cv_);
    pthread_mutex_unlock(&emergency_message_sent_mutex_);
  }

  if (caught_exception_) {
    publishEmergencyMessage(caught_exception_text_);
    pthread_mutex_lock(&emergency_message_sent_mutex_);
    pthread_cond_signal(&emergency_message_sent_cv_);
    pthread_mutex_unlock(&emergency_message_sent_mutex_);
  }

  // publish status messages
}

void AWRoadPlanner::publishEmergencyMessage(const std::string& text) {
  hci_msg_.sound = sound_play::SoundRequest::SAY;
  hci_msg_.command = sound_play::SoundRequest::PLAY_ONCE;
  hci_msg_.arg = text;
  hci_pub_.publish(hci_msg_);
}

void AWRoadPlanner::publishTrajectory(const std::vector<driving_common::TrajectoryPoint2D>& trajectory_points) {

  if (trajectory_points.size() == 0) {
    return;
  }

  double offset_x, offset_y;
  chsm_planner_->currentLocalizeOffsets(offset_x, offset_y);

  pthread_mutex_lock(&chsm_planner_->trajectory_mutex_);

  trajectory_msg_.point.resize(trajectory_points.size());
  //  driving_common::TrajectoryPoint2D p;
  for (size_t i = 0; i < trajectory_points.size(); i++) {
    //    recalcForFrontAxle(trajectory_points[i], trajectory_.points[i]);
    //    p = trajectory_points[i];
    //    p.x -= offset_x;
    //    p.y -= offset_y;
    //    trajectory_msg_.point.push_back(p);
    trajectory_msg_.point[i].t = trajectory_points[i].t;
    trajectory_msg_.point[i].x = trajectory_points[i].x - offset_x;
    trajectory_msg_.point[i].y = trajectory_points[i].y - offset_y;
    trajectory_msg_.point[i].theta = trajectory_points[i].theta;
    trajectory_msg_.point[i].kappa = trajectory_points[i].kappa;
    trajectory_msg_.point[i].kappa_dot = trajectory_points[i].kappa_dot;
    trajectory_msg_.point[i].v = trajectory_points[i].v;
    trajectory_msg_.point[i].a = trajectory_points[i].a;
    trajectory_msg_.point[i].jerk = trajectory_points[i].jerk;
    trajectory_msg_.point[i].delta_theta = trajectory_points[i].delta_theta;
    trajectory_msg_.point[i].d = trajectory_points[i].d;
    trajectory_msg_.point[i].a_lat = trajectory_points[i].a_lat;
  }

  trajectory_pub_.publish(trajectory_msg_);
  pthread_mutex_unlock(&chsm_planner_->trajectory_mutex_);

  //  if(err != IPC_OK) {
  //    if(err == IPC_Timeout) {
  //      printf("IPC Timeout occurred. Could not publish %s (error code %i)\n", TrajectoryPoints2DID.name, IPC_errno);
  //    }
  //    else if(IPC_errno != IPC_No_Error) {
  //      printf("IPC ERROR occurred. Could not publish %s (error code %i)\n", TrajectoryPoints2DID.name, IPC_errno);
  //    }
  //  }
}

void AWRoadPlanner::publishTrafficLightRequest() {

  pthread_mutex_lock(&chsm_planner_->traffic_light_poses_mutex_);

  if (chsm_planner_->traffic_light_poses_.size() == 0) {
    std::cout << "Number of traffic lights to request is 0 (Cannot publish request).\n";
    pthread_mutex_unlock(&chsm_planner_->traffic_light_poses_mutex_);
    return;
  }

  for (size_t i = 0; i < chsm_planner_->traffic_light_poses_.size(); i++) {
    tf_request_msg_.light_pose[i].name = chsm_planner_->traffic_light_poses_[i].name;
    tf_request_msg_.light_pose[i].lat = chsm_planner_->traffic_light_poses_[i].lat;
    tf_request_msg_.light_pose[i].lon = chsm_planner_->traffic_light_poses_[i].lon;
    tf_request_msg_.light_pose[i].z = chsm_planner_->traffic_light_poses_[i].z;
    tf_request_msg_.light_pose[i].orientation = chsm_planner_->traffic_light_poses_[i].orientation;
    printf("Requesting state for traffic light %s\n", tf_request_msg_.light_pose[i].name.c_str());
  }

  pthread_mutex_unlock(&chsm_planner_->traffic_light_poses_mutex_);

  tf_request_pub_.publish(tf_request_msg_);

  //  if(err != IPC_OK) {
  //    if(err == IPC_Timeout) {
  //      printf("IPC Timeout occurred. Could not publish %s (error code %i)\n", TrafficLightPoseListMsgID.name, IPC_errno);
  //    }
  //    else if(IPC_errno != IPC_No_Error) {
  //      printf("IPC ERROR occurred. Could not publish %s (error code %i)\n", TrafficLightPoseListMsgID.name, IPC_errno);
  //    }
  //  }
}

void AWRoadPlanner::updatePose(const applanix::ApplanixPose& applanix_pose, const localize::LocalizePose& localize_pose) {
  // TODO: associate localize offset with correct applanix pose
  pthread_mutex_lock(&chsm_planner_->pose_mutex_);

  if (&localize_pose != &localize_pose_msg_) {
    localize_pose_msg_ = localize_pose;
  }

  if (!received_applanix_pose_) {
    pthread_mutex_unlock(&chsm_planner_->pose_mutex_);
    return;
  } // localize pose alone doesn't help..and anyway..shouldn't occur

  if (&applanix_pose != &applanix_pose_msg_) {
    applanix_pose_msg_ = applanix_pose;
  }
  double offset_x = 0, offset_y = 0;

  if (received_localize_pose_) {
    //    printf("received xoff: %f, yoff: %f\n", localize_pose_msg_.x_offset, localize_pose_msg_.y_offset);
    offset_x = localize_pose_msg_.x_offset;
    offset_y = localize_pose_msg_.y_offset;
    utmToLatLong(applanix_pose_msg_.smooth_x + offset_x, applanix_pose_msg_.smooth_y + offset_y, localize_pose_msg_.utmzone, &lat_, &lon_);
  }
  else { // that's the best we can do without localize pose
    lat_ = applanix_pose_msg_.latitude;
    lon_ = applanix_pose_msg_.longitude;
    char utm_zone[4];
    latLongToUtm(lat_, lon_, &offset_x, &offset_y, utm_zone);
    offset_x -= applanix_pose_msg_.smooth_x;
    offset_y -= applanix_pose_msg_.smooth_y;
  }

  pthread_mutex_unlock(&chsm_planner_->pose_mutex_);
  // the localize offset should vary more slowly compared to pose data
  // therefore use applanix pose timestamp until proper interpolation is in place
  if (chsm_planner_) {
    chsm_planner_->updateRobot(applanix_pose_msg_.timestamp, applanix_pose_msg_.smooth_x, applanix_pose_msg_.smooth_y, offset_x, offset_y,
        applanix_pose_msg_.yaw, applanix_pose_msg_.speed, applanix_pose_msg_.accel_x, applanix_pose_msg_.accel_y, applanix_pose_msg_.accel_z,
        applanix_pose_msg_.rate_yaw);
  }

  if (gui) {
    gui->requestPlannerGlRedraw();
  }
}

void AWRoadPlanner::start() {
  pthread_create(&planner_thread_id_, &def_thread_attr_, threadCBWrapper<AWRoadPlanner, &AWRoadPlanner::plannerThread> , this);
  if (!demo_mode_) {
    ros::AsyncSpinner spinner(6); // one thread for each subscribed message..TODO: Proper queue assignment
    spinner.start();
    while (ros::ok() && !quit_planner_) {
      if (data_ready_to_publish_) {
        publish();
        data_ready_to_publish_ = false;
      }
      usleep(0.5 * 1000 * 1000 * PAW2_PUBLISH_CYCLE_TIME);
    }
  }
  else {
    pthread_mutex_lock(&quit_planner_mutex_);
    pthread_cond_wait(&quit_planner_cv_, &quit_planner_mutex_);
    pthread_mutex_unlock(&quit_planner_mutex_);
  }

}

template<class T> void AWRoadPlanner::getParam(std::string key, T& var) {
  if (!nh_.getParam(key, var)) {
    throw VLRException("annot read parameter " + key + std::string("."));
  }
}

void AWRoadPlanner::getParamTransform(std::string key, dgc::dgc_transform_t& tr) {
  ros::Time now = ros::Time::now();
  tf_listener_.waitForTransform("Applanix", key, now, ros::Duration(3.0));

  tf::StampedTransform transform;
  try {
    tf_listener_.lookupTransform("Applanix", key, ros::Time(0), transform);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }

  tr[3][0] = 0;
  tr[3][1] = 0;
  tr[3][2] = 0;
  tr[3][3] = 1;
  btMatrix3x3 R = transform.getBasis();
  for (int32_t r = 0; r < 3; r++) {
    for (int32_t c = 0; c < 3; c++) {
      tr[r][c] = R[r][c];
    }
  }
  btVector3 t = transform.getOrigin();
  tr[0][3] = t[0];
  tr[1][3] = t[1];
  tr[2][3] = t[2];
}

void AWRoadPlanner::readParameters() {
  double cycle_time;
  getParam("rndf_file", rndf_filename_);
  getParam("mdf_file", mdf_filename_);
  getParam("/driving/paw2/cycle_time", cycle_time);
  planner_hz_ = 1.0 / cycle_time;
  getParam("/driving/perception/map_size_x", static_obstacle_map_size_x_);
  getParam("/driving/perception/map_size_y", static_obstacle_map_size_y_);
  getParam("/driving/perception/map_resolution", static_obstacle_map_resolution_);
}

void AWRoadPlanner::waitForVehicle() {
  printf("%s\n", __FUNCTION__);
  if (!received_vehicle_state_) {
    usleep(10000);
  }
  //  pthread_mutex_lock(&received_vehicle_state_mutex_);
  //  pthread_cond_wait(&received_vehicle_state_cv_, &received_vehicle_state_mutex_);
  //  pthread_mutex_unlock(&received_vehicle_state_mutex_);

  printf("Waiting for passat to respond with FORWARD_GO...\n");
  //  pthread_mutex_lock(&vehicle_state_mutex_);
  int state = vehicle_state_msg_.fsm_state;
  //  pthread_mutex_unlock(&vehicle_state_mutex_);
  while (state != Passat::FORWARD_GO) {
    // pthread_mutex_lock(&vehicle_state_mutex_);
    state = vehicle_state_msg_.fsm_state;
    //  pthread_mutex_unlock(&vehicle_state_mutex_);
    switch (state) {
      case Passat::PAUSEFOR_STOP_VEHICLE:
        printf("Got state PAUSEFOR_STOP_VEHICLE\n");
        break;

      case Passat::PAUSEFOR_PRESHIFT_WAIT:
        printf("Got state PPAUSEFOR_PRESHIFT_WAIT\n");
        break;

      case Passat::PAUSEFOR_SHIFT_TO_PARK:
        printf("Got state PAUSEFOR_SHIFT_TO_PARK\n");
        break;

      case Passat::PAUSEFOR_WAIT_WITH_BRAKE:
        printf("Got state PAUSEFOR_WAIT_WITH_BRAKE\n");
        break;

      case Passat::PAUSEFOR_ENABLE_PARKING_BRAKE:
        printf("Got state PAUSEFOR_ENABLE_PARKING_BRAKE\n");
        break;

      case Passat::PAUSEFOR_WAIT:
        printf("Got state PAUSEFOR_WAIT\n");
        break;

      case Passat::PAUSEFOR_ENABLE_BRAKE:
        printf("Got state PAUSEFOR_ENABLE_BRAKE\n");
        break;

      case Passat::PAUSEFOR_DISABLE_PARKING_BRAKE:
        printf("Got state PAUSEFOR_DISABLE_PARKING_BRAKE\n");
        break;

      case Passat::PAUSEFOR_SHIFT_TO_DRIVE:
        printf("Got state PAUSEFOR_SHIFT_TO_DRIVE\n");
        break;

      case Passat::PAUSEFOR_WAIT_5SEC:
        printf("Got state PAUSEFOR_WAIT_5SEC\n");
        break;

      case Passat::FORWARD_STOP_VEHICLE:
        printf("Got state FORWARD_STOP_VEHICLE\n");
        break;

      case Passat::FORWARD_PRESHIFT_WAIT:
        printf("Got state FORWARD_PRESHIFT_WAIT\n");
        break;

      case Passat::FORWARD_SHIFT_TO_DRIVE:
        printf("Got state FORWARD_SHIFT_TO_DRIVE\n");
        break;

      case Passat::FORWARD_GO:
        printf("There we go :-D\n");
        break;
      default:
        printf("Got state %i\n", state);
    }
    usleep(100000);
  }
}

void* AWRoadPlanner::plannerThread() {
  run();
  return NULL;
}

void* AWRoadPlanner::guiThread() {
  try {
    qtapp = new Paw2App(argc_, argv_);
    qtapp->connect(qtapp, SIGNAL(lastWindowClosed()), qtapp, SLOT(quit()));
    //    BufferedStringBuf sbuf;
    //    std::cout.rdbuf(&sbuf);

    glutInit(&argc_, argv_);

    gui = new Paw2Gui(*qtapp, *this, nh_);

    gui->raise();
    gui->show();

    // ...and loop (QT)
    qtapp->exec();
  }
  catch (...) {
    quitPlanner();
  }
  return NULL;
}

} // namespace vlr

int main(int argc, char **argv) {

  drc::Time::scale(1);

  ros::init(argc, argv, "paw2");

  awp = new vlr::AWRoadPlanner(argc, argv);

  awp->start();

  delete awp;
  return 0;
}
