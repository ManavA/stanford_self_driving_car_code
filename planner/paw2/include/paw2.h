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


#ifndef PAW2_H_
#define PAW2_H_

#include <vector>
#include <boost/signals.hpp>

#include <global.h>
#include <transform.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sound_play/SoundRequest.h>
#include <driving_common/Trajectory2D.h>
#include <driving_common/TrafficLightStateRequest.h>
#include <driving_common/TurnSignal.h>
#include <driving_common/HCIString.h>
#include <driving_common/EStopRequest.h>

#include <driving_common/EStopStatus.h>
#include <driving_common/TrafficLightStates.h>
#include <passat/PassatState.h>
#include <applanix/ApplanixPose.h>
#include <localize/LocalizePose.h>
#include <perception/PerceptionObstacles.h>

#include <lltransform.h>

#include <aw_ChsmPlanner.hpp>

#include <circle_demo.h>
#include <static_map_demo.h>

namespace vlr {

#define PAW2_PUBLISH_CYCLE_TIME .2

class AWRoadPlanner {
public:
  AWRoadPlanner(int argc, char** argv);
  ~AWRoadPlanner();


  ChsmPlanner* chsm_planner_;

  int last_estop_state_;

  std::string rndf_filename_;
  std::string mdf_filename_;
  double planner_hz_;

  // robot pose
  double lat_, lon_;
  dgc::dgc_pose_t pose_accuracy_;
  std::vector<dgc::dgc_pose_t> pose_history_;

  pthread_mutex_t estop_mutex_;
  pthread_mutex_t traffic_light_mutex_;
  pthread_mutex_t vehicle_state_mutex_;

  bool show_gui_;
  bool received_applanix_pose_;
  bool received_localize_pose_;
  bool received_estop_;
  bool received_vehicle_state_;
  bool data_ready_to_publish_;

  double static_obstacle_map_size_x_;
  double static_obstacle_map_size_y_;
  double static_obstacle_map_resolution_;

  bool run_planner_cycle_;
  bool wait_for_vehicle_;

      // demo related stuff
  bool demo_mode_;
  driving_common::TrajectoryPoint2D demo_start_point_;
  bool show_static_map_demo_;
  bool show_fourway_stop1_demo_;
  bool show_circle_demo_;

  BaseDemo* demo_;

  static const double circle_demo_r_;
  static const double circle_demo_start_lat_;
  static const double circle_demo_start_lon_;
  static const std::string circle_demo_rndf_filename_;
  static const std::string circle_demo_mdf_filename_;

  static const double static_map_demo_start_lat_;
  static const double static_map_demo_start_lon_;
  static const double static_map_demo_start_yaw_;
  static const std::string static_map_demo_rndf_filename_;
  static const std::string static_map_demo_mdf_filename_;
  static const std::string static_map_demo_map_name_;

public: //needed for demo(s)
  void applanixHandler(const applanix::ApplanixPose& applanix_pose);
  void localizePoseHandler(const localize::LocalizePose& localize_pose);

public:
  void start();
  void run();
  void publish();

  inline void sendEStopRun() {estop_run_requested_=true;}
  inline void sendEStopPause() {estop_pause_requested_=true;};
  inline void quitPlanner() {
    pthread_mutex_lock(&quit_planner_mutex_);
    quit_planner_ = true;
    pthread_cond_signal(&quit_planner_cv_);
    pthread_mutex_unlock(&quit_planner_mutex_);
  }

  void* guiThread();
  void* plannerThread();

private:
//  void initializePose();
  void updatePose(const applanix::ApplanixPose& applanix_pose, const localize::LocalizePose& localize_pose);
  void updateRoadMap(int32_t width, int32_t height, double res);

  void readParameters();
  void estopHandler(const driving_common::EStopStatus& estop_status);
  void perceptionHandler(const perception::PerceptionObstacles& obstacles);
  void trafficLightHandler(const driving_common::TrafficLightStates& tf_states);
  void vehicleStateHandler(const passat::PassatState& vehicle_state);

  void publishTrajectory(const std::vector<driving_common::TrajectoryPoint2D>& trajectory_points);
  void publishTrafficLightRequest();
  void publishEstopRequest();
  void publishTurnSignalState();
  void publishEmergencyMessage(const std::string& text);

  void obstacleMapInitialize(int x_size, int y_size, double resolution);
  void clearObstacleMap();
  void waitForVehicle();

  template <class T> void getParam(std::string key, T& var);
  void getParamTransform(std::string key, dgc::dgc_transform_t& t);


private:
  int argc_;
  char** argv_;
  bool estop_run_requested_;
  bool estop_pause_requested_;

  bool caught_exception_;
  std::string caught_exception_text_;

  pthread_attr_t def_thread_attr_;

  pthread_t planner_thread_id_;
  pthread_t spin_thread_id_;
  pthread_t gui_thread_id_;

  bool quit_planner_;
  pthread_mutex_t quit_planner_mutex_;
  pthread_cond_t quit_planner_cv_;

  pthread_mutex_t emergency_message_sent_mutex_;
  pthread_cond_t emergency_message_sent_cv_;

  pthread_mutex_t received_vehicle_state_mutex_;
  pthread_cond_t received_vehicle_state_cv_;
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  ros::Publisher trajectory_pub_, tf_request_pub_, hci_pub_;
  ros::Publisher turnsignal_pub_, estop_request_pub_;
  ros::Subscriber applanix_sub_, localize_sub_;
  ros::Subscriber perception_sub_, estop_status_sub_;
  ros::Subscriber vehicle_state_sub_,tf_states_sub_;
//  sensor_msgs::Image left_img_, right_img_;

    // subscribed messages
  applanix::ApplanixPose applanix_pose_msg_;
  localize::LocalizePose localize_pose_msg_;
  perception::PerceptionObstacles perception_msg_;
  driving_common::EStopStatus estop_status_msg_;
  passat::PassatState vehicle_state_msg_;
  driving_common::TrafficLightStates tf_states_msg_;


    // published messages
  driving_common::TurnSignal turn_signal_msg_;
  driving_common::Trajectory2D trajectory_msg_;
  driving_common::EStopRequest estop_request_msg_;
  driving_common::TrafficLightStateRequest tf_request_msg_;
  sound_play::SoundRequest hci_msg_;
};

} // namespace vlr
#endif

