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


#ifndef CONTROLLER_MPC_H
#define CONTROLLER_MPC_H

#include <Eigen/Core>
#include <ros/ros.h>
#include <global.h>
#include <applanix/ApplanixPose.h>
#include <controller/ControllerTarget.h>
#include <driving_common/Actuator.h>
#include <driving_common/CanStatus.h>
#include <driving_common/Trajectory2D.h>
#include <passat_constants.h>
#include <vehicle.h>
//#include <planner_messages.h>
//#include <can_messages.h>
//#include <poly_traj.h>


#define NUM_STATES 6
#define NUM_CONTROLS 2

namespace vlr {

class MPCController {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MPCController();
  ~MPCController();

  void run();
  void controlLoop();
  void changeParams();

private:
  template <class T> void getParam(std::string key, T& var);

  void applanixHandler(const applanix::ApplanixPose& applanix_pose);
  void canStatusHandler(const driving_common::CanStatus& can_status);
  void trajectoryHandler(const driving_common::Trajectory2D& trajectory);

  void getState();
  void getDesiredStates();
  void getThrottleTorque();

  void mpcLQR(const Eigen::Matrix<double, NUM_STATES, 1> &s0, const Eigen::Matrix<double, NUM_CONTROLS, Eigen::Dynamic> &u0, const Eigen::Matrix<double,
      NUM_STATES, Eigen::Dynamic> &s_star, const Eigen::Matrix<double, NUM_CONTROLS, 1> &u_prev, Eigen::Matrix<double, NUM_CONTROLS, Eigen::Dynamic> *u_out);

  void laneFollow(const Eigen::Matrix<double, NUM_STATES, 1> &s0, const Eigen::Matrix<double, NUM_STATES, 1> &s_star,
      Eigen::Matrix<double, NUM_CONTROLS, 1> *u_out);

  void getThrottleTorqueOriginal();

  void dynamics(const Eigen::Matrix<double, NUM_STATES, 1> &s, const Eigen::Matrix<double, NUM_CONTROLS, 1> &u, Eigen::Matrix<double, NUM_STATES, 1> *s_dot,
      Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A = 0, Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B = 0);

  void simulateEuler(const Eigen::Matrix<double, NUM_STATES, 1> &s, const Eigen::Matrix<double, NUM_CONTROLS, 1> &u,
      Eigen::Matrix<double, NUM_STATES, 1> *s_next, Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A = 0, Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B = 0);

  void simulateRK4(const Eigen::Matrix<double, NUM_STATES, 1> &s, const Eigen::Matrix<double, NUM_CONTROLS, 1> &u,
      Eigen::Matrix<double, NUM_STATES, 1> *s_next, Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A = 0, Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B = 0);

  double simulatorThrottle(double u, double v, double th_dot, double u_d, double del);

private:
  ros::NodeHandle nh_;
  ros::Subscriber applanix_sub_, can_status_sub_, trajectory_sub_;
  ros::Publisher controller_target_pub_, actuator_pub_;

  applanix::ApplanixPose applanix_;
  driving_common::CanStatus can_;
  driving_common::Trajectory2D traj_;
  driving_common::Actuator actuator_;
  controller::ControllerTarget controller_target_;

  bool received_applanix_pose_;
  bool received_can_status_;
  bool received_trajectory_;

  bool run_controller_;

  Eigen::Matrix<double, NUM_STATES, 1> state_;
  Eigen::Matrix<double, NUM_CONTROLS, Eigen::Dynamic> controls_;
  Eigen::Matrix<double, NUM_STATES, Eigen::Dynamic> des_states_;
  Eigen::Vector2d errors_;

  Eigen::Matrix<double, NUM_CONTROLS, 1> throttle_torque_;
  Eigen::Matrix<double, NUM_CONTROLS, 1> ctl_err_;
  Eigen::Matrix<double, NUM_CONTROLS, 1> ctl_err_vel_;
  Eigen::Matrix<double, NUM_CONTROLS, 1> ctl_err_int_;

  //MSKenv_t m_mosek_env;

  double vel_err_int_;
  double last_dtheta_;

  // parameters
  vehicle_state p_vs;
  bool p_torque_mode;
  int p_horizon;
  double p_hertz;

  Eigen::Matrix<double, NUM_STATES, NUM_STATES> p_Q;
  Eigen::Matrix<double, NUM_CONTROLS, NUM_CONTROLS> p_R;
  Eigen::Matrix<double, NUM_CONTROLS, NUM_CONTROLS> p_R_delta;

  double p_q_lon, p_q_lat, p_q_theta, p_q_u, p_q_v, p_q_theta_dot;
  double p_r_udot, p_r_delta, p_rd_udot, p_rd_delta;

  double p_vel_smooth, p_int_decay;
  double p_k_throttle, p_d_throttle, p_i_throttle, p_ff_throttle;
  double p_k_torque, p_d_torque, p_i_torque, p_ff_torque;
  double p_throttle_smooth, p_torque_smooth;

  double p_p_cte, p_d_cte, p_k_yawrate, p_k_aggressive;
  double p_p_lon_err;

  double p_max_vel_int, p_k_cruise_int, p_k_accel, p_k_decel;
  double p_kp_torque, p_kd_torque;
};

} // namespace vlr
#endif  
