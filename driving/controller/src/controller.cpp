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


#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <global.h>
#include <vlrException.h>

#include <controller.h>

using namespace std;
using namespace Eigen;

ofstream f_data("data.dat");

vlr::MPCController* mpc=NULL;

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller");
  try {
    mpc = new vlr::MPCController;
  }
  catch(vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
  }

  try {
    mpc->run();
  }
  catch(vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
  }

  delete mpc;
  return 0;
}

namespace vlr {

MPCController::MPCController() : nh_("/driving"), received_applanix_pose_(false),
                                 received_can_status_(false), received_trajectory_(false),
                                 run_controller_(false) {

  getParam("controller/hz", p_hertz);
  getParam("controller/horizon", p_horizon);
  getParam("controller/torque_mode", p_torque_mode);
  getParam("controller/cost_lon", p_q_lon);
  getParam("controller/cost_lat", p_q_lat);
  getParam("controller/cost_theta", p_q_theta);
  getParam("controller/cost_vx", p_q_u);
  getParam("controller/cost_vy", p_q_v);
  getParam("controller/cost_theta_dot", p_q_theta_dot);
  getParam("controller/cost_accel", p_r_udot);
  getParam("controller/cost_steer", p_r_delta);
  getParam("controller/cost_delta_accel", p_rd_udot);
  getParam("controller/cost_delta_steer", p_rd_delta);
  getParam("controller/vel_smooth", p_vel_smooth);
  getParam("controller/int_decay", p_int_decay);
  getParam("controller/k_throttle", p_k_throttle);
  getParam("controller/d_throttle", p_d_throttle);
  getParam("controller/i_throttle", p_i_throttle);
  getParam("controller/ff_throttle", p_ff_throttle);
  getParam("controller/k_torque", p_k_torque);
  getParam("controller/d_torque", p_d_torque);
  getParam("controller/i_torque", p_i_torque);
  getParam("controller/ff_torque", p_ff_torque);
  getParam("controller/throttle_smooth", p_throttle_smooth);
  getParam("controller/torque_smooth", p_torque_smooth);

  getParam("controller/p_cte", p_p_cte);
  getParam("controller/d_cte", p_d_cte);
  getParam("controller/k_yawrate", p_k_yawrate);
  getParam("controller/k_aggressive", p_k_aggressive);
  getParam("controller/vel_int_cap", p_max_vel_int);
  getParam("controller/k_cruise_int", p_k_cruise_int);
  getParam("controller/k_accel", p_k_accel);
  getParam("controller/k_decel", p_k_decel);
  getParam("controller/kp_torque", p_kp_torque);
  getParam("controller/kd_torque", p_kd_torque);


  p_Q.setZero();
  p_R.setZero();
  p_R_delta.setZero();
  p_vs.set_passat_params();
  changeParams();
  des_states_.resize(NUM_STATES, p_horizon + 1);
  controls_.resize(NUM_CONTROLS, p_horizon);
  controls_.setZero();
  errors_.setZero();


  ctl_err_.setZero();
  ctl_err_vel_.setZero();
  ctl_err_int_.setZero();
  vel_err_int_ = 0;
  last_dtheta_ = 0;

  // initialize mosek
  //MSK_makeenv(&m_mosek_env, NULL, NULL, NULL, NULL);
  //MSK_linkfunctoenvstream(m_mosek_env, MSK_STREAM_LOG, NULL, printstr);
  //MSK_initenv(m_mosek_env);

  applanix_sub_   = nh_.subscribe("ApplanixPose", 5, &MPCController::applanixHandler, this);
  can_status_sub_ = nh_.subscribe("CanStatus", 5, &MPCController::canStatusHandler, this);
  trajectory_sub_ = nh_.subscribe("Trajectory2D", 5, &MPCController::trajectoryHandler, this);

  actuator_pub_  = nh_.advertise<driving_common::Actuator>("Actuator", 1);
  controller_target_pub_  = nh_.advertise<controller::ControllerTarget>("ControllerTarget", 1);
}

MPCController::~MPCController() {
}


void MPCController::run() {
  ros::Rate r(p_hertz);

  while (ros::ok()) {
    controlLoop();
    ros::spinOnce();
    r.sleep();
  }
}

  // parameter change callback
void MPCController::changeParams() {
  p_Q(0, 0) = p_q_lon;
  p_Q(1, 1) = p_q_lat;
  p_Q(2, 2) = p_q_theta;
  p_Q(3, 3) = p_q_u;
  p_Q(4, 4) = p_q_v;
  p_Q(5, 5) = p_q_theta_dot;

  p_R(0, 0) = p_r_udot;
  p_R(1, 1) = p_r_delta;
  p_R_delta(0, 0) = p_rd_udot;
  p_R_delta(1, 1) = p_rd_delta;
}

void MPCController::applanixHandler(const applanix::ApplanixPose& applanix_pose) {
  applanix_ = applanix_pose;
  received_applanix_pose_ = true;
}

void MPCController::canStatusHandler(const driving_common::CanStatus& can_status) {
  can_ = can_status;
  received_can_status_ = true;
}

void MPCController::trajectoryHandler(const driving_common::Trajectory2D& trajectory) {
  traj_ = trajectory;
  received_trajectory_ = true;
}

// main controller loop
void MPCController::controlLoop() {
  if (!received_applanix_pose_ || !received_can_status_ || !received_trajectory_) {return;}

    // if we haven't gotten anything for a while, reset
  if (traj_.point[0].t < applanix_.timestamp - 10.0) {
    controls_.setZero();
    throttle_torque_.setZero();
    ctl_err_.setZero();
    ctl_err_vel_.setZero();
    ctl_err_int_.setZero();
    vel_err_int_ = 0;
    last_dtheta_ = 0;
    received_applanix_pose_ = false;
    received_can_status_ = false;
    received_trajectory_ = false;
    cout << "No commands in a while, resetting controller..." << endl;
    return;
  }

  // compute desired control (delta, velocity)
  getState();
  getDesiredStates();
  //cout << errors_.transpose() << endl;

  // shift controls and run MPC
  Matrix<double, NUM_CONTROLS, 1> u_prev = controls_.col(0);
  Matrix<double, NUM_STATES, Dynamic> s_pred(NUM_STATES, p_horizon + 1);
  Matrix<double, NUM_STATES, 1> s;

  controls_.block(0, 0, NUM_CONTROLS, p_horizon - 1) = controls_.block(0, 1, NUM_CONTROLS, p_horizon - 1);

  for (int i = 0; i < 1; i++) {
    mpcLQR(state_, controls_, des_states_, u_prev, &controls_);
  }
  //laneFollow(state_, des_states_.col(0), &u_prev);
  //controls_.col(0) = u_prev;
  f_data << des_states_(0, 0) << " " << des_states_(0, 1) << " " << des_states_(0, 2) << " " << state_(0) << " " << state_(1) << " " << state_(2) << endl;

  if (!p_torque_mode) {
    double throttle = simulatorThrottle(state_(3, 0), state_(4, 0), state_(5, 0), controls_(0, 0), controls_(1, 0));
    double steering_angle = controls_(1, 0) * p_vs.param.steering_ratio;

    actuator_.steering_mode     = driving_common::Actuator::ANGLE_CONTROL;
    actuator_.steering_value    = steering_angle;
    actuator_.brake_pressure    = 30.0 * std::max(0.0, -throttle);
    actuator_.throttle_fraction = std::max(0.0, throttle);

    cout << throttle << " " << steering_angle << endl;

//    PassatActuatorAngleCommand(m_ipc, DGC_PASSAT_DIRECTION_FORWARD, steering_angle, 30.0 * max(0.0, -throttle), max(0.0, throttle));
  }
  else {
    getThrottleTorque();

    actuator_.steering_mode     = driving_common::Actuator::TORQUE_CONTROL;
    actuator_.steering_value    = throttle_torque_(1);
    actuator_.brake_pressure    = 30.0 * std::max(0.0, -throttle_torque_(0));
    actuator_.throttle_fraction = std::max(0.0, throttle_torque_(0));

    cout << "steering: " << controls_(1, 0) << " " << throttle_torque_(1) << endl;

//    PassatActuatorTorqueCommand(m_ipc, DGC_PASSAT_DIRECTION_FORWARD, throttle_torque_(1), 30.0 * max(0.0, -throttle_torque_(0)), max(0.0, throttle_torque_(0)));
  }

  actuator_.direction = driving_common::Actuator::DIRECTION_FORWARD;
  actuator_.timestamp = driving_common::Time::current();
  actuator_pub_.publish(actuator_);

    // publish errors
  controller_target_.target_velocity = des_states_(3, 0);
  controller_target_.target_steering_angle = controls_(1, 0) * p_vs.param.steering_ratio;
  controller_target_.cross_track_error = -sin(des_states_(2, 0)) * (state_(0) - des_states_(0, 0)) + cos(des_states_(2, 0)) * (state_(1) - des_states_(1, 0));
  controller_target_.heading_error = (state_(2) - des_states_(2, 0));
  controller_target_.timestamp = driving_common::Time::current();
  controller_target_pub_.publish(controller_target_);
}

// get current state of the car (really a delta state from desired state)
void MPCController::getState() {
  double imu_to_cg = DGC_PASSAT_IMU_TO_FA_DIST; //p_vs.param.imu_to_cg_dist;
  state_(0) = applanix_.smooth_x + cos(applanix_.yaw) * imu_to_cg;
  state_(1) = applanix_.smooth_y + sin(applanix_.yaw) * imu_to_cg;

  state_(2) = applanix_.yaw;
  state_(5) = applanix_.rate_yaw;

  double cos_th = cos(state_(2)), sin_th = sin(state_(2));
  double v_e = applanix_.vel_east - imu_to_cg * state_(5) * sin(state_(2));
  double v_n = applanix_.vel_north + imu_to_cg * state_(5) * cos(state_(2));

  state_(3) = cos_th * v_e + sin_th * v_n;
  state_(4) = -sin_th * v_e + cos_th * v_n;
}

// get desired states from trajecotry and controls using Newton's method
void MPCController::getDesiredStates() {
  double t = applanix_.timestamp, alpha;
  double ra_to_cg = DGC_PASSAT_WHEEL_BASE;
  int j = 0;

  // find all desired positions
//  driving_common::TrajectoryPoint2D *p1, *p2;
  double dt = 1.0 / p_hertz;
  for (int i = 0; i < p_horizon + 1; i++) {
    while (traj_.point[j + 1].t < t && j < (int)traj_.point.size() - 2)
      j++;
    driving_common::TrajectoryPoint2D& p1 = traj_.point[j];       // TODO: Check if copy is better than reference
    driving_common::TrajectoryPoint2D& p2 = traj_.point[j + 1];
    while (p2.theta - p1.theta > M_PI)
      p2.theta -= 2 * M_PI;
    while (p2.theta - p1.theta < -M_PI)
      p2.theta += 2 * M_PI;

    // integrate to create a smoothed trajectory
    alpha = (t - p1.t) / (p2.t - p1.t);
    if (alpha > 1) alpha = 1;
    if (alpha < 0) alpha = 0;

    des_states_(0, i) = (1 - alpha) * p1.x + alpha * p2.x;
    des_states_(1, i) = (1 - alpha) * p1.y + alpha * p2.y;
    des_states_(2, i) = (1 - alpha) * p1.theta + alpha * p2.theta;
    des_states_(0, i) += cos(des_states_(2, i)) * ra_to_cg;
    des_states_(1, i) += sin(des_states_(2, i)) * ra_to_cg;
    des_states_(3, i) = (1 - alpha) * p1.v + alpha * p2.v;
    des_states_(4, i) = 0.0;
    des_states_(5, i) = des_states_(3, i) * ((1 - alpha) * p1.kappa + alpha * p2.kappa);
    t += dt;
  }

  // normalize desired angles properly
  while (des_states_(2, 0) - state_(2) > M_PI)
    des_states_(2, 0) -= 2 * M_PI;
  while (des_states_(2, 0) - state_(2) < -M_PI)
    des_states_(2, 0) += 2 * M_PI;
  for (int i = 1; i < p_horizon + 1; i++) {
    while (des_states_(2, i) - des_states_(2, i - 1) > M_PI)
      des_states_(2, i) -= 2 * M_PI;
    while (des_states_(2, i) - des_states_(2, i - 1) < -M_PI)
      des_states_(2, i) += 2 * M_PI;
  }

  errors_ += 0.05 * (Rotation2D<double> (-state_(2)).toRotationMatrix() * (state_.block(0, 0, 2, 1) - des_states_.block(0, 0, 2, 1)) - errors_);
}

void MPCController::getThrottleTorque() {
  // integrate forward to see where velocity should be
  Matrix<double, NUM_STATES, 1> s;
  Matrix<double, NUM_CONTROLS, 1> err;

  simulateEuler(state_, controls_.col(0), &s);

  // compute errors, error derivatives (finite differences), a integrators
  err(0) = state_(3) - s(3);
  err(1) = (M_PI / 180) * can_.steering_angle / DGC_PASSAT_STEERING_RATIO - controls_(1, 0);
  ctl_err_vel_ = p_vel_smooth * ctl_err_vel_ + (1 - p_vel_smooth) * (err - ctl_err_) * p_hertz;
  ctl_err_int_ = ctl_err_int_ * p_int_decay + err;
  ctl_err_ = err;

  // apply control
  throttle_torque_(0) = p_throttle_smooth * throttle_torque_(0) + (1 - p_throttle_smooth) * (p_k_throttle * ctl_err_(0) + p_d_throttle * ctl_err_vel_(0)
      + p_i_throttle * ctl_err_int_(0) + p_ff_throttle * controls_(0, 0));

  throttle_torque_(1) = p_torque_smooth * throttle_torque_(1) + (1 - p_torque_smooth) * (p_k_torque * ctl_err_(1) + p_d_torque * ctl_err_vel_(1)
      + p_i_torque * ctl_err_int_(1) + p_ff_torque * controls_(1, 0));

}

// model predictive control using LQR for optimization
#define NUM_EXT_STATES (NUM_STATES+2*NUM_CONTROLS)

void MPCController::mpcLQR(const Matrix<double, NUM_STATES, 1> &s0, const Matrix<double, NUM_CONTROLS, Dynamic> &u0,
    const Matrix<double, NUM_STATES, Dynamic> &s_star, const Matrix<double, NUM_CONTROLS, 1> &u_prev, Matrix<double, NUM_CONTROLS, Dynamic> *u_out) {
  Matrix<double, NUM_STATES, NUM_STATES> A[p_horizon];
  Matrix<double, NUM_STATES, NUM_CONTROLS> B[p_horizon];
  Matrix<double, NUM_EXT_STATES, NUM_EXT_STATES> Ae, P, Q;
  Matrix<double, NUM_EXT_STATES, NUM_CONTROLS> Be;
  Matrix<double, NUM_CONTROLS, NUM_EXT_STATES> K[p_horizon];
  Matrix<double, NUM_CONTROLS, 1> g[p_horizon], u_opt;
  Matrix<double, NUM_STATES, Dynamic> s(NUM_STATES, p_horizon + 1);
  Matrix<double, NUM_EXT_STATES, 1> s_opt, q, ds;
  Matrix<double, NUM_STATES, 1> s_next;
  Matrix<double, NUM_CONTROLS, NUM_CONTROLS> Z;
  Matrix2d R;

  s.col(0) = s0;
  // initialize cost and dynamics matrices
  for (int i = 0; i < p_horizon; i++) {
    simulateRK4(s.col(i), u0.col(i), &s_next, &A[i], &B[i]);
    s.col(i + 1) = s_next;
  }

  // initialize extended dynamics matrices
  Ae.setZero();
  Ae.block(NUM_STATES + NUM_CONTROLS, NUM_STATES, NUM_CONTROLS, NUM_CONTROLS) = Matrix<double, NUM_CONTROLS, NUM_CONTROLS>::Identity();
  Be.setZero();
  Be.block(NUM_STATES, 0, NUM_CONTROLS, NUM_CONTROLS) = Matrix<double, NUM_CONTROLS, NUM_CONTROLS>::Identity();

  Q.setZero();
  Q.block(0, 0, NUM_STATES, NUM_STATES) = p_Q;
  R = Rotation2D<double> (s_star(2, p_horizon)).toRotationMatrix();
  Q.block(0, 0, 2, 2) = R.transpose() * p_Q.block(0, 0, 2, 2) * R;
  Q.block(NUM_STATES, NUM_STATES, NUM_CONTROLS, NUM_CONTROLS) = p_R_delta;
  Q.block(NUM_STATES + NUM_CONTROLS, NUM_STATES, NUM_CONTROLS, NUM_CONTROLS) = -p_R_delta;
  Q.block(NUM_STATES, NUM_STATES + NUM_CONTROLS, NUM_CONTROLS, NUM_CONTROLS) = -p_R_delta;
  Q.block(NUM_STATES + NUM_CONTROLS, NUM_STATES + NUM_CONTROLS, NUM_CONTROLS, NUM_CONTROLS) = p_R_delta;

  s_opt.block(0, 0, NUM_STATES, 1) = s_star.col(p_horizon) - s.col(p_horizon);
  s_opt.block(NUM_STATES, 0, NUM_CONTROLS, 1) = -u0.col(p_horizon - 1);
  s_opt.block(NUM_STATES + NUM_CONTROLS, 0, NUM_CONTROLS, 1) = -u0.col(p_horizon - 2);

  // Ricatti recursion
  P = Q;
  q = -Q * s_opt;
  for (int i = p_horizon - 1; i >= 0; i--) {
    R = Rotation2D<double> (s_star(2, i)).toRotationMatrix();
    Q.block(0, 0, 2, 2) = R.transpose() * p_Q.block(0, 0, 2, 2) * R;

    s_opt.block(0, 0, NUM_STATES, 1) = s_star.col(i) - s.col(i);
    if (i >= 1) {
      s_opt.block(NUM_STATES, 0, NUM_CONTROLS, 1) = -u0.col(i - 1);
    }
    else {
      s_opt.block(NUM_STATES, 0, NUM_CONTROLS, 1) = -u_prev;
    }
    if (i >= 2) {
      s_opt.block(NUM_STATES + NUM_CONTROLS, 0, NUM_CONTROLS, 1) = -u0.col(i - 2);
    }
    else {
      s_opt.block(NUM_STATES + NUM_CONTROLS, 0, NUM_CONTROLS, 1) = -u_prev;
    }
    u_opt = -u0.col(i);

    Ae.block(0, 0, NUM_STATES, NUM_STATES) = A[i];
    Be.block(0, 0, NUM_STATES, NUM_CONTROLS) = B[i];

    Z = (p_R + Be.transpose() * P * Be).inverse();
    K[i] = -Z * Be.transpose() * P * Ae;
    g[i] = -Z * (Be.transpose() * q - p_R * u_opt);
    P = Q + Ae.transpose() * P * Ae + Ae.transpose() * P * Be * K[i];
    P = 0.5 * (P + P.transpose());
    q = (Ae + Be * K[i]).transpose() * q - K[i].transpose() * p_R * u_opt - Q * s_opt;
  }

  // simulate forward
  s_next = s0;
  for (int i = 0; i < p_horizon; i++) {
    ds.block(0, 0, NUM_STATES, 1) = s_next - s.col(i);
    if (i >= 1) {
      ds.block(NUM_STATES, 0, NUM_CONTROLS, 1) = u_out->col(i - 1) - u0.col(i - 1);
    }
    else {
      ds.block(NUM_STATES, 0, NUM_CONTROLS, 1).setZero();
    }
    if (i >= 2) {
      ds.block(NUM_STATES + NUM_CONTROLS, 0, NUM_CONTROLS, 1) = u_out->col(i - 2) - u0.col(i - 2);
    }
    else {
      ds.block(NUM_STATES + NUM_CONTROLS, 0, NUM_CONTROLS, 1).setZero();
    }

    u_out->col(i) = u0.col(i) + K[i] * ds + g[i];
    (*u_out)(1, i) = max((*u_out)(1, i), -p_vs.param.max_wheel_angle);
    (*u_out)(1, i) = min((*u_out)(1, i), p_vs.param.max_wheel_angle);
    simulateRK4(s_next, u_out->col(i), &s_next);
  }
}

#define EPSILON 1e-5

// dynamics of the car (bicycle model with velocity/steering input)
void MPCController::dynamics(const Matrix<double, NUM_STATES, 1> &s, const Matrix<double, NUM_CONTROLS, 1> &u_, Matrix<double, NUM_STATES, 1> *s_dot, Matrix<
    double, NUM_STATES, NUM_STATES> *A, Matrix<double, NUM_STATES, NUM_CONTROLS> *B) {
  double u = s(3), v = s(4), cos_th = cos(s(2)), sin_th = sin(s(2));
  double th_dot = s(5), u_dot = u_(0);
  double del = u_(1), tan_del = tan(u_(1)), cos_del = cos(u_(1));
  double Fyf, Fyr, Ca = p_vs.param.tire_stiffness, m = p_vs.param.mass, a = p_vs.param.a, b = p_vs.param.b, I = p_vs.param.iz;

  // compute slip angles and lateral forces
  Fyf = -Ca * (atan2(v + th_dot * a, max(u, dgc::dgc_mph2ms(5))) - del);
  Fyr = -Ca * atan2(v - th_dot * b, max(u, dgc::dgc_mph2ms(5)));

  // compute derivatives
  (*s_dot)(0) = u * cos_th - v * sin_th;
  (*s_dot)(1) = u * sin_th + v * cos_th;
  (*s_dot)(2) = th_dot;

  (*s_dot)(3) = u_dot;
  (*s_dot)(4) = tan_del * (u_dot - th_dot * v) + (Fyf / cos_del + Fyr) / m - th_dot * u;
  (*s_dot)(5) = m * a / I * tan_del * (u_dot - th_dot * v) + a * Fyf / (I * cos_del) - b * Fyr / I;

  // compute Jacobians (numerically) if desired
  if (A != 0) {
    Matrix<double, NUM_STATES, 1> s2 = s;
    Matrix<double, NUM_STATES, 1> s_dot1, s_dot2;
    for (int i = 0; i < NUM_STATES; i++) {
      s2(i) += EPSILON;
      dynamics(s2, u_, &s_dot1, 0, 0);
      s2(i) -= 2 * EPSILON;
      dynamics(s2, u_, &s_dot2, 0, 0);
      s2(i) += EPSILON;
      A->col(i) = (s_dot1 - s_dot2) / (2 * EPSILON);
    }
  }

  if (B != 0) {
    Matrix<double, NUM_CONTROLS, 1> u2 = u_;
    Matrix<double, NUM_STATES, 1> s_dot1, s_dot2;
    for (int i = 0; i < NUM_CONTROLS; i++) {
      u2(i) += EPSILON;
      dynamics(s, u2, &s_dot1, 0, 0);
      u2(i) -= 2 * EPSILON;
      dynamics(s, u2, &s_dot2, 0, 0);
      u2(i) += EPSILON;
      B->col(i) = (s_dot1 - s_dot2) / (2 * EPSILON);
    }
  }
}

void MPCController::simulateEuler(const Matrix<double, NUM_STATES, 1> &s, const Matrix<double, NUM_CONTROLS, 1> &u, Matrix<double, NUM_STATES, 1> *s_next,
    Matrix<double, NUM_STATES, NUM_STATES> *A, Matrix<double, NUM_STATES, NUM_CONTROLS> *B) {
  Matrix<double, NUM_STATES, 1> s_dot;
  dynamics(s, u, &s_dot, A, B);
  (*s_next) = s + s_dot / p_hertz;

  if (A) {
    (*A) /= p_hertz;
    (*A) += Matrix<double, NUM_STATES, NUM_STATES>::Identity();
  }

  if (B) (*B) /= p_hertz;
}

void MPCController::simulateRK4(const Matrix<double, NUM_STATES, 1> &s, const Matrix<double, NUM_CONTROLS, 1> &u, Matrix<double, NUM_STATES, 1> *s_next,
    Matrix<double, NUM_STATES, NUM_STATES> *A, Matrix<double, NUM_STATES, NUM_CONTROLS> *B) {
  Matrix<double, NUM_STATES, 1> k1, k2, k3, k4;
  double dt = 1 / p_hertz;

  dynamics(s, u, &k1);
  dynamics(s + 0.5 * dt * k1, u, &k2);
  dynamics(s + 0.5 * dt * k2, u, &k3);
  dynamics(s + dt * k3, u, &k4);
  (*s_next) = s + dt * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0);

  // compute Jacobians (numerically) if desired
  if (A != 0) {
    Matrix<double, NUM_STATES, 1> s2 = s;
    Matrix<double, NUM_STATES, 1> sn1, sn2;
    for (int i = 0; i < NUM_STATES; i++) {
      s2(i) += EPSILON;
      simulateRK4(s2, u, &sn1, 0, 0);
      s2(i) -= 2 * EPSILON;
      simulateRK4(s2, u, &sn2, 0, 0);
      s2(i) += EPSILON;
      A->col(i) = (sn1 - sn2) / (2 * EPSILON);
    }
  }

  if (B != 0) {
    Matrix<double, NUM_CONTROLS, 1> u2 = u;
    Matrix<double, NUM_STATES, 1> sn1, sn2;
    for (int i = 0; i < NUM_CONTROLS; i++) {
      u2(i) += EPSILON;
      simulateRK4(s, u2, &sn1, 0, 0);
      u2(i) -= 2 * EPSILON;
      simulateRK4(s, u2, &sn2, 0, 0);
      u2(i) += EPSILON;
      B->col(i) = (sn1 - sn2) / (2 * EPSILON);
    }
  }
}

// use simulator to convert change in velocity to "throttle"
double MPCController::simulatorThrottle(double u, double v, double th_dot, double u_dot, double del) {
  double Fyf, Fxf;
  double Ca = p_vs.param.tire_stiffness, m = p_vs.param.mass;

  Fyf = -Ca * (atan2(v + th_dot * p_vs.param.a, max(u, dgc::dgc_mph2ms(5))) - del);
  Fxf = (m * u_dot - m * th_dot * v + Fyf * sin(del)) / cos(del);

  if (Fxf > 0) {
    return Fxf / (m * p_vs.param.throttle_accel_coef);
  }
  else {
    return Fxf / (m * 100.0 * p_vs.param.brake_decel_coef);
  }
}

template <class T> void MPCController::getParam(std::string key, T& var) {
  if(!nh_.getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + std::string("."));
  }
}

} // namespace vlr
