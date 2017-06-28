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


#include <float.h>
#include <map>
#include <fstream>
#include <limits>
#include <global.h>

#include <vlrException.h>
#include <trajectoryEvaluator.h>

namespace drc = driving_common;

namespace vlr {

const double TrajectoryEvaluator::parameters::passat_length;
const double TrajectoryEvaluator::parameters::passat_width;
const double TrajectoryEvaluator::parameters::passat_offset;


const double TrajectoryEvaluator::parameters::safety_width;
const double TrajectoryEvaluator::parameters::no_safety_width;
const double TrajectoryEvaluator::parameters::safety_length;
const double TrajectoryEvaluator::parameters::no_safety_length;

//#define TRACE(x)  std::cout << context<ChsmPlanner>().name() << " [StDrive] " << x << std::endl;
#define TRACE(x)

TrajectoryEvaluator::parameters::parameters() : weight(5.), time_sample_res(0.1), checked_horizon(5.0), a_min(-8), a_max(8),
               t_to_next_step(0.22), pull_away_time(2.0), generation_velocity_threshold(3.5) {

  velocity_keeping.t_horizon        = checked_horizon;
  velocity_keeping.a_max            = a_max;
  velocity_keeping.a_min            = a_min;
  velocity_keeping.time_res         = 1.0;
  velocity_keeping.time_max         = 5.0;
  velocity_keeping.v_res            = 1.0;
  velocity_keeping.v_offset_max     = 3.0;
  velocity_keeping.v_offset_min     = -8.0;
  velocity_keeping.k_t              = 1.2;
  velocity_keeping.k_j              = 1.0;
  velocity_keeping.k_v              = 4.0;
  velocity_keeping.time_sample_res  = time_sample_res;

  stop.t_horizon        = checked_horizon;
  stop.a_max            = a_max;
  stop.a_min            = a_min; // have to be identical to above
  stop.time_res         = 1.0;
  stop.time_max         = 8.0;
  stop.s_res            = 2.0;
  stop.s_offset_max     = 2.0;
  stop.s_offset_min     = -10.0;
  stop.k_t              = 2.0;
  stop.k_j              = 2.0;
  stop.k_s              = 4.0;
  stop.time_sample_res  = time_sample_res;

  following.t_horizon        = checked_horizon;
  following.a_max            = a_max;
  following.a_min            = a_min; // have to be identical to above
  following.time_res         = 1; //0.5;
  following.time_max         = 5.0;
  following.s_res            = 1;// 0.5;
  following.s_offset_max     = 3.0;
  following.s_offset_min     = -3.0;
  following.k_t              = .3;
  following.k_j              = 1.0;
  following.k_s              = 100.;
  following.time_sample_res  = time_sample_res;


  lane_keeping.t_horizon            = checked_horizon;
  lane_keeping.d_res                = .75;
  lane_keeping.d_offset_max         = 3.1;
  lane_keeping.d_offset_min         = -3.1;

  lane_keeping.holon.d_ddot_max     = 10.0;
  lane_keeping.holon.d_ddot_min     = -10.0;
  lane_keeping.holon.time_res       = 1.0;
  lane_keeping.holon.time_max       = 10.0;

  lane_keeping.holon.k_t            = 0.1 * weight;
  lane_keeping.holon.k_j            = 0.3 * weight;
  lane_keeping.holon.k_d            = 4.5 * weight;

  lane_keeping.holon.time_sample_res = time_sample_res;

  lane_keeping.nonhol.d_pprime_max  = 10.0;
  lane_keeping.nonhol.d_pprime_min  = -10.0;
  lane_keeping.nonhol.s_res         = 2.0;
  lane_keeping.nonhol.s_max         = 20.0;

  lane_keeping.nonhol.k_s           = 0.1 * weight;
  lane_keeping.nonhol.k_j           = 0.3 * weight;
  lane_keeping.nonhol.k_d           = 4.5 * weight;

  ptraj_2d.max_curvature                  = 1; //0.2; //[1/m]
  ptraj_2d.max_center_line_offset         = lane_keeping.d_offset_max;// + 0.25;
  ptraj_2d.max_center_line_angular_offset = 40.* M_PI / 180.; //[]  25.* M_PI / 180.;
  ptraj_2d.max_lat_acceleration           = std::numeric_limits<double>::infinity(); // [m/s²]  //disabled for now since it interferes with planner's velocity calculation
  ptraj_2d.max_lon_acceleration           = 6.0; // [m/s²]
  ptraj_2d.min_lon_acceleration           = -6.0; // [m/s²]
  ptraj_2d.time_delay                     = 0.1; // [s] do not change! Due to numerical effects and after reinitialization the trajectories have to have <time_delay> time to comply with the restrictions

  control_t_horizon = checked_horizon;
  control_t_res = 0.1;
  reinit_dist_thresh = 2.5;
  no_movement_reinit_timeout = 10.0;
  reinit_tangential_error_rate_thresh = 2; // TODO: find good values
  reinit_normal_error_rate_thresh = 1.5;


}

TrajectoryEvaluator::parameters::~parameters() {

}

TrajectoryEvaluator::TrajectoryEvaluator(pthread_mutex_t& obstacle_map_mutex, pthread_mutex_t& dyn_obstacle_mutex) :
                                             lane_and_velocity_set_(params().lane_keeping, params().velocity_keeping, params().ptraj_2d),
                                             lane_and_stop_set_(params().lane_keeping, params().stop, params().ptraj_2d),
                                             lane_and_follow_set_(params().lane_keeping, params().following, params().ptraj_2d),
                                             reinit_(true),
                                             obstacle_map_mutex_(obstacle_map_mutex),
                                             dyn_obstacle_mutex_(dyn_obstacle_mutex)
{
  active_traj_ = lane_and_velocity_set_.set_data_.end();
}

TrajectoryEvaluator::~TrajectoryEvaluator() {

}

void TrajectoryEvaluator::setParams(parameters& new_params) {
  params_=new_params;
  lane_and_velocity_set_.params(params().lane_keeping, params().velocity_keeping, params().ptraj_2d);
  lane_and_stop_set_.params(params().lane_keeping, params().stop, params().ptraj_2d);
  lane_and_follow_set_.params(params().lane_keeping, params().following, params().ptraj_2d);
}

  // stop_s - arc length value we want to stop at
  // v_follow - velocity along center_line of car we want to follow
  // a_follow - acceleration along center_line of car we want to follow
  // *_reinit - values we need for (re)initialization in case
  // e.g. control deviation becomes too large

void TrajectoryEvaluator::makeTrajectory(const std::vector<CurvePoint>& center_line,
    const std::map<int, Vehicle>& predicted_obstacles, double desired_v, double desired_a,
    double current_s, double stop_s, double lead_vehicle_s, double lead_vehicle_v, double lead_vehicle_a, const driving_common::GlobalPose& current_pose,
    double current_yaw_rate, std::vector<driving_common::TrajectoryPoint2D>& trajectory) { //, std::vector<Frame>& frames) {

    double clck_t_start_trajectory_generation, clck_t_end_trajectory_generation,
         clck_t_start_collision_check, clck_t_end_collision_check;

//      printf("-1- %s: desired_v: %f, target speed: %f, follow distance: %f\n", __FUNCTION__, dgc::dgc_ms2mph(desired_v),  dgc::dgc_ms2mph(lead_vehicle_v), lead_vehicle_s);
//    printf("%s: weight: %f\n", __FUNCTION__, params().weight);
//    printf("max curvature: %f\n", params().ptraj_2d.max_curvature);
//    printf("max_center_line_angular_offset: %f\n", params().ptraj_2d.max_center_line_angular_offset);
//    printf("max_lat_acceleration: %f\n", params().ptraj_2d.max_lat_acceleration);
//    printf("max_lon_acceleration: %f\n", params().ptraj_2d.max_lon_acceleration);
//    printf("min_lon_acceleration: %f\n", params().ptraj_2d.min_lon_acceleration);
//    printf("orientation_thresh: %f\n", params().orientation_thresh);

  double t_current = drc::Time::current();

//  printf("%s: %i; first cycle: %i\n", __FUNCTION__, __LINE__,  reinit_);
    // check for replanning

  if (!reinit_) {
    try {
      getCurrentTrajectoryPoint(t_current, current_trajectory_point_);
    }
    catch(vlr::Ex<>& e) {
      std::cout << e.what() << std::endl;
    }
  }

  reinit_ = checkReinit(current_pose, current_yaw_rate,
                        stop_s != std::numeric_limits<double>::infinity(),
                        lead_vehicle_s != std::numeric_limits<double>::infinity(),
                        current_trajectory_point_);

//  reinit_ = true;
  double t_next_deadline = t_current + params().t_to_next_step;
  bool had_to_reinitialize = reinit_; // if we had to reinitialize and no trajectories were found we throw an exception

  if (!reinit_) {
    try {
      // calculate start point for next trajectory generation, which continuous the old trajectory at t_next_deadline
      start_trajectory_point_ = active_traj_->calculateNextStartTrajectoryPoint(t_next_deadline);
    }
    catch(vlr::Ex<>& e) {
      std::cout << "Next start trajectory point -> " << e.what() << std::endl;
    }
  }
  else {
      // for startup
    start_trajectory_point_.theta = normalizeAngle(current_pose.yaw() + current_yaw_rate*params().t_to_next_step);
    start_trajectory_point_.v = current_pose.v() + current_pose.a() * params().t_to_next_step;
    //start_trajectory_point_.a = current_pose.a();
// also get first trajectory point here for initial curvature and acceleration
      // will throw exception on startup
    try {
      // calculate start point for next trajectory generation, which continuous the old trajectory at t_next_deadline
      if(active_traj_ != lane_and_velocity_set_.set_data_.end()) {
        start_trajectory_point_ = active_traj_->calculateNextStartTrajectoryPoint(t_next_deadline);
      }
    }
    catch(vlr::Ex<>& e) {

    }
    calculateNextStartTrajectoryPointReinit(t_current, params().t_to_next_step, current_pose, current_yaw_rate, start_trajectory_point_);
    printf("new start point: (%f, %f / %f): k: %f, v: %f, a: %f\n", start_trajectory_point_.x, start_trajectory_point_.y, start_trajectory_point_.theta, start_trajectory_point_.kappa, start_trajectory_point_.v, start_trajectory_point_.a);
    reinit_ = false;
  }

//  static driving_common::GlobalPose last_pose=current_pose;
//  static double last_v = current_pose.v();
//  static double last_a = current_pose.a();
//  static double last_yaw_rate = current_yaw_rate;
//  driving_common::TrajectoryPoint2D tp;
//  calculateNextStartTrajectoryPointReinit(t_current, params().t_to_next_step, current_pose, current_yaw_rate, tp);
//  printf("diff: (%f, %f / %f): k: %f, v: %f, a %f (pred a: %f, cur a: %f)\n",
//      start_trajectory_point_.x-tp.x, start_trajectory_point_.y-tp.y,
//      normalizeAngle(start_trajectory_point_.theta-tp.theta),
//      start_trajectory_point_.kappa-tp.kappa, start_trajectory_point_.v-tp.v, start_trajectory_point_.a-tp.a, start_trajectory_point_.a, tp.a);
//
//  last_pose=current_pose;
//  last_v = current_pose.v();
//  last_a = current_pose.a();
//  last_yaw_rate = current_yaw_rate;

  // Switch between different generation modes
  GenerationMode generation_mode;
  if (start_trajectory_point_.v > params().generation_velocity_threshold) {
    generation_mode = time_based;
  }
  else {
    generation_mode = arclength_based;
  }

  bool skip_velocity_set_generation=false, skip_stop_set_generation=false, skip_follow_set_generation=false;
  std::multiset<PolyTraj2D, PolyTraj2D::CompPolyTraj2D> current_set;
  std::multiset<PolyTraj2D>::const_iterator best_current_traj;
  traj_mode_t last_traj_mode = active_traj_mode_;
  if(false) {
  //  if(!control_traj_.empty() && !had_to_reinitialize &&
//      (control_traj_[control_traj_.size()-1].t - t_current > 0.5*params().checked_horizon)) {
    current_set.insert(*active_traj_);
    double current_traj_collision_time = -std::numeric_limits<double>::infinity();
    bool current_trj_collision = checkTrajectoriesForCollision(current_set, predicted_obstacles,
                                         "current", best_current_traj, current_traj_collision_time);

    if(!current_trj_collision) {
      switch(active_traj_mode_) {
      case TRAJ_MODE_VELOCITY:
        skip_velocity_set_generation = true;
        break;

      case TRAJ_MODE_STOP:
        skip_stop_set_generation = true;
        break;

      case TRAJ_MODE_FOLLOW:
        skip_follow_set_generation = true;
        break;

      }
    }
  }
  /* ------------------------------>  */
  clck_t_start_trajectory_generation = drc::Time::current();

    // trajectory set for keeping desired velocity
  std::multiset<PolyTraj2D>::const_iterator best_velocity_traj;
  double velocity_traj_collision_time = -std::numeric_limits<double>::infinity();
  TrjOccupancyState_t have_valid_velocity_trajectories;
  if(skip_velocity_set_generation) {
    best_velocity_traj=best_current_traj;
    have_valid_velocity_trajectories=TRJ_FREE;
  }
  else {
    have_valid_velocity_trajectories = makeAndCheckVelocityTrajectories(center_line, t_next_deadline,
                        generation_mode, predicted_obstacles, desired_v, desired_a, best_velocity_traj, velocity_traj_collision_time);
  }

  // trajectory set for stopping
  std::multiset<PolyTraj2D>::const_iterator best_stop_traj;
  double stop_traj_collision_time = -std::numeric_limits<double>::infinity();
  TrjOccupancyState_t have_valid_stop_trajectories;
  if(skip_stop_set_generation) {
    best_stop_traj=best_current_traj;
    have_valid_stop_trajectories=TRJ_FREE;
  }
  else {
   have_valid_stop_trajectories = makeAndCheckStopTrajectories(center_line, t_next_deadline,
                        generation_mode, predicted_obstacles, stop_s, best_stop_traj, stop_traj_collision_time);
  }

    // trajectory set for car following
  std::multiset<PolyTraj2D>::const_iterator best_follow_traj;
  double follow_traj_collision_time = -std::numeric_limits<double>::infinity();
  TrjOccupancyState_t have_valid_follow_trajectories;
  if(skip_follow_set_generation) {
    best_follow_traj=best_current_traj;
    have_valid_follow_trajectories=TRJ_FREE;
  }
  else {
    have_valid_follow_trajectories = makeAndCheckFollowTrajectories(center_line, t_next_deadline,
                        generation_mode, predicted_obstacles, lead_vehicle_s, lead_vehicle_v, lead_vehicle_a, best_follow_traj, follow_traj_collision_time);
  }

  // ------------------------------>
  clck_t_end_trajectory_generation = drc::Time::current();

  clck_t_start_collision_check = drc::Time::current();


  clck_t_end_collision_check = drc::Time::current();

  // Override control strategy: take collision free trajectory, which slows down the most (minimum signed jerk)
  // or the one with longest time to collision if no set gave us a free trajectory

  std::map<double, traj_mode_t> traj_mode_map;
  if(have_valid_velocity_trajectories == TRJ_FREE) {
      traj_mode_map.insert(std::make_pair(best_velocity_traj->getInitialJerk(), TRAJ_MODE_VELOCITY));
  }
  if (have_valid_stop_trajectories == TRJ_FREE) {
    traj_mode_map.insert(std::make_pair(best_stop_traj->getInitialJerk(), TRAJ_MODE_STOP));
  }
  if (have_valid_follow_trajectories == TRJ_FREE) {
    traj_mode_map.insert(std::make_pair(best_follow_traj->getInitialJerk(), TRAJ_MODE_FOLLOW));
  }

  if(!traj_mode_map.empty()) {
    active_traj_mode_ = traj_mode_map.begin()->second;
  }
  else {
    std::cout << "No collision-free trajectory available! Using trajectory with longest time to collision.\n";

    if(have_valid_velocity_trajectories != TRJ_UNAVAILABLE) {traj_mode_map.insert(std::make_pair(velocity_traj_collision_time, TRAJ_MODE_VELOCITY));}
    if(have_valid_stop_trajectories != TRJ_UNAVAILABLE) {traj_mode_map.insert(std::make_pair(stop_traj_collision_time, TRAJ_MODE_STOP));}
    if(have_valid_follow_trajectories != TRJ_UNAVAILABLE) {traj_mode_map.insert(std::make_pair(follow_traj_collision_time, TRAJ_MODE_FOLLOW));}

      // is there any trajectory at all?!?
    if(traj_mode_map.empty()) {
      reinit_ = true;
      if(had_to_reinitialize) {throw VLRException("Emergency stop required (no trajectory could be generated).");}
      return;
    }
    active_traj_mode_ = traj_mode_map.rbegin()->second;

    std::cout << "Collision in = " << traj_mode_map.rbegin()->first << " s\n";
    reinit_ = true;
    throw VLRException("Emergency stop required (collision unavoidable, time's up).");
  }

//  if( (active_traj_mode_!=TRAJ_MODE_STOP) && (have_valid_stop_trajectories != TRJ_UNAVAILABLE) && (stop_s != std::numeric_limits<double>::infinity()) ) {
//   printf ("overriding..stop dist: %f\n", stop_s-current_s);
//    active_traj_mode_=TRAJ_MODE_STOP;
//  }

    // in case we follow another car and have to stop, make sure we pick the stop trajectory
    // if it does not bring us too close to the lead vehicle
  if( (active_traj_mode_!=TRAJ_MODE_STOP) && (have_valid_stop_trajectories != TRJ_UNAVAILABLE) && (have_valid_follow_trajectories != TRJ_UNAVAILABLE) &&
      (stop_s != std::numeric_limits<double>::infinity()) && (lead_vehicle_s != std::numeric_limits<double>::infinity()) ) {
    std::multiset<PolyTraj2D>::const_iterator best_follow_stop_traj;
    const std::vector<driving_common::TrajectoryPoint2D>& stop_traj = (*best_stop_traj).trajectory2D_;
    const std::vector<driving_common::TrajectoryPoint2D>& follow_traj = (*best_follow_traj).trajectory2D_;

    size_t trj_size;
    if(follow_traj.size() != stop_traj.size()) {
      printf("sizes of stop and follow trajectories differ. If you don't know what it means you probably want to ignore it.\n");
      trj_size = std::min(follow_traj.size(), stop_traj.size());
    }
    else {
      trj_size = stop_traj.size();
    }

    bool prefer_stop_traj=true;
    double s_stop=0;
    double s_follow=0;
      // index 1 is ok since both trajectories start at same position
    for(size_t i=1; i<trj_size; i++) {
      double stop_delta_s = hypot(stop_traj[i].x - stop_traj[i-1].x, stop_traj[i].y - stop_traj[i-1].y);
      s_stop += stop_delta_s;

      double follow_delta_s = hypot(follow_traj[i].x - follow_traj[i-1].x, follow_traj[i].y - follow_traj[i-1].y);
      s_follow += follow_delta_s;
      if(s_stop > s_follow) {
        prefer_stop_traj=false;
        break;
      }
    }

    if(prefer_stop_traj) {
      active_traj_mode_ = TRAJ_MODE_STOP;
    }
  }

    // if we stay on last valid trajectory, we can return here
  if((active_traj_mode_ == last_traj_mode) &&
     (skip_velocity_set_generation || skip_stop_set_generation || skip_follow_set_generation)) {
    return;
  }

  switch (active_traj_mode_) {
    case TRAJ_MODE_VELOCITY: {
      active_traj_ = best_velocity_traj;
      }
    //      std::cout << "Active trajectory mode: velocity\n";
     break;

    case TRAJ_MODE_FOLLOW:
      active_traj_ = best_follow_traj;
      std::cout << "Active trajectory mode: follow\n";
      break;

    case TRAJ_MODE_STOP:
      active_traj_ = best_stop_traj;
      std::cout << "Active trajectory mode: stop\n";
      break;

    default:
      TRACE(__FUNCTION__ << "at line : " << __LINE__ << " : BUG! Invalid trajectory mode.");
      // throw(VLRException("Emergency stop required!!"));
  }

  try {
    active_traj_->generateNewControlTrajectory(t_current, params().control_t_horizon, params().control_t_res, control_traj_);
    trajectory=control_traj_;
  }
  catch(vlr::Ex<>& e) {
    std::cout << "Control trajectory -> " << e.what() << std::endl;
  }

  // ------------------ DEBUG OUTPUT ------------------------------------
  if (generation_mode == time_based) {
    TRACE("generation_mode = " << "time_based");
  }
  else {
    TRACE("generation_mode = " << "arclength_based");
  }
  TRACE("Trajectory generation time = " << ((clck_t_end_trajectory_generation
      - clck_t_start_trajectory_generation) / CLOCKS_PER_SEC));
  TRACE("Collision check time = " << ((clck_t_end_collision_check - clck_t_start_collision_check)
      / CLOCKS_PER_SEC));
}

TrjOccupancyState_t TrajectoryEvaluator::makeAndCheckVelocityTrajectories(const std::vector<CurvePoint>& center_line, double time_to_next_deadline,
                          GenerationMode generation_mode, const std::map<int, Vehicle>& predicted_obstacles,
                          double desired_v, double /*desired_a*/, std::multiset<PolyTraj2D>::const_iterator& best_traj, double& time_to_collision) {
  try {
    lane_and_velocity_set_.generate(center_line, time_to_next_deadline, start_trajectory_point_, generation_mode, desired_v);
//    printf("%u velocity trajectories.\n", (uint32_t)lane_and_velocity_set_.set_data_.size());
  }
  catch(vlr::Ex<>& e) {
    std::cout << "Velocity -> " << e.what() << std::endl;
//    printf("-1- %s: desired_v: %f, stop_distance: %f, target speed: %f, follow distance: %f\n",
//        __FUNCTION__, dgc::dgc_ms2mph(desired_v),  stop_s, dgc::dgc_ms2mph(lead_vehicle_v), lead_vehicle_s);
    printf("-1- %s: desired_v: %f\n", __FUNCTION__, dgc::dgc_ms2mph(desired_v));
    return TRJ_UNAVAILABLE;
  }

    bool collision = checkTrajectoriesForCollision(lane_and_velocity_set_.set_data_, predicted_obstacles,
                                      "velocity", best_traj, time_to_collision);

    return (collision ? TRJ_BLOCKED : TRJ_FREE);
}

TrjOccupancyState_t TrajectoryEvaluator::makeAndCheckStopTrajectories(const std::vector<CurvePoint>& center_line, double time_to_next_deadline,
                          GenerationMode generation_mode, const std::map<int, Vehicle>& predicted_obstacles,
                          double stop_s, std::multiset<PolyTraj2D>::const_iterator& best_traj, double& time_to_collision) {

  if(stop_s == std::numeric_limits<double>::infinity()) {
    return TRJ_UNAVAILABLE;
  }

  static const double desired_stop_velocity = 0;
  static const double desired_stop_acceleration = 0;

  try {
    lane_and_stop_set_.generate(center_line, time_to_next_deadline, start_trajectory_point_, generation_mode, stop_s, desired_stop_velocity, desired_stop_acceleration);
 //   printf("%u stop trajectories.\n", (uint32_t)lane_and_stop_set_.set_data_.size());
  }
  catch(vlr::Ex<>& e) {
    printf("Current center line:\n");
    for (std::vector<CurvePoint>::const_iterator it = center_line.begin(); it != center_line.end(); it++) {
      printf("%lf %lf %lf %lf %lf %lf\n", (*it).s, (*it).x, (*it).y, (*it).theta, (*it).kappa, (*it).kappa_prime);
    }
    printf("\n");
    std::cout << "Stopping -> " << e.what() << std::endl;
    return TRJ_UNAVAILABLE;
  }

  bool collision = checkTrajectoriesForCollision(lane_and_stop_set_.set_data_, predicted_obstacles,
                                                             std::string("stop"), best_traj, time_to_collision);

  return (collision ? TRJ_BLOCKED : TRJ_FREE);
}

TrjOccupancyState_t TrajectoryEvaluator::makeAndCheckFollowTrajectories(const std::vector<CurvePoint>& center_line, double time_to_next_deadline,
                          GenerationMode generation_mode, const std::map<int, Vehicle>& predicted_obstacles,
                          double lead_vehicle_s, double lead_vehicle_v, double lead_vehicle_a,
                          std::multiset<PolyTraj2D>::const_iterator& best_traj, double& time_to_collision) {

  if(lead_vehicle_s == std::numeric_limits<double>::infinity()) {
    return TRJ_UNAVAILABLE;
  }

  LeadingVehicle2Target leading_vehicle_to_track;
  double  s_target, s_dot_target, s_ddot_target;
  leading_vehicle_to_track.calc(lead_vehicle_s, lead_vehicle_v, lead_vehicle_a, s_target, s_dot_target, s_ddot_target);
//  printf("lead_vehicle_s: %f, lead_vehicle_v: %f, lead_vehicle_a: %f, s_target: %f, s_dot_target: %f, s_ddot_target: %f\n", lead_vehicle_s, lead_vehicle_v, lead_vehicle_a, s_target, s_dot_target, s_ddot_target);

  try {
    lane_and_follow_set_.generate(center_line, time_to_next_deadline, start_trajectory_point_, generation_mode, s_target, s_dot_target, s_ddot_target);
//    printf("%u follow trajectories.\n", (uint32_t)lane_and_follow_set_.set_data_.size());
  }
  catch (vlr::Ex<>& e) {
 //   std::cout << "Following -> " << e.what() << std::endl;
    return TRJ_UNAVAILABLE;
  }

 bool collision = checkTrajectoriesForCollision(lane_and_follow_set_.set_data_, predicted_obstacles,
                                      "follow", best_traj, time_to_collision);

 return (collision ? TRJ_BLOCKED : TRJ_FREE);
}

void TrajectoryEvaluator::getCurrentTrajectoryPoint(double t, driving_common::TrajectoryPoint2D& current_tp) {

  if(control_traj_.empty()) {
    throw VLRException("Control trajectory is empty.");
  }

  std::vector<driving_common::TrajectoryPoint2D>::const_iterator it2 = control_traj_.begin();
  while(it2 != control_traj_.end() && (*it2).t < t) {it2++;}

  if(it2 == control_traj_.begin()) {
  reinit_ = true;
  throw VLRException("Negative time index => replanning.");
}
  if(it2 == control_traj_.end()) {
    reinit_ = true;
    //    std::cout << "getCurrentTrajectoryPoint: trajectory too old => replanning.\n";
    throw VLRException("Trajectory too old => replanning.");
}

std::vector<driving_common::TrajectoryPoint2D>::const_iterator it = it2;
it--;

double alpha = (t-(*it).t)/((*it2).t-(*it).t);
double om_alpha = 1.0 - alpha;

current_tp.t          = t;
current_tp.x          = om_alpha * (*it).x + alpha * (*it2).x;
current_tp.y          = om_alpha * (*it).y + alpha * (*it2).y;
current_tp.theta      = interpolateAngles((*it).theta, (*it2).theta, alpha, om_alpha);
current_tp.kappa      = om_alpha * (*it).kappa + alpha * (*it2).kappa;
current_tp.kappa_dot  = om_alpha * (*it).kappa_dot + alpha * (*it2).kappa_dot;
current_tp.jerk       = om_alpha * (*it).jerk + alpha * (*it2).jerk;
current_tp.v          = om_alpha * (*it).v + alpha * (*it2).v;
current_tp.a          = om_alpha * (*it).a + alpha * (*it2).a;
}

double TrajectoryEvaluator::interpolateAngles(double theta1, double theta2, double alpha, double om_alpha) {
  return atan2(om_alpha*sin(theta1)+alpha*sin(theta2), om_alpha*cos(theta1)+alpha*cos(theta2));
}

bool TrajectoryEvaluator::checkTrajectoriesForCollision(const std::multiset<PolyTraj2D, PolyTraj2D::CompPolyTraj2D>& traj_set,
    const std::map<int, Vehicle>& predicted_obstacles, const std::string set_name,
    std::multiset<PolyTraj2D>::const_iterator& best_traj, double& longest_time_to_collision) {

  bool collision = true, found_maybe = false;
  double collision_time;
  longest_time_to_collision = 0;
  std::multiset<PolyTraj2D>::const_iterator it_traj;
  uint32_t j = 0;
  for (it_traj = traj_set.begin(), j = 0; it_traj != traj_set.end(); it_traj++, j++) {

    TrjOccupancyState_t static_collision = TRJ_BLOCKED;
    if (obstacle_map_) {
      pthread_mutex_lock(&obstacle_map_mutex_);
      static_collision = checkCollisionStatic(it_traj->trajectory2D_); // TODO: add collision_time and longest_time_to_collision to static check
      pthread_mutex_unlock(&obstacle_map_mutex_);

      if (static_collision==TRJ_BLOCKED) {
        continue;
      }
//      std::vector<driving_common::TrajectoryPoint2D>::const_iterator tit = it_traj->trajectory2D_.begin(), tit_end = it_traj->trajectory2D_.end();
//      bool zero_velocity=true;
//      for(; tit!=tit_end; tit++) {
//      if(std::abs((*tit).v)>.01) {zero_velocity=false; break;}
//      }
//      if(zero_velocity) {continue;}
    }

    if (!checkCollisionOfTrajectoriesDynamic(it_traj->trajectory2D_, predicted_obstacles, collision_time)) {
      if (j != 0) {std::cout << j << "th trajectory (" << set_name << ") of " << traj_set.size() << " is free.\n";}
      longest_time_to_collision = std::numeric_limits<double>::infinity();
        // we found a collision-free trajectory regardless if it's in maybe range or not
      collision = false;
        // if we did not find any free before or we found a free now and had a maybe before => update best trajectory
      if(!found_maybe || static_collision==TRJ_FREE) {best_traj = it_traj;}
        // if there are only maybes we want the one that was found first since it's the best :-)
        // otherwise we found a free one and are done
      if(static_collision==TRJ_MAYBE_BLOCKED) {found_maybe=true;}
      else {
        break;
      }
    }
    else {
      if (collision_time > longest_time_to_collision) {
        longest_time_to_collision = collision_time;
        best_traj = it_traj;
      }
    }
  }

  return collision;
}

bool TrajectoryEvaluator::checkReinit(const driving_common::GlobalPose& current_pose, double current_yaw_rate,
                                      bool stop_in_sight, bool lead_vehicle_in_sight, driving_common::TrajectoryPoint2D& pred_state) {


  if(reinit_) {return true;}

    // angular error
  double e_theta = normalizeAngle(pred_state.theta - current_pose.yaw());

  // tangential (=> longitudinal) error rate
  double e_t = cos(pred_state.theta)*(current_pose.utmX() - pred_state.x) + sin(pred_state.theta)*(current_pose.utmY() - pred_state.y);

    // normal (=> lateral) error
  double e_n = -sin(pred_state.theta)*(current_pose.utmX() - pred_state.x) + cos(pred_state.theta)*(current_pose.utmY() - pred_state.y);

  // tangential (=> longitudinal) error rate
  double e_t_dot = current_pose.v() * cos(e_theta) - pred_state.v * (1.0 - pred_state.kappa*e_n);

  // normal (=> lateral) error rate
  double e_n_dot = current_pose.v() * sin(e_theta) - pred_state.v * pred_state.kappa*e_t;

  if(e_t_dot*e_t_dot/(params().reinit_tangential_error_rate_thresh*params().reinit_tangential_error_rate_thresh) +
     e_n_dot*e_n_dot/(params().reinit_normal_error_rate_thresh*params().reinit_normal_error_rate_thresh) > 1) {
    printf("e_t_dot: %f, e_n_dot: %f\n", e_t_dot, e_n_dot);
    return true;
  }
//  static double last_x=0, last_y=0;
//  static uint32_t no_movement_c=0;
//
//  double dx = std::abs(current_pose.utmX()-last_x);
//  double dy = std::abs(current_pose.utmY()-last_y);
//
//  last_x=current_pose.utmX();
//  last_y=current_pose.utmY();
//  if(dx<0.001 && dy <0.001 && desired_v != 0 && !stop_in_sight && !lead_vehicle_in_sight) {
//    no_movement_c++;
//  }
//  else {
//    no_movement_c=0;
//  }
//
//    // If didn't move for xx but should have we'd better reinitalize since trajectories are gone already
//  if(no_movement_c * params().t_to_next_step > params().no_movement_reinit_timeout) {
//    printf("No movement replanning - last start point:\nt: %f\nx: %f\ny: %f\ntheta: %f\nkappa: %f\nkappa_prime: %f\nv: %f\na: %fjerk: %f\n",
//        start_trajectory_point_.t, start_trajectory_point_.x, start_trajectory_point_.y, start_trajectory_point_.theta,
//        start_trajectory_point_.kappa, start_trajectory_point_.kappa_dot,
//        start_trajectory_point_.v, start_trajectory_point_.a, start_trajectory_point_.jerk);
//    no_movement_c=0;
//    return true;
//  }
//
//
//
///*  static double err_rate=0, prev_err_rate=0;
//  static double prev_err;
//  static double inc_err_time0=drc::Time::current();
//  static double inc_err_thresh=.2;
//  static double inc_err_time_thresh=5;
//  static uint32_t big_inc_err_time_thresh = .5;
//*/
//
//
//  dx = current_trajectory_point_.x - current_pose.utmX();
//  dy = current_trajectory_point_.y - current_pose.utmY();
//  double err = sqrt(dx * dx + dy * dy);
///*
//  err_rate=err-prev_err;
//  if(err_rate > prev_err_rate) {
//    if(drc::Time::current()-inc_err_time0 > inc_err_time_thresh ||
//       (err_rate > inc_err_thresh && (drc::Time::current()-inc_err_time0 > big_inc_err_time_thresh)) ) {
//      reinit_=true;
//      err_rate=0;
//      prev_err_rate=0;
//      prev_err=0;
//    }
//    prev_err = err;
//    prev_err_rate = err_rate;
//  }
//  else {
//    inc_err_time0=drc::Time::current();
//  }
//*/
// if (err > params().reinit_dist_thresh) {
//    // TODO: Use arc instead of line
//    printf("Distance (%f) between current and planned position too big (> %f) => replanning.\n", sqrt(dx * dx + dy * dy), params().reinit_dist_thresh);
//    printf("current position: (%f, %f), planned position (%f, %f)\n", current_pose.utmX(), current_pose.utmY(), current_trajectory_point_.x, current_trajectory_point_.y);
//    return true;
//  }

return false;
}

void TrajectoryEvaluator::calculateNextStartTrajectoryPointReinit(double t0, double dt, const driving_common::GlobalPose& current_pose, double current_yaw_rate, driving_common::TrajectoryPoint2D& pred_pose) {

double dt2 = dt*dt;
double dt3 = dt*dt2;
double dt4 = dt2*dt2;

double theta_0_dot_2 = current_yaw_rate*current_yaw_rate;

pred_pose.t = t0 + dt;
//pred_pose.kappa = (current_pose.v() == 0 ? 0 : current_yaw_rate / current_pose.v());
pred_pose.theta = normalizeAngle(current_pose.yaw() + current_yaw_rate*dt);
pred_pose.v = current_pose.v() + current_pose.a() * dt;
//pred_pose.a = current_pose.a();


  // angle used to transform from car cs to utm cs
double theta_back = -current_pose.yaw();
double ds_t = current_pose.v()*dt + 0.5*current_pose.a()*dt2 - current_pose.v()*theta_0_dot_2/6*dt3 - current_pose.a()*theta_0_dot_2/8*dt4;
double ds_n = current_yaw_rate*(current_pose.v()/2*dt2 + current_pose.a()/3*dt3 - current_pose.v()*theta_0_dot_2/24*dt4);

pred_pose.x = current_pose.utmX() + cos(theta_back)*ds_t + sin(theta_back)*ds_n;
pred_pose.y = current_pose.utmY() - sin(theta_back)*ds_t + cos(theta_back)*ds_n;
}

} // namespace vlr
