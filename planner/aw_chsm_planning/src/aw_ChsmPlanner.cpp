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


#include <boost/format.hpp>

#include <aw_CGAL.h>
#include <vlrException.h>
#include <perception/PerceptionObstacles.h>
//#include <clothoid.h>
//#include <bezCtxPS.h>

#include <aw_ChsmPlanner.hpp>
#include <aw_StPause.hpp>
#include <aw_StActive.hpp>
#include <aw_LaneChangeManager.hpp>
#include <obstaclePrediction.h>
//#include <aw_SituationInterpretation.hpp>

namespace drc = driving_common;

namespace vlr {

#undef TRACE
#define TRACE(str) std::cout << "[ChsmPlanner] "<< str << std::endl

template<class T, void*(T::*mem_fn)(void*)>
void* threadCBWrapper(void* ptr) {
  return (static_cast<T*>(ptr)->*mem_fn)(NULL);
  }

using namespace std;
using namespace CGAL_Geometry;
using namespace driving_common;

#define EPS 0.01

/*---------------------------------------------------------------------------
 * ChsmPlanner::Parameters
 *---------------------------------------------------------------------------*/
ChsmPlanner::Parameters::Parameters() {

  center_line_back_sample_length = -15;      // in what distance are curvepoints sampled behind the car
  center_line_sample_dist  = 7;    // distance between (re)sampled waypoints
  center_line_post_smoothing_sample_dist = 1;      // distance between sampled point after clothoid fitting
  center_line_min_lookahead_dist = 20; // we look at least 20m ahead, regardless of speed
  smoothing_range = 8;                // smooth over 8 points
  // kturn params
  kturn_delta = 4.7;                  // kturn distance offset
  kturn_radius = 6.0;                 // kturn radius
  kturn_switch_speed = 0.1;           // switches to next phase if speed below this threshold
  kturn_switch_distance = 0.5;        // switches to next phase if position difference below this threshold

  // message buffer
  message_buffer_size = 15;           // size of the internal message buffer

  safety_stop_infront_vehicle_distance = 1.5*SAFETY_DISTANCE; // 1.5 = compensate for measurement failures

  enable_recovery = false;
  enable_blockade_detection = false;

  // speed related params
  max_speed_global                      = dgc::dgc_mph2ms(25.0); // maximal velocity used by the planner in m/s
  curvature_slowdown_factor             = 1.5;              // slowdown factor for curvatures in tentacle mode
  surface_quality_slowdown_factor       = .5;               // slowdown factor for rough surface areas
  max_speed_drive                       = dgc::dgc_mph2ms(65);
  max_speed_kturn                       = dgc::dgc_mph2ms(6);
  max_speed_intersection                = dgc::dgc_mph2ms(35);
  max_speed_merge_intersection          = dgc::dgc_mph2ms(35);
  max_speed_intersection_approach       = dgc::dgc_mph2ms(55);
  max_speed_traffic_light_approach      = dgc::dgc_mph2ms(45);
  max_speed_crosswalk_approach_empty    = dgc::dgc_mph2ms(25);
  max_speed_crosswalk_approach_occupied = dgc::dgc_mph2ms(15);
  max_speed_goal                        = dgc::dgc_mph2ms(5);
  max_speed_pass_obstacle               = dgc::dgc_mph2ms(25);
  max_speed_pass_switch_lane            = dgc::dgc_mph2ms(5); // ?!?
  max_speed_enter_zone                  = dgc::dgc_mph2ms(5);
  max_speed_lane_merge_false            = dgc::dgc_mph2ms(5); // ?!?
}

ChsmPlanner::Parameters::~Parameters() {

}

/*---------------------------------------------------------------------------
 * ChsmPlanner
 *---------------------------------------------------------------------------*/
ChsmPlanner::ChsmPlanner(std::string rndf_filename, std::string mdf_filename,
                        double static_obstacle_map_width, double static_obstacle_map_height,
                        double static_obstacle_map_resolution)
                        : pose_queue_(pose_queue_size_), robot_yaw_rate_(0),
                          vehicle_manager(NULL), topology_(NULL), route_sampler(NULL),
                          publish_traffic_lights_(false),
                          current_index_on_mission_(0),
                          transitionCounter(0),
                          inPause(true),
                          traj_eval_(NULL),
                          last_start_idx_(0),
                          min_lookahead_s_(0),
                          lane_change_data(NULL),
                          stop_before_replanning(true),
                          forced_start_edge(NULL),
                          has_pause_lanechange(false),
                          road_boundaries_enabled( false ),
                          replan_reason( ChsmPlanner::UNKNOWN ),
                          static_obstacle_map_raw_(NULL),
                          static_obstacle_map_tmp_(NULL),
                          static_obstacle_map_(NULL),
                          emergency_map_clear_(false),
                          emergency_stop_initiated_(false),
                          quit_chsm_planner_(false),
                          new_static_obstacle_map_ready_(false) {

  pthread_attr_init(&def_thread_attr_);

  pthread_mutex_init(&pose_mutex_, NULL);
  pthread_mutex_init(&trajectory_mutex_, NULL);
  pthread_mutex_init(&dyn_obstacles_mutex_, NULL);
  pthread_mutex_init(&static_obstacle_map_mutex_, NULL);
  pthread_mutex_init(&static_obstacle_points_mutex_, NULL);
  pthread_mutex_init(&mission_mutex_, NULL);
  pthread_mutex_init(&topology_mutex_, NULL);
  pthread_mutex_init(&center_line_mutex_, NULL);
  pthread_mutex_init(&intersection_predictor_mutex_, NULL);
  pthread_mutex_init(&traffic_light_states_mutex_, NULL);
  pthread_mutex_init(&traffic_light_poses_mutex_, NULL);
  pthread_mutex_init(&pedestrian_prediction_mutex_, NULL);

  pthread_mutex_init(&static_obstacle_map_cycle_mutex_, NULL);
  pthread_cond_init(&static_obstacle_map_cycle_cv_, NULL);

  // create helper
  topology_ = new Topology(rndf_filename.c_str(), mdf_filename.c_str(), "Topo");
  vehicle_manager = new VehicleManager(*topology_);
  topology_->vehicle_manager = vehicle_manager;
  route_sampler = new RouteSampler(topology_);
  blockade_manager = new BlockadeManager(topology_);
  topology_->blockade_manager = blockade_manager;
  traj_eval_ = new TrajectoryEvaluator(static_obstacle_map_mutex_, dyn_obstacles_mutex_);

  velocity_desired_ = 0;
  velocity_following_ = 0;
  follow_distance_ = 0;
  stop_distance_ = 0;

  turn_signal_.signal = driving_common::TurnSignal::NONE;

  updateStaticObstacleMapSize(static_obstacle_map_width, static_obstacle_map_height, static_obstacle_map_resolution);

  pthread_create(&obstacle_map_update_thread_id_, &def_thread_attr_,
      threadCBWrapper<ChsmPlanner, &ChsmPlanner::updateStaticObstacleMapThread>, this);

  bIntersection = false;
  bOffroad = false;

    // TODO: remove...
  replan_distance_map_.insert(make_pair(0, 2));
  replan_distance_map_.insert(make_pair(.5, 2.5));
  replan_distance_map_.insert(make_pair(6, 3));
  replan_distance_map_.insert(make_pair(8, 4));
  replan_distance_map_.insert(make_pair(15, 5));
  replan_distance_map_.insert(make_pair(20, 6));
}

ChsmPlanner::~ChsmPlanner()
{
  delete topology_;
  delete vehicle_manager;
  delete route_sampler;
  delete blockade_manager;
  quit_chsm_planner_ = true;

  pthread_mutex_lock(&static_obstacle_map_cycle_mutex_);
  pthread_cond_signal(&static_obstacle_map_cycle_cv_);
  pthread_mutex_unlock(&static_obstacle_map_cycle_mutex_);

  pthread_join(obstacle_map_update_thread_id_, NULL);
  if(static_obstacle_map_raw_) {delete[] static_obstacle_map_raw_;}
  if(static_obstacle_map_tmp_) {delete[] static_obstacle_map_tmp_;}
  if(static_obstacle_map_) {delete[] static_obstacle_map_;}

  pthread_attr_destroy(&def_thread_attr_);

  pthread_mutex_destroy(&pose_mutex_);
  pthread_mutex_destroy(&trajectory_mutex_);
  pthread_mutex_destroy(&dyn_obstacles_mutex_);
  pthread_mutex_destroy(&static_obstacle_map_mutex_);
  pthread_mutex_destroy(&static_obstacle_points_mutex_);
  pthread_mutex_destroy(&mission_mutex_);
  pthread_mutex_destroy(&topology_mutex_);
  pthread_mutex_destroy(&center_line_mutex_);
  pthread_mutex_destroy(&traffic_light_states_mutex_);
  pthread_mutex_destroy(&traffic_light_poses_mutex_);
  pthread_mutex_destroy(&pedestrian_prediction_mutex_);

  pthread_mutex_destroy(&static_obstacle_map_cycle_mutex_);
  pthread_cond_destroy(&static_obstacle_map_cycle_cv_);

}

void ChsmPlanner::activate() {
  process_event(EvActivate());
}

void ChsmPlanner::pause() {
  process_event(EvPause());
}

void ChsmPlanner::start() {

  if(pose_queue_.empty()) {
    throw VLRException("Could not start chsm planner since current position is unknown (pose queue is empty).");
  }

  drc::GlobalPose pose = pose_queue_.pose(drc::Time::current());
  cout << "[CHSM] plan the mission" << endl;
  addMessage("plan the mission");
  pthread_mutex_lock(&mission_mutex_);
  topology_->planMission(pose.utmX(), pose.utmY(), pose.yaw());

  createSmoothedMissionLine();

  last_start_idx_ = 0;

  static bool dump_mission_lines = true;

  if(dump_mission_lines) {
    std::string ml_name = "mission_line.txt";
    std::string ml_raw_name = "mission_line_raw.txt";
    std::string ml_rndf_name = "mission_line_rndf.txt";
    try {
      dumpMissionLines(ml_name, ml_raw_name, ml_rndf_name);
    }
    catch(vlr::Ex<>& e) {
      std::cout << "Could not write center line data for current mission: " << e.what() << std::endl;
    }
  }

  pthread_mutex_unlock(&mission_mutex_);

  pthread_mutex_lock(&topology_mutex_);
  LaneChangeManager::annotateLaneChanges(topology_);
  pthread_mutex_unlock(&topology_mutex_);

  // initiate state machine
  cout << "[CHSM] initiate state machine" << endl;
  addMessage("initiate state machine");
  initiate();
}

void ChsmPlanner::process()
{
  transitionCounter = 0;
  process_event(EvProcess());
  process_event(EvAfterProcess());
}

void ChsmPlanner::stop() {
  //  process_event(EvStop());
}

uint32_t ChsmPlanner::extendAtMissionStart(std::vector<CurvePoint>& mission) {
  CurvePoint cp;
  double extended_length = - 2 * params().center_line_back_sample_length;
  std::vector<CurvePoint> pre_mission_points;
  while(extended_length > 0) {
    extended_length -= params().center_line_post_smoothing_sample_dist;
    cp.s = mission[0].s - extended_length; // negative s since it's before mission
    cp.kappa = 0;// smoothed_mission_points_[0].kappa;
    cp.theta = mission[0].theta;
    cp.kappa_prime = 0;
    cp.x = mission[0].x - extended_length*cos(cp.theta);
    cp.y = mission[0].y - extended_length*sin(cp.theta);
    pre_mission_points.push_back(cp);
  }
  mission.insert(mission.begin(), pre_mission_points.begin(), pre_mission_points.end());

return pre_mission_points.size();
}

uint32_t ChsmPlanner::extendAtMissionEnd(std::vector<CurvePoint>& mission, double front_sample_dist) {
  CurvePoint cp;
  CurvePoint& cp_last = mission[mission.size()-1];
  double extended_length=0;
  uint32_t num_post_sample_points=0;
  while(extended_length < front_sample_dist) {
    extended_length += params().center_line_post_smoothing_sample_dist;
    cp.s = cp_last.s + extended_length; // negative s since it's before mission
    cp.kappa = 0;// cp_last.kappa;
    cp.theta = cp_last.theta;
    cp.kappa_prime = 0;
    cp.x = cp_last.x + extended_length*cos(cp.theta);
    cp.y = cp_last.y + extended_length*sin(cp.theta);
    mission.push_back(cp);
    num_post_sample_points++;
  }

  return num_post_sample_points;
}

void ChsmPlanner::createSmoothedMissionLine() {
  std::vector<bool> mission_points_to_ignore;
  route_sampler->sampleMission(mission_points_, mission_points_to_ignore);

  smoother.sampleLinearEquidist(mission_points_, mission_points_to_ignore, params().center_line_sample_dist, sampled_mission_points_);

  if(sampled_mission_points_.size() < 2) {
    throw VLRException("Mission must contain at least 2 waypoints; resampling failed.");
  }

    // replacement for clothoid due to licence issues :-(
  mission_bezier_points_.clear();
  for(int32_t i=0; i<(int32_t)sampled_mission_points_.size()-1; i++) {
    CurvePoint& p0 = sampled_mission_points_[i];
    CurvePoint& p1 = sampled_mission_points_[i+1];
    CurvePoint tp1, tp2;
    if(i==0) {
      CurvePoint& p2 = sampled_mission_points_[i+2];
      double len0 = hypot(p1.x-p0.x, p1.y-p0.y);
      double len1 = hypot(p2.x-p1.x, p2.y-p1.y);
      double scale1 = len0/len1;
      tp2.x = p1.x - 1./3. * (p2.x-p1.x)*scale1;
      tp2.y = p1.y - 1./3. * (p2.y-p1.y)*scale1;
      double lenm1 = hypot(tp2.x-p0.x, tp2.y-p0.y);
      double scale0 = len0/lenm1;
      tp1.x = p0.x + 1./3. * (tp2.x-p0.x)*scale0;
      tp1.y = p0.y + 1./3. * (tp2.y-p0.y)*scale0;
    }
    else if((size_t)i==sampled_mission_points_.size()-2) {
      CurvePoint& pm1 = sampled_mission_points_[i-1];
      double lenm1 = hypot(p0.x-pm1.x, p0.y-pm1.y);
      double len0 = hypot(p1.x-p0.x, p1.y-p0.y);
      double scale0 = len0/lenm1;
      tp1.x = p0.x + 1./3. * (p0.x-pm1.x)*scale0;
      tp1.y = p0.y + 1./3. * (p0.y-pm1.y)*scale0;
      double len1 = hypot(p1.x-tp1.x, p1.y-tp1.y);
      double scale1 = len0/len1;
      tp2.x = p1.x - 1./3. * (p1.x-tp1.x)*scale1;
      tp2.y = p1.y - 1./3. * (p1.y-tp1.y)*scale1;
    }
    else {
      CurvePoint& pm1 = sampled_mission_points_[i-1];
      CurvePoint& p2 = sampled_mission_points_[i+2];
      double lenm1 = hypot(p0.x-pm1.x, p0.y-pm1.y);
      double len0 = hypot(p1.x-p0.x, p1.y-p0.y);
      double len1 = hypot(p2.x-p1.x, p2.y-p1.y);
      double scale0 = len0/lenm1;
      double scale1 = len0/len1;
      tp1.x = p0.x + 1./3. * (p0.x-pm1.x)*scale0;
      tp1.y = p0.y + 1./3. * (p0.y-pm1.y)*scale0;
      tp2.x = p1.x - 1./3. * (p2.x-p1.x)*scale1;
      tp2.y = p1.y - 1./3. * (p2.y-p1.y)*scale1;
    }
    mission_bezier_points_.push_back(p0);
    mission_bezier_points_.push_back(tp1);
    mission_bezier_points_.push_back(tp2);
  }

//  smoother.cubicBezierFromClothoid(sampled_mission_points_, mission_bezier_points_);

  mission_bezier_points_.push_back(sampled_mission_points_[sampled_mission_points_.size()-1]);
  std::vector<CurvePoint> temp_smoothed_mission_points;
  std::vector<CurvePoint> temp_smoothed_mission_points2;
  smoother.sampleCubicBezierEquidist(mission_bezier_points_, 1, temp_smoothed_mission_points);

  extendAtMissionStart(temp_smoothed_mission_points);
  double post_sample_dist = dgc::dgc_mph2ms(75)*traj_eval_->params().checked_horizon;  // assumed worst case
  extendAtMissionEnd(temp_smoothed_mission_points, post_sample_dist);

  smoother.clothoideSpline(temp_smoothed_mission_points, temp_smoothed_mission_points[0].theta, temp_smoothed_mission_points[0].kappa, temp_smoothed_mission_points[0].s, params().smoothing_range, smoothed_mission_points_);

  mission_points_set_.clear();
  std::vector<CurvePoint>::const_iterator cit = temp_smoothed_mission_points.begin();
  double x0=mission_points_[0].x;
  double y0=mission_points_[0].y;
  uint32_t mp_idx=0;
  for(uint32_t i=0; i<temp_smoothed_mission_points.size(); i++) {
     if(temp_smoothed_mission_points[i].x == x0 && temp_smoothed_mission_points[i].y == y0) {
       mission_points_[mp_idx].s=(double)i;   // TODO: index is stored as s, find different structure to store mission data..
       mission_points_set_.insert(&mission_points_[mp_idx]);
       mp_idx++;
       x0=mission_points_[mp_idx].x;
       y0=mission_points_[mp_idx].y;
     }
   }

createVelocityProfile();
}

void ChsmPlanner::createVelocityProfile() {
  // step 1: annotate each points with its maximum velocity
  Route::RouteEdgeList::const_iterator it = topology_->route.route.begin(), it_end = topology_->route.route.end();
  Route::RouteEdgeList::const_iterator it_last = --topology_->route.route.end();
  uint32_t last_idx=0;
  for(; it != it_end; it++ ) {
//    const RoutePlanner::AnnotatedRouteEdge::AnnotationList& annotation_list = (*it)->annotations();

    if(it != it_last) {
      Route::RouteEdgeList::const_iterator it2=it;
      it2++;

      uint32_t idx = 0, idx2 = 0;
      try {
        idx = missionIndex((*it)->edge()->fromVertex()->x(), (*it)->edge()->fromVertex()->y(), last_idx);
      }
      catch(vlr::Ex<>& e) {
        std::cout << e.what() << std::endl;
      }
      try {
        idx2 = missionIndex((*it2)->edge()->fromVertex()->x(), (*it2)->edge()->fromVertex()->y(), idx);
      }
      catch(vlr::Ex<>& e) {
        std::cout << e.what() << std::endl;
      }
      last_idx=idx;

        // TODO: Do proper preflight/run to determine state (machine) velocities
      double max_state_speed = params().max_speed_global;
//      if(annotation_list.empty()) {continue;}
//      RoutePlanner::AnnotatedRouteEdge::AnnotationList::const_iterator ait = annotation_list.begin(), ait_end = annotation_list.end();
//      std::stringstream s;
//      for(;ait != ait_end; ait++) {
//        if((*ait)->maneuver() == UC_MANEUVER_TRAVEL) {continue;}
//      }

      double min_edge_speed=(*it)->edge()->minSpeed();
      double max_edge_speed=(*it)->edge()->maxSpeed();
      double max_speed = std::max(min_edge_speed, std::min(max_state_speed, std::min(params().max_speed_global, max_edge_speed)));
      if(smoothed_mission_points_[idx].kappa >=EPS) {
        max_speed = std::min(max_speed, sqrt(params().curvature_slowdown_factor/smoothed_mission_points_[idx].kappa));
      }
//      if((*it)->edge()->fromVertex()->isStopVertex()) {max_speed=1;}
      while(idx<idx2) {
        smoothed_mission_points_[idx++].v=max_speed;
      }
    }
    else {
      uint32_t idx = 0;
      try {
        idx = missionIndex((*it)->edge()->fromVertex()->x(), (*it)->edge()->fromVertex()->y(), last_idx);
      }
      catch(vlr::Ex<>& e) {
        std::cout << e.what() << std::endl;
      }
      while(idx<smoothed_mission_points_.size()) {
        smoothed_mission_points_[idx++].v = 0;
      }
    }
  }
  double max_accel = 3;
  double max_decel = -2;

  // 2. Step forward through the mission and make velocities achievable with given accel
  std::vector<CurvePoint>::iterator mit = smoothed_mission_points_.begin(), mit_end = smoothed_mission_points_.end();
  for(; mit != mit_end; mit++ ) {
    std::vector<CurvePoint>::iterator mit2 = mit;
    mit2++;
    while(mit2 != mit_end) {
      if(checkAndReplaceVelocityForward(*mit, *mit2, max_accel)) {break;}
      mit2++;
    }
  }


  // 3. Step backwards through the mission and make velocities achievable with given decel
  std::vector<CurvePoint>::reverse_iterator rit = smoothed_mission_points_.rbegin(), rit_end = smoothed_mission_points_.rend();
  for(; rit != rit_end; rit++ ) {
    std::vector<CurvePoint>::reverse_iterator rit2 = rit;
    rit2++;
    while(rit2 != rit_end) {
      if(checkAndReplaceVelocityBackward(*rit, *rit2, max_decel)) {break;}
      rit2++;
    }
  }

    // 4. add time indices
  mission_time_map_.clear();
  mit = smoothed_mission_points_.begin(), mit_end = smoothed_mission_points_.end();
  double last_s, last_v, last_t;
  if(mit != mit_end) {
    CurvePoint& cp = *mit;
    cp.t = 0;
    last_s=cp.s;
    last_v=cp.v;
    last_t=cp.t;
    mit++;
  }
  for(uint32_t i=0; mit != mit_end; mit++, i++) {
    CurvePoint& cp = *mit;
    double ds=cp.s-last_s;
    if(ds==0) {
      cp.t=last_t;
      continue;
    }

    double dv=cp.v-last_v;
    double a=(cp.v*cp.v-last_v*last_v)/(2*ds);
    double dt;
    if (a != 0) {
      if (dv >= 0) {
        dt = -last_v / a + sqrt((last_v / a) * (last_v / a) + 2 * ds / a);
      }
      else {
         dt = -last_v / a - sqrt((last_v / a) * (last_v / a) + 2 * ds / a);
      }
    }
    else {
      if(last_v !=0) {
        dt = ds/last_v;
      } else {
      dt=std::numeric_limits<double>::infinity();
      }
    }
    cp.t = last_t + dt;
    last_t = cp.t;
    last_v = cp.v;
    last_s = cp.s;
    mission_time_map_.insert(std::make_pair(cp.t, &cp));
  }

//  {
//  printf("with t:\n");
//  std::vector<CurvePoint>::iterator tmit = smoothed_mission_points_.begin(), tmit_end = smoothed_mission_points_.end();
//  FILE* f = fopen("vt.txt", "w");
//  for(uint32_t i=0; tmit != tmit_end; tmit++, i++ ) {
//    fprintf(f, "%u. %f %f %f\n", i, tmit->s, dgc::dgc_ms2mph(tmit->v), tmit->t);
//  }
//  fclose(f);
//  }
}

bool ChsmPlanner::checkAndReplaceVelocityBackward(CurvePoint& p_ref, CurvePoint& p, double max_decel) {

  if (p.v > p_ref.v) {
    double t = -p_ref.v/max_decel-sqrt((p_ref.v/max_decel)*(p_ref.v/max_decel)+2*(p.s-p_ref.s)/max_decel);
    double v_feasible = p_ref.v + max_decel * t;
    if (p.v > v_feasible) {
      p.v = v_feasible;
      return false;
    }
  }
  return true;
}

bool ChsmPlanner::checkAndReplaceVelocityForward(CurvePoint& p_ref, CurvePoint& p, double max_accel) {
  if (p.v > p_ref.v) {
    double t = -p_ref.v/max_accel+sqrt((p_ref.v/max_accel)*(p_ref.v/max_accel)+2*(p.s-p_ref.s)/max_accel);
    double v_feasible = p_ref.v + max_accel * t;
    if (v_feasible < p.v) {
      p.v = v_feasible;
      return false;
    }
  }
  return true;
}

void ChsmPlanner::updateRobot(double timestamp, double smooth_x, double smooth_y, double offset_x, double offset_y, double yaw, double speed, double ax, double ay, double az, double yaw_rate)
{
  pthread_mutex_lock(&pose_mutex_);
  robot_yaw_rate_ = yaw_rate;
  double a = ax * cos(-yaw) - ay*sin(-yaw); // TODO: check this...+-theta?!?
  double a_lat = ax * sin(-yaw) + ay*cos(-yaw);// TODO: check this...+-theta?!?

  pose_queue_.push(GlobalPose(smooth_x, smooth_y, offset_x, offset_y, yaw, 0, 0, speed, 0, 0, a, a_lat, az), timestamp);

  pthread_mutex_unlock(&pose_mutex_);

  if(!topology_->isMissionPlanned()) {return;}

  // update ego vehicle
  pthread_mutex_lock(&topology_mutex_);
  topology_->updateEgoVehicle(smooth_x+offset_x, smooth_y+offset_y, yaw, speed);
  pthread_mutex_unlock(&topology_mutex_);

  // set offroad flag
  bOffroad = (*topology_->current_edge_it_on_complete_mission_graph)->edge()->isOffroadEdge();
}

void ChsmPlanner::updateTrafficLightStates(const driving_common::TrafficLightStates& tf_states) {
  pthread_mutex_lock(&traffic_light_states_mutex_);
  std::vector<driving_common::TrafficLightState>::const_iterator tlsit=tf_states.light_state.begin(), tlsit_end = tf_states.light_state.end();

  for (;tlsit != tlsit_end; tlsit++) {

    // TODO: remove old traffic lights...
//    printf("Got traffic light update for tl id %s (state: %c)\n", (*tlsit).name.c_str(), (*tlsit).state);
    std::map<std::string, driving_common::TrafficLightState>::iterator exists_tlsit = traffic_light_states_.find((*tlsit).name);
    if (exists_tlsit != traffic_light_states_.end()) {
      (*exists_tlsit).second = *tlsit;
    }
    else {
      traffic_light_states_.insert(std::make_pair((*tlsit).name, (*tlsit)));
    }
  }
  pthread_mutex_unlock(&traffic_light_states_mutex_);
}

   // find minimum lane width along center line piece used
   // for trajectory planning in the next time step
double ChsmPlanner::minLookaheadLaneWidth(Route::RouteEdgeList::iterator edge_it, double front_sample_dist) {
  double d = 0, lane_width = std::numeric_limits<double>::infinity();
  while(edge_it != topology_->route.route.end() && d < front_sample_dist) {
    lane_width = std::min(lane_width, (*edge_it)->edge()->width());
    d += (*edge_it)->edge()->length();
    edge_it++;
  }

  if(lane_width == std::numeric_limits<double>::infinity()) {
    lane_width = rndf::Lane::default_width;  // TODO: make independent of rndf lib...
  }

  return lane_width;
}

uint32_t ChsmPlanner::missionIndex(double x, double y, uint32_t last_idx) {
  CurvePoint cp;
  cp.x=x; cp.y=y;
  return missionIndex(cp, last_idx);
}

uint32_t ChsmPlanner::missionIndex(CurvePoint& cp, uint32_t last_idx) {
  std::pair<std::multiset<CurvePoint*, CurvePointComp>::const_iterator,
            std::multiset<CurvePoint*, CurvePointComp>::const_iterator> range = mission_points_set_.equal_range(&cp);

  if(range.first == range.second) {
    std::stringstream s;
    s << "Mission invalid: requested point (" << cp.x << ", " << cp.y << ") is not contained in mission.";
    throw VLRException(s.str());
  }

  std::multiset<CurvePoint*, CurvePointComp>::const_iterator cpit=range.first;
  for(; cpit != range.second; cpit++) {
    uint32_t idx = uint32_t((*cpit)->s); // TODO: find a proper way to handle index
    if(idx>=last_idx) {return idx;}
  }

  std::stringstream s;
  s << "Mission invalid: requested point (" << cp.x << ", " << cp.y << ") is not contained in mission before last index (" << last_idx << ")";
  throw VLRException(s.str());
}

void ChsmPlanner::generateCurvePoints(double front_sample_dist, double maxSpeed) {

 //printf("stop distance: %f\n", curvepoints.stop_distance);
 //double dt = Time::current();
 Route::RouteEdgeList::iterator edge_it = topology_->current_edge_it;
 double dist_from_start = topology_->ego_vehicle.distFromStart();


   // find current position in smoothed mission data set
 CurvePoint ocp;
 ocp.x = (*edge_it)->edge()->fromVertex()->x();
 ocp.y = (*edge_it)->edge()->fromVertex()->y();

 uint32_t min_idx=0;
 try {
   min_idx = missionIndex(ocp, last_start_idx_);
 }
 catch(vlr::Ex<>& e) {
   std::cout << "Error: requesting point on edge " << (*edge_it)->edge()->name() << ": " << e.what() << std::endl;
   throw e;
 }

 double s0 = smoothed_mission_points_[min_idx].s + dist_from_start;

    // Hysteresis for lookahead range to avoid velocity oscillations
  if(min_lookahead_s_ - s0 > front_sample_dist) {
    front_sample_dist = min_lookahead_s_ - s0;
  }
  else {
    min_lookahead_s_ = s0 + front_sample_dist;
  }

    // used later on for velocity calculation, this is discretized in original wp steps!
  current_index_on_mission_ = min_idx;

    // find closest (regarding to s) smoothly sampled index
  while(smoothed_mission_points_[current_index_on_mission_].s < s0 && current_index_on_mission_<smoothed_mission_points_.size()) {
      current_index_on_mission_++;
  }
      // find index for first back-sampled point
  while(s0-smoothed_mission_points_[min_idx].s < -params().center_line_back_sample_length) {
    if(min_idx==0) {break;}
    min_idx--;
  }

  last_start_idx_ = min_idx;

  pthread_mutex_lock(&center_line_mutex_);
  center_line_.clear();

//    // dependent on angle between car and centerline at mission start,
//    // it's not always possible to project current position onto centerline => extend backwards
//  if(min_idx == 0 && s0-smoothed_mission_points_[0].s < -params().center_line_back_sample_length) {
//    extendCenterLineAtMissionStart(s0);
//   }

  uint32_t idx = min_idx;
  while(idx<smoothed_mission_points_.size() && smoothed_mission_points_[idx].s-s0 < front_sample_dist) {
    center_line_.push_back(smoothed_mission_points_[idx]);
    idx++;
  }

//  if(idx == smoothed_mission_points_.size()) {
//    extendCenterLineAtMissionEnd(s0, front_sample_dist);
//  }

    // for the trajectory generation we need a lookahead range taking the max speed into account, for everything
    // else, the lookahead range is based on the last desired velocity (replace with last actual?!?)
  double current_front_sample_dist = std::max(params().center_line_min_lookahead_dist, traj_eval_->params().checked_horizon * velocity_desired_);
  // calculate desired velocity based on speed limits and curvature
  velocity_desired_ = calculateVelocity(maxSpeed, current_front_sample_dist);
  double accel_desired = 0;
/*  if (current_index_on_mission_ < smoothed_mission_points_.size() - 1) {
    double mission_t_current = smoothed_mission_points_[current_index_on_mission_].t;
    uint32_t lookahead_index_ = current_index_on_mission_ + 1;
    while (lookahead_index_ < smoothed_mission_points_.size()
            && smoothed_mission_points_[lookahead_index_].t - mission_t_current  < traj_eval_->params().checked_horizon) {
//      printf("mission_t_current: %f, lookahead t: smoothed_mission_points_[current_index_on_mission_].t: %f, dt: %f\n",
//          mission_t_current, smoothed_mission_points_[current_index_on_mission_].t, smoothed_mission_points_[current_index_on_mission_].t- mission_t_current);
      lookahead_index_++;
    }
    if (lookahead_index_ == smoothed_mission_points_.size()) {
      lookahead_index_--;
    }
    velocity_desired_ = std::min(maxSpeed, smoothed_mission_points_[lookahead_index_].v);
  }
  else {
    velocity_desired_ = 0;
  }
*/
  // -------
  double stop_s;
  if (stop_distance_ == std::numeric_limits<double>::infinity()) {
    stop_s = std::numeric_limits<double>::infinity();
  }
  else {
    uint32_t stop_idx = missionIndex(stop_point_, last_start_idx_);
    stop_s = smoothed_mission_points_[stop_idx].s - FRONT_BUMPER_DELTA;
  }

  drc::GlobalPose pose = pose_queue_.pose(drc::Time::current());

  double lane_width = minLookaheadLaneWidth(edge_it, current_front_sample_dist)-2.0; // since we want to stay inside lane
//  lane_width*=3.5; //for circledemo

  double follow_s = s0 + follow_distance_ - 5.0;  // TODO: for at 1m safety distance
  double accel_following = 0;
  double current_s = smoothed_mission_points_[current_index_on_mission_].s;
  //  printf("init time: %f\n", drc::Time::current() - dt);
//  printf("velocity_desired: %f\n", velocity_desired_);

  try {
    pthread_mutex_lock(&trajectory_mutex_);
    TrajectoryEvaluator::parameters te_params = traj_eval_->params();
    te_params.lane_keeping.d_offset_min = -0.5*lane_width;
    te_params.lane_keeping.d_offset_max =  0.5*lane_width;
    if(pose.v() < 10) { // TODO: make velocity dependent..more than this parking lot hack...
      te_params.velocity_keeping.v_offset_max     = 1.0;
      te_params.velocity_keeping.v_offset_min     = -4.0;
      te_params.velocity_keeping.v_res            = .5;
    }
    else {
      te_params.velocity_keeping.v_offset_max     = 3.0;
      te_params.velocity_keeping.v_offset_min     = -8.0;
      te_params.velocity_keeping.v_res            = 1;
    }
    te_params.reinit_dist_thresh = replanDistance(velocity_desired_);
    traj_eval_->setParams(te_params);

    traj_eval_->makeTrajectory(center_line_, vehicle_manager->vehicle_map, velocity_desired_,
                               accel_desired, current_s, stop_s, follow_s, velocity_following_, accel_following, pose,
                               robot_yaw_rate_, trajectory_points_);
  }
  catch(vlr::Ex<>& e) {
    printf("Error in trajectory generation: %s\n", e.what().c_str());
    pthread_mutex_unlock(&trajectory_mutex_);
    pthread_mutex_unlock(&center_line_mutex_);

   // TODO: integrate vehicle interface
//    vehiclecmd.beeper_on = 1;
//   vehiclecmd.hazard_lights_on = 1;

//    vehicle_manager->vehicle_map.clear(); // TODO: this will invalidate edge iterators :-(
    emergency_map_clear_ = true;
    pthread_mutex_lock(&static_obstacle_map_mutex_);
    memset(static_obstacle_map_raw_, 0, static_obstacle_map_width_*static_obstacle_map_height_*sizeof(uint8_t));
    memset(static_obstacle_map_, 0, static_obstacle_map_width_*static_obstacle_map_height_*sizeof(uint8_t));
    pthread_mutex_unlock(&static_obstacle_map_mutex_);

    generateStopTrajectory();
    emergency_stop_initiated_=true;
  }
  catch(...) {
    printf("Error in trajectory generation.\n");
  }

  pthread_mutex_unlock(&trajectory_mutex_);
  pthread_mutex_unlock(&center_line_mutex_);
//  printf("generation time: %f\n", drc::Time::current() - dt);
}

void ChsmPlanner::generateCurvePoints(double stop_distance, double follow_distance, double maxSpeed) {
  double front_sample_length = std::max(params().center_line_min_lookahead_dist, traj_eval_->params().checked_horizon * maxSpeed);
//  double front_sample_length = std::max(params.center_line_min_lookahead_dist, traj_eval_->params().checked_horizon * velocity_desired_);

  if (follow_distance < stop_distance && follow_distance < TRIGGER_DIST_FOLLOWING) { // following mode

    follow_distance_ = follow_distance;
    stop_distance_ = std::numeric_limits<double>::infinity();

    Vehicle* veh = topology_->get_next_vehicle();
    if (veh) {
        // TODO: matched/matched_from incosistent but pose is not used right now...
      dgc::dgc_pose_t pose;
      pose.x = veh->xMatched();
      pose.y = veh->yMatched();
      pose.z = 0.0;
      pose.yaw = veh->yawMatchedFrom();
      pose.pitch = 0.0;
      pose.roll = 0.0;
      velocity_following_ = calculateFollowingSpeed(veh->speed(), pose, veh->angleToMatchedEdge());
    }
    else {
      velocity_following_ = 0;
    }
    generateCurvePoints(front_sample_length, maxSpeed);
  }
  else if (stop_distance < TRIGGER_DIST_STOPLINE) { // stop mode
   follow_distance_  = std::numeric_limits<double>::infinity();
    stop_distance_ = stop_distance;
    generateCurvePoints(front_sample_length, maxSpeed);
  }
  else { // drive mode
    follow_distance_  = std::numeric_limits<double>::infinity();
    stop_distance_ = std::numeric_limits<double>::infinity();
    generateCurvePoints(front_sample_length, maxSpeed);
  }
}

void ChsmPlanner::generateCurvePoints(double maxSpeed) {
  double stop_distance,follow_distance;
  double goal_dist = topology_->distToMissionEnd();
  double kturn_dist = topology_->distToNextKTurn();


  if(goal_dist < kturn_dist) {
    stop_distance = goal_dist;
    stop_point_.x = mission_points_[mission_points_.size()-1].x;
    stop_point_.y = mission_points_[mission_points_.size()-1].y;
    if(stop_distance < 0) {maxSpeed = 0;} // TODO: Otherwise car might start again :-(
  }
  else {
    stop_distance = kturn_dist;
//    printf("TODO: set k turn start point.\n");
//    stop_point_.x = mission_points_[mission_points_.size()-1].x;
//    stop_point_.y = mission_points_[mission_points_.size()-1].y;
  }

  double mv_veh_dist, st_veh_dist;
  getVehicleDistances(st_veh_dist, mv_veh_dist);
  follow_distance = min(mv_veh_dist, st_veh_dist);

  generateCurvePoints(stop_distance, follow_distance, maxSpeed);
}

void ChsmPlanner::generateStopTrajectory() {
  velocity_desired_= 0;
  velocity_following_ = 0;
  follow_distance_ = std::numeric_limits<double>::infinity();
  stop_distance_ = std::numeric_limits<double>::infinity();
  generateCurvePoints(params().center_line_min_lookahead_dist, 0.0);
}

void ChsmPlanner::getVehicleDistances(double& standing, double& moving) {
  standing = topology_->distToNextStandingVehicle() - params().safety_stop_infront_vehicle_distance;
  moving = topology_->distToNextMovingVehicle();
}

double ChsmPlanner::calculateCurvatureVelocity(double max_speed, double curvature_slowdown_factor, double sample_length_front) {

  if(current_index_on_mission_>=smoothed_mission_points_.size()) {
    std::cout << "Error: position index not in mission.\n";
    return 0.0;
  }

  double maxAbsCurvature=0;//, meanAbsCurvature=0;
  double s0 = smoothed_mission_points_[current_index_on_mission_].s;

   // early acceleration: ignore curvature right in front of car
  s0 += max_speed * 0.5;
  uint32_t num_points=0, idx=current_index_on_mission_;
  while(smoothed_mission_points_[idx].s-s0 < sample_length_front) {
    idx++; num_points++;
    if(idx==smoothed_mission_points_.size()) {break;}

    maxAbsCurvature = std::max(maxAbsCurvature, std::abs(smoothed_mission_points_[idx].kappa));
//    meanAbsCurvature += std::abs(smoothed_mission_points_[idx].kappa);
  }

//  if(num_points>0) {
//    meanAbsCurvature/=((double)num_points);
//  }

  //  return max_speed/(1.0+curvature_slowdown_factor*sqrt(maxAbsCurvature));
//  printf("s: %f - macu: %f -> max vel: %f, or: %f or %f or %f\n", s0, maxAbsCurvature,
//      dgc::dgc_ms2mph(std::min(max_speed, sqrt(curvature_slowdown_factor/maxAbsCurvature))),
//      dgc::dgc_ms2mph(max_speed/(1.0+curvature_slowdown_factor*sqrt(maxAbsCurvature))),
//      dgc::dgc_ms2mph(max_speed/(1.0+curvature_slowdown_factor*maxAbsCurvature*maxAbsCurvature)),
//      dgc::dgc_ms2mph(std::min(max_speed, curvature_slowdown_factor/pow(maxAbsCurvature,1.5))));

  if(maxAbsCurvature < EPS) {
    return max_speed;
  }
  else {
//      return std::min(max_speed, curvature_slowdown_factor/pow(maxAbsCurvature,1.5));
//    return max_speed/(1.0+curvature_slowdown_factor*maxAbsCurvature*maxAbsCurvature);
//    return max_speed/(1.0+curvature_slowdown_factor*sqrt(maxAbsCurvature));
      return std::min(max_speed, sqrt(curvature_slowdown_factor/maxAbsCurvature));
  }
}

double ChsmPlanner::calculateRoughSurfaceVelocity(double max_speed) {
//  int i;
//  double mean=0, var=0;
//
//  for(i=0; i<ACCEL_HISTORY_SIZE; i++) {mean+=robot_az[i];}
//  mean/=((double)ACCEL_HISTORY_SIZE);
//
//  for(i=0; i<ACCEL_HISTORY_SIZE; i++) {var+=(robot_az[i]-mean)*(robot_az[i]-mean);}
//  var/=((double)ACCEL_HISTORY_SIZE); // no bias and simpler :-)

  double var = 0;
  //cout << " var " << var << endl;

  return max_speed/(1.0+params().surface_quality_slowdown_factor*var);
}

double ChsmPlanner::calculateVelocity(double max_state_speed, double sample_length_front) {
  double min_edge_speed=(*topology_->current_edge_it)->edge()->minSpeed();
  double max_edge_speed=(*topology_->current_edge_it)->edge()->maxSpeed();

  double max_speed = std::max(min_edge_speed, std::min(max_state_speed, std::min(params().max_speed_global, max_edge_speed)));

  double max_curvature_speed = calculateCurvatureVelocity(max_speed, params().curvature_slowdown_factor, sample_length_front);
  double max_roughness_speed = calculateRoughSurfaceVelocity(max_curvature_speed);

  double speed = max_roughness_speed; // std::max(min_edge_speed, std::min(max_state_speed, std::min(max_edge_speed, std::min(max_speed, std::min(max_roughness_speed, max_curvature_speed)))));
//  std::cout << "max speed:\nstate: " << dgc::dgc_ms2mph(max_state_speed) << " edge: " << dgc::dgc_ms2mph(max_edge_speed) << " global: " << dgc::dgc_ms2mph(max_speed) << " curvature: " << dgc::dgc_ms2mph(max_curvature_speed) << " roughness: " << dgc::dgc_ms2mph(max_roughness_speed) << " -> " << dgc::dgc_ms2mph(speed) << endl;
  return speed;
}

int ChsmPlanner::addMessage(const string& message) {
  last_message_timestamp_ = drc::Time::current();
  message_buffer_.push_back(message);
  while(message_buffer_.size()>params().message_buffer_size)
  message_buffer_.pop_front();
  return message_buffer_.size();
}

double ChsmPlanner::distance(const GlobalPose& pose) {
  GlobalPose current_pose = currentPose();
  return hypot(current_pose.utmX() - pose.utmX(), current_pose.utmY() - pose.utmY());
}

const std::string& ChsmPlanner::name() const
{
  return name_;
}

GlobalPose ChsmPlanner::pose(double timestamp) {
  pthread_mutex_lock(&pose_mutex_);
  try {
    GlobalPose pose = pose_queue_.pose(timestamp);
    pthread_mutex_unlock(&pose_mutex_);
    return pose;
  }
  catch(vlr::Ex<>& e) {
    pthread_mutex_unlock(&pose_mutex_);
    throw e;
  }
}

void ChsmPlanner::latestPose(GlobalPose& latest_pose, double& latest_timestamp) {
  pthread_mutex_lock(&pose_mutex_);
  try {
    pose_queue_.latestPose(latest_pose, latest_timestamp);
    pthread_mutex_unlock(&pose_mutex_);
  }
  catch(vlr::Ex<>& e) {
    pthread_mutex_unlock(&pose_mutex_);
    throw e;
  }
}

void ChsmPlanner::currentLocalizeOffsets(double& offset_x, double& offset_y) {
  pthread_mutex_lock(&pose_mutex_);
  pose_queue_.offsets(drc::Time::current(), offset_x, offset_y);
  pthread_mutex_unlock(&pose_mutex_);
}

// if the car we want to follow comes towards us the speed will be set to zero
double ChsmPlanner::calculateFollowingSpeed(const double nextVehicleSpeed, const dgc::dgc_pose_t& /*nextVehiclePose*/, const double angleToEdge)
{

  //double angle_diff = deltaAngle(nextVehiclePose.yaw, robot_pose.yaw);
  bool is_opposite_angle = angleToEdge > M_PI_2/*  && distance(nextVehiclePose) < 30*/;
  return is_opposite_angle?-nextVehicleSpeed:nextVehicleSpeed;
}

void perpBisector(double x1, double y1, double x2, double y2, double& AM, double& BM, double& CM) {
  double A = y2-y1;
  double B = x1-x2;

  double xm1 = 0.5*(x1+x2);
  double ym1 = 0.5*(y1+y2);

  double D = -B*xm1 + A*ym1;

  AM = -B;
  BM = A;
  CM = D;
}

void ChsmPlanner::estimateKappa(CurvePoint& p1, CurvePoint& p2, CurvePoint& p3, double& kappa) {

    // Ax+By=C
  double A1, B1, C1, A2, B2, C2;
  perpBisector(p1.x, p1.y, p2.x, p2.y, A1, B1, C1);
  perpBisector(p2.x, p2.y, p3.x, p3.y, A2, B2, C2);

  double det = A1*B2 - A2*B1;
  if(det == 0) {
     kappa=0;
     return;
  }

  double cx = (B2 * C1 - B1 * C2) / det;
  double cy = (A1 * C2 - A2 * C1) / det;

  kappa = 1.0/hypot(p1.x-cx, p1.y-cy);
}

void ChsmPlanner::dumpMissionLines(std::string& ml_name, std::string& ml_raw_name, std::string& ml_rndf_name) {

  FILE* dbgf = NULL;
  char buf[1000];

  static const std::string err_str_open = "Cannot open file ";
  static const std::string err_str_write = "Cannot write to file ";

  if(!(dbgf = fopen(ml_name.c_str(), "w"))) {
    throw VLRException(err_str_open + ml_name);
  }

  for (std::vector<CurvePoint>::const_iterator it = smoothed_mission_points_.begin(); it
      != smoothed_mission_points_.end(); it++) {
    sprintf(buf, "%lf %lf %lf %lf %lf %lf\n", (*it).s, (*it).x, (*it).y, (*it).theta, (*it).kappa, (*it).kappa_prime);
    size_t len = fwrite(buf, 1, strlen(buf), dbgf);
    if(len != strlen(buf)) {
      throw VLRException(err_str_write + ml_name);
    }
  }
  fclose(dbgf);
  dbgf=NULL;

  if(!(dbgf = fopen(ml_rndf_name.c_str(), "w"))) {
    throw VLRException(err_str_open + ml_rndf_name);
  }

  for (std::vector<CurvePoint>::const_iterator it = mission_points_.begin(); it != mission_points_.end(); it++) {
    sprintf(buf, "%lf %lf %lf %lf %lf %lf\n", (*it).s, (*it).x, (*it).y, (*it).theta, (*it).kappa, (*it).kappa_prime);
    size_t len = fwrite(buf, 1, strlen(buf), dbgf);
    if(len != strlen(buf)) {
      throw VLRException(err_str_write + ml_rndf_name);
    }
  }
  fclose(dbgf);
  dbgf=NULL;

  if(!(dbgf = fopen(ml_raw_name.c_str(), "w"))) {
    throw VLRException(err_str_open + ml_raw_name);
  }

  for (std::vector<CurvePoint>::const_iterator it = sampled_mission_points_.begin(); it
      != sampled_mission_points_.end(); it++) {
    sprintf(buf, "%lf %lf %lf %lf %lf %lf\n", (*it).s, (*it).x, (*it).y, (*it).theta, (*it).kappa, (*it).kappa_prime);
    size_t len = fwrite(buf, 1, strlen(buf), dbgf);
    if(len != strlen(buf)) {
      throw VLRException(err_str_write + ml_raw_name);
    }
  }
  fclose(dbgf);
}

double ChsmPlanner::replanDistance(double velocity) {
std::map<double, double>::const_iterator rpit = replan_distance_map_.lower_bound(velocity);
if(rpit != replan_distance_map_.end()) {
  return rpit->second;
}

return (*(--replan_distance_map_.end())).second;
}

const uint32_t ChsmPlanner::pose_queue_size_ = 2000; // buffer 10s @ current applanix rate (or equivalent of localizer rate)

} // namespace vlr
