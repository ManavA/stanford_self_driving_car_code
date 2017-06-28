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
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vlrException.h>
#include <obstacle_types.h>
#include <perception/PerceptionObstacles.h>

#include <aw_ChsmPlanner.hpp>
//#include <aw_SituationInterpretation.hpp>
#include <collisionCheck.h>
#include <obstaclePrediction.h>


namespace drc = driving_common;

namespace vlr {

#undef TRACE
#define TRACE(str) std::cout << "[ChsmPlanner] "<< str << std::endl

using namespace std;

void ChsmPlanner::updateStaticObstacleMapSize(double new_width, double new_height, double new_resolution) {
  pthread_mutex_lock(&static_obstacle_map_mutex_);
  pthread_mutex_lock(&static_obstacle_points_mutex_);
  if(static_obstacle_map_raw_) {delete[] static_obstacle_map_raw_;}
  if(static_obstacle_map_tmp_) {delete[] static_obstacle_map_tmp_;}
  if(static_obstacle_map_) {delete[] static_obstacle_map_;}

  static_obstacle_map_width_  = int(new_width  / new_resolution + .5);
  static_obstacle_map_height_ = int(new_height / new_resolution + .5);
  static_obstacle_map_res_    = new_resolution;
  static_obstacle_map_raw_    = new uint8_t[static_obstacle_map_width_ * static_obstacle_map_height_];
  static_obstacle_map_tmp_    = new float[static_obstacle_map_width_ * static_obstacle_map_height_];
  static_obstacle_map_        = new uint8_t[static_obstacle_map_width_ * static_obstacle_map_height_];
  memset(static_obstacle_map_raw_, 0, static_obstacle_map_width_ * static_obstacle_map_height_ * sizeof(uint8_t));
  memset(static_obstacle_map_tmp_, 0, static_obstacle_map_width_ * static_obstacle_map_height_ * sizeof(float));
  memset(static_obstacle_map_, 0, static_obstacle_map_width_ * static_obstacle_map_height_ * sizeof(uint8_t));

  double rc = traj_eval_->params().safety_width / 2;
  static_obstacle_map_cs_mask_size_ = uint32_t(rc / static_obstacle_map_res_ + .5);

  pthread_mutex_unlock(&static_obstacle_points_mutex_);
  pthread_mutex_unlock(&static_obstacle_map_mutex_);
}

void ChsmPlanner::updateStaticObstacleMap(const std::vector<perception::StaticObstaclePoint>& points, double timestamp) {

  if(emergency_map_clear_) {return;}

  pthread_mutex_lock(&static_obstacle_points_mutex_);
  if(points.size()>0) {
    static_obstacle_points_ = points;
  }
  static_obstacle_timestamp_ = timestamp;

  new_static_obstacle_map_ready_=true;
  pthread_mutex_lock(&static_obstacle_map_cycle_mutex_);
  pthread_cond_signal(&static_obstacle_map_cycle_cv_);
  pthread_mutex_unlock(&static_obstacle_map_cycle_mutex_);

  pthread_mutex_unlock(&static_obstacle_points_mutex_);
}

void* ChsmPlanner::updateStaticObstacleMapThread(void*) {
  while (!quit_chsm_planner_) {

    pthread_mutex_lock(&static_obstacle_map_cycle_mutex_);
    pthread_cond_wait(&static_obstacle_map_cycle_cv_, &static_obstacle_map_cycle_mutex_);
    pthread_mutex_unlock(&static_obstacle_map_cycle_mutex_);

    double dt = drc::Time::current();

    if(quit_chsm_planner_) {return NULL;}

    double cx, cy, cx_utm, cy_utm;
    pthread_mutex_lock(&pose_mutex_);
    try {
    pose_queue_.xyAndUtmXY(static_obstacle_timestamp_, cx, cy, cx_utm, cy_utm);
    }
    catch(vlr::Ex<>& e) {
      pthread_mutex_unlock(&pose_mutex_);
      std::cout << "Warning: Updating static obstacle map failed because position could not be determined: " << e.what() << std::endl;
      continue;
    }
    pthread_mutex_unlock(&pose_mutex_);


    pthread_mutex_lock(&static_obstacle_map_mutex_);
    pthread_mutex_lock(&static_obstacle_points_mutex_);
    staticObstacleMessage2Map(cx, cy);
    new_static_obstacle_map_ready_ = false;
    pthread_mutex_unlock(&static_obstacle_points_mutex_);

    cv::Mat_<uint8_t> mat_raw(static_obstacle_map_height_, static_obstacle_map_width_, static_obstacle_map_raw_);
    cv::Mat_<float> mat_dist(static_obstacle_map_height_, static_obstacle_map_width_);
    cv::distanceTransform(mat_raw, mat_dist, CV_DIST_L2, CV_DIST_MASK_PRECISE);

    float* dist_data = (float*)mat_dist.data;
    uint8_t* data = static_obstacle_map_;
    uint8_t* data_end = static_obstacle_map_ + static_obstacle_map_width_*static_obstacle_map_height_;
    float dist_thresh_ = static_obstacle_map_cs_mask_size_;
    float dist_maybe_thresh_ = static_obstacle_map_cs_mask_size_ + 5;

    for(; data !=data_end; data++, dist_data++) {
      if(*dist_data > dist_maybe_thresh_) {
        *data=OSM_FREE;
      }
      else if(*dist_data > dist_thresh_) {
        *data=OSM_MAYBE_BLOCKED;
      }
      else {
        *data=OSM_BLOCKED;
      }
    }

    traj_eval_->setStaticObstacleMap(static_obstacle_map_, static_obstacle_map_width_, static_obstacle_map_height_,
                                    static_obstacle_map_res_, cx_utm, cy_utm, static_obstacle_timestamp_);

//    char imagename[100];
//    static uint32_t frame_num=0;
//    sprintf(imagename, "static_map%04d.png", frame_num);
//    printf("%s\n", imagename);
//    cv::Mat_<uint8_t> mat_res(static_obstacle_map_height_, static_obstacle_map_width_, static_obstacle_map_);
//    cv::imwrite(imagename, mat_res);
//    frame_num++;
//    printf("obstacle map update time: %f (mask size: %u)\n", drc::Time::current() - dt, static_obstacle_map_cs_mask_size_);
    pthread_mutex_unlock(&static_obstacle_map_mutex_);
  }

  return NULL;
}

void ChsmPlanner::updateVehicles(const std::vector<perception::DynamicObstacle>& dyn_obstacles, double t0) {
  double offset_x, offset_y;
  pthread_mutex_lock(&pose_mutex_);
  try {
    pose_queue_.offsets(t0, offset_x, offset_y);
  }
  catch(vlr::Ex<>& e) {
    pthread_mutex_unlock(&pose_mutex_);
    std::cout << "Warning: Updating dynamic objects (e.g. vehicles) failed because position could not be determined: " << e.what() << std::endl;
    return;
  }
  pthread_mutex_unlock(&pose_mutex_);

  pthread_mutex_lock(&dyn_obstacles_mutex_);

  pthread_mutex_lock(&topology_mutex_);

  // update vehicles in vehicle manager
  vehicle_manager->updateVehicles(dyn_obstacles, offset_x, offset_y, t0, traj_eval_->params().checked_horizon, traj_eval_->params().time_sample_res);

  // update blockages
  vehicle_manager->updateBlockages();

  predicted_pedestrians_.clear();

  pthread_mutex_unlock(&topology_mutex_);

  pthread_mutex_lock(&pedestrian_prediction_mutex_);
  MovingBox mb;
  ObstaclePrediction op;
  for(uint32_t i=0; i<dyn_obstacles.size(); i++) {
    if(dyn_obstacles[i].type != OBSTACLE_PEDESTRIAN) {continue;}
    double x_step = dyn_obstacles[i].velocity * cos(dyn_obstacles[i].direction);
    double y_step = dyn_obstacles[i].velocity * sin(dyn_obstacles[i].direction);

    mb.length       = dyn_obstacles[i].length;
    mb.width        = dyn_obstacles[i].width;
    mb.ref_offset   = dyn_obstacles[i].length*0.5;
    for(double t=0; t<traj_eval_->params().checked_horizon; t+=traj_eval_->params().time_sample_res) {
      mb.t            = t0 + t;
      mb.x            = dyn_obstacles[i].x + t*x_step + offset_x;
      mb.y            = dyn_obstacles[i].y + t*y_step + offset_y;
//      mb.psi          = (dyn_obstacles[i].velocity >=0 ? dyn_obstacles[i].direction : dyn_obstacles[i].direction-M_PI); // TODO: velocity is not directly part of prediction
      mb.psi          = atan2(y_step, x_step);
      op.predicted_traj_.push_back(mb);
    }

    predicted_pedestrians_.push_back(op);
    op.predicted_traj_.clear();
  }
  pthread_mutex_unlock(&pedestrian_prediction_mutex_);
  //  std::map<int, Vehicle>::const_iterator vit = vehicle_manager->vehicle_map.begin(),  vit_end = vehicle_manager->vehicle_map.end();
  //  for(; vit!=vit_end; vit++) {
  //      const Vehicle& v = (*vit).second;
  //      printf(" Found vehicle (w: %f, l: %f) @ (%f, %f):\n matched on lane %s (%f, %f; dist from start: %f\n",
  //              v.width(), v.length(), v.xMatchedFrom(), v.yMatchedFrom(), v.edge()->name().c_str(), v.xMatched(), v.yMatched(), v.distFromStart());
  //      if(!v.edge()->leftEdges.empty()) {
  //          TRndfEdgeSet::const_iterator eit=v.edge()->leftEdges.begin(), eit_end=v.edge()->leftEdges.end();
  //          for(; eit!=eit_end; eit++){
  //              printf("lane has left neighbor %s\n", (*eit)->name().c_str());
  //          }
  //      }
  //      if(!v.edge()->rightEdges.empty()) {
  //          TRndfEdgeSet::const_iterator eit=v.edge()->rightEdges.begin(), eit_end=v.edge()->rightEdges.end();
  //          for(; eit!=eit_end; eit++){
  //              printf("lane has right neighbor %s\n", (*eit)->name().c_str());
  //          }
  //      }
  //      if(!v.edge()->leftOppositeEdges.empty()) {
  //          TRndfEdgeSet::const_iterator eit=v.edge()->leftOppositeEdges.begin(), eit_end=v.edge()->leftOppositeEdges.end();
  //          for(; eit!=eit_end; eit++){
  //              printf("lane has oncoming neighbor %s\n", (*eit)->name().c_str());
  //          }
  //      }
  //  }

  //  std::map<int, Vehicle>::const_iterator vit =
  //      vehicle_manager->vehicle_map.begin(), vit_end = vehicle_manager->vehicle_map.end();
  //  for(; vit != vit_end; vit++) {
  //    const Vehicle& veh = (*vit).second;
  //    predictor_->predict(veh);
  //  }

  pthread_mutex_unlock(&dyn_obstacles_mutex_);


  // interpret the current overall situation and estimate the object behaviors and trajectories
  pthread_mutex_lock(&topology_mutex_);

//  situation_interpretation_->estimate( t0 );

  pthread_mutex_unlock(&topology_mutex_);
}

  RoutePlanner::RndfEdge* ChsmPlanner::bestPredictedEdge(const std::map<RoutePlanner::RndfEdge*, double>& edges, double& dist_to_end) {
    if(edges.empty()) {return NULL;}
    std::map<RoutePlanner::RndfEdge*, double>::const_iterator eit = edges.begin(), eit_end = edges.end();
    for(; eit!=eit_end; eit++) {
      if(!(*eit).first->isVirtualEdge()) {
        dist_to_end = (*eit).first->length() - (*eit).second;
        return eit->first;
      }
    }

      // only virtual edges available, pick first one
    dist_to_end = (*edges.begin()).first->length() - (*edges.begin()).second;
    return edges.begin()->first;
  }

  RoutePlanner::RndfEdge* ChsmPlanner::bestPredictedEdge(const RoutePlanner::TRndfEdgeSet& edges) {
  if(edges.empty()) {return NULL;}
  RoutePlanner::TRndfEdgeSet::const_iterator eit = edges.begin(), eit_end = edges.end();
  for(; eit!=eit_end; eit++) {
    if(!(*eit)->isVirtualEdge()) {return *eit;}
  }

    // only virtual edges available, pick first one
  return *edges.begin();
}

void ChsmPlanner::detectBlockades() {
  printf("%s: reimplement...\n", __FUNCTION__);
//  const float thres_critical_zdif = 0.3;
//  const double borderDist = 1.8;
//  const int thres_cirticalN = 14;
//  const double back_sampl_dist = 0.0;
//  const double length = 30.0;
//  const int point_anz = 20;
//  int current_curvepoint=0;
//  double latDist, lonDist, lonOffset;
//  BlockadeInfo blockade_info;
//  bool valid;
//  CurvePoints sampled_curvepoints;
//  static CurvePoints stripped_curvepoints[1000];
//  std::vector<BlockadeInfo> blockade_info_list;
//  std::vector<CurvePoints*> curvepoints_list;
//  std::vector<RndfEdge*> edge_list;
//
//  if( !centered_obstacle_map ) return;
//  dgc_pose_t pose = centered_obstacle_map->robot_pose;
//
//  blockade_info.thres_critical_zdif = thres_critical_zdif;
//  blockade_info.borderDist = borderDist;
//  blockade_info.thres_cirticalN = thres_cirticalN;
//
//  // get current edge
//  RndfEdge* current_edge = (*topology->current_edge_it)->edge();
//
//  // get opposite edge
//  const TRndfEdgeSet& opposite_edges = current_edge->leftOncomingEdges();
//  if(opposite_edges.size()==0) return;
//  RndfEdge* opposite_edge = (*opposite_edges.begin());
//
//  // calculate offset to opposite edge
//  RndfVertex* v = opposite_edge->fromVertex();
//  current_edge->computeDistance(v->x(), v->y(), latDist, lonDist, lonOffset);
//  double sample_offset = fabs(latDist)/2.0;
//
//  // sample curvepoints in the middle between current edge and opposite edge
//  valid = route_sampler->samplePoints(&sampled_curvepoints, back_sampl_dist, length, point_anz, sample_offset);
//
//  if(valid && sampled_curvepoints.numberValidPoints > 2) {
//    // strip curvepoints for current lane
//    for(int i=0;i<sampled_curvepoints.numberValidPoints-2;++i) {
//      CurvePoints* cp = &stripped_curvepoints[current_curvepoint];
//      cp->numberValidPoints=3;
//      cp->curvepoints[0] = sampled_curvepoints.curvepoints[i];
//      cp->curvepoints[1] = sampled_curvepoints.curvepoints[i+1];
//      cp->curvepoints[2] = sampled_curvepoints.curvepoints[i+2];
//      blockade_info_list.push_back(blockade_info);
//      curvepoints_list.push_back(cp);
//      current_curvepoint++;
//    }
//  }
//
//  // do we have valid curvepoints?
//  if (curvepoints_list.size() == 0) return;
//
////  // check blockages
////  pTMaster->checkForBlockades(blockade_info_list, pose.x, pose.y, pose.yaw, curvepoints_list,centered_obstacle_map);
//
//  // add blocked edges to blockage manager
//  for(int i=0;i<current_curvepoint;++i) {
//    //fprintf(stderr,"[BM] id %d na %d av %f \n",i,blockade_info_list[i].nAboveThreshold,blockade_info_list[i].averageValue);
//
//    if(blockade_info_list[i].bVerificationWasSuccessfull &&
//       blockade_info_list[i].bBlocked) {
//      //fprintf(stderr,"[BM] blocked!!!!\n",blockade_info_list[i].averageValue);
//      CurvePoints* cp = curvepoints_list[i];
//      // get edge for blocked curvepoints
//      double x1 = cp->curvepoints[1].x;
//      double y1 = cp->curvepoints[1].y;
//      RndfEdge* edge = topology->complete_graph->findClosestEdge(x1,y1);
//      if(!edge) continue;
//      if(!edge->isLaneEdge()) continue;
//
//      // check if there is a vehicle on this edge
//      std::map<int, Vehicle>& vehicle_map = vehicle_manager->vehicle_map;
//      std::map<int, Vehicle>::iterator itv, itv_end;
//      bool vehicle_close = false;
//      for(itv=vehicle_map.begin(),itv_end=vehicle_map.end();itv!=itv_end;++itv) {
//        Vehicle& vehicle = itv->second;
//        double dist = hypot(x1-vehicle.x_matched_from,y1-vehicle.y_matched_from);
//
//        if(edge==vehicle.edge || dist < 10.0) {
//          vehicle_close = true;
//          break;
//        }
//      }
//
//      if(vehicle_close) continue;
//
//      // this lane is blocked -> check if blockade already exists
//      BlockadeManager::Blockade* blockade = blockade_manager->getBlockadeByEdge(edge);
//      if(blockade) {
//        // update blockade
//        blockade->update();
//      }
//      else {
//        // add new blockade
//        blockade_manager->addBlockade(edge);
//      }
//
//      // update left opposites edges
//      if(edge->leftOncomingEdges().size()>0) {
//      RndfEdge* edge2 = (*edge->leftOncomingEdges().begin());
//      BlockadeManager::Blockade* blockade = blockade_manager->getBlockadeByEdge(edge2);
//        if(blockade) {
//          // update blockade
//          blockade->update();
//        }
//        else {
//          // add new blockade
//          blockade_manager->addBlockade(edge2);
//        }
//      }
//    }
//  }
}

void ChsmPlanner::updateBlockades() {

  // detect new blockades
  if(!inPause && !bIntersection && params().enable_blockade_detection) detectBlockades();
  // update blockade manager
  blockade_manager->update();
}

void ChsmPlanner::clearBlockages()
{
  topology_->complete_graph->clearBlockades();
}

void ChsmPlanner::staticObstacleMessage2Map(double cx, double cy) {
  memset(static_obstacle_map_raw_, 255, static_obstacle_map_width_ * static_obstacle_map_height_ * sizeof(uint8_t));

  static_obstacle_map_cx_ = cx;
  static_obstacle_map_cy_ = cy;

  for (uint32_t i = 0; i < uint32_t(static_obstacle_points_.size()); i++) {
    int32_t xi = static_obstacle_map_width_ / 2 - (int32_t) ((static_obstacle_points_[i].x - cx) / static_obstacle_map_res_+.5);
    int32_t yi = static_obstacle_map_height_ / 2 - (int32_t) ((static_obstacle_points_[i].y - cy) / static_obstacle_map_res_+.5);
    if (xi >= 0 && xi < int32_t(static_obstacle_map_width_) && yi >= 0 && yi < int32_t(static_obstacle_map_height_)) {
      uint8_t val = static_obstacle_points_[i].type;

      //        if (val == PERCEPTION_MAP_OBSTACLE_LOW || val == PERCEPTION_MAP_OBSTACLE_HIGH || val
      //            == PERCEPTION_MAP_OBSTACLE_UNKNOWN) { // val == PERCEPTION_MAP_OBSTACLE_DYNAMIC) {
//      if (val == PERCEPTION_MAP_OBSTACLE_FREE)
      if(val != perception::StaticObstaclePoint::DYNAMIC)
      {
        val = 0;
        static_obstacle_map_raw_[yi * static_obstacle_map_width_ + xi] = val;
      }
    }
  }
}

} // namespace vlr
