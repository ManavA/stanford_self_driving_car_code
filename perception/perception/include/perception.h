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


#ifndef PERCEPTION_H_
#define PERCEPTION_H_

#define USE_GRID_SEGMENTER

#ifndef USE_GRID_SEGMENTER
#define  USE_LASER_SEGMENTER
#else
#undef  USE_LASER_SEGMENTER
#endif

#undef  USE_LASER_SEGMENTER

#include <stdint.h>
#include <limits.h>
#include <tr1/memory>
#include <unordered_set>
#include <unordered_map>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <global.h>
#include <PoseQueue.h>
#include <applanix/ApplanixPose.h>
#include <localize/LocalizePose.h>
#include <driving_common/Heartbeat.h>
#include <driving_common/RadarSensor.h>
#include <driving_common/RadarSensorLRR3.h>
#include <perception/PerceptionObstacles.h>
#include <perception/VirtualScan.h>
#include <driving_common/RadarSensor.h>
#include <driving_common/RadarSensorLRR3.h>
#include <velodyne.h>
#include <velo_support.h>

#include <grid.h>
#include <passat_constants.h>
#include <gls.h>
#include <aw_roadNetwork.h>

#include <obstacle.h>
#include <tracker.h>
#include <tracked_obstacle.h>

#include <perception_defines.h>
#include <perception_types.h>

#include <multibooster_support2.h>
#include <veloClient.h>
#include <pointCloudClient.h>

namespace perception {

void grid_line(ivec2_t start, ivec2_t end, grid_line_p line);

class Perception {
public:
  typedef std::unordered_multimap<uintptr_t, uintptr_t> map_type;

public:
  Perception(const std::string&);
  virtual ~Perception();

  void run();
  int shutdown() {return shutdown_;}
  void setShutdown(int shutdown) {shutdown_ = shutdown;}
  pthread_mutex_t& shutdownMutex() {return shutdown_mutex;}

  driving_common::GlobalPose pose(double timestamp) {
    boost::lock_guard<boost::mutex> lock(pose_mutex_);
    return pose_queue_.pose(timestamp);
  }

  void initBooster(const std::string& multibooster_data_file);
  void integrateSensors(const std::vector<velodyne::Block>& scans);
  void integrateSensors(const pcl::PointCloud<pcl::PointXYZI>& cloud);

  uint16_t counter_diff(uint16_t last, uint16_t now);
  uint16_t counter() const {return counter_;}

  void publish();

  void publishObstacles(const Grid<PerceptionCell>& grid, dgc_perception_map_cells_t* points, std::vector<boost::shared_ptr<TrackedObstacle> > obstacles,
      uint16_t counter);

  void freeSpaceRayTracing(uint16_t counter);

//  void set_cell_min(PerceptionCell* cell, float z, uint16_t counter);
//  void set_cell_max(PerceptionCell* cell, float z, uint16_t counter);
  void sync_with_terrain(PerceptionCell* t, PerceptionCell* c);

  void * integration_thread(void * ptr __attribute__ ((unused)) );
  void * tracking_thread(void * ptr __attribute__ ((unused)) );

  void dgc_transform_integrate_pose(dgc::dgc_transform_t t, dgc::dgc_pose_t pose);

  double pose_dist(applanix::ApplanixPose *p1, applanix::ApplanixPose *p2);


  MultiBooster* multiBooster() {return booster;}
  ClassifierPipeline* classifierPipeline() const {return classifier_pipeline_;}
  std::vector<boost::shared_ptr<Obstacle> >& segmentedObstacles() {return obstacles_segmented;}
  std::vector<boost::shared_ptr<TrackedObstacle> >& predictedObstacles() {return obstacles_predicted;}
  std::vector<boost::shared_ptr<TrackedObstacle> >& trackedObstacles() {return obstacles_tracked;}
  std::vector<boost::shared_ptr<TrackedObstacle> >& trackedPublishedObstacles() {return obstacles_tracked_publish;}

  dgc_perception_map_cells_t* staticObstacles() const {return obstacles_s;}
  const Grid<PerceptionCell>& grid() {return *grid_;}
  const Grid<ZCell>& zGrid() {return *z_grid_;}
  map_type& cellToPoints() {return cell_to_points_;}
  const segmentation_settings_t& segmentationSettings() const {return settings_.segmentation_settings;}
  const perception_settings_t settings() const {return settings_;}

private:
  template<class T> void getParam(std::string key, T& var);
  void getParamTransform(std::string key, dgc::dgc_transform_t& t);
  void readParameters();

  void perceptionThread();
  void prepareObstacles(const Grid<PerceptionCell>& grid, dgc_perception_map_cells_t* points, std::vector<boost::shared_ptr<TrackedObstacle> > obstacles, uint16_t counter);
  void applanixHandler(const applanix::ApplanixPose& pose);
  void localizePoseHandler(const localize::LocalizePose& pose);
  void updatePose(const applanix::ApplanixPose& applanix_pose, const localize::LocalizePose& localize_pose);

private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  ros::Publisher perception_obstacles_pub_;
  ros::Publisher heartbeat_pub_;
  ros::Subscriber applanix_sub_;
  ros::Subscriber localize_sub_;
  ros::Subscriber radar1_sub_, radar2_sub_, radar3_sub_;
  ros::Subscriber radar4_sub_, radar5_sub_, radar6_sub_;

  boost::thread* perception_thread_;
  pthread_mutex_t shutdown_mutex;
  pthread_mutex_t applanix_mutex;
  boost::mutex pose_mutex_;
  pthread_mutex_t integration_mutex;
  pthread_mutex_t tracking_mutex;
  pthread_mutex_t publish_mutex;
  pthread_mutex_t velodyne_mutex;
  pthread_mutex_t radar_mutex[NUM_RADARS];
  pthread_mutex_t fsm_mutex;
  //  vlr::PerceptionObstacleList     dynamic_msg;
  pthread_mutex_t dynamic_msg_mutex;
  pthread_mutex_t dynamic_semaphore_mutex;
  pthread_mutex_t virtual_scan_mutex;
  pthread_cond_t virtual_tracking_cond;

  driving_common::PoseQueue pose_queue_;
  bool received_applanix_pose_;
  applanix::ApplanixPose applanix_pose_msg_;
  bool received_localize_pose_;
  localize::LocalizePose localize_pose_msg_;

  perception_settings_t settings_;

  int shutdown_;
  bool data_ready_to_publish;
  vlr::GLS gls;

  driving_common::Heartbeat heartbeat_;

  driving_common::RadarSensor radar_lrr2[NUM_LRR2_RADARS];
  driving_common::RadarSensorLRR3 radar_lrr3[NUM_LRR3_RADARS];
  dgc::dgc_transform_t radar_offset[NUM_RADARS];


  VelodyneClient* velo_client_;
  PointCloudClient* point_cloud_client_;
  std::vector<velodyne::Block> velo_blocks_;
  pcl::PointCloud<pcl::PointXYZI> packet_;
  double scan_resolution;
  uint16_t counter_;
  int velodyne_ctr;
  int virtual_scan_counter;
  perception::VirtualScan virtual_scan[NUM_VIRTUAL_SCANS];

  perception::PerceptionObstacles msg_;
  Grid<PerceptionCell>* grid_;
  Grid<PerceptionCell>* grid_publish_;
  Grid<ZCell>* z_grid_;
  grid_stat_t grid_stat;
  bool send_map_reset;

  std::vector<boost::shared_ptr<Obstacle> > obstacles_segmented;
  std::vector<boost::shared_ptr<TrackedObstacle> > obstacles_predicted;
  std::vector<boost::shared_ptr<TrackedObstacle> > obstacles_tracked;
  std::vector<boost::shared_ptr<TrackedObstacle> > obstacles_tracked_publish;


  dgc_perception_map_cells_t* obstacles_s;
  dgc_perception_map_cells_t obstacles_s_publish;
  dgc_perception_map_cells_t* map_s;

  std::vector<dgc_perception_map_region_p> regions;

  bool publish_ready;
  bool dynamic_msg_ready;

  MultiBooster* booster;
  ClassifierPipeline* classifier_pipeline_;

  dgc_global_pose_t global_pos; // TODO: remove
  std::string rndf_filename;
  Tracker* tracker_;

  uint32_t img_id;
  double last_update_;
  double last_integration_time_;

  #ifdef USE_GRID_SEGMENTER
  map_type cell_to_points_;
  #endif
};

} // namespace perception

#endif
