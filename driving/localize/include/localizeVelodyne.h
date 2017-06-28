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


#ifndef VLR_LOCALIZE_VELODYNE_H_
#define VLR_LOCALIZE_VELODYNE_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <applanix/ApplanixPose.h>
#include <localize/LocalizePose.h>
#include <PoseQueue.h>
#include <gls.h>
#include <grid.h>
#include <laserMap.h>

// filter parameters
#define GPS_ERR 1.5
#define SENSOR_WEIGHT 0.005
#define MEASUREMENT_WEIGHT 0.1
#define MOTION_NOISE 0.1

#define SHOW_GLS 1
#define GRID_RADIUS 16


#define DIM 500
#define RES .15
#define DEBUG 0
#define PUBLISH_INTERVAL 0.1
#define LOCALIZE_DISTANCE_INTERVAL 0.2
#define LOCALIZE_TIME_INTERVAL 0.2
#define SMOOTHNESS 0.9

#define UTM_ZONE "10S"

namespace vlr {

class VelodyneLocalizer {
public:
  VelodyneLocalizer();
  virtual ~VelodyneLocalizer();

  void* velodyneThread();
  void* applanixThread();
  void* localizeThread();
  void run();

public:
  typedef struct {
    TerrainTile* tile;
  } grid_cell_t;

private:
  typedef struct {
    double intensity_sum;
    int32_t intensity_count;
    int32_t n;
    float xa;
    float sxi2;
    float stdev;
  } map_cell_t;

private:
  template <class T> void getParam(std::string key, T& var);
  void getParamTransform(std::string key, dgc::dgc_transform_t& t);
  void readParameters();
  void initColors();
  void initGaussians();
  void initializeGrid();
  void initializeTileGrid();
  double normalProbability(double a, double b);
  void plot2D();
  void normalizePosterior();
  void motionModel(double speed);
  void initializePosterior();
  void precomputePointers();
  float correlation(float* x, float* y, int32_t n);
  double alignStrength(int32_t dx, int32_t dy);
  void localize();
  void computeBestOffset();
  void publishPoseMessage(localize::LocalizePose& pose);
  void publish();
  void applanixPoseHandler(const applanix::ApplanixPose& applanix_pose);
  void trackApplanix();

private:
  ros::NodeHandle nh_;
  ros::Subscriber applanix_sub_;
  ros::Publisher localize_pose_pub_;
  tf::TransformListener tf_listener_;

  FILE* outlog_;
  GLS gls_;

  bool quit_localizer_;
  pthread_t velodyne_thread_id_;
  pthread_t applanix_thread_id_;
  pthread_t localize_thread_id_;

  dgc::dgc_grid_t* grid_;
  bool grid_updated_;

  float gaussian_weights_[100][40];
  double hist_z0_[GRID_RADIUS * 3][GRID_RADIUS * 3];
  double hist_z_[GRID_RADIUS * 3][GRID_RADIUS * 3];
  double posterior_[GRID_RADIUS * 3][GRID_RADIUS * 3];
  double posterior1_[GRID_RADIUS * 3][GRID_RADIUS * 3];
  bool has_localized_;
  double timestamp_;
  double last_x_offset_, last_y_offset_;
  double localize_x_offset_, localize_y_offset_;
  applanix::ApplanixPose applanix_pose_;
  double utm_x_, utm_y_;//smooth_x_, smooth_y_, smooth_z_, yaw_;
//  double speed_;
  std::string utm_zone_;

  dgc::dgc_grid_t* tile_grid_;

  float r_[101], g_[101], b_[101];

  pthread_mutex_t posterior_mutex_;
  pthread_mutex_t velodyne_mutex_;
  pthread_mutex_t applanix_pose_mutex_;
  bool received_applanix_pose_;
  driving_common::PoseQueue applanix_pose_queue_;

    // parameters
  std::string imagery_root_;
  std::string int_filename_;
  std::string cal_filename_;
  bool calibrate_intensities_;
  dgc::dgc_transform_t velodyne_offset_;

  float* diff_array_;
  float* stdev_array_;

  //map_cell_t sensor_data_[DIM * DIM];
  map_cell_t* sensor_data_;
  TerrainTileCell** map_data_;
};

} // namespace vlr

#endif
