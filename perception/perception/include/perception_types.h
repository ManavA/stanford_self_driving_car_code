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


#ifndef PERCEPTION_TYPES_H_
#define PERCEPTION_TYPES_H_

#include <global.h>
#include <applanix/ApplanixPose.h>
#include <perception/PerceptionObstacles.h>
#include <velodyne/ScanPoint.h>
#include <grid.h>
#include <velodyne.h>
#include <perception_defines.h>
#include <obstacle_types.h>

namespace perception {

typedef struct {
  double                                x;
  double                                y;
} point2d_t;

typedef struct {
  double                                x;
  double                                y;
  double                                z;
  double                                intensity;
} point3d_t;

typedef struct {
  int                                   x;
  int                                   y;
} ipoint2d_t;

typedef struct {
  point2d_t                             center;
  ipoint2d_t                            mapsize;
  float                                 resolution;
  float                                 z_resolution;
} grid_stat_t;

typedef struct {
  double                                min_x;
  double                                max_x;
  double                                min_y;
  double                                max_y;
} bounding_box2d_t;

typedef struct {
  float             x1;
  float             x2;
  float             y1;
  float             y2;
  float             min_angle;
  float             max_angle;
} restricted_2d_area_t;

typedef struct {
  std::vector<velodyne::Block> blocks;
  velodyne::Config*      config;
  int                    preprocessed;
} VelodyneData;

class ZCell : public CellBase<ZCell> {
public:
  ZCell() : z_value(std::numeric_limits<int16_t>::max()) {
  }
  int16_t z_value;
};

class PerceptionCell : public CellBase<PerceptionCell> {
 public:
  PerceptionCell() : hits(0), seen(0), obstacle(perception::StaticObstaclePoint::FREE),
                     min(FLT_MAX), max(-FLT_MAX), street(1), region(0),
                     last_obstacle(0), last_observed(0), last_dynamic(0),
                     last_mod(0), last_min(0), last_max(0) {
  }
  uint16_t          hits;
  uint16_t          seen;

  uint8_t           obstacle;
  float                   min;
  float                   max;
  uint8_t           street;

  uint16_t          region; // region in segmentation

  uint16_t          last_obstacle; // counter when cell was last observed as an obstacle
  uint16_t          last_observed; // counter when cell was last observed
  uint16_t          last_dynamic;  // counter when cell was last observed as dynamic

  /* counter compare modified for the map diff list */
  uint16_t          last_mod;

  uint16_t          last_min;
  uint16_t          last_max;
};

typedef struct {

  float                   min;
  float                   max;
  unsigned short          last_min;
  unsigned short          last_max;

} terrain_map_cell_t, *terrain_map_cell_p;

// a grid region
typedef struct {
  int id;
  int num_points;
  std::vector<PerceptionCell*> cells;
} dgc_perception_map_region_t, *dgc_perception_map_region_p;

typedef struct {
  double                 x;
  double                 y;
  double                 z;
  double                 yaw;
  double                 roll;
  double                 pitch;
  std::string            utmzone;
} dgc_global_pose_t;

typedef struct {
  int                            num;
  PerceptionCell*    * cell;
  double                         timestamp;
} dgc_perception_map_cells_t, *dgc_perception_map_cells_p;

typedef struct {
//  const velodyne::Block* block;
  geometry_msgs::Point32 point;
//  const geometry_msgs::Point32* point;
  uint16_t                      range;
  uint8_t                       intensity;
  ZCell* z_cell;
  int32_t encoder;
  uint8_t obstacle;
  uint8_t valid;
  double                        timestamp;
} laser_point_t, *laser_point_p;

typedef struct {
  int num_points;
  laser_point_t* laser_point;
  driving_common::Pose robot;
} laser_scan_t, *laser_scan_p;

typedef std::vector<velodyne::ScanPoint*> laser_cluster_t;
typedef laser_cluster_t* laser_cluster_p;

typedef struct {
  int         x;
  int         y;
} ivec2_t, *ivec2_p;

typedef struct {
  int         max;
  int         numgrids;
  ivec2_p     grid;
} grid_line_t, *grid_line_p;

typedef enum {
  SENSOR_RADAR1,
  SENSOR_RADAR2,
  SENSOR_RADAR3,
  SENSOR_RADAR4,
  SENSOR_RADAR5,
  SENSOR_RADAR6,
  SENSOR_VELODYNE
} dgc_sensor_type;

typedef enum {
  TRACKING_STATE_INITIALIZED,
  TRACKING_STATE_TRACKING,
  TRACKING_STATE_LOST,
  TRACKING_STATE_SEGMENTED,
  TRACKING_STATE_PREDICTED,
  TRACKING_STATE_FILTERED
} dgc_tracking_state;

typedef struct {
  int kernel_size;                    // kernel size for segmentation
  int min_points;                     // min number of points
  int max_points;                     // max number of points
  double min_height;                  // minimum height
  bool gls_output;                     // send visualization to perception_view
} segmentation_settings_t, *segmentation_settings_p;

typedef struct {
  double correspondence_threshold;
  double pruning_threshold;
  double measurement_variance;
  double position_variance;
  double velocity_variance;
  double initial_position_variance;
  double initial_velocity_variance;
} tracker_kf_settings_t, *tracker_kf_settings_p;

typedef struct {
  double merge_dist;
  double lateral_merge_dist;
  double filter_rndf_max_distance;
  double filter_rndf_max_pedestrian_distance;
  bool filter_rndf;
//  double filter_graph_scaler;
//  int    predictive_clustering;
//  double max_car_width;
//  double max_car_length;
  std::string classifier_filename;
} tracker_settings_t, *tracker_settings_p;

typedef struct {
  bool        gls_output;
  bool        show_virtual_scan;
  double      virtual_scan_resolution;

  bool         show_ray_tracing;
  bool         extract_dynamic;

  bool         clear_sensor_data;
  double      max_sensor_delay;

  double      map_resolution;
  double      map_size_x;
  double      map_size_y;
  double      map_cell_threshold;
  int         map_cell_min_hits;
  int         map_cell_increase;
  bool         map_ray_tracing;

  double      z_resolution;
  double      z_obstacle_height;

  bool        use_velodyne;
  bool        use_point_cloud;
  double      velodyne_threshold_factor;
  double      velodyne_max_range;
  double      velodyne_min_beam_diff;
//  bool         velodyne_sync;
  char       *velodyne_cal;

  double      overpass_height;

  double      rate_in_hz;
  double      publish_interval;

  int         num_threads;

  bool         rerun;

  tracker_settings_t        tracker_settings;
  tracker_kf_settings_t     kf_settings;
  segmentation_settings_t   segmentation_settings;
} perception_settings_t, *perception_settings_p;

} // namespace perception

#endif
