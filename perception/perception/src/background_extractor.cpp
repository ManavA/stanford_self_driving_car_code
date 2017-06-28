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


#include "perception.h"
#include "utils.h"
#include <ipc_std_interface.h>
#include <velodyne_shm_interface.h>

#include <gui3D.h>
#include <lltransform.h>
#include <imagery.h>
#include <rndfgl.h>
#include <passatmodel.h>
#include <car_list.h>

#include <multibooster/multibooster.h>
#include <cluster_descriptors/cluster_descriptors.h>
#include "track_manager.h"

using namespace Eigen;
using std::tr1::shared_ptr;
using namespace std;
using namespace dgc;
using namespace vlr;
using namespace vlr::rndf;
using namespace track_manager;

IpcInterface *ipc = NULL;

pthread_mutex_t                 applanix_mutex                = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                 integration_mutex             = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                 publish_mutex                 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t                 ldlrs_mutex[NUM_LDLRS_LASERS] = { PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER };
pthread_mutex_t                 radar_mutex[NUM_RADARS]       = { PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER};
pthread_mutex_t                 fsm_mutex                     = PTHREAD_MUTEX_INITIALIZER;

LdlrsLaser                      ldlrs[NUM_LDLRS_LASERS];
double                          ldlrs_ts[NUM_LDLRS_LASERS];

RadarSensor                     radar_lrr2[NUM_LRR2_RADARS];
RadarLRR3Sensor                 radar_lrr3[NUM_LRR3_RADARS];

vlr::GlsOverlay                 * gls = NULL;
unsigned short                  counter                            = 370;
int                             velodyne_ctr                       = 0;

double                          velodyne_ts  = 0;
double                          last_velodyne_ts  = 0;
VelodyneData             velodyne;

double                          scan_resolution                    = 0;
LocalizePose                    localize_pose = { 0, 0.0, 0.0, "", 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, "" };

PlannerFsmState                 fsm_state;

grid_stat_t                     grid_stat;
dgc_grid_p                      grid;
extern dgc_grid_p               z_grid;

//VelodyneInterface              *velo_interface = NULL;

PerceptionCell*       default_map_cell     = NULL;
PerceptionCell*       default_terrain_cell = NULL;
short                           default_z_cell       = std::numeric_limits<short>::max();

double                          publish_interval;
double                          timer_interval;

char                          * rndf_filename = NULL;
dgc_global_pose_t               global = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "10S"};

RoadNetwork* road_network = NULL;
int rndf_valid = 0;

void rndf_load_file(char *filename) {
  /* load the RNDF file, if available */
  fprintf(stderr, "# INFO: load rndf file\n");
  road_network = new RoadNetwork;

//  rndf = new rndf_file;
  if (road_network->loadRNDF(filename))
    rndf_valid = 1;
}

/****************************************************************************/

perception_settings_t           settings;

MultiBooster* booster = NULL;
ClassifierPipeline* classifier_pipeline_ = NULL;

/****************************************************************************/

/*********************************************************************
 *
 *   Log files
 *
 *********************************************************************/
char vlf_filename[300], index_filename[300];

dgc_velodyne_file_p velodyne_file = NULL;
dgc_velodyne_index velodyne_index;
dgc_velodyne_config_p velodyne_config = NULL;
dgc_velodyne_spin spin, last_spin;

car_list_t cars;

int current_spin_num = 0;

void run_tracker()
{
  VelodyneData v;

  if (current_spin_num > 0) {
    v.config = velodyne_config;
    v.num_scans = last_spin.num_scans;
    v.allocated_scans = last_spin.num_scans;
    v.scans = last_spin.scans;
    v.preprocessed = 0;

    ApplanixPose pose;

    pose.smooth_x =  velodyne_index.spin[current_spin_num-1].pose[0].smooth_x;
    pose.smooth_y =  velodyne_index.spin[current_spin_num-1].pose[0].smooth_y;
    pose.smooth_z =  velodyne_index.spin[current_spin_num-1].pose[0].smooth_z;
    pose.longitude =  velodyne_index.spin[current_spin_num-1].pose[0].longitude;
    pose.latitude =  velodyne_index.spin[current_spin_num-1].pose[0].latitude;
    pose.altitude =  velodyne_index.spin[current_spin_num-1].pose[0].altitude;
    pose.v_east =  velodyne_index.spin[current_spin_num-1].pose[0].v_east;
    pose.v_north =  velodyne_index.spin[current_spin_num-1].pose[0].v_north;
    pose.v_up =  velodyne_index.spin[current_spin_num-1].pose[0].v_up;
    pose.roll =  velodyne_index.spin[current_spin_num-1].pose[0].roll;
    pose.pitch =  velodyne_index.spin[current_spin_num-1].pose[0].pitch;
    pose.yaw =  velodyne_index.spin[current_spin_num-1].pose[0].yaw;
    pose.timestamp = velodyne_index.spin[current_spin_num-1].pose[0].timestamp;

    v.scans->timestamp = pose.timestamp;
    last_velodyne_ts = velodyne_ts;
    velodyne_ts = pose.timestamp;

    localize_pose.x_offset = velodyne_index.spin[current_spin_num-1].pose[0].x_offset;
    localize_pose.y_offset = velodyne_index.spin[current_spin_num-1].pose[0].y_offset;

    if (localize_pose.x_offset == 0.0) { // if the first pose doesn't have a localize offset, try the last pose
      int num_poses = velodyne_index.spin[current_spin_num-1].num_poses;
      if (num_poses > 1) {
        localize_pose.x_offset = velodyne_index.spin[current_spin_num-1].pose[num_poses-1].x_offset;
        localize_pose.y_offset = velodyne_index.spin[current_spin_num-1].pose[num_poses-1].y_offset;
      }
    }

    if (localize_pose.x_offset == 0.0) { // if we still don't have an offset assume applanix is right
      printf("# WARNING: localize message not found in vlf index\n");
      double x,y;
      char utmzone[10];
      latLongToUtm(pose.latitude, pose.longitude, &x, &y, utmzone);
      localize_pose.x_offset = x - pose.smooth_x;
      localize_pose.y_offset = y - pose.smooth_y;
    }

    applanix_history_add( &pose );

    global.x = pose.smooth_x + localize_pose.x_offset;
    global.y = pose.smooth_y + localize_pose.y_offset;

    counter = current_spin_num-1 + 370;

    obstacles_s->num = 0;
    integrate_sensors(&v);
  }

  v.config = velodyne_config;
  v.num_scans = spin.num_scans;
  v.allocated_scans = spin.num_scans;
  v.scans = spin.scans;
  v.preprocessed = 0;

  ApplanixPose pose;

  pose.smooth_x =  velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  pose.smooth_y =  velodyne_index.spin[current_spin_num].pose[0].smooth_y;
  pose.smooth_z =  velodyne_index.spin[current_spin_num].pose[0].smooth_z;
  pose.longitude =  velodyne_index.spin[current_spin_num].pose[0].longitude;
  pose.latitude =  velodyne_index.spin[current_spin_num].pose[0].latitude;
  pose.altitude =  velodyne_index.spin[current_spin_num].pose[0].altitude;
  pose.v_east =  velodyne_index.spin[current_spin_num].pose[0].v_east;
  pose.v_north =  velodyne_index.spin[current_spin_num].pose[0].v_north;
  pose.v_up =  velodyne_index.spin[current_spin_num].pose[0].v_up;
  pose.roll =  velodyne_index.spin[current_spin_num].pose[0].roll;
  pose.pitch =  velodyne_index.spin[current_spin_num].pose[0].pitch;
  pose.yaw =  velodyne_index.spin[current_spin_num].pose[0].yaw;
  pose.timestamp = velodyne_index.spin[current_spin_num].pose[0].timestamp;

  v.scans->timestamp = pose.timestamp;
  last_velodyne_ts = velodyne_ts;
  velodyne_ts = pose.timestamp;

  localize_pose.x_offset = velodyne_index.spin[current_spin_num].pose[0].x_offset;
  localize_pose.y_offset = velodyne_index.spin[current_spin_num].pose[0].y_offset;

  if (localize_pose.x_offset == 0.0) { // if the first pose doesn't have a localize offset, try the last pose
    int num_poses = velodyne_index.spin[current_spin_num].num_poses;
    if (num_poses > 1) {
      localize_pose.x_offset = velodyne_index.spin[current_spin_num].pose[num_poses-1].x_offset;
      localize_pose.y_offset = velodyne_index.spin[current_spin_num].pose[num_poses-1].y_offset;
    }
  }

  if (localize_pose.x_offset == 0.0) { // if we still don't have an offset assume applanix is right
    printf("# WARNING: localize message not found in vlf index\n");
    double x,y;
    char utmzone[10];
    latLongToUtm(pose.latitude, pose.longitude, &x, &y, utmzone);
    localize_pose.x_offset = x - pose.smooth_x;
    localize_pose.y_offset = y - pose.smooth_y;
  }

  applanix_history_add( &pose );

  global.x = pose.smooth_x + localize_pose.x_offset;
  global.y = pose.smooth_y + localize_pose.y_offset;

  counter = current_spin_num + 370;

  obstacles_s->num = 0;
  integrate_sensors(&v);
}


void dgcToEigen(const vector<point3d_t>& points, boost::shared_ptr<MatrixXf> cloud,
		boost::shared_ptr<VectorXf> intensities) {
  *cloud = MatrixXf(points.size(), 3);
  *intensities = VectorXf(points.size());
  for(size_t i = 0; i < points.size(); ++i) {
    cloud->coeffRef(i, 0) = points[i].x;
    cloud->coeffRef(i, 1) = points[i].y;
    cloud->coeffRef(i, 2) = points[i].z;
    intensities->coeffRef(i) = points[i].intensity;
  }
}

boost::shared_ptr<sensor_msgs::PointCloud> dgcToRos(const vector<point3d_t>& dgc) {
  boost::shared_ptr<sensor_msgs::PointCloud> ros(new sensor_msgs::PointCloud());
  ros->set_points_size(dgc.size());
  ros->set_channels_size(1);
  ros->channels[0].set_values_size(dgc.size());
  for(size_t i = 0; i < dgc.size(); ++i) {
    ros->points[i].x = dgc[i].x;
    ros->points[i].y = dgc[i].y;
    ros->points[i].z = dgc[i].z;
    ros->channels[0].values[i] = dgc[i].intensity;
  }
  return ros;
}
 
void collectDataset() {
  int start_frame = 0;
  if(getenv("START_FRAME"))
    start_frame = atoi(getenv("START_FRAME"));
  
  int end_frame = velodyne_index.num_spins;
  if(getenv("END_FRAME"))
    end_frame = atoi(getenv("END_FRAME"));

  int num_threads = 1;
  if(getenv("NUM_THREADS"))
    num_threads = atoi(getenv("NUM_THREADS"));
  DescriptorPipeline dp(num_threads);

  vector<Object*> objects;
  objects.reserve(1e6);
  
  vector< boost::shared_ptr<Track> > tracks;
  for(int i = start_frame; i < end_frame; ++i) {
    cout << "Working on spin " << i << " / " << end_frame << endl;
    double applanix_lat, applanix_lon, applanix_alt;

    // -- Run the tracker forward.
    int delta = 1;
    current_spin_num += delta;
    if(current_spin_num >= velodyne_index.num_spins)
      current_spin_num = velodyne_index.num_spins - 1;
    if(current_spin_num < 0)
      current_spin_num = 0;
      
    if (current_spin_num > start_frame) {
      last_spin.copy(spin);
    } else {
      last_spin.num_scans = 0;
    }
    spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num,
	      &applanix_lat, &applanix_lon, &applanix_alt);
    run_tracker();


    if(obstacles_segmented.empty())
      continue;
    cout << "num segmented objects: " << obstacles_segmented.size() << endl;

    // -- Store clusters for later observation.
    if(getenv("SAVE_CLUSTERS")) { 
      for(size_t j = 0; j < obstacles_segmented.size(); ++j) { 
	vector< boost::shared_ptr<sensor_msgs::PointCloud> > trclouds;
	trclouds.push_back(dgcToRos(obstacles_segmented[j]->getPoints()));
	Track* tr = new Track(trclouds);
	tr->label_ = "background";
    	tracks.push_back(boost::shared_ptr<Track>(tr));
      }
    }
        
    // -- Compute the descriptors.
    vector< boost::shared_ptr<MatrixXf> > clouds;
    vector< boost::shared_ptr<VectorXf> > intensities;
    clouds.reserve(obstacles_segmented.size());
    intensities.reserve(obstacles_segmented.size());
    for (size_t j=0; j < obstacles_segmented.size(); j++) {
      std::vector<point3d_t>& points = obstacles_segmented[j]->getPoints();
      if(points.size() < 5)
	continue;
      boost::shared_ptr<MatrixXf> cloud(new MatrixXf());
      boost::shared_ptr<VectorXf> intensity(new VectorXf());
      dgcToEigen(points, cloud, intensity);
      clouds.push_back(cloud);
      intensities.push_back(intensity);
    }

    vector<Object*> spin_objs = dp.computeDescriptors(clouds, intensities);
    for(size_t i = 0; i < spin_objs.size(); ++i) {
      objects.push_back(spin_objs[i]);
    }
  }

  // -- Set all the labels to be background.
  for(size_t i = 0; i < objects.size(); ++i)
    objects[i]->label_ = -1;
  
  // -- Save the clusters for viewing.
  if(getenv("SAVE_CLUSTERS")) {
    TrackManager tm(tracks);
    tm.save("bgtracks.tm");
  }
  
  // -- Save the dataset.
  assert(!objects.empty());
  MultiBoosterDataset mbd(getClassNames(), getDescriptorNames());
  mbd.setObjs(objects);

  string savename("negatives.mbd");
  if(getenv("MBD_SAVENAME"))
    savename = string(getenv("MBD_SAVENAME"));
  mbd.save(savename);
  
}
      


/******************************************************************
 * READ parameters
 ******************************************************************/
void
frequency_change_handler(void)
{
  publish_interval = (1.0 / (double) settings.rate_in_hz);
  timer_interval   = (1.0 / (double) (2.0*settings.rate_in_hz) );
}

void
scan_resolution_handler(void)
{
  settings.virtual_scan_resolution = dgc_d2r(scan_resolution);
}

void
rndf_change_handler(void)
{
  fprintf( stderr, "# INFO: rndf changed. Quit program.\n" );
  exit(0);
}


void
read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {

      {"perception", "hz",   DGC_PARAM_DOUBLE,                 &settings.rate_in_hz,  1, ParamCB(frequency_change_handler)},
      {"perception", "max_sensor_delay",   DGC_PARAM_DOUBLE,   &settings.max_sensor_delay,  1, NULL },
      {"perception", "clear_sensor_data",   DGC_PARAM_ONOFF,   &settings.clear_sensor_data,  1, NULL },
      {"perception", "extract_dynamic",   DGC_PARAM_ONOFF,     &settings.extract_dynamic,       1, NULL},
      {"perception", "overpass_height",   DGC_PARAM_DOUBLE,    &settings.overpass_height,      1, NULL},

      {"perception", "map_resolution",   DGC_PARAM_DOUBLE,     &settings.map_resolution,  0, NULL },
      {"perception", "map_size_x",   DGC_PARAM_DOUBLE,         &settings.map_size_x,  0, NULL },
      {"perception", "map_size_y",   DGC_PARAM_DOUBLE,         &settings.map_size_y,  0, NULL },
      {"perception", "map_cell_threshold",   DGC_PARAM_DOUBLE, &settings.map_cell_threshold,  1, NULL },
      {"perception", "map_cell_min_hits",   DGC_PARAM_INT,     &settings.map_cell_min_hits,  1, NULL },
      {"perception", "map_cell_increase",   DGC_PARAM_INT,     &settings.map_cell_increase,  1, NULL },
      {"perception", "map_ray_tracing" , DGC_PARAM_ONOFF,      &settings.map_ray_tracing,      1, NULL},
      {"perception", "z_resolution",   DGC_PARAM_DOUBLE,       &settings.z_resolution,  0, NULL },
      {"perception", "z_obstacle_height",   DGC_PARAM_DOUBLE,  &settings.z_obstacle_height,  0, NULL },

      {"perception", "gls_output" , DGC_PARAM_ONOFF,     &settings.gls_output,      1, NULL},
      {"perception", "show_virtual_scan" , DGC_PARAM_ONOFF,    &settings.show_virtual_scan,      1, NULL},
      {"perception", "show_ray_tracing" , DGC_PARAM_ONOFF,     &settings.show_ray_tracing,      1, NULL},

      {"rndf",      "rndf_file",    DGC_PARAM_FILENAME,    &rndf_filename, 1, ParamCB(rndf_change_handler) },


      {"perception", "use_ldlrs1",  DGC_PARAM_ONOFF,     &settings.use_ldlrs[0],      1, NULL},
      {"perception", "use_ldlrs2" , DGC_PARAM_ONOFF,     &settings.use_ldlrs[1],      1, NULL},
      {"perception", "ldlrs_min_distance" , DGC_PARAM_DOUBLE,   &settings.ldlrs_min_dist,  1, NULL},
      {"perception", "ldlrs_max_distance" , DGC_PARAM_DOUBLE,   &settings.ldlrs_max_dist,  1, NULL},
      {"transform", "ldlrs_laser1", DGC_PARAM_TRANSFORM, &ldlrs_offset[0],   0, NULL},
      {"transform", "ldlrs_laser2", DGC_PARAM_TRANSFORM, &ldlrs_offset[1],   0, NULL},

      {"transform", "radar1", DGC_PARAM_TRANSFORM, &radar_offset[0],   0, NULL},
      {"transform", "radar2", DGC_PARAM_TRANSFORM, &radar_offset[1],   0, NULL},
      {"transform", "radar3", DGC_PARAM_TRANSFORM, &radar_offset[2],   0, NULL},
      {"transform", "radar4", DGC_PARAM_TRANSFORM, &radar_offset[3],   0, NULL},
      {"transform", "radar5", DGC_PARAM_TRANSFORM, &radar_offset[4],   0, NULL},
      {"transform", "radar6", DGC_PARAM_TRANSFORM, &radar_offset[5],   0, NULL},

      {"perception", "use_velodyne" , DGC_PARAM_ONOFF,     &settings.use_velodyne,      1, NULL},
      {"perception", "velodyne_threshold_factor", DGC_PARAM_DOUBLE,   &settings.velodyne_threshold_factor, 1, NULL },
      {"perception", "velodyne_max_range", DGC_PARAM_DOUBLE,   &settings.velodyne_max_range, 1, NULL },
      {"perception", "velodyne_min_beam_diff", DGC_PARAM_DOUBLE,   &settings.velodyne_min_beam_diff, 0, NULL },
      {"perception", "velodyne_sync",   DGC_PARAM_ONOFF,    &settings.velodyne_sync,  1, NULL },
      {"transform", "velodyne",  DGC_PARAM_TRANSFORM, &velodyne_offset,    0, NULL},
      {"velodyne", "cal_file", DGC_PARAM_FILENAME, &settings.velodyne_cal, 0, NULL},

      {"perception", "virtual_scan_resolution", DGC_PARAM_DOUBLE,   &scan_resolution, 1, ParamCB(scan_resolution_handler) },

      { "segmentation", "min_points", DGC_PARAM_INT, &settings.segmentation_settings.min_points, 1, NULL },
      { "segmentation", "max_points", DGC_PARAM_INT, &settings.segmentation_settings.max_points, 1, NULL },
      { "segmentation", "min_height", DGC_PARAM_DOUBLE, &settings.segmentation_settings.min_height, 1, NULL },
      { "segmentation", "kernel_size", DGC_PARAM_INT, &settings.segmentation_settings.kernel_size, 1, NULL },
      { "segmentation", "gls_output", DGC_PARAM_ONOFF, &settings.segmentation_settings.gls_output, 1, NULL },


      { "tracker", "min_car_width", DGC_PARAM_INT, &settings.kf_settings.min_car_width, 1, NULL },
      { "tracker", "min_car_length", DGC_PARAM_INT, &settings.kf_settings.min_car_length, 1, NULL },
      { "tracker", "filter_rndf_max_distance", DGC_PARAM_DOUBLE, &settings.tracker_settings.filter_rndf_max_distance, 1, NULL },
      { "tracker", "filter_rndf_max_pedestrian_distance", DGC_PARAM_DOUBLE, &settings.tracker_settings.filter_rndf_max_pedestrian_distance, 1, NULL },
      { "tracker", "merge_distance", DGC_PARAM_DOUBLE, &settings.tracker_settings.merge_dist, 1, NULL },
      { "tracker", "lateral_merge_dist", DGC_PARAM_DOUBLE, &settings.tracker_settings.lateral_merge_dist, 1, NULL },

      { "tracker", "default_loc_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.default_loc_stddev, 1, NULL },
      { "tracker", "default_vel_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.default_vel_stddev, 1, NULL },
      { "tracker", "transition_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.transition_stddev, 1, NULL },
      { "tracker", "velodyne_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.velodyne_stddev, 1, NULL },
//      { "tracker", "velodyne_max_range", DGC_PARAM_DOUBLE, &settings.kf_settings.velodyne_max_range, 1, NULL },
      { "tracker", "radar_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.radar_stddev, 1, NULL },
      { "tracker", "radar_max_range", DGC_PARAM_DOUBLE, &settings.kf_settings.radar_max_range, 1, NULL },
      { "tracker", "max_dist_correspondence", DGC_PARAM_DOUBLE, &settings.kf_settings.max_dist_correspondence, 1, NULL },
      { "tracker", "confidence_increment_obs", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_increment_obs, 1, NULL },
      { "tracker", "confidence_increment_unobs", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_increment_unobs, 1, NULL },

      { "tracker", "confidence_decay", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_decay, 1, NULL },
      { "tracker", "confidence_max", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_max, 1, NULL },
      { "tracker", "confidence_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_min, 1, NULL },
      { "tracker", "confidence_initial_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_initial_min, 1, NULL },
      { "tracker", "confidence_initial_max", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_initial_max, 1, NULL },
      { "tracker", "confidence_track_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_track_min, 1, NULL },
      { "tracker", "confidence_publish_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_publish_min, 1, NULL },

      { "tracker", "pedestrian_classifier", DGC_PARAM_FILENAME, &settings.tracker_settings.classifier_filename, 1, NULL },
  };

  pint->InstallParams(argc, argv,
      params, sizeof(params)/sizeof(params[0]));

  settings.kf_settings.velodyne_max_range = settings.velodyne_max_range;
  settings.publish_interval = (1.0 / (double) settings.rate_in_hz);
  timer_interval   = (1.0 / (double) (2.0*settings.rate_in_hz) );
  settings.virtual_scan_resolution = dgc_d2r(scan_resolution);
}

/******************************************************************
 *
 *    MAIN
 *
 ******************************************************************/
int
main(int argc, char **argv)
{
  ParamInterface *pint;

  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);
  if (ipc->ConnectLocked("perception_viz") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  settings.velodyne_cal = NULL;

  read_parameters(pint, argc, argv);

  perception_register_ipc_messages(ipc);

  /* allocate rolling grid */
  default_map_cell =
    (PerceptionCell*)calloc(1, sizeof(PerceptionCell));
  dgc_test_alloc(default_map_cell);

  default_map_cell->max       = -FLT_MAX;
  default_map_cell->min       = FLT_MAX;
  default_map_cell->hits      = 0;
  default_map_cell->seen      = 0;
  default_map_cell->last_min  = 0;
  default_map_cell->last_max  = 0;
  default_map_cell->last_observed  = 0;
  default_map_cell->last_obstacle  = 0;
  default_map_cell->last_dynamic   = 0;
  default_map_cell->last_mod  = 0;
  default_map_cell->region    = 0;
  default_map_cell->obstacle  = PERCEPTION_MAP_OBSTACLE_FREE;
  default_map_cell->street    = 1;

  default_terrain_cell =
        (PerceptionCell*)calloc(1, sizeof(PerceptionCell));
  dgc_test_alloc(default_terrain_cell);
  *default_terrain_cell = *default_map_cell;

  grid_stat.mapsize.x    = (int) (settings.map_size_x/settings.map_resolution);
  grid_stat.mapsize.y    = (int) (settings.map_size_y/settings.map_resolution);
  grid_stat.resolution   = settings.map_resolution;
  grid_stat.center.x     = 0;
  grid_stat.center.y     = 0;
  grid_stat.z_resolution = settings.z_resolution;

  fprintf( stderr, "# INFO: initialize grid map (%.1fm x %.1fm - %.2fm resolution)\n",
      settings.map_size_x, settings.map_size_y, settings.map_resolution );

  grid =
    dgc_grid_initialize( grid_stat.resolution,
        grid_stat.mapsize.x,
        grid_stat.mapsize.y,
        sizeof(PerceptionCell),
        default_map_cell );

  applanix_history_init( APPLANIX_HISTORY_LENGTH );


  /*********************************************************************
   *
   *   Log files
   *
   *********************************************************************/
  strcpy(vlf_filename, argv[1]);
  if(strlen(vlf_filename) < 4 || strcmp(vlf_filename + strlen(vlf_filename) - 4, ".vlf"))
    dgc_die("Error: first argument must end in .vlf\n");

  if(argc >= 3)
    strcpy(index_filename, argv[2]);
  else {
    strcpy(index_filename, argv[1]);
    strcat(index_filename, ".index.gz");
  }

  velodyne_file = dgc_velodyne_open_file(vlf_filename);
  if(velodyne_file == NULL)
    dgc_die("Error: Could not open velodyne file %s for reading.\n", vlf_filename);

  /* load the velodyne index */
  velodyne_index.load(index_filename);

  dgc_velodyne_get_config(&velodyne_config);
  if(dgc_velodyne_read_calibration(settings.velodyne_cal, velodyne_config) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  dgc_velodyne_integrate_offset(velodyne_offset, velodyne_config);

  double applanix_lat, applanix_lon, applanix_alt;
  spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num, &applanix_lat, &applanix_lon, &applanix_alt);

  rndf_load_file( rndf_filename );

  velodyne_init( &velodyne );
  perception_init();

  collectDataset();

  return 0;
}
