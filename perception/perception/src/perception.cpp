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


#include <ros/ros.h>
#include <perception.h>
#include <utils.h>
#include <velodyne.h>
#include <gls.h>
#include <driving_common/Heartbeat.h>

using namespace std;
using boost::shared_ptr;
using namespace dgc;
using namespace driving_common;
using namespace vlr;

namespace drc = driving_common;

namespace perception {

//VelodyneInterface *velo_interface = NULL;
//uint32_t velo_shm_key = 0;

double publish_interval;
double timer_interval;

Perception::Perception(const std::string& multibooster_data_file) :
  nh_("/driving"), shutdown_mutex(PTHREAD_MUTEX_INITIALIZER), integration_mutex(PTHREAD_MUTEX_INITIALIZER), tracking_mutex(PTHREAD_MUTEX_INITIALIZER),
        publish_mutex(PTHREAD_MUTEX_INITIALIZER), velodyne_mutex(PTHREAD_MUTEX_INITIALIZER), fsm_mutex(PTHREAD_MUTEX_INITIALIZER),
        dynamic_msg_mutex(PTHREAD_MUTEX_INITIALIZER), dynamic_semaphore_mutex(PTHREAD_MUTEX_INITIALIZER), virtual_scan_mutex(PTHREAD_MUTEX_INITIALIZER),
        pose_queue_(2000), received_applanix_pose_(false), received_localize_pose_(false), shutdown_(false), data_ready_to_publish(false), gls("PERCEPTION"),
        velo_client_(NULL), point_cloud_client_(NULL), scan_resolution(0), counter_(0), velodyne_ctr(0), virtual_scan_counter(-1), grid_(NULL), z_grid_(NULL),
      publish_ready(false), dynamic_msg_ready(false),
      booster(NULL), classifier_pipeline_(NULL), tracker_(NULL), last_update_(drc::Time::current()), last_integration_time_(drc::Time::current()) {

  global_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "10S"};

  //  for(int32_t i=0; i<NUM_LRR2_RADARS; i++) {
      for (int32_t i = 0; i < NUM_RADARS; i++) {
        radar_mutex[i] = PTHREAD_MUTEX_INITIALIZER;
      }

      settings_.velodyne_cal = NULL;

      readParameters();

      velo_client_ = new VelodyneClient(*this);
      point_cloud_client_ = new PointCloudClient(*this);
      tracker_ = new Tracker(rndf_filename, settings_.tracker_settings, settings_.kf_settings, *this);
      perception_obstacles_pub_ = nh_.advertise<perception::PerceptionObstacles>("PerceptionObstacles", 1);
      heartbeat_pub_ = nh_.advertise<driving_common::Heartbeat>("Heartbeat", 1);

      // poses
      applanix_sub_ = nh_.subscribe("ApplanixPose", 5, &Perception::applanixHandler, this);
      localize_sub_ = nh_.subscribe("LocalizePose", 5, &Perception::localizePoseHandler, this);
      //  radar1_sub_ = nh_.subscribe("sensors/RadarSensor1", 5, &Perception::radar1Handler, this);
      //  radar2_sub_ = nh_.subscribe("sensors/RadarSensor2", 5, &Perception::radar2Handler, this);
      //  radar3_sub_ = nh_.subscribe("sensors/RadarSensor3", 5, &Perception::radar3Handler, this);
      //  radar4_sub_ = nh_.subscribe("sensors/RadarSensor4", 5, &Perception::radar4Handler, this);
      //  radar5_sub_ = nh_.subscribe("sensors/RadarSensor5", 5, &Perception::radar5Handler, this);
      //  radar6_sub_ = nh_.subscribe("sensors/RadarSensor6", 5, &Perception::radar6Handler, this);

      obstacles_s = (dgc_perception_map_cells_t*) malloc(sizeof(dgc_perception_map_cells_t));
      obstacles_s->num = 0;
      obstacles_s->cell = (PerceptionCell* *) malloc( MAX_NUM_POINTS * sizeof(PerceptionCell*) );
      obstacles_s_publish.cell = NULL;
      fprintf( stderr, "done\n" );

      // -- Check for a classifier.
      initBooster(multibooster_data_file);

//      default_terrain_cell = (PerceptionCell*) calloc(1, sizeof(PerceptionCell));
//      dgc_test_alloc(default_terrain_cell);
//      *default_terrain_cell = *default_map_cell;

      grid_stat.mapsize.x = (int) (settings_.map_size_x / settings_.map_resolution);
      grid_stat.mapsize.y = (int) (settings_.map_size_y / settings_.map_resolution);
      grid_stat.resolution = settings_.map_resolution;
      grid_stat.center.x = 0;
      grid_stat.center.y = 0;
      grid_stat.z_resolution = settings_.z_resolution;

      fprintf(stderr, "# INFO: initialize grid map (%.1fm x %.1fm - %.2fm resolution)\n", settings_.map_size_x, settings_.map_size_y, settings_.map_resolution);

      PerceptionCell default_map_cell;
      grid_ = new Grid<PerceptionCell>(grid_stat.resolution, grid_stat.mapsize.x, grid_stat.mapsize.y, &default_map_cell);

      double z_resolution = grid_stat.z_resolution / CM_TO_METER_FACTOR;
      ZCell default_z_cell;
      z_grid_ = new Grid<ZCell>(z_resolution, (int) ceil((grid_stat.mapsize.x * grid_stat.resolution) / grid_stat.z_resolution), (int) ceil(
              (grid_stat.mapsize.y * grid_stat.resolution) / grid_stat.z_resolution), &default_z_cell);
      z_grid_->recenter(0, 0); // TODO: ?!?

      //    terrain_grid =
      //      dgc_grid_initialize( grid_stat.resolution * 5,
      //          (int) ceil(grid_stat.mapsize.x/5),
      //          (int) ceil(grid_stat.mapsize.y/5),
      //          sizeof(PerceptionCell),
      //          default_terrain_cell);

      //  /* create connection to velodyne interface */
      //  velo_interface = new VelodyneShmInterface;
      //  if (velo_shm_key != 0) velo_interface->SetKey(velo_shm_key);
      //  if (velo_interface->CreateClient() < 0) dgc_die("Error: could not connect to velodyne interface.\n");

      std::cout << "Loading RNDF: " << rndf_filename << ".\n";

      perception_thread_ = new boost::thread(boost::bind(&Perception::perceptionThread, this));
    }

    Perception::~Perception() {
      if(tracker_) {delete tracker_;}
      if(velo_client_) {delete velo_client_;}
      if(point_cloud_client_) {delete point_cloud_client_;}
      if (grid_) {delete grid_;}
      if (z_grid_) {delete z_grid_;}
    }

    void Perception::run() {
      ros::Rate r(200);
      while (ros::ok()) {
        double current_time = drc::Time::current();
//        static double time = 0.0;
        if (data_ready_to_publish) {
//          if (time != 0.0) {
//            printf("Publishing again in %f seconds\n", Time::current() - time);
//          }
//          time = Time::current();
          publish();
          data_ready_to_publish = false;
        }

        if (current_time - last_update_ > 1.0) {
          heartbeat_pub_.publish(heartbeat_);
          last_update_ = drc::Time::current();
//          if (current_time - last_publish_ < 1.0) fprintf(stderr, ".");
        }

        r.sleep();
      }
    }

    void Perception::initBooster(const std::string& multibooster_data_file) {

      if(!multibooster_data_file.empty()) {
        booster = new MultiBooster(multibooster_data_file);
      }
      else if (dgc_file_exists(settings_.tracker_settings.classifier_filename.c_str())) {
        booster = new MultiBooster(settings_.tracker_settings.classifier_filename);
      }

      if(!booster) {
        std::cout << "#WARNING: no classifier found at " << settings_.tracker_settings.classifier_filename << std::endl;
        return;
      }

      //cout << booster->status(false) << endl;

      // -- Translate it to use the features that we have currently.
      assert(booster->class_map_.isPermutation(getClassNames()));
      NameMapping feature_map(getDescriptorNames());
      if (!booster->feature_map_.isPermutation(feature_map)) {
        vector<string> not_in_code;
        vector<string> not_in_classifier;
        booster->feature_map_.diff(feature_map, &not_in_code, &not_in_classifier);
        cout << "Descriptors in classifier " << settings_.tracker_settings.classifier_filename << " but not present in code: " << endl;
        for (size_t i = 0; i < not_in_code.size(); ++i) {
          cout << i << ":   " << not_in_code[i] << endl;
        }
        cout << endl << endl;

        cout << "Descriptors in code but not in classifier: " << endl;
        for (size_t i = 0; i < not_in_classifier.size(); ++i) {
          cout << i << ":   " << not_in_classifier[i] << endl;
        }
        cout << endl << endl;

        delete booster;
        booster = NULL;
        return;
      }
      booster->applyNewMappings(NameMapping(getClassNames()), NameMapping(getDescriptorNames()));

      // -- Construct the classifier pipeline.
      int num_threads = settings_.num_threads;
      if (getenv("NUM_THREADS")) num_threads = atoi(getenv("NUM_THREADS"));
      classifier_pipeline_ = new ClassifierPipeline(booster, num_threads);
      std::cout << "Loaded " << settings_.tracker_settings.classifier_filename << ", using " << num_threads << " threads for pipeline classification\n";
    }

    void Perception::publish() {
      static bool do_publishing = false;

      if (!do_publishing) {
        do_publishing = true;
        pthread_mutex_lock(&publish_mutex);
        {
          printf("(3)Tracks: %d\n", (int)obstacles_tracked.size());
          publishObstacles(*grid_publish_, &obstacles_s_publish, obstacles_tracked_publish, counter_);

          if (settings_.gls_output) {
            gls.send();
            gls.clear();
          }

          //fprintf( stderr, "#INFO: [OBSTACLES: %05d]\n", obstacles_s->num);

          obstacles_s->num = 0;
        }
        pthread_mutex_unlock(&publish_mutex);
        do_publishing = false;
      }
    }

    void Perception::publishObstacles(const Grid<PerceptionCell>&, dgc_perception_map_cells_t*, vector< boost::shared_ptr<TrackedObstacle> >, uint16_t) {
//      static double last_time;

//      msg_.timestamp  = points->timestamp;
//      int32_t mc = grid->cols/2;
//      int32_t mr = grid->rows/2;
//      float ll_x = (grid->map_c0)*grid->resolution;
//      float ll_y = (grid->map_r0)*grid->resolution;
//
//      msg_.static_point.clear();
//      int32_t max_num = points->num;
//
//      perception::StaticObstaclePoint point;
//      for (int32_t i=0; i<max_num; i++) {
//        short px, py;
//        // cell_to_coord( grid,  points->cell[i], &px, &py );
//        grid_.cellToRCLocal(points->cell[i], &py, &px);
//        point.x     = ll_x + ( (px+0.5) * grid->resolution );
//        point.y     = ll_y + ( (py+0.5) * grid->resolution );
//
//    //    point.type = points->cell[i]->region;
//        if (points->cell[i]->last_dynamic == counter) {
//          point.type = 1;
//        } else {
//          point.type = 0;
//        }
//
//        point.z_min = points->cell[i]->min;
//        point.z_max = points->cell[i]->max;
//        if (points->cell[i]->hits > 4) {
//          msg_.static_point.push_back(point);
////          msg_.num_points++;
//    //    assert(points->cell[i]->hits > -1);
//        }
//      }



      //int i, err;
      int num_obstacles = 0;

      //    printf("%s %fx%f @ %f,%f\n", obstacle_type2str(obstacles[i]->type),
      //        msg_.dynamic_obstacle[i].x, msg_.dynamic_obstacle[i].y,
      //        msg_.dynamic_obstacle[i].length, msg_.dynamic_obstacle[i].width);

      num_obstacles = msg_.dynamic_obstacle.size(); // ?!?
//      printf("Really publishing %d obstacles\n", num_obstacles);
      double time = drc::Time::current();
      drc::GlobalPose current_pose = pose(time);
      dgc::dgc_transform_t t;
      dgc::dgc_transform_t* radar;
//      if(last_time != 0.0)
//      {
//        printf("Interval: %f\n", time - last_time);
//      }
//      last_time = time;
      for (int r = 0; r < NUM_LRR3_RADARS; r++) {
        switch(r) {
          case 0:
          radar = &radar_offset[2];
          pthread_mutex_lock(&radar_mutex[2]);
          break;
          case 1:
          radar = &radar_offset[5];
          pthread_mutex_lock(&radar_mutex[5]);
          break;
        }

        dgc_transform_copy(t, *radar);
        dgc_transform_rotate_x( t, current_pose.roll());
        dgc_transform_rotate_y( t, current_pose.pitch());
        dgc_transform_rotate_z( t, current_pose.yaw());
        dgc_transform_translate( t, current_pose.x(), current_pose.y(), current_pose.z() );
        /*
         for (i = 0; i < radar_lrr3[r].num_targets; i++) {

         RadarLRR3Target* target = &radar_lrr3[r].target[i];
         if (target->prob_exist < 0.9 || target->prob_obstacle < 0.4)
         continue;

         double x = target->long_distance + 2;
         double y = target->lateral_distance;
         double z = 0.0;
         dgc_transform_point(&x, &y, &z, t);

         double x_vel = target->long_relative_velocity + pose->speed;
         double y_vel = target->lateral_relative_velocity;
         double z_vel = 0.0;

         msg_.dynamic_obstacle[num_obstacles].id = target->id;
         msg_.dynamic_obstacle[num_obstacles].obstacleType = OBSTACLE_CAR;
         msg_.dynamic_obstacle[num_obstacles].x = x;
         msg_.dynamic_obstacle[num_obstacles].y = y;

         msg_.dynamic_obstacle[num_obstacles].velocity = sqrt(x_vel * x_vel + y_vel * y_vel);
         msg_.dynamic_obstacle[num_obstacles].direction = atan2(y_vel, x_vel) + pose->yaw;

         msg_.dynamic_obstacle[num_obstacles].length = 4.0;
         msg_.dynamic_obstacle[num_obstacles].width = 1.75;

         x = target->long_distance_std;
         y = target->lateral_distance_std;

         z = 0.0;
         dgc_transform_point(&x, &y, &z, t);
         msg_.dynamic_obstacle[num_obstacles].confidence = 30.0;
         msg_.dynamic_obstacle[num_obstacles].x_var = x;
         msg_.dynamic_obstacle[num_obstacles].y_var = y;
         msg_.dynamic_obstacle[num_obstacles].xy_cov = 0;
         num_obstacles++;
         }*/
        switch(r) {
          case 0:
          pthread_mutex_unlock(&radar_mutex[2]);
          break;
          case 1:
          pthread_mutex_unlock(&radar_mutex[5]);
          break;
        }
      }

//      msg_.num_dynamic_obstacles = num_obstacles;
//      printf("%d obstacles with radar\n", num_obstacles);

      if (settings_.gls_output) {
        // setup GLS header //
        gls.coordinates = GLSOverlay::SMOOTH_COORDINATES;
        gls.origin_x = 0;
        gls.origin_y = 0;
        gls.origin_z = pose_queue_.pose(drc::Time::current()).z();
        gls.color3f( 1.0, 0.0, 0.0 );
        gls.lineWidth(2.0);
        for (int i=0; i < num_obstacles; i++) {
          gls.pushMatrix();
          perception::DynamicObstacle* obstacle = &msg_.dynamic_obstacle[i];
          gls.translatef(obstacle->x, obstacle->y, 0);
          gls.rotatef(dgc_r2d(obstacle->direction), 0, 0, 1);

          float l = obstacle->length;
          float w = obstacle->width;
          gls.begin(GLSOverlay::LINE_LOOP);
          gls.vertex3f(l / 2, w / 2, 0);
          gls.vertex3f(l / 2, -w / 2, 0);
          gls.vertex3f(-l / 2, -w / 2, 0);
          gls.vertex3f(-l / 2, w / 2, 0);
          gls.vertex3f(l / 2, 0, 0);
          gls.vertex3f(l / 2 + obstacle->velocity, 0, 0);
          gls.vertex3f(l / 2, 0, 0);
          gls.vertex3f(-l / 2, -w / 2, 0);
          gls.vertex3f(-l / 2, w / 2, 0);
          gls.end();

          gls.popMatrix();
        }
      }

      perception_obstacles_pub_.publish(msg_);
    }

    /******************************************************************
     * APPLANIX handler
     ******************************************************************/
    void Perception::applanixHandler(const applanix::ApplanixPose& pose) {
      received_applanix_pose_ = true;
    //  printf("%s: %f, %f\n", __PRETTY_FUNCTION__, applanix_pose.latitude, applanix_pose.longitude);
      updatePose(pose, localize_pose_msg_);
    }


    void Perception::localizePoseHandler(const localize::LocalizePose& pose) {
      received_localize_pose_ = true;
    //  printf("%s: %f, %f\n", __PRETTY_FUNCTION__, localize_pose.x_offset, localize_pose.y_offset);
      updatePose(applanix_pose_msg_, pose);
    }

    void Perception::updatePose(const applanix::ApplanixPose& applanix_pose, const localize::LocalizePose& localize_pose) {
      boost::lock_guard<boost::mutex> lock(pose_mutex_);

      if(&localize_pose != &localize_pose_msg_) {
        localize_pose_msg_ = localize_pose;
      }

      if (!received_applanix_pose_) {return;} // localize pose alone doesn't help..and anyway..shouldn't occur

      if(&applanix_pose != &applanix_pose_msg_) {
        applanix_pose_msg_ = applanix_pose;
      }
      double offset_x=0, offset_y=0;

      if (received_localize_pose_) {
    //    printf("received xoff: %f, yoff: %f\n", localize_pose_msg_.x_offset, localize_pose_msg_.y_offset);
        offset_x = localize_pose_msg_.x_offset;
        offset_y = localize_pose_msg_.y_offset;
      }
      else { // that's the best we can do without localize pose
        std::string utm_zone;
        latLongToUtm(applanix_pose_msg_.latitude, applanix_pose_msg_.longitude, &offset_x, &offset_y, utm_zone);
        offset_x -= applanix_pose_msg_.smooth_x;
        offset_y -= applanix_pose_msg_.smooth_y;
      }

      double a     = applanix_pose_msg_.accel_x * cos(-applanix_pose_msg_.yaw) - applanix_pose_msg_.accel_y*sin(-applanix_pose_msg_.yaw); // TODO: check this...+-theta?!?
      double a_lat = applanix_pose_msg_.accel_x * sin(-applanix_pose_msg_.yaw) + applanix_pose_msg_.accel_y*cos(-applanix_pose_msg_.yaw); // TODO: check this...+-theta?!?

      pose_queue_.push(drc::GlobalPose(applanix_pose_msg_.smooth_x, applanix_pose_msg_.smooth_y, offset_x, offset_y, applanix_pose_msg_.yaw, 0, 0, applanix_pose_msg_.speed, 0, 0, a, a_lat, applanix_pose_msg_.accel_z), applanix_pose_msg_.timestamp);
    }

    void Perception::perceptionThread() {
      while (ros::ok()) {
        double t1 = drc::Time::current();

        /*---------------------------------------*/
        if ( settings_.use_velodyne ) {
          bool new_data = false;
          if (velo_client_->blocksAvailable()) {
            velo_client_->readSpin(velo_blocks_);  // TODO: don't wait for complete spin, use readBlocks()

            velodyne_ctr++;
            new_data = true;
          }
          else {
            velo_blocks_.clear();
          }

          if (new_data) {
            integrateSensors(velo_blocks_);
            //publish();
            data_ready_to_publish = true;
          }
        }

        if(settings_.use_point_cloud) {
          bool new_data = false;
          if (point_cloud_client_->blocksAvailable()) {
            point_cloud_client_->read(packet_);

            velodyne_ctr++;
            new_data = true;
          }

          if (new_data) {
            integrateSensors(packet_);
            //publish();
            data_ready_to_publish = true;
          }
        }
        /*---------------------------------------*/

        double t2 = Time::current();
        double extra = 1.0 / 10 - (t2 - t1);
        if (extra > 0) {
          usleep((int) rint(extra * 1e6));
        }
        else if (extra < 0) {
//          printf("SLOW %f\n", t2 - t1);
        }
        pthread_mutex_lock(&shutdown_mutex);
        if (shutdown_ == 2) {
          printf("Perception shutting down\n");
          shutdown_ = 0;
          pthread_mutex_unlock(&shutdown_mutex);
          return;
        }
        pthread_mutex_unlock(&shutdown_mutex);
      }
    }

    /******************************************************************
     * READ parameters
     ******************************************************************/
    //void
    //frequency_change_handler(void)
    //{
    //  publish_interval = (1.0 / (double) settings_.rate_in_hz);
    //  timer_interval   = (1.0 / (double) (2.0*settings_.rate_in_hz) );
    //}

    template<class T> void Perception::getParam(std::string key, T& var) {
      if (!nh_.getParam(key, var)) {
        throw VLRException("Cannot read parameter " + key + std::string("."));
      }
    }

    void Perception::getParamTransform(std::string key, dgc::dgc_transform_t& tr) {
      ros::Time now = ros::Time::now();
      tf_listener_.waitForTransform("Applanix", key, now, ros::Duration(3.0));

      tf::StampedTransform transform;
      try {
        tf_listener_.lookupTransform("Applanix", key, ros::Time(0), transform);
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
      }

      tr[3][0] = 0;
      tr[3][1] = 0;
      tr[3][2] = 0;
      tr[3][3] = 1;
      btMatrix3x3 R = transform.getBasis();
      for (int32_t r = 0; r < 3; r++) {
        for (int32_t c = 0; c < 3; c++) {
          tr[r][c] = R[r][c];
        }
      }
      btVector3 t = transform.getOrigin();
      tr[0][3] = t[0];
      tr[1][3] = t[1];
      tr[2][3] = t[2];
    }

    void Perception::readParameters() {
      getParam("perception/hz", settings_.rate_in_hz);
      getParam("perception/max_sensor_delay", settings_.max_sensor_delay);
      getParam("perception/clear_sensor_data", settings_.clear_sensor_data);
      getParam("perception/overpass_height", settings_.overpass_height);
      getParam("perception/num_threads", settings_.num_threads);

      getParam("perception/map_resolution", settings_.map_resolution);

      getParam("perception/map_size_x", settings_.map_size_x);
      getParam("perception/map_size_y", settings_.map_size_y);
      getParam("perception/map_cell_threshold", settings_.map_cell_threshold);
      getParam("perception/map_cell_min_hits", settings_.map_cell_min_hits);
      getParam("perception/map_cell_increase", settings_.map_cell_increase);
      getParam("perception/map_ray_tracing", settings_.map_ray_tracing);
      getParam("perception/z_resolution", settings_.z_resolution);
      getParam("perception/z_obstacle_height", settings_.z_obstacle_height);

      getParam("perception/gls_output", settings_.gls_output);
      getParam("perception/show_virtual_scan", settings_.show_virtual_scan);
      getParam("perception/show_ray_tracing", settings_.show_ray_tracing);

      getParam("perception/virtual_scan_resolution", scan_resolution);

      getParam("rndf_file", rndf_filename);

      getParamTransform("RadarSensor1", radar_offset[0]);
      getParamTransform("RadarSensor2", radar_offset[1]);
      getParamTransform("RadarSensor3", radar_offset[2]);
      getParamTransform("RadarSensor4", radar_offset[3]);
      getParamTransform("RadarSensor5", radar_offset[4]);
//      getParamTransform("RadarSensor6", radar_offset[5]);


      try { getParam("perception/use_point_cloud", settings_.use_point_cloud); }
      catch(...) { settings_.use_point_cloud = false; }

      try { getParam("perception/use_velodyne", settings_.use_velodyne); }
      catch(...) { settings_.use_velodyne = false; }

      if (settings_.use_velodyne)
      {
        getParam("perception/velodyne_threshold_factor", settings_.velodyne_threshold_factor);
        getParam("perception/velodyne_max_range", settings_.velodyne_max_range);
        getParam("perception/velodyne_min_beam_diff", settings_.velodyne_min_beam_diff);
//      getParam("perception/velodyne_sync", settings_.velodyne_sync);
      }

      getParam("perception/segmentation/min_points", settings_.segmentation_settings.min_points);
      getParam("perception/segmentation/max_points", settings_.segmentation_settings.max_points);
      getParam("perception/segmentation/min_height", settings_.segmentation_settings.min_height);
      getParam("perception/segmentation/kernel_size", settings_.segmentation_settings.kernel_size);
      getParam("perception/segmentation/gls_output", settings_.segmentation_settings.gls_output);

      getParam("perception/tracker/filter_rndf_max_distance", settings_.tracker_settings.filter_rndf_max_distance);
      getParam("perception/tracker/filter_rndf_max_pedestrian_distance", settings_.tracker_settings.filter_rndf_max_pedestrian_distance);
      getParam("perception/tracker/filter_rndf", settings_.tracker_settings.filter_rndf);
      getParam("perception/tracker/merge_distance", settings_.tracker_settings.merge_dist);
      getParam("perception/tracker/lateral_merge_dist", settings_.tracker_settings.lateral_merge_dist);
      getParam("perception/tracker/pedestrian_classifier", settings_.tracker_settings.classifier_filename);
      /*getParam("default_loc_stddev", settings_.kf_settings.default_loc_stddev);
       getParam("perception/tracker/min_car_width", settings_.kf_settings.min_car_width);
       getParam("perception/tracker/min_car_length", settings_.kf_settings.min_car_length);
       getParam("perception/tracker/default_vel_stddev", settings_.kf_settings.default_vel_stddev);
       getParam("perception/tracker/transition_stddev", settings_.kf_settings.transition_stddev);
       getParam("perception/tracker/velodyne_stddev", settings_.kf_settings.velodyne_stddev);
       //      getParam("perception/tracker/velodyne_max_range", settings_.kf_settings.velodyne_max_range);
       getParam("perception/tracker/radar_stddev", settings_.kf_settings.radar_stddev);
       getParam("perception/tracker/radar_max_range", settings_.kf_settings.radar_max_range);
       getParam("perception/tracker/max_dist_correspondence", settings_.kf_settings.max_dist_correspondence);
       getParam("perception/tracker/confidence_increment_obs", settings_.kf_settings.confidence_increment_obs);
       getParam("perception/tracker/confidence_increment_unobs", settings_.kf_settings.confidence_increment_unobs);

       getParam("perception/tracker/confidence_decay", settings_.kf_settings.confidence_decay);
       getParam("perception/tracker/confidence_max", settings_.kf_settings.confidence_max);
       getParam("perception/tracker/confidence_min", settings_.kf_settings.confidence_min);
       getParam("perception/tracker/confidence_initial_min", settings_.kf_settings.confidence_initial_min);
       getParam("perception/tracker/confidence_initial_max", settings_.kf_settings.confidence_initial_max);
       getParam("perception/tracker/confidence_track_min", settings_.kf_settings.confidence_track_min);
       getParam("perception/tracker/confidence_publish_min", settings_.kf_settings.confidence_publish_min);*/
      getParam("perception/tracker/correspondence_threshold", settings_.kf_settings.correspondence_threshold);
      getParam("perception/tracker/pruning_threshold", settings_.kf_settings.pruning_threshold);
      getParam("perception/tracker/measurement_variance", settings_.kf_settings.measurement_variance);
      getParam("perception/tracker/position_variance", settings_.kf_settings.position_variance);
      getParam("perception/tracker/velocity_variance", settings_.kf_settings.velocity_variance);
      getParam("perception/tracker/initial_position_variance", settings_.kf_settings.initial_position_variance);
      getParam("perception/tracker/initial_velocity_variance", settings_.kf_settings.initial_velocity_variance);

      //settings_.kf_settings.velodyne_max_range = settings_.velodyne_max_range;
      settings_.publish_interval = (1.0 / (double) settings_.rate_in_hz);
      timer_interval = (1.0 / (double) (2.0 * settings_.rate_in_hz));
      settings_.virtual_scan_resolution = dgc_d2r(scan_resolution);
    }

  } // namespace perception
