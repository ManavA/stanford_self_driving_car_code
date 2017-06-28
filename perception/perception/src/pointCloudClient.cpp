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

#ifdef MULTITHREAD
#include <omp.h>
#endif

#include <global.h>
#include <velo_support.h>
#include <pointCloudClient.h>
#include <perception.h>
#include <utils.h>
#include <velodyne_rings.h>

using namespace vlr;

namespace drc = driving_common;

namespace perception {

#define    MAX_NUM_VELODYNE_BLOCKS  50000
#define    MAX_BEAMS_IN_BIN         10
#define    MAX_POINTS_PER_SCAN      8000
#define    CM_TO_METER_FACTOR       0.01
#define    VELODYNE_MIN_RANGE       2.0
#define    VELODYNE_MIN_RANGE_S     200
#define    NO_HEIGHT               -100.0
#define    MIN_DISTANCE_IN_CM       100
#define    VELO_BLIND_SPOT_START    17000
#define    VELO_BLIND_SPOT_STOP     19000
#define    MAX_COUNTER_DIFF         15

#define OCCUPIED    1
#define FREE        0

#define NUM_SAMPLES    1000
#define EPSILON        0.000000001
#define MAX_RANGE      70.0

PointCloudClient::PointCloudClient(Perception& perception) :
  perception_(perception), nh_("/driving"), velodyne_ts_(0), last_velodyne_ts_(0), processed_(false), spin_ready_(false), spin_read_(true) {

  std::string calib_filename;
  getParam("velodyne/cal_file", calib_filename);

  std::string intensity_calib_filename;
  getParam("velodyne/int_file", intensity_calib_filename);
  getParamTransform("Velodyne", velodyne_offset_);

  if (!config_.readCalibration(calib_filename)) {
    throw VLRException("# ERROR: could not read calibration file!");
  }
  if (!config_.readIntensity(intensity_calib_filename)) {
    throw VLRException("# ERROR: could not read intensity calibration file!");
  }

  // save a copy of the offset transform
  config_.integrateOffset(velodyne_offset_);

#ifdef MULTITHREAD
  omp_set_dynamic(0);
  omp_set_num_threads(perception_.settings().num_threads);
#endif

  packet_sub_ = nh_.subscribe("/driving/Cam3D", 5, &PointCloudClient::update, this);
  spin_thread_ = new boost::thread(boost::bind(&PointCloudClient::spinThread, this));
}

PointCloudClient::~PointCloudClient() {
}

template<class T> void PointCloudClient::getParam(std::string key, T& var) {
  if (!nh_.getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + std::string("."));
  }
}

void PointCloudClient::getParamTransform(std::string key, dgc::dgc_transform_t& tr) {
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
void PointCloudClient::spinThread() {
  ros::spin();
}

void PointCloudClient::update(const pcl::PointCloud<pcl::PointXYZI>& packet) {
  boost::unique_lock<boost::mutex> lock(mutex_);

  std::cout<<"Update point cloud with "<<packet.size()<<" points" <<std::endl;
  packet_ = packet;
  spin_read_ = false;
  spin_ready_ = true;

}

void PointCloudClient::read(pcl::PointCloud<pcl::PointXYZI>& packet) {
  boost::unique_lock<boost::mutex> lock(mutex_);

  std::cout<<"Point cloud with "<<packet.size()<<" points read" <<std::endl;

  packet = packet_;
  spin_ready_ = false;
  spin_read_ = true;
}

void PointCloudClient::setCellMin(PerceptionCell& cell, float z, unsigned short counter) {
  if (counter_diff(cell.last_min, counter) > MAX_COUNTER_DIFF || cell.min > z) {
    cell.min = z;
    cell.last_min = counter;
  }
}

void PointCloudClient::setCellMax(PerceptionCell& cell, float z, unsigned short counter) {
  if (counter_diff(cell.last_max, counter) > 5) {
    cell.max = z;
    cell.last_max = counter;
  }
  else if (z > cell.max && z - cell.min < perception_.settings().overpass_height) {
    cell.max = z;
    cell.last_max = counter;
  }
}

void PointCloudClient::syncWithTerrain(const PerceptionCell& t, PerceptionCell& c) {

  if (t.min < c.min) {
    c.min = t.min;
  }
//  if (0 && t->max > c->max) {c->max = t->max;}
}

void PointCloudClient::fillScan(const velodyne::Block& block, int encoder, int point_num, laser_scan_t& scan) {
  int32_t n = scan.num_points;
  if (n < MAX_POINTS_PER_SCAN) {
//    scan.laser_point[n].block = &(block);
    scan.laser_point[n].point = block.point[point_num];
    scan.laser_point[n].range = block.laser[point_num].distance;
    scan.laser_point[n].intensity = block.laser[point_num].intensity;
    //          scan.laser_point[n].cell       = NULL;
    scan.laser_point[n].z_cell = NULL;
    scan.laser_point[n].encoder = encoder;
    scan.laser_point[n].obstacle = false;
    if (!config_.laser_enabled[point_num]) {
      scan.laser_point[n].valid = false;
    }
    else {
      scan.laser_point[n].valid = true;
    }
    scan.laser_point[n].timestamp = block.timestamp;
    scan.num_points++;
  }
  drc::GlobalPose pose = perception_.pose(block.timestamp);
  scan.robot.x =pose.x();
  scan.robot.y =pose.y();
  scan.robot.z =pose.z();
  scan.robot.yaw =pose.yaw();
  scan.robot.pitch =pose.pitch();
  scan.robot.roll =pose.roll();
}

bool PointCloudClient::isObstacle(const pcl::PointXYZI& pt, double ground_threshold) {
  //return pt.z > ground_threshold;
  return true;
}

/**
 * TODO: speed this up (by at least a factor of 2)
 * IDEAS:
 *   1) X get rid of terrain_grid (use z-grid if we really need to)
 *   2) X get rid of map_s (called by include_cell)
 *   3) get rid of min range filtering
 *   4) X get rid of cell_to_points (only OK if we go to Laser Segmenter)
 *   5) X speed up grid_get_xy
 *   6) move local to global transform ( * CM_TO_METER_FUN + scan->robot.x)  outside (we're doing the same work in Obstacle.addPoint())
 *   7) OMP Parallel
 *   8) Get rid of overpass checking
 */

//void PointCloudClient::labelObstacleCells(unsigned short counter) {
//  PerceptionCell* cell = NULL;
//
//  double time = drc::Time::current();
//#ifdef USE_GRID_SEGMENTER
//  perception_.cellToPoints().clear();
//#endif
//  //  int c = 0;
//#pragma omp parallel for private(b,k,x,y,z,cell) shared(counter)
//  for (uint32_t l = 0; l < NUM_LASER_BEAMS - 1; l++) {
//    for (uint32_t b = 0; b < BINS_PER_REV; b++) {
//      for (uint32_t k = 0; k < (uint32_t)bin_[l][b].num_beams; k++) {
//
//        if (bin_[l][b].beam[k]->valid) {
//          drc::GlobalPose robot_pose = perception_.pose(bin_[l][b].beam[k]->timestamp);
//          float x = bin_[l][b].beam[k]->point.x + perception_.grid()->map_c0*perception_.grid()->resolution;// + robot_pose.x();
//          float y = bin_[l][b].beam[k]->point.y + perception_.grid()->map_r0*perception_.grid()->resolution;// + robot_pose.y();
//          cell = (PerceptionCell*) grid_get_xy(perception_.grid(), x, y);
//          if (cell) {
//            int r, c;
//            grid_xy_to_rc(perception_.grid(), x, y, &r, &c);
//#ifdef USE_GRID_SEGMENTER
//#pragma omp critical
//            {
//              perception_.cellToPoints().insert(Perception::map_type::value_type((uintptr_t) cell, (uintptr_t) bin_[l][b].beam[k]));
//            }
//#endif
//
//            //            c++;
//            float z = bin_[l][b].beam[k]->point.z * CM_TO_METER_FACTOR + robot_pose.z();
//            setCellMin(cell, z, counter);
//            setCellMax(cell, z, counter);
//
//            cell->last_observed = counter;
//            if (cell->last_obstacle != counter) {
//              float h = z - cell->min;
//              if (h < perception_.settings().overpass_height) {
//                if (bin_[l][b].beam[k]->range > VELODYNE_MIN_RANGE_S) {
//                  //                  printf("%d, %d\n", bin[l][b].beam[k]->point->x, bin[l][b].beam[k]->point->y );
//                  if (bin_[l][b].beam[k]->obstacle) {
//#pragma omp critical
//                    {
//                      perception_.staticObstacles()->cell[perception_.staticObstacles()->num++] = cell;
//                    }
//
//                    cell->last_obstacle = counter;
//                    cell->hits += 2; //= perception_.settings().map_cell_increase;
//                  }
//                  else {
//                    cell->seen++;
//                  }
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  assert(perception_.staticObstacles()->num < MAX_NUM_POINTS);
//  displayTime("Cells", time);
//  printf("labeled %d cells as obstacles\n", perception_.staticObstacles()->num);
//}

// Add obstacles near car that the Velodyne can't see
//void PointCloudClient::labelObstacleCellsNear(int64_t counter, const std::vector<velodyne::Block>& blocks) {
//  static const int near_obstacle_buffer = 10;
//  static const float near_obstacle_radius = 4.2;
//
//  double time = drc::Time::current();
//
//  int origin_r, origin_c;
//  dgc::dgc_transform_t t;
//  double velodyne_x = 0.0, velodyne_y = 0.0, velodyne_z = 0.0;
//
//  drc::GlobalPose robot_pose = perception_.pose(blocks[0].timestamp);
//
//  dgc::dgc_transform_rpy(t, config_.offset_, robot_pose.roll(), robot_pose.pitch(), robot_pose.yaw());
//  dgc::dgc_transform_translate(t, robot_pose.x(), robot_pose.y(), robot_pose.z());
//  dgc::dgc_transform_point(&velodyne_x, &velodyne_y, &velodyne_z, t);
//
//  float velodyne_height = config_.offset_[2][3] + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT;
//  // TODO: figure out why we need to add 40 cm here to get a realistic ground_z
//  float ground_z = perception_.pose(time).z() - DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT + config_.offset_[2][3];
//
//  PerceptionCell* origin_cell = (PerceptionCell*) grid_get_xy(perception_.grid(), velodyne_x, velodyne_y);
//  cellToRCLocal(perception_.grid(), origin_cell, &origin_r, &origin_c);
//
//  int invisible_radius = ceil(near_obstacle_radius / perception_.grid()->resolution);
//  int invisible_radius2 = invisible_radius * invisible_radius;
//  for (int r = -invisible_radius; r <= invisible_radius; r++) {
//    int r2 = r * r;
//    for (int c = -invisible_radius; c <= invisible_radius; c++) {
//      int c2 = c * c;
//      int d2 = (r2 + c2);
//      if (d2 > invisible_radius2) continue;
//
//      PerceptionCell* cell = (PerceptionCell*) getRCLocal(perception_.grid(), origin_r + r, origin_c + c);
//      if (cell && (cell->last_observed != counter) && (cell->last_obstacle > 0) && ((cell->last_observed - cell->last_obstacle) < near_obstacle_buffer)
//          && ((cell->last_obstacle - cell->last_dynamic) > near_obstacle_buffer) && (cell->hits > 2)) {
//        // remove obstacles that we should be able to see because of their height,
//        // this solves the problem of high obstacles passing through/near the occluded area and leaving behind static obstacles
//        double distance = sqrt(d2) * perception_.grid()->resolution;
//        double max_z = ((near_obstacle_radius - distance) / near_obstacle_radius) * velodyne_height + ground_z;
//
//        // following 2 lines are for visualization purposes only:
//        //        cell->max = max_z;
//        //        cell->min = ground_z;
//
//        if (cell->max > max_z) continue;
//
//        perception_.staticObstacles()->cell[perception_.staticObstacles()->num] = cell;
//        if (perception_.staticObstacles()->num < MAX_NUM_POINTS) {
//          perception_.staticObstacles()->num++;
//        }
//      }
//    }
//  }
//
//  displayTime("Cells Near", time);
//}

void PointCloudClient::labelObstacleCells(const pcl::PointCloud<pcl::PointXYZI>& cloud, unsigned short counter) {

  double time = drc::Time::current();

#ifdef USE_GRID_SEGMENTER
  // perception_.cellToPoints().clear(); // check...
#endif

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator it = cloud.begin(); it != cloud.end(); ++it) {
      drc::GlobalPose robot_pose = perception_.pose(cloud.header.stamp.toNSec()*1.e-6);
    const Grid<PerceptionCell>& grid = perception_.grid();

      float sy = sin(robot_pose.yaw());
      float cy = cos(robot_pose.yaw());

      float r = (it->y * cy + it->x * sy)  / grid.resolution_ + grid.rows_/2;
      float c = (-it->y * sy + it->x * cy) / grid.resolution_ + grid.cols_/2;

    PerceptionCell* cell = grid.getRCLocal(r, c);
          if (cell) {
            float z = it->z + robot_pose.z();
//            float z = it->z * CM_TO_METER_FACTOR + robot_pose.z();
      setCellMin(*cell, z, counter);
      setCellMax(*cell, z, counter);

            cell->last_observed = counter;
            if (cell->last_obstacle != counter) {
              float h = z - cell->min;
              if (h < perception_.settings().overpass_height) {

                  //                  printf("%d, %d\n", bin[l][b].beam[k]->point->x, bin[l][b].beam[k]->point->y );
                  if (isObstacle(*it, perception_.settings().z_obstacle_height) ){ // TODO: check if this is the right threshold...

                    perception_.staticObstacles()->cell[perception_.staticObstacles()->num++] = cell;

                    cell->last_obstacle = counter;
                    cell->hits += 2; //= perception_.settings().map_cell_increase;
                  }
                  else {
                    cell->seen++;
                  }

              }
            }
          }
  }

  assert(perception_.staticObstacles()->num < MAX_NUM_POINTS);
//  displayTime("Cells", time);
  printf("labeled %d cells as obstacles\n", perception_.staticObstacles()->num);
}

void PointCloudClient::integrate(const pcl::PointCloud<pcl::PointXYZI>& cloud, unsigned short counter) {

  std::cout<<"integrating "<<cloud.size()<<" points"<< std::endl;

 // if (!blocks.empty() && (!perception_.settings().clear_sensor_data || drc::Time::current() - velodyne_ts_ < perception_.settings().max_sensor_delay)) {

    //labelObstaclePointsThreshold(cloud, 1.);
    //labelObstaclePoints();

    labelObstacleCells(cloud, counter);
  //  labelObstacleCellsNear(counter, blocks);

    //  segmentVelodyne();
    //  draw_segments();
    //  draw_velodyne();
    last_velodyne_ts_ = velodyne_ts_;
}
} // namespace perception

