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
#include <veloClient.h>
#include <perception.h>
#include <utils.h>
#include <velodyne_rings.h>

namespace drc = driving_common;

using namespace vlr;

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

float velodyne_upper_block_max_range_diff = 200.0;
float velodyne_lower_block_max_range_diff = -15.0;


#define OCCUPIED    1
#define FREE        0

#define NUM_SAMPLES    1000
#define EPSILON        0.000000001
#define MAX_RANGE      70.0

/*
 * Used to filter obviously bad returns.
 * Assume the ground slopes down at no more than 11 degrees
 * and that all returns below ground are invalid
 *
 * Values are recalculated directly from config file on each load.
 * These values are from a particular config file (ID89.cal)
 */
unsigned short max_valid_range[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1025, 1070, 0, 0, 1117,
    1171, 868, 904, 1245, 1336, 940, 996, 2225, 2400, 1431, 1473, 2650, 3002, 1582, 1676, 3542, 4293, 1841, 2007, 0, 0, 5453, 6522, 0, 0, 8513, 13529 };


VelodyneClient::VelodyneClient(Perception& perception) :
    perception_(perception), nh_("/driving"), velodyne_ts_(0),
    last_velodyne_ts_(0), processed_(false), spin_ready_(false), spin_read_(true) {

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

  data_.config = &config_;
  data_.blocks.reserve(MAX_NUM_VELODYNE_BLOCKS);

  last_data_.config = &config_;
  last_data_.blocks.reserve(MAX_NUM_VELODYNE_BLOCKS);

#ifdef MULTITHREAD
  omp_set_dynamic(0);
  omp_set_num_threads(perception_.settings().num_threads);
#endif

  for (uint32_t l = 0; l < NUM_LASER_BEAMS; l++) {
    for (uint32_t i = 0; i < BINS_PER_REV; i++) {
      bin_[l][i].num_beams = 0;
      bin_[l][i].beam = new laser_point_t*[MAX_BEAMS_IN_BIN];
    }
  }

  lscan_ = new laser_scan_t[NUM_LASER_BEAMS];
  nscan_ = new laser_scan_t[NUM_LASER_BEAMS];

  for (uint32_t l = 0; l < NUM_LASER_BEAMS; l++) {
    lscan_[l].num_points = 0;
    lscan_[l].laser_point = new laser_point_t[MAX_POINTS_PER_SCAN];

    nscan_[l].num_points = 0;
    nscan_[l].laser_point = new laser_point_t[MAX_POINTS_PER_SCAN];
  }

  rings_ = new VelodyneRings(config_, perception_.settings().velodyne_min_beam_diff);

  double theta = atan2(0.2, 1);
  double L = M_PI_2 + theta;
  double velodyne_height = config_.offset_[2][3] + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT;
  for (uint32_t l = 0; l < NUM_LASER_BEAMS; l++) {
    double R = config_.vert_angle[l];
    if (R > -theta) {
      max_valid_range[l] = 0;
      continue;
    }
    R = M_PI_2 + R;

    double max_ground_range = 100.0 * velodyne_height * sin(L) / sin(M_PI - R - L);
    if (max_ground_range > std::numeric_limits<short>::max()) max_valid_range[l] = 0;
    else max_valid_range[l] = (unsigned short) (max_ground_range);
  }

  packet_sub_ = nh_.subscribe("velodyne/projected", 100, &VelodyneClient::update, this);
  spin_thread_ = new boost::thread(boost::bind(&VelodyneClient::spinThread, this));
}

VelodyneClient::~VelodyneClient() {
}

template<class T> void VelodyneClient::getParam(std::string key, T& var) {
  if (!nh_.getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + std::string("."));
  }
}

void VelodyneClient::getParamTransform(std::string key, dgc::dgc_transform_t& tr) {
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
void VelodyneClient::spinThread() {
  ros::spin();
}

static uint32_t spcount=0;

void VelodyneClient::update(const velodyne::Projected& packet) {

  boost::unique_lock<boost::mutex> lock(mutex_);

  static uint32_t last_packet_id = packet.header.seq;
  if(packet.header.seq - last_packet_id != 1) {
    std::cout << __PRETTY_FUNCTION__ << ": Lost Velodyne packet\n";
  }
  last_packet_id = packet.header.seq;
  if (blocks_.empty()) {
    for (uint32_t i = 0; i < velodyne::Packet::NUM_BLOCKS; i++) {
      blocks_.push_back(packet.block[i]);
    }
    return;
  }

  uint16_t last_encoder;
  if(blocks_.empty()) {last_encoder=0;}
  else {
    last_encoder = (blocks_.rbegin())->encoder;
  }

  for (uint32_t i = 0; i < velodyne::Packet::NUM_BLOCKS; i++) {
    if (packet.block[i].encoder < last_encoder) {
      if(blocks_.empty()) {
        continue;
      }
      spin_ready_ = true;
      printf("spin %u with size %u ready\n", spcount, (uint32_t)blocks_.size());
      cond_spin_ready_.notify_all();

      while (!spin_read_) {
        cond_spin_read_.wait(lock);
      }

      blocks_.clear();
      spin_ready_ = false;
      spcount++;
    }
    else {
      blocks_.push_back(packet.block[i]);
    }
  }
}

void VelodyneClient::readSpin(std::vector<velodyne::Block>& blocks) {
  boost::unique_lock<boost::mutex> lock(mutex_);

  spin_read_ = false;
  while(!spin_ready_) {
    cond_spin_ready_.wait(lock);
  }

  blocks = blocks_;
  velodyne_ts_ = drc::Time::current();  // TODO: change timestamps to original
  printf("Using spin %u with size %u ready\n", spcount, (uint32_t)blocks.size());

  spin_read_ = true;
  cond_spin_read_.notify_all();
}

void VelodyneClient::readBlocks(std::vector<velodyne::Block>& blocks) {
  boost::unique_lock<boost::mutex> lock(mutex_);

  if(!blocks_.empty()) {
    blocks = blocks_;
    velodyne_ts_ = drc::Time::current();
  }

  blocks_.clear();

  spin_ready_ = false;
  spin_read_ = true;
  cond_spin_read_.notify_all();
}

void displayTime(const std::string& label, double time) {
  return; // TODO: disabled for debugging...
  int ms = (int) ((drc::Time::current() - time) * 1000);
  if (label.length() < 9) printf("#TIME: %s\t\t%02d", label.c_str(), ms);
  else {printf("#TIME: %s\t%02d", label.c_str(), ms);}

  if (ms > 20) printf(" * ");

  printf("\n");
}

void VelodyneClient::setCellMin(PerceptionCell& cell, float z, unsigned short counter) {
  if (counter_diff(cell.last_min, counter) > MAX_COUNTER_DIFF || cell.min > z) {
    cell.min = z;
    cell.last_min = counter;
  }
}

void VelodyneClient::setCellMax(PerceptionCell& cell, float z, unsigned short counter) {
  if (counter_diff(cell.last_max, counter) > 5) {
    cell.max = z;
    cell.last_max = counter;
  }
  else if (z > cell.max && z - cell.min < perception_.settings().overpass_height) {
    cell.max = z;
    cell.last_max = counter;
  }
}

void VelodyneClient::syncWithTerrain(const PerceptionCell& t, PerceptionCell& c) {

  if (t.min < c.min) {c.min = t.min;}
//  if (0 && t->max > c->max) {c->max = t->max;}
}


void VelodyneClient::fillScan(const velodyne::Block& block, int encoder, int point_num, laser_scan_t& scan) {
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
    if (!config_.laser_enabled[point_num]) {scan.laser_point[n].valid = false;}
    else {scan.laser_point[n].valid = true;}
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

void VelodyneClient::process(const std::vector<velodyne::Block>& blocks) {
  // *******************************************************************
  // *    Set beams to individual laser units
  // *******************************************************************

  laser_scan_p tmp = lscan_;
  lscan_ = nscan_;
  nscan_ = tmp;

//  if ((velodyne_ts_ - last_velodyne_ts_) > 0.19) {
//    fprintf(stderr, "#WARN: dropping blocks can cause blind spots! Delay is %f s\n", (velodyne_ts_ - last_velodyne_ts_));
//    for (int l = 0; l < NUM_LASER_BEAMS; l++) {
//      lscan_[l].num_points = 0;
//    }
//  }

  for (uint32_t l = 0; l < NUM_LASER_BEAMS; l++) {
    nscan_[l].num_points = 0;
  }

  /*  LASER 00-31: upper block
   LASER 32-64: lower block  */
  double time = drc::Time::current();
  //    int count_zero = 0;
  //    int count_max = 0;
  //    int less_zero = 0;
  for (int32_t i = 0; i < (int32_t)blocks.size(); i++) {
    int encoder = (blocks[i].encoder + VELO_SPIN_START) % VELO_NUM_TICKS; // unwrap back to 0 ... 36000 range
    encoder -= rings_->minHorizontalOffset();

#pragma omp parallel for private(l, e, n)
    for (int32_t j = 0; j < 32; j++) {
      unsigned short range = blocks[i].laser[j].distance;
      if (range == 0) {
        //          count_zero++;
        // TODO: we may not want to throw these all away
        continue;
      }

      int32_t l = j + blocks[i].block * 32;

      if ((max_valid_range[l]) && (range > max_valid_range[l])) {
        //          count_max++;
        continue;
      }

      int32_t e = encoder + rings_->horizontalOffset(l);
      assert (e >= 0);
      if (e >= VELO_NUM_TICKS) {
        e -= VELO_NUM_TICKS;
        fillScan(blocks[i], e, j, nscan_[l]);
      }
      else {
        fillScan(blocks[i], e, j, lscan_[l]);
      }
    }
  }
  displayTime("Projecting", time);
  time = drc::Time::current();
  // *************************************************************
  // * Put beams in bins
  // *************************************************************
#pragma omp parallel for private(i)
  for (int32_t l = 0; l < NUM_LASER_BEAMS; l++) {
    for (int32_t i = 0; i < BINS_PER_REV; i++) {
      bin_[l][i].num_beams = 0;
    }
  }

#pragma omp parallel for private(i,e,b,n)
  for (int32_t l = 0; l < NUM_LASER_BEAMS; l++) {
    for (int32_t i = 0; i < lscan_[l].num_points; i++) {
      if (lscan_[l].laser_point[i].valid) {
        int32_t e = lscan_[l].laser_point[i].encoder;
        int32_t b = (int) floor(e / ((float) VELO_NUM_TICKS / (float) BINS_PER_REV));
        assert(bin_[l][b].num_beams < MAX_BEAMS_IN_BIN);
        bin_[l][b].beam[bin_[l][b].num_beams++] = &lscan_[l].laser_point[i];
      }
    }
  }
  displayTime("Binning", time);
  //    printf("zeros: %d\n", count_zero);
  //    printf("max filtered: %d\n", count_max);
}

void VelodyneClient::labelObstaclePointsRings() {
  //  int c;
  //  float threshold, range0, range1;

  double max_range = perception_.settings().velodyne_max_range * 100.0;
  double factor = 0.01 * perception_.settings().velodyne_threshold_factor;

  //  c = 0;
#pragma omp parallel for private(l, l0, l1, b)
  for (uint32_t l = 0; l < NUM_LASER_BEAMS - 1; l++) {
    if (l != (uint32_t)rings_->partnerBeam(l)) {
      uint32_t l1 = rings_->beamToIndex(l); // ring[l].idx;            /* shorter beam */
      uint32_t l0 = rings_->partnerIndex(l); // ring[ring[l].pb].idx;   /* longer beam */
      for (uint32_t b = 0; b < BINS_PER_REV; b++) {
        for (int k = 0; k < bin_[l0][b].num_beams; k++) {
          float range0 = bin_[l0][b].beam[k]->range;
          if (bin_[l0][b].beam[k]->valid && (range0 < max_range * 100.0)) {
            for (uint32_t m = 0; m < (uint32_t)bin_[l1][b].num_beams; m++) {
              if (bin_[l1][b].beam[m]->valid) {
                float range1 = bin_[l1][b].beam[m]->range;
                float threshold = rings_->factor(l) * range1 * range1 * factor;
                if (fabs(range1 - range0) < threshold) {
                  bin_[l0][b].beam[k]->obstacle = TRUE;
                  //                  bin[l1][b].beam[m]->obstacle = TRUE;
                  //                  c++;
                  //                  break;
                }
              }
            }
          }
        }
      }
    }
  }
  //  printf("%d points labeled as obstacles\n", c);
}

//void labelObstaclePointsThreshold(double ground_threshold) {
//  short th = short(ground_threshold * 100);
//  //
//  int count = 0;
//  int total = 0;
//  for (int l = 0; l < NUM_LASER_BEAMS; l++) {
//    total += lscan_[l].num_points;
//    for (int k = 0; k < lscan_[l].num_points; k++) {
//      lscan_[l].laser_point[k].obstacle = (lscan_[l].laser_point[k].point->z > th);
//      count += (lscan_[l].laser_point[k].point->z > th);
//    }
//  }
//  //  printf("%d / %d points are obstacles\n", count, total);
//}

unsigned short min_ground_range[] = { 65000, 65000, 65000, 65000, 65000, 65000, 65000, 4559, 4022, 3718, 3336, 3000, 2711, 2479, 2287, 2151, 2004, 1921, 1842,
    1741, 1657, 1590, 1510, 1380, 1306, 1258, 1265, 1238, 1078, 1045, 983, 944, 917, 829, 810, 759, 756, 687, 684, 622, 630, 569, 594, 539, 539, 494, 515, 462,
    471, 427, 436, 401, 436, 380, 401, 363, 377, 343, 354, 333, 347, 321, 332, 308 };

unsigned short min_ground_range_adjusted[] = { 65000, 65000, 65000, 65000, 65000, 65000, 65000, 4559, 4022, 3718, 3336, 3000, 2711, 2479, 2287, 2151, 2004,
    1921, 1842, 1741, 1657, 1590, 1510, 1380, 1306, 1258, 1265, 1238, 1078, 1045, 983, 944, 917, 829, 810, 759, 756, 687, 684, 622, 630, 569, 594, 539, 539,
    494, 515, 462, 471, 427, 436, 401, 436, 380, 401, 363, 377, 343, 354, 333, 347, 321, 332, 308 };

unsigned short max_ground_range = 7000; // 70 m

//void labelObstaclePointsRange() {
//    static bool init = true;
//    if (init) {
//      for (int i=0; i < NUM_LASER_BEAMS; i++)
//        min_ground_range_adjusted[i] = (short)( min_ground_range[i] * 0.75);
//      init = false;
//    }
//
//    int l;
//
//    #pragma omp parallel for
//    for (l=0; l<NUM_LASER_BEAMS; l++) {
//      for (int i = 0; i < lscan_[l].num_points; i++) {
//        short r = lscan_[l].laser_point[i].point->range;
//        int b = rings->indexToBeam(l); //ridx[l];
//        if ((r < min_ground_range_adjusted[b]) || (r > max_ground_range))
//          lscan_[l].laser_point[i].obstacle = 1;
//      }
//    }
//}

//TODO: cut from 8ms to 5ms
void VelodyneClient::labelObstaclePointsTerrain() {

  // EXPAND_Z_GRID adds about 3ms to processing time, and
  // it does a single step of expansion on z_grid
  //#define EXPAND_Z_GRID

#ifdef EXPAND_Z_GRID
  __gnu_cxx::hash_set<int> open;
#endif

  static bool init = true;
  static short obstacle_threshold = 0;
  if (init) {
    obstacle_threshold = (short) (perception_.settings().z_obstacle_height / 0.01);
    init = false;
  }

  static short max_z = std::numeric_limits<short>::max();
  int l;
#pragma omp parallel for
  for (l = 0; l < NUM_LASER_BEAMS; l++) {
    for (int i = 0; i < lscan_[l].num_points; i++) {
      const Grid<ZCell>& z_grid = perception_.zGrid();
      float x = lscan_[l].laser_point[i].point.x;
      float y = lscan_[l].laser_point[i].point.y;
      ZCell* zcell = z_grid.getXY(x, y);
//      short* zcell = (short*) getXY(perception_.zGrid(), lscan_[l].laser_point[i].point->x, lscan_[l].laser_point[i].point->y);
      if (zcell) {
        zcell->z_value = max_z;
        lscan_[l].laser_point[i].z_cell = zcell;
      }
    }
  }

#pragma omp parallel for
  for (l = 0; l < NUM_LASER_BEAMS; l++) {
    for (int i = 0; i < lscan_[l].num_points; i++) {
      ZCell* zcell = lscan_[l].laser_point[i].z_cell;
      if (zcell) {
        zcell->z_value = std::min((float)zcell->z_value, lscan_[l].laser_point[i].point.z);
#ifdef EXPAND_Z_GRID
        open.insert((int)zcell);
#endif
      }
    }
  }

  //  printf("open size: %d\n", open.size());

#ifdef EXPAND_Z_GRID
  __gnu_cxx::hash_set<int>::iterator end = open.end();
  for (__gnu_cxx::hash_set<int>::iterator it = open.begin(); it != end; it++) {
    short* cell = (short*) *it;
    //    printf("processing %d %d\n", (int)cell, *cell);
    int r,c;
    z_grid->cellToRCLocal(cell, &r, &c);

    // look at four neighborhood
    ZCell* neighbor = (ZCell*)z_grid->getRCLocal(r+1, c);
    //    if ((neighbor) && (*cell > (*neighbor + 20)))
    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
    *cell = *neighbor + 20;

    neighbor = (short*)getRCLocal(z_grid, r-1, c);
    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
    *cell = *neighbor + 20;

    neighbor = (short*)getRCLocal(z_grid, r, c-1);
    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
    *cell = *neighbor + 20;

    neighbor = (short*)getRCLocal(z_grid, r, c+1);
    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
    *cell = *neighbor + 20;
  }
#endif

#pragma omp parallel for
  for (int l = 0; l < NUM_LASER_BEAMS; l++) {
    for (int i = 0; i < lscan_[l].num_points; i++) {
      short z = lscan_[l].laser_point[i].point.z;
      ZCell* zcell = lscan_[l].laser_point[i].z_cell;
      if ((zcell) && ((z - zcell->z_value) > obstacle_threshold)) {lscan_[l].laser_point[i].obstacle = 1;}
    }
  }
}

//void label_obstacle_points_terrain_range()
//{
//  static bool init = true;
//  static short obstacle_threshold = 0;
//  if (init) {
//    obstacle_threshold = (short)(perception_.settings().terrain_obstacle_height / 0.01);
//
//    for (int i=0; i < NUM_LASER_BEAMS; i++)
//      min_ground_range_adjusted[i] = (short)( min_ground_range[i] * 0.8);
//
//    init = false;
//  }
//
//  perception_.zGrid()->clear();
//  for (int l=0; l<NUM_LASER_BEAMS; l++) {
//    for (int i = 0; i < lscan_[l].num_points; i++) {
//      if (!lscan_[l].laser_point[i].valid)
//        continue;
//      double x = lscan_[l].laser_point[i].point->x * CM_TO_METER_FACTOR;
//      double y = lscan_[l].laser_point[i].point->y * CM_TO_METER_FACTOR;
//      short z = lscan_[l].laser_point[i].point->z;
//      short* grid_z;
//      grid_z = (short*)getXY(z_grid, x, y);
//      if (grid_z)
//        *grid_z = std::min(*grid_z, z);
//      lscan_[l].laser_point[i].z_cell = grid_z;
//    }
//  }
//
//  for (int l=0; l<NUM_LASER_BEAMS; l++) {
//    for (int i = 0; i < lscan_[l].num_points; i++) {
//      short z = lscan_[l].laser_point[i].point->z;
//      short* grid_z = lscan_[l].laser_point[i].z_cell;
//      if ((grid_z) && ( (z - *grid_z) > obstacle_threshold))
//         lscan_[l].laser_point[i].obstacle = 1;
//      else {
//        short r = lscan_[l].laser_point[i].point->range;
//        int b = ridx[l];
//        if ((r < min_ground_range_adjusted[b]) || (r > max_ground_range))
//          lscan_[l].laser_point[i].obstacle = 1;
//      }
//    }
//  }
//}

// TODO: cut from 30ms to 10ms
void VelodyneClient::labelObstaclePoints() {
  double time = drc::Time::current();
  labelObstaclePointsRings();
  displayTime("Rings", time);
//  time = drc::Time::current();
//  labelObstaclePointsRange();
//  displayTime("Range", time);
  time = drc::Time::current();
  labelObstaclePointsTerrain();
  displayTime("Z", time);
}

void report_num_threads(int level) {
  //  #pragma omp single
  //  {
  //    printf("Level %d: number of threads in the team - %d\n", level, omp_get_num_threads());
  //  }
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

void VelodyneClient::labelObstacleCells(unsigned short counter) {

  double time = drc::Time::current();
#ifdef USE_GRID_SEGMENTER
  perception_.cellToPoints().clear();
#endif

  const Grid<PerceptionCell>& grid = perception_.grid();
  //  int c = 0;
#pragma omp parallel for private(b,k,x,y,z,cell) shared(counter)
  for (uint32_t l = 0; l < NUM_LASER_BEAMS - 1; l++) {
    for (uint32_t b = 0; b < BINS_PER_REV; b++) {
      for (uint32_t k = 0; k < (uint32_t)bin_[l][b].num_beams; k++) {

        if (bin_[l][b].beam[k]->valid) {
          drc::GlobalPose robot_pose = perception_.pose(bin_[l][b].beam[k]->timestamp);
          int32_t xi =   grid.cols_ / 2 - (int32_t) (bin_[l][b].beam[k]->point.x * CM_TO_METER_FACTOR / grid.resolution_ +.5);
          int32_t yi =   grid.rows_ / 2 - (int32_t) (bin_[l][b].beam[k]->point.y * CM_TO_METER_FACTOR / grid.resolution_ +.5);
          PerceptionCell* cell = grid.getRCLocal(yi, xi);
          if (cell) {
#ifdef USE_GRID_SEGMENTER
#pragma omp critical
            {
              perception_.cellToPoints().insert(Perception::map_type::value_type((uintptr_t) cell, (uintptr_t) bin_[l][b].beam[k]));
            }
#endif

            //            c++;
//            float z = bin_[l][b].beam[k]->point.z + robot_pose.z();
            float z = bin_[l][b].beam[k]->point.z * CM_TO_METER_FACTOR + robot_pose.z();
            setCellMin(*cell, z, counter);
            setCellMax(*cell, z, counter);

            cell->last_observed = counter;
            if (cell->last_obstacle != counter) {
              float h = z - cell->min;
              if (h < perception_.settings().overpass_height) {
                if (bin_[l][b].beam[k]->range > VELODYNE_MIN_RANGE_S) {
                  //                  printf("%d, %d\n", bin[l][b].beam[k]->point->x, bin[l][b].beam[k]->point->y );
                  if (bin_[l][b].beam[k]->obstacle) {
#pragma omp critical
                    {
                      perception_.staticObstacles()->cell[perception_.staticObstacles()->num++] = cell;
                    }

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
        }
        else {
          // TODO: add proper handling of unhit cells....
        }
      }
    }
  }

  assert(perception_.staticObstacles()->num < MAX_NUM_POINTS);
  displayTime("Cells", time);
 // printf("labeled %d cells as obstacles\n", perception_.staticObstacles()->num);
}

// Add obstacles near car that the Velodyne can't see
void VelodyneClient::labelObstacleCellsNear(int64_t counter, const std::vector<velodyne::Block>& blocks) {
  static const int near_obstacle_buffer = 10;
  static const float near_obstacle_radius = 4.2;

  double time = drc::Time::current();

  int16_t origin_r, origin_c;
  dgc::dgc_transform_t t;
  double velodyne_x = 0.0, velodyne_y = 0.0, velodyne_z = 0.0;

  drc::GlobalPose robot_pose = perception_.pose(blocks[0].timestamp);

  dgc::dgc_transform_rpy(t, config_.offset_, robot_pose.roll(), robot_pose.pitch(), robot_pose.yaw());
  dgc::dgc_transform_translate(t, robot_pose.x(), robot_pose.y(), robot_pose.z());
  dgc::dgc_transform_point(&velodyne_x, &velodyne_y, &velodyne_z, t);

  float velodyne_height = config_.offset_[2][3] + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT;
  // TODO: figure out why we need to add 40 cm here to get a realistic ground_z
  float ground_z = perception_.pose(time).z() - DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT + config_.offset_[2][3];

  PerceptionCell* origin_cell = perception_.grid().getXY(velodyne_x, velodyne_y);
  perception_.grid().cellToRCLocal(origin_cell, &origin_r, &origin_c);

  int invisible_radius = ceil(near_obstacle_radius / perception_.grid().resolution_);
  int invisible_radius2 = invisible_radius * invisible_radius;
  for (int r = -invisible_radius; r <= invisible_radius; r++) {
    int r2 = r * r;
    for (int c = -invisible_radius; c <= invisible_radius; c++) {
      int c2 = c * c;
      int d2 = (r2 + c2);
      if (d2 > invisible_radius2) continue;

      PerceptionCell* cell = perception_.grid().getRCLocal(origin_r + r, origin_c + c);
      if (cell && (cell->last_observed != counter) && (cell->last_obstacle > 0) && ((cell->last_observed - cell->last_obstacle) < near_obstacle_buffer)
          && ((cell->last_obstacle - cell->last_dynamic) > near_obstacle_buffer) && (cell->hits > 2)) {
        // remove obstacles that we should be able to see because of their height,
        // this solves the problem of high obstacles passing through/near the occluded area and leaving behind static obstacles
        double distance = sqrt(d2) * perception_.grid().resolution_;
        double max_z = ((near_obstacle_radius - distance) / near_obstacle_radius) * velodyne_height + ground_z;

        // following 2 lines are for visualization purposes only:
        //        cell->max = max_z;
        //        cell->min = ground_z;

        if (cell->max > max_z) continue;

        perception_.staticObstacles()->cell[perception_.staticObstacles()->num] = cell;
        if (perception_.staticObstacles()->num < MAX_NUM_POINTS) {
          perception_.staticObstacles()->num++;
        }
      }
    }
  }

  displayTime("Cells Near", time);
}

//void label_obstacle_points_from_cells()
//{
//  // create a hash look up from obstacle cells
//  __gnu_cxx::hash_set<int> obstacle_set;
//
//  for (int i=0; i < perception_.staticObstacles()->num; i++) {
//    obstacle_set.insert((int)perception_.staticObstacles()->cell[i]);
//  }
//
//  int c = 0;
//  __gnu_cxx::hash_set<int>::iterator end =  obstacle_set.end();
//  for (int l=0; l<NUM_LASER_BEAMS; l++) {
//    for (int k=0; k < lscan_[l].num_points; k++) {
//      lscan_[l].laser_point[k].obstacle = (obstacle_set.find((int)lscan_[l].laser_point[k].cell) != end);
//    }
//  }
//
//  printf("%d points labeled as obstacles\n", c);
//}

void VelodyneClient::integrate(const std::vector<velodyne::Block>& blocks, unsigned short counter) {

  printf("now (%f) - ts (%f) = dt: %f\n", drc::Time::current(), velodyne_ts_, drc::Time::current() - velodyne_ts_);
  if (!blocks.empty() && (!perception_.settings().clear_sensor_data || drc::Time::current() - velodyne_ts_ < perception_.settings().max_sensor_delay)) {

    process(blocks);
    labelObstaclePoints();
    labelObstacleCells(counter);
    labelObstacleCellsNear(counter, blocks);

    //  segmentVelodyne();
    //  draw_segments();
    //  draw_velodyne();
    last_velodyne_ts_ = velodyne_ts_;
  }
}

} // namespace perception

