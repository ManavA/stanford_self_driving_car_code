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


#include <perception.h>
//#include "utils.h"
#include <velodyne_rings.h>

#ifdef MULTITHREAD
#include "omp.h"
#endif

#define    MAX_NUM_VELODYNE_BLOCKS   50000
#define    BINS_PER_REV             720
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


using namespace dgc;

namespace drc = driving_common;

namespace perception {

//float      velodyne_upper_block_max_range_diff  = 200.0;
//float      velodyne_lower_block_max_range_diff  = -15.0;
//
//typedef struct {
//  int                 num_beams;
//  laser_point_p     * beam;
//} beam_bin_t;
//
//beam_bin_t           bin[NUM_LASER_BEAMS][BINS_PER_REV];
//
//typedef struct {
//  int     idx;      /* velodyne index beam */
//  int     pb;       /* partner beam for the comparison */
//  float   v_angle;  /* vertical angle of the laser beam */
//  float   h_angle;  /* horizontal angle of the laser beam */
//  int     h_offset; /* horizontal offset of the beam in velodyne ticks */
//  float   fac;      /* approximation factor of function with pb */
//} velodyne_ring_settings_t;
//
//#define OCCUPIED    1
//#define FREE        0
//
//#define NUM_SAMPLES    1000
//#define EPSILON        0.000000001
//#define MAX_RANGE      70.0
//
//typedef __gnu_cxx::hash_multimap<uintptr_t, uintptr_t> map_type;
//
//#ifdef USE_GRID_SEGMENTER
//map_type cell_to_points;
//#endif
//
//void display_time(char* label, double time)
//{
//  int ms = (int)((driving_common::Time::current() - time) * 1000);
//  if (strlen(label) < 9)
//    printf("#TIME: %s\t\t%02d", label, ms);
//  else
//    printf("#TIME: %s\t%02d", label, ms);
//
//  if (ms > 20)
//    printf(" * ");
//
//  printf("\n");
//}
//
//
///*
// * Used to filter obviously bad returns.
// * Assume the ground slopes down at no more than 11 degrees
// * and that all returns below ground are invalid
// *
// * Values are recalculated directly from config file on each load.
// * These values are from a particular config file (ID89.cal)
// */
//unsigned short max_valid_range[] =
//{
//  0,    0,    0,    0,    0,    0,    0,    0,
//  0,    0,    0,    0,    0,    0,    0,    0,
//  0,    0,    0,    0,    0,    0,    0,    0,
//  0,    0,    0,    0,    0,    0,    0,    0,
//  1025, 1070, 0,    0,    1117, 1171, 868,  904,
//  1245, 1336, 940,  996,  2225, 2400, 1431, 1473,
//  2650, 3002, 1582, 1676, 3542, 4293, 1841, 2007,
//  0,    0,    5453, 6522, 0,    0,    8513, 13529
//};
//
//
//void label_obstacle_points_rings()
//{
//  int l, b, l0, l1;
////  int c;
////  float threshold, range0, range1;
//
//  double max_range = settings_.velodyne_max_range * 100.0;
//  double factor = 0.01 * settings_.velodyne_threshold_factor;
//
////  c = 0;
//  #pragma omp parallel for private(l, l0, l1, b)
//  for (l=0; l<NUM_LASER_BEAMS-1; l++) {
//    if (l != rings->partnerBeam(l)) {
//      l1 = rings->beamToIndex(l); // ring[l].idx;            /* shorter beam */
//      l0 = rings->partnerIndex(l); // ring[ring[l].pb].idx;   /* longer beam */
//      for( b = 0; b < BINS_PER_REV; b++) {
//        for(int k=0; k<bin[l0][b].num_beams; k++) {
//          float range0 = bin[l0][b].beam[k]->point->range;
//          if (bin[l0][b].beam[k]->valid && (range0 < max_range * 100.0)) {
//            for(int m=0; m<bin[l1][b].num_beams; m++) {
//              if (bin[l1][b].beam[m]->valid) {
//                float range1 = bin[l1][b].beam[m]->point->range;
//                float threshold = rings->factor(l) * range1 * range1 * factor;
//                if ( fabs(range1-range0) < threshold ) {
//                  bin[l0][b].beam[k]->obstacle = TRUE;
////                  bin[l1][b].beam[m]->obstacle = TRUE;
////                  c++;
////                  break;
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
////  printf("%d points labeled as obstacles\n", c);
//}
//
//void label_obstacle_points_threshold(double ground_threshold)
//{
//  short th = short(ground_threshold * 100);
////
//  int count = 0;
//  int total = 0;
//  for (int l=0; l<NUM_LASER_BEAMS; l++) {
//    total += lscan[l].num_points;
//    for (int k=0; k < lscan[l].num_points; k++) {
//      lscan[l].laser_point[k].obstacle = (lscan[l].laser_point[k].point->z > th);
//      count += (lscan[l].laser_point[k].point->z > th);
//    }
//  }
////  printf("%d / %d points are obstacles\n", count, total);
//}
//
//unsigned short min_ground_range[] =
//{
//    65000, 65000, 65000, 65000, 65000, 65000, 65000, 4559,
//    4022,  3718,  3336,  3000,  2711,  2479,  2287,  2151,
//    2004,  1921,  1842,  1741,  1657,  1590,  1510,  1380,
//    1306,  1258,  1265,  1238,  1078,  1045,   983,   944,
//    917,   829,   810,   759,   756,   687,   684,   622,
//    630,   569,   594,   539,   539,   494,   515,   462,
//    471,   427,   436,   401,   436,   380,   401,   363,
//    377,   343,   354,   333,   347,   321,   332,   308
//};
//
//unsigned short min_ground_range_adjusted[] =
//{
//    65000, 65000, 65000, 65000, 65000, 65000, 65000, 4559,
//    4022,  3718,  3336,  3000,  2711,  2479,  2287,  2151,
//    2004,  1921,  1842,  1741,  1657,  1590,  1510,  1380,
//    1306,  1258,  1265,  1238,  1078,  1045,   983,   944,
//    917,   829,   810,   759,   756,   687,   684,   622,
//    630,   569,   594,   539,   539,   494,   515,   462,
//    471,   427,   436,   401,   436,   380,   401,   363,
//    377,   343,   354,   333,   347,   321,   332,   308
//};
//
//unsigned short max_ground_range = 7000; // 70 m
//
//void label_obstacle_points_range() {
////  static bool init = true;
////  if (init) {
////    for (int i=0; i < NUM_LASER_BEAMS; i++)
////      min_ground_range_adjusted[i] = (short)( min_ground_range[i] * 0.75);
////    init = false;
////  }
////
////  int l;
////
////  #pragma omp parallel for
////  for (l=0; l<NUM_LASER_BEAMS; l++) {
////    for (int i = 0; i < lscan[l].num_points; i++) {
////      short r = lscan[l].laser_point[i].point->range;
////      int b = rings->indexToBeam(l); //ridx[l];
////      if ((r < min_ground_range_adjusted[b]) || (r > max_ground_range))
////        lscan[l].laser_point[i].obstacle = 1;
////    }
////  }
//}
//
////TODO: cut from 8ms to 5ms
//void label_obstacle_points_terrain() {
//
//// EXPAND_Z_GRID adds about 3ms to processing time, and
//// it does a single step of expansion on z_grid
////#define EXPAND_Z_GRID
//
//#ifdef EXPAND_Z_GRID
//  __gnu_cxx::hash_set<int> open;
//#endif
//
//  static bool init = true;
//  static short obstacle_threshold = 0;
//  if (init) {
//    obstacle_threshold = (short)(settings_.z_obstacle_height / 0.01);
//    init = false;
//  }
//
//  static short max_z = std::numeric_limits<short>::max();
//  int l;
//  #pragma omp parallel for
//  for (l=0; l<NUM_LASER_BEAMS; l++) {
//    for (int i = 0; i < lscan[l].num_points; i++) {
//      short* grid_z = (short*)getXY(z_grid, lscan[l].laser_point[i].point->x, lscan[l].laser_point[i].point->y);
//      if (grid_z) {
//        *grid_z = max_z;
//        lscan[l].laser_point[i].z_cell = grid_z;
//      }
//    }
//  }
//
//  #pragma omp parallel for
//  for (l=0; l<NUM_LASER_BEAMS; l++) {
//    for (int i = 0; i < lscan[l].num_points; i++) {
//      short* grid_z = lscan[l].laser_point[i].z_cell;
//      if (grid_z) {
//        *grid_z = std::min(*grid_z, lscan[l].laser_point[i].point->z);
//#ifdef EXPAND_Z_GRID
//        open.insert((int)grid_z);
//#endif
//      }
//    }
//  }
//
////  printf("open size: %d\n", open.size());
//
//#ifdef EXPAND_Z_GRID
//  __gnu_cxx::hash_set<int>::iterator end = open.end();
//  for (__gnu_cxx::hash_set<int>::iterator it = open.begin(); it != end; it++) {
//    short* cell = (short*) *it;
//    //    printf("processing %d %d\n", (int)cell, *cell);
//    int r,c;
//    cellToRCLocal(z_grid, cell, &r, &c);
//
//    // look at four neighborhood
//    short* neighbor = (short*)getRCLocal(z_grid, r+1, c);
////    if ((neighbor) && (*cell > (*neighbor + 20)))
//    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
//      *cell = *neighbor + 20;
//
//    neighbor = (short*)getRCLocal(z_grid, r-1, c);
//    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
//      *cell = *neighbor + 20;
//
//    neighbor = (short*)getRCLocal(z_grid, r, c-1);
//    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
//      *cell = *neighbor + 20;
//
//    neighbor = (short*)getRCLocal(z_grid, r, c+1);
//    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
//      *cell = *neighbor + 20;
//  }
//#endif
//
//  #pragma omp parallel for
//  for (int l=0; l<NUM_LASER_BEAMS; l++) {
//    for (int i = 0; i < lscan[l].num_points; i++) {
//      short z = lscan[l].laser_point[i].point->z;
//      short* grid_z = lscan[l].laser_point[i].z_cell;
//      if ((grid_z) && ( (z - *grid_z) > obstacle_threshold))
//         lscan[l].laser_point[i].obstacle = 1;
//    }
//  }
//}
//
////void label_obstacle_points_terrain_range()
////{
////  static bool init = true;
////  static short obstacle_threshold = 0;
////  if (init) {
////    obstacle_threshold = (short)(settings_.terrain_obstacle_height / 0.01);
////
////    for (int i=0; i < NUM_LASER_BEAMS; i++)
////      min_ground_range_adjusted[i] = (short)( min_ground_range[i] * 0.8);
////
////    init = false;
////  }
////
////  clear(z_grid);
////  for (int l=0; l<NUM_LASER_BEAMS; l++) {
////    for (int i = 0; i < lscan[l].num_points; i++) {
////      if (!lscan[l].laser_point[i].valid)
////        continue;
////      double x = lscan[l].laser_point[i].point->x * CM_TO_METER_FACTOR;
////      double y = lscan[l].laser_point[i].point->y * CM_TO_METER_FACTOR;
////      short z = lscan[l].laser_point[i].point->z;
////      short* grid_z;
////      grid_z = (short*)getXY(z_grid, x, y);
////      if (grid_z)
////        *grid_z = std::min(*grid_z, z);
////      lscan[l].laser_point[i].z_cell = grid_z;
////    }
////  }
////
////  for (int l=0; l<NUM_LASER_BEAMS; l++) {
////    for (int i = 0; i < lscan[l].num_points; i++) {
////      short z = lscan[l].laser_point[i].point->z;
////      short* grid_z = lscan[l].laser_point[i].z_cell;
////      if ((grid_z) && ( (z - *grid_z) > obstacle_threshold))
////         lscan[l].laser_point[i].obstacle = 1;
////      else {
////        short r = lscan[l].laser_point[i].point->range;
////        int b = ridx[l];
////        if ((r < min_ground_range_adjusted[b]) || (r > max_ground_range))
////          lscan[l].laser_point[i].obstacle = 1;
////      }
////    }
////  }
////}
//
//void report_num_threads(int level)
//{
////  #pragma omp single
////  {
////    printf("Level %d: number of threads in the team - %d\n", level, omp_get_num_threads());
////  }
//}
//
//
//void points_in_cell(PerceptionCell* cell, std::vector<point3d_t>& points) {
//#ifdef USE_GRID_SEGMENTER
//  points.reserve(cell_to_points.count((uintptr_t)cell));
//  __gnu_cxx::pair<map_type::iterator, map_type::iterator> p = cell_to_points.equal_range((uintptr_t)cell);
//  point3d_t point;
//  for (map_type::iterator it = p.first; it != p.second; it++) {
//    laser_point_p pt = (laser_point_p)(it->second);
//    point.x = pt->point->x * CM_TO_METER_FACTOR + pt->scan->robot.x;
//    point.y = pt->point->y * CM_TO_METER_FACTOR + pt->scan->robot.y;
//    point.z = pt->point->z * CM_TO_METER_FACTOR + pt->scan->robot.z;
//    point.intensity = pt->point->intensity;
//    points.push_back(point);
//  }
//#endif
//}
//
////void label_obstacle_points_from_cells()
////{
////  // create a hash look up from obstacle cells
////  __gnu_cxx::hash_set<int> obstacle_set;
////
////  for (int i=0; i < obstacles_s->num; i++) {
////    obstacle_set.insert((int)obstacles_s->cell[i]);
////  }
////
////  int c = 0;
////  __gnu_cxx::hash_set<int>::iterator end =  obstacle_set.end();
////  for (int l=0; l<NUM_LASER_BEAMS; l++) {
////    for (int k=0; k < lscan[l].num_points; k++) {
////      lscan[l].laser_point[k].obstacle = (obstacle_set.find((int)lscan[l].laser_point[k].cell) != end);
////    }
////  }
////
////  printf("%d points labeled as obstacles\n", c);
////}
//
//
//void
//test_clusters(int num_scans, dgc_velodyne_scan_p scans, dgc_velodyne_config_p config, double ground_threshold, std::vector<laser_cluster_p>& clusters, double x, double y)
//{
//  VelodyneData v;
//
//  v.config = config;
//  v.num_scans = num_scans;
//  v.allocated_scans = num_scans;
//  v.scans = scans;
//  v.preprocessed = 0;
//
//  static int                   firsttime = 1;
//
//  if (v.num_scans==0)
//    return;
//
//  if (firsttime) {
//    initialize(&v);
//    firsttime = 0;
//  }
//
//  process_velodyne(&v);
//  label_obstacle_points_threshold(ground_threshold);
////  test_segmentVelodyne(x, y);
////  collect_obstacles(clusters);
//}

} // namespace perception
