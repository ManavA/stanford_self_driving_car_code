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
#include <applanix/ApplanixPose.h>
#include <grid.h>
#include <lltransform.h>
#include <velo_support.h>
#include <terrainmap.h>

#define MAX_RANGE 50.0

using namespace dgc;
using namespace vlr;

ros::NodeHandle* nh_ = NULL;
tf::TransformListener* tf_listener_=NULL;

  // parameters
std::string cal_filename_;
std::string int_filename_;
bool calibrate_intensities_ = true;
dgc_transform_t velodyne_offset_;

bool first = true;
double last_x = 0, last_y = 0;

typedef struct {
  terrain_tile* tile;
} grid_cell_t;

dgc_grid_p tile_grid = NULL;

FILE *rndf = NULL;

void rndf_create(const std::string& name) {
  char fname[255];
  sprintf(fname, "%s.rndf", name.c_str());
  rndf = fopen(fname, "w");
  fprintf(rndf, "RNDF_name %s\n", name.c_str());
  fprintf(rndf, "num_segments\t1\n");
  fprintf(rndf, "num_zones\t0\n");
  fprintf(rndf, "format_version\t1.0\n");
  fprintf(rndf, "creation_date\t06/25/2009\n");
  fprintf(rndf, "segment\t1\n");
  fprintf(rndf, "num_lanes\t1\n");
  fprintf(rndf, "lane\t1.1\n");
  fprintf(rndf, "num_waypoints\t2000\n");
  fprintf(rndf, "lane_width\t12\n");
}

void rndf_add_waypoint(applanix::ApplanixPose* pose) {
  static int w_num = 1;
  static float last_yaw = 0.0;
  static double last_x = 0.0, last_y = 0.0;
  double utm_x = 0, utm_y = 0;
  std::string utm_zone;
  vlr::latLongToUtm(pose->latitude, pose->longitude, &utm_x, &utm_y, utm_zone);
  float dist = hypot(utm_x - last_x, utm_y - last_y);
  if (dist > 2.0 && (fabs(pose->yaw - last_yaw) > M_PI / 60 || dist > 30.0)) {
    printf("1.1.%d\t%.7lf\t%.7lf\n", w_num, pose->latitude, pose->longitude);
    fprintf(rndf, "1.1.%d\t%.7lf\t%.7lf\n", w_num, pose->latitude, pose->longitude);
    last_x = utm_x;
    last_y = utm_y;
    last_yaw = pose->yaw;
    w_num++;
  }
  else {
    printf("Skipping waypoint. Distance: %f    Yaw: %f\n", dist, pose->yaw);
  }
}

void rndf_close() {
  fprintf(rndf, "end_lane\n");
  fprintf(rndf, "end_segment\n");
  fclose(rndf);
}

void grid_clear_handler(void *cell) {
  terrain_tile *tile = ((grid_cell_t *) cell)->tile;
  if (tile != NULL) {
    fprintf(stderr, "Saving %s\n", tile->filename);
    tile->save(tile->filename);
    delete ((grid_cell_t *) cell)->tile;
  }
}

int spin_counter = 0;
int move_counter = 0;

void my_spin_func(VelodyneSpin* spin, VelodyneConfig* config, applanix::ApplanixPose* applanix_pose) {
  //printf("spin!\n");
  int r, c, tile_y, tile_x, beam_num, ring_num;
  double p_x, p_y, p_z, utm_x, utm_y;
  grid_cell_t *cell;
  std::string utm_zone;
  terrain_tile_cell *grid_cell = NULL;
  static double utm_offset_x = 0, utm_offset_y = 0;
  static double utm_offset_x0 = 0, utm_offset_y0 = 0;
  double u;

  spin_counter++;

  /* if(spin_counter > 100)
   return; */

  if (spin->numScans() <= 0) {
    fprintf(stderr, "Warning: spin has zero scans.  Shouldn't happen\n");
    return;
  }

  vlr::latLongToUtm(applanix_pose->latitude, applanix_pose->longitude, &utm_x, &utm_y, utm_zone);

  utm_offset_x = utm_x - applanix_pose->smooth_x;
  utm_offset_y = utm_y - applanix_pose->smooth_y;
  if (first) {
    utm_offset_x0 = utm_offset_x;
    utm_offset_y0 = utm_offset_y;
  }

  if (first || hypot(utm_x - last_x, utm_y - last_y) > 1.0) {
    last_x = utm_x;
    last_y = utm_y;
    dgc_grid_recenter_grid(tile_grid, utm_x, utm_y);
    rndf_add_waypoint(applanix_pose);
  }
  first = 0;

  if (fabs(applanix_pose->speed) < dgc_mph2ms(1.0)) {
    //printf("Skipping, speed is %f\n", applanix_pose->speed);
    return;
  }
  //printf("  moving spin!\n");
  move_counter++;

  for (size_t i = 0; i < spin->numScans(); i++) {
    for (int32_t j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      /* skip bad readings */
      if (spin->scans_[i].p[j].range < 1.0) continue;

      if (spin->scans_[i].p[j].range * .01 > MAX_RANGE) // skip long readings
      continue;
      beam_num = j + spin->scans_[i].block * 32;
      if (spin->scans_[i].block == 6) continue;

      ring_num = config->inv_beam_order[beam_num];

      /* project beam */
      //p_x = spin->scans_[i].p[j].x * 0.01 + spin->scans_[i].robot.x;
      //p_y = spin->scans_[i].p[j].y * 0.01 + spin->scans_[i].robot.y;
      p_x = spin->scans_[i].p[j].x * 0.01 + spin->scans_[i].robot.x + utm_offset_x;
      p_y = spin->scans_[i].p[j].y * 0.01 + spin->scans_[i].robot.y + utm_offset_y;
      p_z = spin->scans_[i].p[j].z * 0.01 + 0 * spin->scans_[i].robot.z;

      /* find the beam's terrain tile */
      r = (int) floor(p_y / tile_grid->resolution);
      c = (int) floor(p_x / tile_grid->resolution);
      cell = (grid_cell_t *) dgc_grid_get_rc_global(tile_grid, r, c);
      if (cell != NULL) {
        /* initialize the tile */
        if (cell->tile == NULL) {
          cell->tile = new terrain_tile(TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE);
          sprintf(cell->tile->filename, "lmap-%s-%d-%d-%06d-%06d.tf.gz", utm_zone.c_str(), (int) rint(TERRAIN_TILE_RESOLUTION * 100), TERRAIN_TILE_SIZE, c, r);
          fprintf(stderr, "Loading %s\n", cell->tile->filename);
          if (cell->tile->load(cell->tile->filename) < 0) {
            //	    fprintf(stderr, "  Could not load file.\n");
            cell->tile->utm_x0 = c * tile_grid->resolution;
            cell->tile->utm_y0 = r * tile_grid->resolution;
            cell->tile->resolution = TERRAIN_TILE_RESOLUTION;
            strcpy(cell->tile->utmzone, utm_zone.c_str());
          }
        }

        tile_x = (int) floor((p_x - cell->tile->utm_x0) / TERRAIN_TILE_RESOLUTION);
        tile_y = (int) floor((p_y - cell->tile->utm_y0) / TERRAIN_TILE_RESOLUTION);
        grid_cell = &(cell->tile->cell[tile_x][tile_y]);

        u = spin->scans_[i].p[j].intensity;
        grid_cell->intensity += u;
        grid_cell->i_count++;
        if (i > 3 && i < spin->numScans() - 3) {
          grid_cell->z_count++;
          double dx = spin->scans_[i + 2].p[j].x - spin->scans_[i - 2].p[j].x;
          double dy = spin->scans_[i + 2].p[j].y - spin->scans_[i - 2].p[j].y;
          double points_theta = atan2(dy, dx);
          double ray_theta = atan2(spin->scans_[i].p[j].y, spin->scans_[i].p[j].x);
          double curbness = fabs(cos(points_theta - ray_theta));
          curbness *= 1;
          //grid_cell->z += .0005 * fabs(applanix_pose->speed) * pow(curbness, 5) * (pow(.01 * spin->scans_[i].p[j].range, 1.0));
          grid_cell->z_response += pow(curbness, 6) * .0005 * fabs(applanix_pose->speed) * spin->scans_[i].p[j].range;
          if (p_z < grid_cell->z) grid_cell->z = p_z;
          if (p_z > grid_cell->current_highest) grid_cell->current_highest = p_z;
        }
        p_z = u;
        if (grid_cell->n == 0) {
          grid_cell->n = 1;
          grid_cell->xa = p_z;
          grid_cell->sxi2 = p_z * p_z;
        }
        else {
          grid_cell->n++;
          grid_cell->xa = ((grid_cell->n - 1) * grid_cell->xa + p_z) / grid_cell->n;
          grid_cell->sxi2 += p_z * p_z;
        }
        grid_cell->stdev = pow((grid_cell->sxi2 / grid_cell->n - grid_cell->xa * grid_cell->xa), .5);
      }
    }
  }
}

template <class T> void getParam(std::string key, T& var) {
  if(!nh_->getParam(key, var)) {
    throw Exception("Cannot read parameter " + key + std::string("."));
  }
}

void getParamTransform(std::string key, dgc::dgc_transform_t& tr) {
  ros::Time now = ros::Time::now();
  tf_listener_->waitForTransform("Applanix", key, now, ros::Duration(3.0));

  tf::StampedTransform transform;
  try {
    tf_listener_->lookupTransform("Applanix", key, ros::Time(0), transform);
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

void readParameters() {
  getParamTransform("Velodyne", velodyne_offset_);
  getParam("velodyne/cal_file", cal_filename_);
  getParam("velodyne/calibrate_intensities", calibrate_intensities_);

  if (calibrate_intensities_) {
    getParam("velodyne/int_file", int_filename_);
  }
  else {
    std::cout << "WARNING: not calibrating Velodyne intensities.\n";
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, argv[0]);

  if (argc < 3) {
    std::cout << "Not enough arguments.\nUsage: " << argv[0] << " log-file velodyne-file\n";
  }

  nh_ = new ros::NodeHandle("/driving");
  tf_listener_ = new tf::TransformListener;

  readParameters();

  grid_cell_t* default_tile = (grid_cell_t *) calloc(1, sizeof(grid_cell_t));
  dgc_test_alloc(default_tile);

  tile_grid = dgc_grid_initialize(TERRAIN_TILE_RESOLUTION * TERRAIN_TILE_SIZE, TERRAIN_GRID_NUM_TILES, TERRAIN_GRID_NUM_TILES, sizeof(grid_cell_t), default_tile);
  dgc_grid_set_clear_handler(tile_grid, grid_clear_handler);

  rndf_create("OneLane");
  vlr::vlf_projector(argv[2], argv[1], cal_filename_, int_filename_, velodyne_offset_, my_spin_func);
  dgc_grid_clear(tile_grid);
  std::cout << "Map-making complete. Processed " << spin_counter << " velodyne spins, " << move_counter << " moving\n";
  rndf_close();
  return 0;
}
