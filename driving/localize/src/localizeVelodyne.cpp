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


#include <sstream>
#include <iomanip>

#include <global.h>
#include <lltransform.h>
//#include <velodyne_interface.h>
#include <velodyne/Projected.h>
#include <applanix/ApplanixPose.h>
#include <localize/LocalizePose.h>
#include <gls.h>
//#include <velocore.h>
#include <velo_support.h>

#include <veloClient.h>
#include <localizeVelodyne.h>

using namespace dgc;

vlr::VelodyneLocalizer* velo=NULL;

namespace drc = driving_common;

namespace vlr {

  // c helper functions
void tile_clear_handler(void* cell) {
  TerrainTile* tile = ((VelodyneLocalizer::grid_cell_t*) cell)->tile;
  if (tile != NULL) {
    delete ((VelodyneLocalizer::grid_cell_t *) cell)->tile;
  }
}

VelodyneLocalizer::VelodyneLocalizer() : nh_("/driving"), outlog_(NULL), gls_("localize_velodyne"), quit_localizer_(false),
                                         grid_(NULL), grid_updated_(false), has_localized_(false),
                                         timestamp_(0), last_x_offset_(0), last_y_offset_(0),
                                         utm_x_(0), utm_y_(0), //smooth_x_(0), smooth_y_(0), smooth_z_(0), yaw_(0), speed_(0),
                                         tile_grid_(NULL), received_applanix_pose_(false), applanix_pose_queue_(2000),
                                         sensor_data_(NULL), map_data_(NULL) {
  posterior_mutex_ = PTHREAD_MUTEX_INITIALIZER;
  velodyne_mutex_ = PTHREAD_MUTEX_INITIALIZER;
  applanix_pose_mutex_ = PTHREAD_MUTEX_INITIALIZER;
  map_data_ =  new TerrainTileCell*[DIM*DIM];
  memset(map_data_, 0, DIM*DIM*sizeof(TerrainTileCell*));
  sensor_data_ = new map_cell_t[DIM*DIM];
  diff_array_= new float[DIM * DIM];
  stdev_array_= new float[DIM * DIM];

  initializeGrid();
  initializeTileGrid();
  initializePosterior();

  readParameters();
  initColors();
  initGaussians();

  applanix_sub_ = nh_.subscribe("ApplanixPose", 5, &VelodyneLocalizer::applanixPoseHandler, this);
  localize_pose_pub_ = nh_.advertise<localize::LocalizePose> ("LocalizePose", 5);

  pthread_create(&velodyne_thread_id_, NULL,
      threadCBWrapper<VelodyneLocalizer, &VelodyneLocalizer::velodyneThread>, this);
//  pthread_create(&applanix_thread_id_, NULL,
//      threadCBWrapper<VelodyneLocalizer, &VelodyneLocalizer::applanixThread>, this);
  pthread_create(&localize_thread_id_, NULL,
      threadCBWrapper<VelodyneLocalizer, &VelodyneLocalizer::localizeThread>, this);
}

VelodyneLocalizer::~VelodyneLocalizer() {
  quit_localizer_ = true;
  if (outlog_) {fclose(outlog_);}

  delete[] diff_array_;
  delete[] stdev_array_;
  delete map_data_;
  delete sensor_data_;
  fprintf(stderr, "\nDisconnecting.\n");
}

void VelodyneLocalizer::run() {
  ros::spin();
}

void VelodyneLocalizer::initColors() {
  for (int32_t i = 0; i < 50; i++) {
    r_[i] = .5 + .01 * i;
    g_[i] = .5 + .01 * i;
    b_[i] = .5 - .01 * i;
  }
  for (int32_t i = 50; i <= 100; i++) {
    r_[i] = 1;
    g_[i] = 2 - .02 * i;
    b_[i] = 0;
  }
}

void VelodyneLocalizer::initializeGrid() {
    // allocate rolling grid_
  map_cell_t* default_map_cell = (map_cell_t*) calloc(1, sizeof(map_cell_t));
  dgc_test_alloc(default_map_cell);
  default_map_cell->intensity_sum = 0;
  default_map_cell->intensity_count = 0;

  grid_ = dgc_grid_initialize(RES, DIM, DIM, sizeof(map_cell_t), default_map_cell);
  fprintf(stderr, "Grid memory required: %.2f MB\n", grid_->rows * grid_->cols * grid_->bytes_per_cell / 1024.0 / 1024.0);
}

void VelodyneLocalizer::initializeTileGrid() {
  grid_cell_t* default_tile = (grid_cell_t *) calloc(1, sizeof(grid_cell_t));
  tile_grid_ = dgc_grid_initialize(TERRAIN_TILE_RESOLUTION * TERRAIN_TILE_SIZE, TERRAIN_GRID_NUM_TILES, TERRAIN_GRID_NUM_TILES, sizeof(grid_cell_t),
      default_tile);
  dgc_grid_set_clear_handler(tile_grid_, tile_clear_handler);
}

double VelodyneLocalizer::normalProbability(double a, double b) { // probability of a given stdev b
  //float p = pow(2 * 3.14159 * b * b, -.5) * exp(-.5 * a * a / (b * b));
  float p = exp(-.5 * a * a / (b * b));
  //if(p < .001) return .001;
  return p;
}

void VelodyneLocalizer::initGaussians() {
  for (int32_t i = 0; i < 100; i++) {
    for (int32_t j = 0; j < 40; j++) {
      float p = 2 * normalProbability(1.0 * i, 1.0 * j);
      if (p < .02) {p = .02;}
      gaussian_weights_[i][j] = log(p);
      //printf("%d %d: %.6f %.6f\n", i, j, normalProbability(1.0*i,1.0*j), gaussian_weights_[i][j]);
    }
  }
}

void VelodyneLocalizer::plot2D() {
  //printf("plotting 2D histogram\n");

  pthread_mutex_lock(&applanix_pose_mutex_);
  applanix::ApplanixPose ap = applanix_pose_;
  double utm_x = utm_x_;
  double utm_y = utm_y_;
  pthread_mutex_unlock(&applanix_pose_mutex_);

  gls_.clear();
  gls_.coordinates = driving_common::GLSOverlay::LOCAL_COORDINATES;
  gls_.color3f(1, 1, 0);
  gls_.lineWidth(2.0);
  double max_z = 0, sum_z = 0;

  for (int32_t xpos = 0; xpos < GRID_RADIUS; xpos++) {
    for (int32_t ypos = 0; ypos < GRID_RADIUS; ypos++) {
      if (hist_z_[xpos][ypos] > 0) sum_z += hist_z_[xpos][ypos];
      if (hist_z_[xpos][ypos] > max_z) max_z = hist_z_[xpos][ypos];
    }
  }

  //max_z = sum_z / 50;
  double dx0 = -1 * (last_x_offset_ - utm_x + ap.smooth_x);
  double dy0 = -1 * (last_y_offset_ - utm_y + ap.smooth_y);
  for (int32_t xpos = 0; xpos < GRID_RADIUS; xpos++) {
    gls_.begin(driving_common::GLSOverlay::LINE_STRIP);
    for (int32_t ypos = 0; ypos < GRID_RADIUS; ypos++) {
      if ((xpos - GRID_RADIUS / 2) * (xpos - GRID_RADIUS / 2) + (ypos - GRID_RADIUS / 2) * (ypos - GRID_RADIUS / 2) > GRID_RADIUS * GRID_RADIUS / 4) continue;
      float dx = dx0 + (2 * xpos - GRID_RADIUS) * RES;
      float dy = dy0 + (2 * ypos - GRID_RADIUS) * RES;
      double z = hist_z_[xpos][ypos] / max_z;
      double d = pow(dx * dx + dy * dy, .5);
      double theta = M_PI / 2 + atan2(dy, dx) - ap.yaw;
      double dlat = d * cos(theta);
      double dlon = d * sin(theta);
      int32_t c = (int32_t) (100 * z);
      gls_.color3f(r_[c], g_[c], b_[c]);
      gls_.vertex3f(dlon, dlat, 2 * z - 1.4);
    }
    gls_.end();
  }
  for (int32_t ypos = 0; ypos < GRID_RADIUS; ypos++) {
    gls_.begin(driving_common::GLSOverlay::LINE_STRIP);
    for (int32_t xpos = 0; xpos < GRID_RADIUS; xpos++) {
      if ((xpos - GRID_RADIUS / 2) * (xpos - GRID_RADIUS / 2) + (ypos - GRID_RADIUS / 2) * (ypos - GRID_RADIUS / 2) > GRID_RADIUS * GRID_RADIUS / 4) continue;
      float dx = dx0 + (2 * xpos - GRID_RADIUS) * RES;
      float dy = dy0 + (2 * ypos - GRID_RADIUS) * RES;
      double z = hist_z_[xpos][ypos] / max_z;
      double d = pow(dx * dx + dy * dy, .5);
      double theta = M_PI / 2 + atan2(dy, dx) - ap.yaw;
      double dlat = d * cos(theta);
      double dlon = d * sin(theta);
      int32_t c = (int32_t) (100 * z);
      gls_.color3f(r_[c], g_[c], b_[c]);
      gls_.vertex3f(dlon, dlat, 2 * z - 1.4);
    }
    gls_.end();
  }
  for (int32_t xpos = -GRID_RADIUS; xpos < GRID_RADIUS; xpos++) {
    gls_.begin(driving_common::GLSOverlay::LINE_STRIP);
    for (int32_t ypos = 0; ypos < GRID_RADIUS; ypos++) {
      int32_t x = xpos + ypos;
      int32_t y = ypos;
      if (x < 0 || x >= GRID_RADIUS) continue;
      if ((x - GRID_RADIUS / 2) * (x - GRID_RADIUS / 2) + (y - GRID_RADIUS / 2) * (y - GRID_RADIUS / 2) > GRID_RADIUS * GRID_RADIUS / 4) continue;
      float dx = dx0 + (2 * x - GRID_RADIUS) * RES;
      float dy = dy0 + (2 * y - GRID_RADIUS) * RES;
      double z = hist_z_[x][y] / max_z;
      double d = pow(dx * dx + dy * dy, .5);
      double theta = M_PI / 2 + atan2(dy, dx) - ap.yaw;
      double dlat = d * cos(theta);
      double dlon = d * sin(theta);
      int32_t c = (int32_t) (100 * z);
      gls_.color3f(r_[c], g_[c], b_[c]);
      gls_.vertex3f(dlon, dlat, 2 * z - 1.4);
    }
    gls_.end();
  }
  for (int32_t xpos = 0; xpos < 2 * GRID_RADIUS; xpos++) {
    gls_.begin(driving_common::GLSOverlay::LINE_STRIP);
    for (int32_t ypos = 0; ypos < GRID_RADIUS; ypos++) {
      int32_t x = xpos - ypos;
      int32_t y = ypos;
      if (x < 0 || x >= GRID_RADIUS) continue;
      if ((x - GRID_RADIUS / 2) * (x - GRID_RADIUS / 2) + (y - GRID_RADIUS / 2) * (y - GRID_RADIUS / 2) > GRID_RADIUS * GRID_RADIUS / 4) continue;
      float dx = dx0 + (2 * x - GRID_RADIUS) * RES;
      float dy = dy0 + (2 * y - GRID_RADIUS) * RES;
      double z = hist_z_[x][y] / max_z;
      double d = pow(dx * dx + dy * dy, .5);
      double theta = M_PI / 2 + atan2(dy, dx) - ap.yaw;
      double dlat = d * cos(theta);
      double dlon = d * sin(theta);
      int32_t c = (int32_t) (100 * z);
      gls_.color3f(r_[c], g_[c], b_[c]);
      gls_.vertex3f(dlon, dlat, 2 * z - 1.4);
    }
    gls_.end();
  }
  /* gls_.color3f(1, 0, 1);
   glsLineWidth(gls_, 8.0);
   gls_.begin(driving_common::GLSOverlay::LINE_STRIP);
   gls_.vertex3f(0, 0, -2.0);
   gls_.vertex3f(0, 0, .5);
   gls_.end(); */

  gls_.send();
  return;
}

void VelodyneLocalizer::normalizePosterior() { // mutex should be locked when this is called
  double sum = 0.00000000000000000000000000000000001;
  for (int32_t i = 0; i <= GRID_RADIUS * 2; i++) {
    for (int32_t j = 0; j <= GRID_RADIUS * 2; j++) {
      //posterior_[i][j] += .000000000001 / (GRID_RADIUS*GRID_RADIUS);
      sum += posterior_[i][j];
    }
  }
  //printf("Sum: %10f\n", sum);
  for (int32_t i = 0; i <= GRID_RADIUS * 2; i++) {
    for (int32_t j = 0; j <= GRID_RADIUS * 2; j++) {
      posterior_[i][j] /= sum;
    }
  }

  for (int32_t i = 0; i <= GRID_RADIUS * 2; i++) {
    for (int32_t j = 0; j <= GRID_RADIUS * 2; j++) {
      //printf("%.6f ", posterior_[i][j]);
    }
    //printf("\n");
  }
  //printf("\n");
}

void VelodyneLocalizer::motionModel(double speed) {
  //printf("motion model! Traveling at %f m/s\n", speed);
  pthread_mutex_lock(&posterior_mutex_);

    // motion model stuff here
  for (int32_t i = 0; i <= GRID_RADIUS * 2; i++) {
    for (int32_t j = 0; j <= GRID_RADIUS * 2; j++) {
      posterior1_[i][j] = 0.0;
      if ((i - GRID_RADIUS) * (i - GRID_RADIUS) + (j - GRID_RADIUS) * (j - GRID_RADIUS) > GRID_RADIUS * GRID_RADIUS) continue;
      double weight_sum = 0.0;
      for (int32_t x = -2; x <= 2; x++) {
        for (int32_t y = -2; y <= 2; y++) {
          int32_t col = j + y;
          int32_t row = i + x;
          if ((col - GRID_RADIUS) * (col - GRID_RADIUS) + (row - GRID_RADIUS) * (row - GRID_RADIUS) > GRID_RADIUS * GRID_RADIUS) continue;
          if (col < 0 || col > 2 * GRID_RADIUS || row < 0 || row > 2 * GRID_RADIUS) continue;
          double dist = pow(x * x + y * y, .5);
          double weight = normalProbability(dist, MOTION_NOISE * (speed + 1));
          posterior1_[i][j] += weight * posterior_[row][col];
          weight_sum += weight;
        }
      }
      posterior1_[i][j] /= weight_sum;
    }
  }
  for (int32_t i = 0; i <= GRID_RADIUS * 2; i++) {
    for (int32_t j = 0; j <= GRID_RADIUS * 2; j++) {
      posterior_[i][j] = posterior1_[i][j];
    }
  }

  normalizePosterior();

  pthread_mutex_unlock(&posterior_mutex_);
}

void VelodyneLocalizer::initializePosterior() {
  printf("initializing posterior_\n");

  for (int32_t i = 0; i < GRID_RADIUS * 3; i++) {
    for (int32_t j = 0; j < GRID_RADIUS * 3; j++) {
      posterior_[i][j] = 0;
      if ((i - GRID_RADIUS) * (i - GRID_RADIUS) + (j - GRID_RADIUS) * (j - GRID_RADIUS) <= GRID_RADIUS * GRID_RADIUS) posterior_[i][j] = 1.0;
    }
  }
  //posterior_[10][10] = 2000.0;
  normalizePosterior();
}

void VelodyneLocalizer::precomputePointers() {
  //printf("Precomputing pointers...\n");
  pthread_mutex_lock(&applanix_pose_mutex_);
  applanix::ApplanixPose ap = applanix_pose_;
  double utm_x = utm_x_;
  double utm_y = utm_y_;
  pthread_mutex_unlock(&applanix_pose_mutex_);

  pthread_mutex_lock(&velodyne_mutex_);
  for (int32_t row = DIM / 8 - GRID_RADIUS; row < 7 * DIM / 8 + GRID_RADIUS; row++) {
    for (int32_t col = DIM / 8 - GRID_RADIUS; col < 7 * DIM / 8 + GRID_RADIUS; col++) {
      map_cell_t* cell = (map_cell_t*) dgc_grid_get_rc_local(grid_, row, col); // sensor data cell

      double x, y;
      dgc_grid_cell_to_xy(grid_, cell, &x, &y);
      x = utm_x + x - ap.smooth_x;
      y = utm_y + y - ap.smooth_y;

      int32_t r2, c2;
      r2 = (int32_t) floor(y / tile_grid_->resolution);
      c2 = (int32_t) floor(x / tile_grid_->resolution);
      grid_cell_t *tile_cell = (grid_cell_t *) dgc_grid_get_rc_global(tile_grid_, r2, c2);
      if (tile_cell == NULL) {
        printf("%s: error: null tile!?\n", __PRETTY_FUNCTION__);
        printf("row = %d, col = %d   x = %f, y = %f\n", row, col, x - utm_x, y - utm_y);
        continue;
      }
      if (tile_cell->tile == NULL) { // need to initialize it
        tile_cell->tile = new TerrainTile(TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE);
        // @TODO: Get actual directory the graphic laser imagery is in for this

        std::stringstream s;
        s << imagery_root_ << "/" << "lasermap" <<"/lmap-"<< UTM_ZONE << "-";
        s << (int32_t) rint(TERRAIN_TILE_RESOLUTION * 100) << "-" << TERRAIN_TILE_SIZE << "-";
        s << std::setfill('0') << std::setw(6) << c2 << "-" <<  std::setw(6) << r2 << ".tf.gz";
       tile_cell->tile->fileName() = s.str();
//            sprintf(tile_cell->tile->fileName(), "%s/%s/lmap-%s-%d-%d-%06d-%06d.tf.gz", imagery_root_.c_str(), "lasermap", UTM_ZONE,
//            (int32_t) rint(TERRAIN_TILE_RESOLUTION * 100), TERRAIN_TILE_SIZE, c2, r2);

        fprintf(stderr, "Loading %s\n", tile_cell->tile->fileName().c_str());
        if (!tile_cell->tile->load(tile_cell->tile->fileName())) {
          printf("... nothing here\n");
          tile_cell->tile->utmX0() = c2 * tile_grid_->resolution;
          tile_cell->tile->utmY0() = r2 * tile_grid_->resolution;
          tile_cell->tile->resolution() = TERRAIN_TILE_RESOLUTION;
          tile_cell->tile->utmZone() =  UTM_ZONE;
        }
      }
      int32_t tile_x = (int32_t) floor((x - tile_cell->tile->utmX0()) / TERRAIN_TILE_RESOLUTION);
      int32_t tile_y = (int32_t) floor((y - tile_cell->tile->utmY0()) / TERRAIN_TILE_RESOLUTION);
      TerrainTileCell* grid_cell = &(tile_cell->tile->cell()[tile_x][tile_y]);

      sensor_data_[DIM * row + col] = *cell;
      map_data_[DIM * row + col] = grid_cell;
    }
  }
  pthread_mutex_unlock(&velodyne_mutex_);
  //printf("  .... done\n");
}

float VelodyneLocalizer::correlation(float* x, float* y, int32_t n) {
  if (n < 4) {return 0.0;}

  double xsum = 0, ysum = 0;
  for (int32_t i = 0; i < n; i++) {
    xsum += x[i];
    ysum += y[i];
  }

  double xavg = xsum / n;
  double yavg = ysum / n;
  double xvar = 0, yvar = 0, cor = 0;

  for (int32_t i = 0; i < n; i++) {
    double dx = x[i] - xavg;
    double dy = y[i] - yavg;
    xvar += dx * dx;
    yvar += dy * dy;
    cor += dx * dy;
  }

  double xstd = pow(xvar / n, .5);
  double ystd = pow(yvar / n, .5);

  if (xstd == 0 || ystd == 0) {return 0;}

  return cor / ((n - 1) * xstd * ystd);
}

  // compute alignment strength between map tiles and current grid with cell offset dx, dy
double VelodyneLocalizer::alignStrength(int32_t dx, int32_t dy) { // turbo means skip every other pixel in both directions
  map_cell_t cell;  // TODO: need to be copied?!?
  double log_sum = 0.0;
  int32_t xrand = rand() % 2; // TODO: ?!?
  int32_t yrand = rand() % 2;
  float map_sum = 0.0, sens_sum = 0.0;
  int32_t align_count = 0;
  for(int32_t row = yrand + DIM / 6; row < 5 * DIM / 6; row += 2) {
    for(int32_t col = xrand + DIM / 6; col < 5 * DIM / 6; col += 2) {
      cell = sensor_data_[DIM * row + col];
      TerrainTileCell* grid_cell = map_data_[DIM * (row + dy) + col + dx];
      if (cell.intensity_count * grid_cell->i_count == 0) {
        continue;
      }
      else {
        float intensity_map = grid_cell->intensity / grid_cell->i_count;
        float intensity_current = cell.intensity_sum / cell.intensity_count;
        map_sum += intensity_map;
        sens_sum += intensity_current;
        stdev_array_[align_count] = (int32_t) (grid_cell->stdev + cell.stdev);
        diff_array_[align_count++] = intensity_current - intensity_map;
      }
    }
  }

  float map_average = map_sum / (align_count + 1);
  float sens_average = sens_sum / (align_count + 1);
  //printf("Map average: %.2f   Sensor average: %.2f\n", map_average, sens_average);
  float to_add = sens_average - map_average;
  float align_intensity_sum = 0.0;
  for (int32_t j = 0; j < align_count; j++) {
    align_intensity_sum += fabs(to_add - diff_array_[j]);
    int32_t a = abs((int32_t) (to_add - diff_array_[j]));
    if (a > 99) {a = 99;}
    int32_t b = stdev_array_[j];
    if (b < 5) {b = 5;}         // TODO: why not symmetric?!?
    else if (b > 39) {b = 39;}
    log_sum += SENSOR_WEIGHT * gaussian_weights_[a][b];
  }

  //printf("old sum: %.6f  ", log_sum);
  //log_sum /= SENSOR_WEIGHT;
  //log_sum /= (align_count + 1);
  //log_sum *= 50;
  //printf("  new sum: %.6f matches: %d\n", log_sum, align_count);
  //float corr = correlation(map_array, sens_array, align_intensity_count) + .2;
  double gps_score = normalProbability(hypot(dx * RES, dy * RES), GPS_ERR);
  double gauss_score = exp(log_sum);
  //printf("    score: %10f\n", smart_score * gps_score);
  return gauss_score * gps_score;
}

void VelodyneLocalizer::computeBestOffset() {
  //printf("Localizing!!!\n");

  double t0 = drc::Time::current();
  precomputePointers();
  double t1 = drc::Time::current();
  if (DEBUG) printf("   t1 = %f\n", t1 - t0);
  int32_t delta = 2, radius = GRID_RADIUS; // units = grid_ cells
  int32_t offset_x = 0, offset_y = 0;
  int32_t best_x = -1000, best_y = -1000;
  double best_strength = -1000;
  double measurement_model[GRID_RADIUS * 3][GRID_RADIUS * 3];
  int32_t i, j;
  for (i = 0; i <= GRID_RADIUS * 2; i++) {
    for (j = 0; j <= GRID_RADIUS * 2; j++) {
      measurement_model[i][j] = 0;
    }
  }
  int32_t xpos = 0, ypos = 0;
  for (offset_x = -radius + 1; offset_x < radius; offset_x += delta, xpos++) {
    ypos = 0;
    for (offset_y = -radius + 1; offset_y < radius; offset_y += delta, ypos++) {
      double strength = 0.0;
      if (offset_x * offset_x + offset_y * offset_y < (radius - 2) * (radius - 2)) {
        strength = alignStrength(offset_x, offset_y);
      }
      hist_z0_[xpos][ypos] = pow(strength, .05);

      measurement_model[2 * ypos][2 * xpos] = strength;
      measurement_model[2 * ypos][2 * xpos + 1] = strength;
      measurement_model[2 * ypos + 1][2 * xpos] = strength;
      measurement_model[2 * ypos + 1][2 * xpos + 1] = strength;

      //printf("(%.2f, %.2f) --> %f\n", offset_x * RES, offset_y * RES, strength);
      if (strength > best_strength) {
        best_x = offset_x;
        best_y = offset_y;
        best_strength = strength;
      }
    }
  }
  double t2 = drc::Time::current();
  //printf("BEST coarse: (%.2f, %.2f) --> %f\n", best_x*RES, best_y*RES, best_strength);
  if (DEBUG) printf("   t2 = %f\n", t2 - t1);
  int32_t best_x2 = best_x, best_y2 = best_y;
  for (offset_x = best_x - 1; offset_x <= best_x + 1; offset_x++) {
    for (offset_y = best_y - 1; offset_y <= best_y + 1; offset_y++) {
      double strength = alignStrength(offset_x, offset_y);
      //printf("(%.2f, %.2f) --> %f\n", offset_x * RES, offset_y * RES, strength);
      measurement_model[offset_y + radius - 1][offset_x + radius - 1] = strength;

      if (strength > best_strength) {
        best_x2 = offset_x;
        best_y2 = offset_y;
        best_strength = strength;
      }
    }
  }
  pthread_mutex_lock(&posterior_mutex_);
  int32_t best_x3 = 0, best_y3 = 0;
  double best_posterior = -1000000000000.0;
  for (i = 0; i <= GRID_RADIUS * 2; i++) {
    for (j = 0; j <= GRID_RADIUS * 2; j++) {
      //hist_z_[i][j] = hist_z0_[i][j];
      posterior_[i][j] *= pow(measurement_model[i][j], MEASUREMENT_WEIGHT);
    }
  }
  normalizePosterior();
  for (i = 0; i <= GRID_RADIUS * 2; i++) {
    for (j = 0; j <= GRID_RADIUS * 2; j++) {
      if (posterior_[i][j] > best_posterior) {
        best_posterior = posterior_[i][j];
        best_x3 = j + 1 - radius;
        best_y3 = i + 1 - radius;
        //printf("NEW BEST: %d, %d\n", best_x3, best_y3);
      }
      hist_z_[i / 2][j / 2] = posterior_[j][i];
    }
  }

  //printf("New best: %d, %d\n", best_x3, best_y3);
  double best_x4 = 0, best_y4 = 0;
  double weight_sum = 0.00000001;
  for (i = best_y3 - 2; i <= best_y3 + 2; i++) {
    for (j = best_x3 - 2; j <= best_x3 + 2; j++) {
      //printf("i, j = %d, %d\n", i, j);
      int32_t x = j + radius - 1;
      int32_t y = i + radius - 1;
      if (x < 0 || y < 0 || x > 2 * GRID_RADIUS || y > 2 * GRID_RADIUS) continue;
      //printf("   this is okay\n");
      double weight = pow(posterior_[y][x], 2);
      weight_sum += weight;
      best_x4 += j * weight;
      best_y4 += i * weight;
    }
  }
  //printf("pre-best4: %f, %f      sum: %f\n", best_x4, best_y4, weight_sum);
  best_x4 /= weight_sum;
  best_y4 /= weight_sum;
  pthread_mutex_unlock(&posterior_mutex_);
  double t3 = drc::Time::current();
  if (DEBUG) printf("   t3 = %f\n", t3 - t2);
  localize_x_offset_ = best_x4 * RES;
  localize_y_offset_ = best_y4 * RES;
  printf("BEST refined: (%.2f, %.2f) --> %f\n", best_x3 * RES, best_y3 * RES, best_strength);
  printf("    BEST ultra-refined: (%.2f, %.2f)\n", localize_x_offset_, localize_y_offset_);
  //printf("   t_total = %f\n", t3 - t0);
}

void VelodyneLocalizer::trackApplanix() {

  if(!received_applanix_pose_) {return;}

  static bool first = true;
  static double last_x = 0, last_y = 0;
  static double last_motion_time = 0;

  pthread_mutex_lock(&applanix_pose_mutex_);
  applanix::ApplanixPose ap = applanix_pose_;
  double utm_x = utm_x_;
  double utm_y = utm_y_;
  pthread_mutex_unlock(&applanix_pose_mutex_);

  if (first) {
    pthread_mutex_lock(&velodyne_mutex_);
    dgc_grid_recenter_grid(grid_, ap.smooth_x, ap.smooth_y);
    pthread_mutex_unlock(&velodyne_mutex_);
  }
  if (first || hypot(utm_x - last_x, utm_y - last_y) > 5.0) {
    //printf("Recentering tile grid_\n");
    dgc_grid_recenter_grid(tile_grid_, utm_x, utm_y);
    last_x = utm_x;
    last_y = utm_y;
  }
  first = false;
  if (timestamp_ > last_motion_time + 0.1 && ap.speed > .1) {
    last_motion_time = ap.timestamp;
    motionModel(ap.speed);
  }
}

void VelodyneLocalizer::localize() {
  if (!received_applanix_pose_) {return;}

  static bool first = true;
  static double last_time = 0;
  static double last_x = 0;
  static double last_y = 0;

  pthread_mutex_lock(&applanix_pose_mutex_);
  applanix::ApplanixPose ap = applanix_pose_;
  pthread_mutex_unlock(&applanix_pose_mutex_);

  if (first || (hypot(ap.smooth_x - last_x, ap.smooth_y - last_y) > LOCALIZE_DISTANCE_INTERVAL && ap.timestamp - last_time > LOCALIZE_TIME_INTERVAL)) {
    //printf("Sup yo, time dif is %f\n", timestamp - last_time);
    computeBestOffset();
    last_x = ap.smooth_x;
    last_y = ap.smooth_y;
    last_time = ap.timestamp;
    first = false;
    has_localized_ = true;
  }
}

void VelodyneLocalizer::publish() {
  if (!received_applanix_pose_ || !has_localized_) {return;}

  static double last_time = 0, last_log_time = 0;
  static bool first = true;
  //static double last_x_offset_ = 0, last_y_offset_ = 0;

  if (first) {
    outlog_ = fopen("localizelog.txt", "w");
    if (!outlog_) {
      fprintf(stderr, "Error: cannot open localizelog.txt for writing. Make sure you have write permission in this directory.\nDisconnecting.\n");
      exit(0);
    }
    fprintf(outlog_, "ts\tutm_x\tutm_y\tdx\tdy\tdlat\tdlon\n");
  }
  if (first || drc::Time::current() - last_time > PUBLISH_INTERVAL) {
    //printf("Actually publish!\n");
    localize::LocalizePose pose_message;
    pose_message.utmzone = UTM_ZONE;
    pthread_mutex_lock(&applanix_pose_mutex_);
    double next_x_offset = utm_x_ + localize_x_offset_ - applanix_pose_.smooth_x;
    double next_y_offset = utm_y_ + localize_y_offset_ - applanix_pose_.smooth_y;
    if (last_x_offset_) {
      next_x_offset = SMOOTHNESS * last_x_offset_ + (1 - SMOOTHNESS) * next_x_offset;
      next_y_offset = SMOOTHNESS * last_y_offset_ + (1 - SMOOTHNESS) * next_y_offset;
    }
    pose_message.x_offset = next_x_offset;
    pose_message.y_offset = next_y_offset;
    if (timestamp_ - last_log_time > 1.0) { // log every second
      double dx = next_x_offset - utm_x_ + applanix_pose_.smooth_x;
      double dy = next_y_offset - utm_y_ + applanix_pose_.smooth_y;
      double d = pow(dx * dx + dy * dy, .5);
      double theta = M_PI / 2 + atan2(dy, dx) - applanix_pose_.yaw;
      double dlat = d * cos(theta);
      double dlon = d * sin(theta);
      printf("lat: %.2f   lon: %.2f   mag: %.2f\n", dlat, dlon, d);
      fprintf(outlog_, "%lf\t%lf\t%lf\t%f\t%f\t%f\t%f\t%f\t%f\n", timestamp_, next_x_offset + applanix_pose_.smooth_x, next_y_offset + applanix_pose_.smooth_y, applanix_pose_.smooth_x + dx, applanix_pose_.smooth_y + dy,
          dx, dy, dlat, dlon);
      last_log_time = applanix_pose_.timestamp;
    }
    pthread_mutex_unlock(&applanix_pose_mutex_);

    pose_message.std_x = pose_message.std_y = pose_message.std_s = 0;
    publishPoseMessage(pose_message);

    first = false;
    last_time = drc::Time::current();
    last_x_offset_ = next_x_offset;
    last_y_offset_ = next_y_offset;

    if (SHOW_GLS) {plot2D();}
  }
}

void VelodyneLocalizer::publishPoseMessage(localize::LocalizePose& pose) {
  pose.timestamp = drc::Time::current();
  localize_pose_pub_.publish(pose);
}

  // currently unused
void* VelodyneLocalizer::applanixThread() {
  printf("Applanix thread says hi!\n");
  while (!quit_localizer_) {
    trackApplanix();
    usleep(1000);
  }
  return NULL;
}

void* VelodyneLocalizer::localizeThread() {
  printf("Localize thread says hi!\n");
  while (!quit_localizer_) {
    localize();
    usleep(1000);
  }
  return NULL;
}

void* VelodyneLocalizer::velodyneThread() {
  printf("Hello velodyne thread!\n");

  VelodyneClient* velodyne = new VelodyneClient(cal_filename_, int_filename_, velodyne_offset_);
  velodyne::Spin* vspin = new velodyne::Spin;

  while (!received_applanix_pose_) {
    usleep(1000);
  }

  while (!quit_localizer_) {
    while (velodyne->blocksAvailable()) {
      //printf("VELODYNE DATA!!\n");
      pthread_mutex_lock(&velodyne_mutex_);
      velodyne->readSpin(vspin);

      /* put the intensitysity data in the rolling grid_ */
      if (vspin->numBlocks() > 0) {
        //printf("ACTUAL VELODYNE DATA!\n");
        pthread_mutex_lock(&applanix_pose_mutex_);
        drc::GlobalPose robot_pose = applanix_pose_queue_.pose(vspin->blocks_[0].timestamp);
        pthread_mutex_unlock(&applanix_pose_mutex_);

        dgc_grid_recenter_grid(grid_, robot_pose.x(), robot_pose.y());

        pthread_mutex_lock(&applanix_pose_mutex_);
        double applanix_vel = applanix_pose_.speed;
        pthread_mutex_unlock(&applanix_pose_mutex_);

            for (size_t i = 0; i < vspin->numBlocks(); i++) {
              pthread_mutex_lock(&applanix_pose_mutex_);  // TODO: Move mutex locking off the loop ?!?
              robot_pose = applanix_pose_queue_.pose(vspin->blocks_[i].timestamp);
              pthread_mutex_unlock(&applanix_pose_mutex_);
              for (int32_t j = 0; j < velodyne::Block::NUM_BEAMS; j++) {
                int32_t beam_num = j + vspin->blocks_[i].block * 32;
//                int32_t ring_num = velodyne->config().inv_beam_order[beam_num];

                if (beam_num == 0 || beam_num == 6 || beam_num == 10 || beam_num == 24) continue;
                if (vspin->blocks_[i].laser[j].distance < 0.01) continue;
                if (std::abs(applanix_vel) < dgc_mph2ms(1.0)) continue;

                double p_x = vspin->blocks_[i].point[j].x * 0.01 + robot_pose.x();
                double p_y = vspin->blocks_[i].point[j].y * 0.01 + robot_pose.y();
                double p_z = vspin->blocks_[i].point[j].z * 0.01 + robot_pose.z();

                map_cell_t* cell = (map_cell_t*) dgc_grid_get_xy(grid_, p_x, p_y);
                if (cell != NULL) {
                  int32_t intensity = vspin->blocks_[i].laser[j].intensity;
                  if (intensity > 1) {
                    cell->intensity_sum += intensity;
                    cell->intensity_count++;
                    if (cell->n == 0) {
                      cell->n = 1;
                      cell->xa = p_z;
                      cell->sxi2 = p_z * p_z;
                    }
                    else {
                      cell->n++;
                      cell->xa = ((cell->n - 1) * cell->xa + p_z) / cell->n;
                      cell->sxi2 += p_z * p_z;
                    }
                    cell->stdev = pow((cell->sxi2 / cell->n - cell->xa * cell->xa), .5);
                  }
                }
              }
            }
            grid_updated_ = true;
          }
          pthread_mutex_unlock(&velodyne_mutex_);
        }
        usleep(1000);
      }

      delete vspin;
      delete velodyne;

  return NULL;
}

void VelodyneLocalizer::applanixPoseHandler(const applanix::ApplanixPose& applanix_pose) {
  pthread_mutex_lock(&applanix_pose_mutex_);
//  smooth_x_ = applanix_pose.smooth_x;
//  smooth_y_ = applanix_pose.smooth_y;
//  smooth_z_ = applanix_pose.smooth_z;
//  yaw_ = applanix_pose.yaw;
//  speed_ = applanix_pose.speed;
//  timestamp_ = applanix_pose.timestamp;
//  vlr::latLongToUtm(applanix_pose.latitude, applanix_pose.longitude, &utm_x_, &utm_y_, utm_zone_);
  applanix_pose_ = applanix_pose;
  applanix_pose_queue_.push(drc::GlobalPose(applanix_pose.smooth_x, applanix_pose.smooth_y, applanix_pose.yaw), applanix_pose.timestamp);
  received_applanix_pose_ = true;
  pthread_mutex_unlock(&applanix_pose_mutex_);

  trackApplanix();
  publish();
}

template <class T> void VelodyneLocalizer::getParam(std::string key, T& var) {
  if(!nh_.getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + std::string("."));
  }
}

void VelodyneLocalizer::getParamTransform(std::string key, dgc::dgc_transform_t& tr) {
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

void VelodyneLocalizer::readParameters() {
  getParam("imagery_root", imagery_root_);
  getParamTransform("Velodyne", velodyne_offset_);
  getParam("velodyne/cal_file", cal_filename_);
  getParam("velodyne/calibrate_intensities", calibrate_intensities_);
  getParam("velodyne/int_file", int_filename_);

  if (!calibrate_intensities_) {
    printf("WARNING: not calibrating Velodyne intensities.\n");
  }
}

} // namespace vlr

int32_t main(int argc, char** argv) {

  ros::init(argc, argv, "localize");

  try {
    vlr::VelodyneLocalizer velo;
    velo.run();
  } catch( vlr::Ex<>& e ) {
    std::cout << e.what() << std::endl;
  }

  return 0;
}
