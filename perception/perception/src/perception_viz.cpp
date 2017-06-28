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
#include "laser_segment.h"


using std::tr1::shared_ptr;

using namespace dgc;
using namespace vlr;
using namespace vlr::rndf;

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
char cars_filename[300];

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

int points_close_threshold_g = 1200;

inline bool points_close(laser_point_p prev, laser_point_p point) {
  int dx = prev->point->x - point->point->x;
  int dy = prev->point->y - point->point->y;
  return ((dx*dx + dy*dy) < (points_close_threshold_g));
}

#define BUFFER 0.15
int point_inside_cars(laser_point_p p) {

  if (!p->obstacle)
    return 0;

  int num_cars = cars.num_cars();
  for (int c = 0; c < num_cars; c++) {
    car_t car = cars.car[c];
    double car_x,car_y,car_th;
    bool extrapolated;
    if (car.estimate_pose(current_spin_num, &car_x, &car_y, &car_th, &extrapolated)) {
      dgc_transform_t t;
      dgc_transform_identity(t);
      dgc_transform_translate(t, -car_x, -car_y, 0);
      dgc_transform_rotate_z(t, -car_th);

      double x = p->point->x * 0.01 + p->scan->robot.x;
      double y = p->point->y * 0.01 + p->scan->robot.y;
      double z = p->point->z * 0.01 + p->scan->robot.z;

      dgc_transform_point(&x, &y, &z, t);
      if ((fabs(x) < ((car.l+ BUFFER) / 2.0)) && (fabs(y) < ((car.w+BUFFER) / 2.0)))
        return c + 1;
    }
  }
  return 0;
}

float score_segmentation()
{
  double t1 = dgc_get_time();
  int fp = 0, tp = 0, fn = 0, tn =0;
  // scan along each scan line
  for(int l=0; l<NUM_LASER_BEAMS; l++) {
    laser_point_p prev_point = NULL;
    int prev_point_inside = 0;

    laser_point_p point = NULL;
    int point_inside = 0;

    // find first point
    int num_points = lscan[l].num_points;
    prev_point = &lscan[l].laser_point[0];
    prev_point_inside = point_inside_cars(prev_point);
    for (int i=1; i<num_points; i++) {
      point = &lscan[l].laser_point[i];
      point_inside = point_inside_cars(point);
      bool close = points_close(point, prev_point);

      if (point_inside == prev_point_inside) {
        if (point_inside > 0) {
          if (close)
            tp++;
          else
            fn++;
        }

      } else if (point_inside != prev_point_inside) {
        if (close)
          fp++;
        else
          tn++;
      }

      prev_point = point;
      prev_point_inside = point_inside;
    }
  }

  float accuracy = (float)(tp + tn) / (tp + tn + fp + fn);
  float score = (float)(tp + tn) / (tp + tn + 5.0 * fp + fn);
  printf("tp: %d\t fp: %d\t tn:%d\t fn:%d\n", tp, fp, tn, fn);
  printf("fp rate: %f\t fn rate: %f\t accuracy: %f\t score: %f\n", (float)fp/(fp+tn), (float)fn/(fn+tp), accuracy, score);

//  printf("time: %d\n", (dgc_get_time() - t1) * 100);
  return score;
}

void train_segmentation() {
  float max_score = 0.0;
  int max_threshold = 1200;
  for (int i = 50; i < 2000; i += 50) {
    points_close_threshold_g = i;
    float score = score_segmentation();
    if (score > max_score) {
      max_score = score;
      max_threshold = points_close_threshold_g;
      printf("max_threshold: %d\nmax_score: %f\n", max_threshold, max_score);
    }
  }

  printf("max_threshold: %d\nmax_score: %f\n", max_threshold, max_score);
}

/*********************************************************************
 *
 *   Graphics
 *
 *********************************************************************/
//vlr::rndf::RoadNetwork   * rn = NULL;
rndf_display_list        * rndf_dl = NULL;

char *imagery_root;

dgc_passatwagonmodel_t* passat = NULL;
int large_points = 1;
int color_mode = 0;
int draw_flat = 0;
int show_distance = 0;
int show_z = 0;
int show_velodyne = 0;
int show_passat = 1;
int show_rndf = 1;
int show_segmentation = 1;
int show_obstacle = 1;
int show_obstacles = 1;
int show_tracked = 1;
int show_cars = 1;
int show_path = 1;

typedef struct {
  double    r;
  double    g;
  double    b;
} dgc_rgb_t, *dgc_rgb_p;

#define NUM_COLORS 16

unsigned char colors[16][3] = {
  {255,0,0},
  {0,255,0},
  {0,0,255},
  {255,255,0},
  {0,255,255},
  {255,0,255},
  {128,51,128},
  {51,128,128},
  {255,51,51},
  {51,255,51},
  {51,51,255},
  {51,179,204},
  {128,255,51},
  {255,128,51},
  {51,128,255},
  {0,0,0},
};


void keyboard(unsigned char key, int x, int y)
{
  double applanix_lat, applanix_lon, applanix_alt, scene_x, scene_y;
  int delta = 0;
  float score;

  /* figure out where the user clicked on the plane */
  gui3D_pick_point(x, y, &scene_x, &scene_y);
  scene_x += velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  scene_y += velodyne_index.spin[current_spin_num].pose[0].smooth_y;

  if(key >= '1' && key <= '9')
    delta = key - '0';
  else
    switch(key) {
    case 'i': case 'I':
      dgc_imagery_cycle_imagery_type();
      break;
    case 'd':
      show_distance = !show_distance;
      break;
    case 'z':
      show_z = 1 - show_z;
      break;
    case ',':
      show_velodyne = !show_velodyne;
      break;
    case 'p':
      show_path = 1 - show_path;
      break;
    case 'c':
      color_mode++;
      if (color_mode > 8)
        color_mode = 0;
      break;
    case 'l':
      show_cars = 1 - show_cars;
      break;
    case 'r':
      show_rndf = !show_rndf;
      break;
    case 27: case 'q': case 'Q':
      exit(0);
      break;

    case 'S':
      score = score_segmentation();
      break;

    case 'T':
      train_segmentation();
      break;

    case 's':
      show_segmentation = !show_segmentation;
      break;
    case 't':
      show_tracked = !show_tracked;
      break;
    case 'o':
      show_obstacles = !show_obstacles;
      break;
    case '!':
      delta = -1;
      break;
    case '@':
      delta = -2;
      break;
    case '#':
      delta = -3;
      break;
    case '$':
      delta = -4;
      break;
    case '%':
      delta = -5;
      break;
    case '^':
      delta = -6;
      break;
    case '&':
      delta = -7;
      break;
    case '*':
      delta = -8;
      break;
    case '(':
      delta = -9;
      break;
    case '0':
      delta = 100;
      break;
    case 'f':
      draw_flat = !draw_flat;
      break;
    default:
      break;
    }

  if(delta) {
    current_spin_num += delta;
    if(current_spin_num >= velodyne_index.num_spins)
      current_spin_num = velodyne_index.num_spins - 1;
    if(current_spin_num < 0)
      current_spin_num = 0;

    if (current_spin_num > 0) {
      if (delta == 1)
        last_spin.copy(spin);
      else
        last_spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num - 1,
            &applanix_lat, &applanix_lon, &applanix_alt);
    } else {
      last_spin.num_scans = 0;
    }

    spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num,
        &applanix_lat, &applanix_lon, &applanix_alt);

    run_tracker();
  }

  gui3D_forceRedraw();

}

void draw_grid(dgc_grid_p grid, double origin_x, double origin_y)
{
  glPushMatrix();
  glTranslatef(origin_x, origin_y, DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT);
  glScalef(0.01, 0.01, 0.01);
  glBegin(GL_QUADS);

  double resolution = grid->resolution;
  double dr = resolution / 2.0;

  glColor4f(0.0, 0.0, 0.0, 0.5);
  for (int c=0; c<grid->cols; c++) {
    for (int r=0; r<grid->rows; r++) {
      double x, y;
      rcLocalToXY(grid, r, c, &x, &y);
      short z = *((short*)getRCLocal(grid, r, c));
      glVertex3f(x-dr, y-dr, z);
      glVertex3f(x-dr, y+dr, z);
      glVertex3f(x+dr, y+dr, z);
      glVertex3f(x+dr, y-dr, z);
    }
  }

  glEnd();
  glPopMatrix();
}


#define    NUM_LASER_BEAMS 64
void draw_points(double origin_x, double origin_y, double origin_z, int flat)
{
  if (lscan == NULL)
    return;

  if (color_mode == 8) {
    glBegin(GL_LINES);
    for(int l=0; l<NUM_LASER_BEAMS; l++) {
      laser_point_p prev_point = NULL;
      int prev_point_inside = 0;

      laser_point_p point = NULL;
      int point_inside = 0;

      double x,y,z;
      double prev_x,prev_y,prev_z;

      // find first point
      int num_points = lscan[l].num_points;
      prev_point = &lscan[l].laser_point[0];
      prev_point_inside = point_inside_cars(prev_point);
      prev_x = prev_point->point->x * 0.01 + prev_point->scan->robot.x;
      prev_y = prev_point->point->y * 0.01 + prev_point->scan->robot.y;
      prev_z = prev_point->point->z * 0.01 + prev_point->scan->robot.z;
      for (int i=1; i<num_points; i++) {
        point = &lscan[l].laser_point[i];
        point_inside = point_inside_cars(point);
        bool close = points_close(point, prev_point);

        x = point->point->x * 0.01 + point->scan->robot.x;
        y = point->point->y * 0.01 + point->scan->robot.y;
        z = point->point->z * 0.01 + point->scan->robot.z;

        if (point_inside == prev_point_inside) {
          if (point_inside > 0) {
            if (close)
              glColor3f(0, 0.8, 0);
            else
              glColor3f(0.8, 0.0, 0);
          } else {
            glColor3f(0.0, 0.0, 0.8);
          }

        } else if (point_inside != prev_point_inside) {
          if (close)
            glColor3f(0.8, 0.0, 0.8);
          else
            glColor3f(0, 0, 0.8);
        }

        glVertex3f(prev_x - origin_x, prev_y - origin_y, prev_z - origin_z);
        glVertex3f(x - origin_x, y - origin_y, z - origin_z);

        prev_point = point;
        prev_point_inside = point_inside;
        prev_x = x; prev_y = y; prev_z = z;
      }
    }

    glEnd();
    return;
  }

  if(flat)
    glDisable(GL_DEPTH_TEST);

  glBegin(GL_POINTS);
  glPointSize(1.0);
  for(int l=0; l<NUM_LASER_BEAMS; l++) {
    for(int i=0; i<lscan[l].num_points; i++) {
      if (lscan[l].laser_point[i].valid) {

        double x = lscan[l].laser_point[i].point->x * 0.01 + lscan[l].laser_point[i].scan->robot.x;
        double y = lscan[l].laser_point[i].point->y * 0.01 + lscan[l].laser_point[i].scan->robot.y;
        double z = lscan[l].laser_point[i].point->z * 0.01 + lscan[l].laser_point[i].scan->robot.z;

        double in;
        int e, b, color;
        switch (color_mode) {
          case 0:
            in = lscan[l].laser_point[i].point->intensity / 255.0;
            glColor3f(in, in, in);
            break;

          case 1:
            in = (lscan[l].laser_point[i].point->z * 0.01 + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT) / 3.0;
            glColor3f(1-in, in, 0);
            break;

          case 2:
            if (lscan[l].laser_point[i].obstacle)
              glColor3f( 1.0, 0.0, 0.0 );
            else
              glColor3f( 0.0, 1.0, 0.0 );
            break;

          case 3:
            in = (lscan[l].laser_point[i].scan->encoder+18000)%36000 / (double)VELO_NUM_TICKS;
            glColor3f(1-in, 0, in);
            break;

          case 4:
            in = (lscan[l].laser_point[i].scan->encoder) / (double)VELO_NUM_TICKS;
            glColor3f(1-in, 0, in);
            break;

          case 5:
            e = lscan[l].laser_point[i].encoder;
            b = (int)floor(e / ((float)VELO_NUM_TICKS/(float)720));
            color = b % NUM_COLORS;
            glColor3f(colors[color][0] / 255.0, colors[color][1] / 255.0, colors[color][2] / 255.0);
            break;

          case 6:
            color = l % NUM_COLORS;
            glColor3f(colors[color][0] / 255.0, colors[color][1] / 255.0, colors[color][2] / 255.0);
            break;

          case 7:
            in = (double)lscan[l].laser_point[i].encoder / VELO_NUM_TICKS;
            glColor3f(1-in, in, 0);
            break;

          case 8:
            if (point_inside_cars(&lscan[l].laser_point[i]))
              glColor3f(0.8, 0, 0);
            else
              glColor3f(0, 0.8, 0);
            break;

        }

        if (flat)
          glVertex3f(x - origin_x, y - origin_y, 0);
        else
          glVertex3f(x - origin_x, y - origin_y, z - origin_z);


      }
    }
  }
  glEnd();

  if(flat)
    glEnable(GL_DEPTH_TEST);
}

void draw_path(double origin_x, double origin_y)
{
  double dx, dy;
  int i;

  /* draw path before and after current scan */
  glDisable(GL_DEPTH_TEST);

  glColor4f(1, 1, 0, 0.3);
  glBegin(GL_QUAD_STRIP);
  for(i = 0; i <= current_spin_num; i++) {
    dx = cos(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    dy = sin(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x + dx - origin_x,
        velodyne_index.spin[i].pose[0].smooth_y + dy - origin_y);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x - dx - origin_x,
        velodyne_index.spin[i].pose[0].smooth_y - dy - origin_y);
  }
  glEnd();

  glColor4f(0, 0, 1, 0.3);
  glBegin(GL_QUAD_STRIP);
  for(i = current_spin_num; i < velodyne_index.num_spins; i++) {
    dx = cos(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    dy = sin(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x + dx - origin_x,
        velodyne_index.spin[i].pose[0].smooth_y + dy - origin_y);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x - dx - origin_x,
        velodyne_index.spin[i].pose[0].smooth_y - dy - origin_y);
  }
  glEnd();

  glEnable(GL_DEPTH_TEST);
}

void draw_info_box(void)
{
  char str[200];
  double u;

  glLineWidth(3);
  glColor4f(0, 0, 0, 0.5);
  glBegin(GL_POLYGON);
  glVertex2f(0, 0);
  glVertex2f(gui3D.window_width, 0);
  glVertex2f(gui3D.window_width, 50);
  glVertex2f(0, 50);
  glEnd();
  glColor3f(1, 1, 1);
  glBegin(GL_LINE_LOOP);
  glVertex2f(0, 0);
  glVertex2f(gui3D.window_width, 0);
  glVertex2f(gui3D.window_width, 50);
  glVertex2f(0, 50);
  glEnd();

  glBegin(GL_LINES);
  glVertex2f(20, 25);
  glVertex2f(gui3D.window_width - 20, 25);
  u = current_spin_num / (double)velodyne_index.num_spins *
      (gui3D.window_width - 40.0);
  glVertex2f(20 + u, 10);
  glVertex2f(20 + u, 40);
  glEnd();

  glColor3f(1, 1, 0);
  sprintf(str, "%d of %d", current_spin_num, velodyne_index.num_spins);
  renderBitmapString(gui3D.window_width - 20 -
      bitmapStringWidth(GLUT_BITMAP_HELVETICA_18, str),
      31, GLUT_BITMAP_HELVETICA_18, str);

//  glColor3f(0, 0, 0);
//  sprintf(str, "Seg Score: %.2f", seg_score);
//  renderBitmapString(gui3D.window_width - 20 -
//      bitmapStringWidth(GLUT_BITMAP_HELVETICA_18, str),
//      91, GLUT_BITMAP_HELVETICA_18, str);
//
//  glColor3f(0, 0, 0);
//  sprintf(str, "Seg Total: %.2f", seg_score_total);
//  renderBitmapString(20, 91, GLUT_BITMAP_HELVETICA_18, str);
//
//  glColor3f(0, 0, 0);
//  sprintf(str, "Tracker Score: %.2f", tracker_score);
//  renderBitmapString(gui3D.window_width - 20 -
//      bitmapStringWidth(GLUT_BITMAP_HELVETICA_18, str),
//      61, GLUT_BITMAP_HELVETICA_18, str);
//
//  glColor3f(0, 0, 0);
//  sprintf(str, "Tracker Total: %.2f", tracker_score_total);
//  renderBitmapString(20, 61, GLUT_BITMAP_HELVETICA_18, str);
}

void
draw_pedestrian( float x, float y, float z1, float height, float size, float direction,
    float magnitude, dgc_rgb_t rgb, float occ)
{
  float head_size = 0.6*size;

  GLUquadricObj *quadratic;
  quadratic = gluNewQuadric();

  glColor4f( rgb.r, rgb.g, rgb.b, occ );

  glPushMatrix();
  {
    /* Body */
    glTranslatef(x, y, z1);
    gluCylinder(quadratic, size, 0.0, height, 32, 32);

    /* Head */
    glTranslatef(0.0, 0.0, height);
    gluSphere(quadratic, head_size, 32, 32);

    /* Direction and magnitude of travel */
    glRotatef(dgc_r2d(direction), 0, 0, 1);
    glLineWidth(2.0);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(magnitude, 0, 0);
    glEnd();
  }
  glPopMatrix();

  gluDeleteQuadric(quadratic);
}

void
draw_bicyclist( float x, float y, float z, float scale, float direction,
    float magnitude, dgc_rgb_t rgb, float occ)
{
  float head_size = 0.1*scale;
  float height = 0.8*scale;
  float width = 0.2*scale;

  GLUquadricObj *quadratic;
  quadratic = gluNewQuadric();

  glColor4f( rgb.r, rgb.g, rgb.b, occ );

  glPushMatrix();
  {
    /* Body */
    glTranslatef(x, y, z);
    gluCylinder(quadratic, width, 0.0, height, 32, 32);

    /* Head */
    glPushMatrix();
    glTranslatef(0.0, 0.0, height);
    gluSphere(quadratic, head_size, 32, 32);
    glPopMatrix();

    /* Wheels */
    glPushMatrix();
    glTranslatef(0.0, 0.0, 0.0*scale);
    glRotatef(90, 1, 0, 0);
    glRotatef(dgc_r2d(direction), 0, 1, 0);
    glTranslatef(0.5*scale, 0.0, 0.0);
    glutWireTorus(0.05*scale, 0.25*scale, 50, 50);
    glTranslatef(-0.8*scale, 0.0, 0.0);
    glutWireTorus(0.05*scale, 0.25*scale, 50, 50);
    glPopMatrix();

    /* Handlebars. */
    glPushMatrix();
    glRotatef(dgc_r2d(direction), 0, 0, 1); // x is bike-forward, z is up.
    glTranslatef(0.3*scale, 0.4*scale, 0.35*scale);
    glRotatef(90, 1, 0, 0);
    gluCylinder(quadratic, 0.03*scale, 0.03*scale, 0.8*scale, 32, 32);
    glPopMatrix();

    /* Direction and magnitude of travel */
    glRotatef(dgc_r2d(direction), 0, 0, 1);
    glTranslatef(0.3, 0.0, 0.0);
    glLineWidth(2.0);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(magnitude, 0, 0);
    glEnd();
  }
  glPopMatrix();

  gluDeleteQuadric(quadratic);
}

void draw_obstacle_frame( float x, float y, float z, float l, float w, float direction,
    float magnitude, dgc_rgb_t rgb)
{
  glColor3f( rgb.r, rgb.g, rgb.b);
  glLineWidth(2.0);

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(dgc_r2d(direction), 0, 0, 1);

  glBegin(GL_LINE_LOOP);
  glVertex3f(l / 2, w / 2, 0);
  glVertex3f(l / 2, -w / 2, 0);
  glVertex3f(-l / 2, -w / 2, 0);
  glVertex3f(-l / 2, w / 2, 0);
  glVertex3f(l / 2, 0, 0);
  glVertex3f(l / 2 + magnitude, 0, 0);
  glVertex3f(l / 2, 0, 0);
  glVertex3f(-l / 2, -w / 2, 0);
  glVertex3f(-l / 2, w / 2, 0);
  glEnd();

  glPopMatrix();
}

void display(void)
{
  double robot_lat, robot_lon, robot_x, robot_y, robot_z, robot_roll;
  double robot_pitch, robot_yaw, robot_smooth_x, robot_smooth_y;
  double robot_smooth_z;
  char utmzone[10];
  int i;

  /*  fprintf(stderr, "%d cars\n", cars.num_cars());
  for(i = 0; i < cars.num_cars(); i++) {
    fprintf(stderr, "  %d : %d : ", i, cars.car[i].num_poses());
    for(j = 0; j < cars.car[i].num_poses(); j++)
      fprintf(stderr, "%d ", cars.car[i].pose[j].t);
    fprintf(stderr, "\n");
    }*/


  /* clear to black */
  glClearColor(1, 1, 1, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  /* turn on smooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  /* calculate origin */
  robot_lat = velodyne_index.spin[current_spin_num].pose[0].latitude;
  robot_lon = velodyne_index.spin[current_spin_num].pose[0].longitude;
  latLongToUtm(robot_lat, robot_lon, &robot_x, &robot_y, utmzone);
  robot_z = velodyne_index.spin[current_spin_num].pose[0].altitude;
  robot_smooth_x = velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  robot_smooth_y = velodyne_index.spin[current_spin_num].pose[0].smooth_y;
  robot_smooth_z = velodyne_index.spin[current_spin_num].pose[0].smooth_z;
  robot_roll = velodyne_index.spin[current_spin_num].pose[0].roll;
  robot_pitch = velodyne_index.spin[current_spin_num].pose[0].pitch;
  robot_yaw = velodyne_index.spin[current_spin_num].pose[0].yaw;

  double x_offset = velodyne_index.spin[current_spin_num].pose[0].x_offset;
  double y_offset = velodyne_index.spin[current_spin_num].pose[0].y_offset;

  if (x_offset == 0.0) { // in case we don't have offset
    x_offset = robot_x - robot_smooth_x;
    y_offset = robot_y - robot_smooth_y;
  }

  robot_x = velodyne_index.spin[current_spin_num].pose[0].smooth_x + x_offset;
  robot_y = velodyne_index.spin[current_spin_num].pose[0].smooth_y + y_offset;

  /* draw aerial imagery */
  glPushMatrix();
  glTranslatef(0, 0, 0.2);
  // TODO: fix - this causes seg fault right now
//  dgc_imagery_draw_3D(imagery_root, gui3D.camera_pose.distance,
//      gui3D.camera_pose.x_offset,
//      gui3D.camera_pose.y_offset,
//      robot_x, robot_y, utmzone, true, 1.0, 1);
  glPopMatrix();

  /* draw robot path */
  if (show_path)
    draw_path(robot_smooth_x, robot_smooth_y);

  // *******************************************************************
  // *    Draw the RNDF / SRNDF
  // *******************************************************************
  if((rndf_dl != NULL) && show_rndf) {
    glPushMatrix();
    {
      if (!draw_flat)
        glTranslatef(0, 0, 0.05);

      draw_rndf_display_list(rndf_dl, 0, 1, 1, 1, 0, 0, robot_x, robot_y);

//      draw_stop_zones(rndf, global.x, global.y);
    }
    glPopMatrix();
  }

  glDisable(GL_DEPTH_TEST);

  /* draw labeled cars */
  if (show_cars) {
    double dt = 0.2;
    if ((current_spin_num > 0) && (current_spin_num < velodyne_index.num_spins - 1)) {
      dt = velodyne_index.spin[current_spin_num+1].pose[0].timestamp - velodyne_index.spin[current_spin_num-1].pose[0].timestamp;
    }
    dgc_rgb_t rgb = {0.0, 0.0, 1.0};
    for(i = 0; i < cars.num_cars(); i++) {
      car_t& car = cars.car[i];
      double x, y, theta, vel;
      bool extrapolated;
      if (car.estimate_pose(current_spin_num, &x, &y, &theta, &extrapolated) &&
          car.estimate_velocity(current_spin_num, &vel, dt))
        draw_obstacle_frame( x - robot_smooth_x, y - robot_smooth_y, 0.0, car.l, car.w, theta, vel, rgb);
    }
  }

  /* draw tracked cars */
  glPushMatrix();
  glTranslatef(-robot_smooth_x, -robot_smooth_y, 0.2);

  /* segmented obstacles */
//  for (i=0; i < obstacles_segmented.size(); i++) {
//    shared_ptr<GridObstacle> obstacle = obstacles_segmented[i];
//    float l = obstacle->length;
//    float w = obstacle->width;
//
//    glPushMatrix();
//    glTranslatef(obstacle->pose.x, obstacle->pose.y, 0);
//    glRotatef(dgc_r2d(obstacle->pose.yaw), 0, 0, 1);
//
//    glColor4f(0.0, 0.6, 0, 0.5);
//    glBegin(GL_POLYGON);
//    glVertex2f(l / 2, w / 2);
//    glVertex2f(l / 2, -w / 2);
//    glVertex2f(-l / 2, -w / 2);
//    glVertex2f(-l / 2, w / 2);
//    glEnd();
//
//    glColor3f(0, 0, 0);
//    glBegin(GL_LINE_LOOP);
//    glVertex2f(l / 2, w / 2);
//    glVertex2f(l / 2, -w / 2);
//    glVertex2f(-l / 2, -w / 2);
//    glVertex2f(-l / 2, w / 2);
//    glEnd();
//    glBegin(GL_LINES);
//    glVertex2f(l / 2 * 0.5, 0);
//    glVertex2f(l / 2 * 1.5, 0);
//    glEnd();
//    glPopMatrix();
//  }

  /* tracked obstacles */
  if (show_tracked) {
    for (i=0; i < obstacles_tracked.size(); i++) {
      shared_ptr<TrackedObstacle> obstacle = obstacles_tracked[i];

      dgc_rgb_t rgb2;
      switch (obstacle->type){

        case OBSTACLE_CAR:
          rgb2.r = 1.0;  /* car color setting */
          rgb2.g = 0.0;
          rgb2.b = 0.0;
          draw_obstacle_frame( obstacle->pose.x,
              obstacle->pose.y,
              0.5,
              obstacle->length,
              obstacle->width,
              obstacle->pose.yaw,
              obstacle->getVelocity(),
              rgb2);
          break;

        case OBSTACLE_PEDESTRIAN:
          rgb2.r = 0.98;
          rgb2.g = 0.76;
          rgb2.b = 0.44;
          draw_pedestrian( obstacle->pose.x,
              obstacle->pose.y,
              -DGC_PASSAT_HEIGHT+2.5,
              1.0,
              0.35,
              obstacle->pose.yaw,
              obstacle->getVelocity(),
              rgb2, 0.8);
          break;

        case OBSTACLE_BICYCLIST:
          rgb2.r = 0.1;  /* skin color setting */
          rgb2.g = 0.9;
          rgb2.b = 0.1;
          draw_bicyclist( obstacle->pose.x,
              obstacle->pose.y,
              -DGC_PASSAT_HEIGHT+2.5,
              2.0,
              obstacle->pose.yaw,
              obstacle->getVelocity(),
              rgb2, 0.8);
          break;

        case OBSTACLE_UNKNOWN:
        default:
          rgb2.r = 1.0;  /* unknown color setting */
          rgb2.g = 1.0;
          rgb2.b = 0.0;
          draw_obstacle_frame( obstacle->pose.x,
              obstacle->pose.y,
              0,
              obstacle->length,
              obstacle->width,
              obstacle->pose.yaw,
              obstacle->getVelocity(),
              rgb2);
          break;
      }

      switch (obstacle->type_this_frame_){
        case OBSTACLE_CAR:
          rgb2.r = 1.0;  /* car color setting */
          rgb2.g = 0.0;
          rgb2.b = 0.0;
          draw_obstacle_frame( obstacle->pose.x,
              obstacle->pose.y,
              0.5,
              obstacle->length / 4.0,
              obstacle->width / 4.0,
              obstacle->pose.yaw,
              obstacle->getVelocity(),
              rgb2);
          break;

        case OBSTACLE_PEDESTRIAN:
          rgb2.r = 0.98;
          rgb2.g = 0.76;
          rgb2.b = 0.44;
          draw_pedestrian( obstacle->pose.x,
              obstacle->pose.y,
              -DGC_PASSAT_HEIGHT+2.0,
              0.3,
              0.15,
              obstacle->pose.yaw,
              obstacle->getVelocity(),
              rgb2, 0.8);
          break;

        case OBSTACLE_BICYCLIST:
          rgb2.r = 0.1;  /* skin color setting */
          rgb2.g = 0.9;
          rgb2.b = 0.1;
          draw_bicyclist( obstacle->pose.x,
              obstacle->pose.y,
              -DGC_PASSAT_HEIGHT+2.0,
              0.5,
              obstacle->pose.yaw,
              obstacle->getVelocity(),
              rgb2, 0.8);
          break;

        default:
        case OBSTACLE_UNKNOWN:
          rgb2.r = 1.0;  /* unknown color setting */
          rgb2.g = 1.0;
          rgb2.b = 0.0;
          draw_obstacle_frame( obstacle->pose.x,
              obstacle->pose.y,
              0.5,
              obstacle->length / 4.0,
              obstacle->width / 4.0,
              obstacle->pose.yaw,
              obstacle->getVelocity(),
              rgb2);
          break;
      }

      // -- Draw small icon for frame classification

//
//      float l = obstacle->length;
//      float w = obstacle->width;
//
//      glPushMatrix();
//      glTranslatef(obstacle->pose.x, obstacle->pose.y, 0);
//      glRotatef(dgc_r2d(obstacle->pose.yaw), 0, 0, 1);
//
//      if (obstacle->getConfidence() < settings.kf_settings.confidence_publish_min) {
//        glColor4f(0.3, 0, 0, 0.3);
//      } else {
//        glColor4f(0.6, 0, 0, 1.0);
//      }
//      glBegin(GL_POLYGON);
//      glVertex2f(l / 2, w / 2);
//      glVertex2f(l / 2, -w / 2);
//      glVertex2f(-l / 2, -w / 2);
//      glVertex2f(-l / 2, w / 2);
//      glEnd();
//
//      glColor3f(0, 0, 0);
//      glBegin(GL_LINE_LOOP);
//      glVertex2f(l / 2, w / 2);
//      glVertex2f(l / 2, -w / 2);
//      glVertex2f(-l / 2, -w / 2);
//      glVertex2f(-l / 2, w / 2);
//      glEnd();
//      glBegin(GL_LINES);
//      glVertex2f(l / 2 * 0.5, 0);
//      glVertex2f(l / 2 * 1.5, 0);
//      glEnd();
//      glPopMatrix();
    }
  }

  /* draw obstacles_s */

  if (show_obstacles) {
    glColor3f(0.0, 0.0, 0.0);
    for (int i=0; i<obstacles_s->num; i++) {
      double x, y;
//      cell_to_coord(grid, obstacles_s->cell[i], &x, &y);
      grid->cellToRCLocal(obstacles_s->cell[i], &y, &x);
      glBegin(GL_QUADS);
      glVertex3f(x-0.05, y-0.05, 0);
      glVertex3f(x-0.05, y+0.05, 0);
      glVertex3f(x+0.05, y+0.05, 0);
      glVertex3f(x+0.05, y-0.05, 0);
      glEnd();
    }
  }

  if (show_z) {
    draw_grid(z_grid, robot_smooth_x, robot_smooth_y);
  }

  if (show_distance) {
    /* draw tracker map extent */
    double x1,y1,x2,y2;
    glColor3f(0.7, 0.7, 0.7);
    glLineWidth(1.0);
    glBegin(GL_LINES);
    grid_rc_to_xy(grid, 0, 0, &x1, &y1);
    grid_rc_to_xy(grid, grid->rows, grid->cols, &x2, &y2);
    glVertex3f(x1, y1, 0);
    glVertex3f(x1, y2, 0);

    glVertex3f(x1, y2, 0);
    glVertex3f(x2, y2, 0);

    glVertex3f(x2, y2, 0);
    glVertex3f(x2, y1, 0);

    glVertex3f(x2, y1, 0);
    glVertex3f(x1, y1, 0);
    glEnd();
  }


  glPopMatrix();

  if (show_distance) {
    // draw distance rings
    glTranslatef(0,0,-0.02);
    for(int i = 10; i <= 120; i += 10) {
      if (i%50) {
        glColor3f(0.5, 0.5, 0.5);
      } else {
        glColor3f(1.0, 0.5, 0.5);
      }
      glBegin(GL_LINE_LOOP);
      for(int j = 0; j < 100; j++) {
        double angle = j / 100.0 * M_PI * 2;
        glVertex3f(i * cos(angle), i * sin(angle), 0);
      }
      glEnd();
    }
  }

  glEnable(GL_DEPTH_TEST);

  if (show_segmentation) {
    glPointSize(2.0);
    glPushMatrix();
    glTranslatef(-robot_smooth_x, -robot_smooth_y, -robot_smooth_z + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT);
    glBegin(GL_POINTS);
    for (unsigned int i=0; i < obstacles_segmented.size(); i++) {
      shared_ptr<Obstacle> obst = obstacles_segmented[i];
      std::vector<point3d_t>& points = obst->getPoints();
      int color = (i % NUM_COLORS);
      glColor3f(colors[color][0] / 255.0, colors[color][1] / 255.0, colors[color][2] / 255.0);

      for (unsigned int j=0; j < points.size(); j++) {
        glVertex3f(points[j].x, points[j].y, points[j].z);
      }
    }
    glEnd();
    glPopMatrix();
  }

  /* draw the velodyne spins */
  if (show_velodyne) {
    glColor3f(1, 1, 1);
    if(large_points)
      glPointSize(3);
    else
      glPointSize(1);
    draw_points(robot_smooth_x, robot_smooth_y, robot_smooth_z - DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, draw_flat);
  }

  /* draw the passat */
  if (show_passat) {
    glEnable(GL_LIGHTING);
    glPushMatrix();
    glRotatef(dgc_r2d(robot_yaw), 0, 0, 1);
    glRotatef(dgc_r2d(robot_pitch), 0, 1, 0);
    glRotatef(dgc_r2d(robot_roll), 1, 0, 0);
    glTranslatef(1.65, 0, -0.6 + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT);
    passatwagonmodel_draw(passat, 0, 0, 0);
    glPopMatrix();
    glDisable(GL_LIGHTING);
  }

  glEnable(GL_BLEND);

  /* go to 2D */
  set_display_mode_2D(gui3D.window_width, gui3D.window_height);

  /* draw the info box */
  draw_info_box();
}

void timer(int)
{
  if(dgc_imagery_update())
    gui3D_forceRedraw();
  gui3D_add_timerFunc(100, timer, 0);
}

double get_z() {
  return velodyne.scans[0].robot.z;
}

void
perception_publish( void )
{
  static int do_publishing = FALSE;

  if (!do_publishing) {
    do_publishing = TRUE;
    pthread_mutex_lock(&integration_mutex);
    {
      perception_publish_obstacles( ipc, grid, obstacles_s, obstacles_tracked, counter );

      if (settings.gls_output) {
        glsSend(ipc, gls);
        gls_clear(gls);
      }

      fprintf( stderr, "#INFO: [OBSTACLES: %05d]\n", obstacles_s->num);

      obstacles_s->num = 0;
    }
    pthread_mutex_unlock(&integration_mutex);
    do_publishing = FALSE;
  }
}

/******************************************************************
 * APPLANIX handler
 ******************************************************************/
void
applanix_handler( ApplanixPose *pose )
{
  pthread_mutex_lock(&applanix_mutex);
  applanix_history_add( pose );
  global.x =  pose->smooth_x + localize_pose.x_offset;
  global.y =  pose->smooth_y + localize_pose.y_offset;
  pthread_mutex_unlock(&applanix_mutex);
}

/******************************************************************
 * SENSOR_DATA handler
 ******************************************************************/
void
ldlrs1_handler( void )
{
  pthread_mutex_lock(&ldlrs_mutex[0]);
  ldlrs_ts[0] = dgc_get_time();
  pthread_mutex_unlock(&ldlrs_mutex[0]);
}

void
ldlrs2_handler( void )
{
  pthread_mutex_lock(&ldlrs_mutex[1]);
  ldlrs_ts[1] = dgc_get_time();
  pthread_mutex_unlock(&ldlrs_mutex[1]);
}

/******************************************************************
 * TIMER  handler
 ******************************************************************/
/*
void
timer(void)
{
  bool new_data = false;
  while(velo_interface->ScanDataWaiting()) {
    velodyne.num_scans =
      velo_interface->ReadScans(velodyne.scans, velodyne.allocated_scans);

    if (velodyne.num_scans>0) {
      velodyne.preprocessed = FALSE;
      velodyne_ts = dgc_get_time();
      velodyne_ctr++;
      new_data = true;
    }
  }

  if (new_data) {
    integrate_sensors(&velodyne);
    perception_publish();
  }
}
*/

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


  // -- Check for a classifier.
  if (dgc_file_exists(settings.tracker_settings.classifier_filename)) {
    booster = new MultiBooster(settings.tracker_settings.classifier_filename);
    
    // -- Translate it to use the features that we have currently.
    booster->applyNewMappings(NameMapping(getClassNames()), NameMapping(getDescriptorNames()), true);
    int num_threads = 1;
    if(getenv("NUM_THREADS"))
      num_threads = atoi(getenv("NUM_THREADS"));
    classifier_pipeline_ = new ClassifierPipeline(booster, num_threads);
    printf("Loaded %s\n", settings.tracker_settings.classifier_filename);
  }
  else {
    printf("#WARNING: no classifier found at %s\n", settings.tracker_settings.classifier_filename);
  }

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
  default_map_cell->last_obstacle = 0;
  default_map_cell->last_observed = 0;
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


  /* create connection to velodyne interface */
//  velo_interface = new VelodyneShmInterface;
//  if(velo_interface->CreateClient() < 0)
//    dgc_die("Error: could not connect to velodyne interface.\n");

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

//  strcpy(labels_filename, vlf_filename);
//  strcat(labels_filename, ".labels");
//
  strcpy(cars_filename, vlf_filename);
  strcat(cars_filename, ".labels");
  cars.load_labels(cars_filename);

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

  /*********************************************************************
   *
   *   IPC
   *
   *********************************************************************/
/*
  ipc->Subscribe(ApplanixPoseID, &applanix_handler, DGC_SUBSCRIBE_ALL,
      &applanix_mutex);

  ipc->Subscribe(LocalizePoseID, &localize_pose);

  ipc->Subscribe(LdlrsLaser1ID, &ldlrs[0], ldlrs1_handler, DGC_SUBSCRIBE_ALL,
      &ldlrs_mutex[0]);
  ipc->Subscribe(LdlrsLaser2ID, &ldlrs[1], ldlrs2_handler, DGC_SUBSCRIBE_ALL,
      &ldlrs_mutex[1]);

  ipc->Subscribe(RadarSensor1ID, &radar_lrr2[0], DGC_SUBSCRIBE_LATEST, &radar_mutex[0]);
  ipc->Subscribe(RadarSensor2ID, &radar_lrr2[1], DGC_SUBSCRIBE_LATEST, &radar_mutex[1]);
  ipc->Subscribe(RadarLRR3Sensor3ID, &radar_lrr3[0], DGC_SUBSCRIBE_LATEST, &radar_mutex[2]);
  ipc->Subscribe(RadarSensor4ID, &radar_lrr2[2], DGC_SUBSCRIBE_LATEST, &radar_mutex[3]);
  ipc->Subscribe(RadarSensor5ID, &radar_lrr2[3], DGC_SUBSCRIBE_LATEST, &radar_mutex[4]);
  ipc->Subscribe(RadarLRR3Sensor6ID, &radar_lrr3[1], DGC_SUBSCRIBE_LATEST, &radar_mutex[5]);

  ipc->Subscribe(PlannerFsmStateID, &fsm_state, DGC_SUBSCRIBE_LATEST,
      &fsm_mutex);

  ipc->Subscribe(PerceptionMapRequestID, &map_request_handler);
  ipc->Subscribe(PerceptionMapResetID, &map_reset_handler);

  gls = gls_alloc("PERCEPTION");
*/

  rndf_load_file( rndf_filename );

  velodyne_init( &velodyne );
  perception_init();

//  ipc->AddTimer( timer_interval/10.0, timer);
//  ipc->Dispatch();

  /*********************************************************************
   *
   *   Graphics
   *
   *********************************************************************/

  gui3D_initialize(argc, argv, 10, 10, 720, 480, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_displayFunc(display);
//  gui3D_set_motionFunc(motion);
//  gui3D_set_mouseFunc(mouse);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_add_timerFunc(100, timer, 0);
  passat = passatwagonmodel_load(0.0, 0.0, 0.5, 1);

  if(rndf_valid) {
    rndf_dl = generate_rndf_display_list(*road_network, 1.0, false);
  }

  dgc_imagery_set_imagery_type(DGC_IMAGERY_TYPE_COLOR);
  gui3D_mainloop();

  return 0;
}
