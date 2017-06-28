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


#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <global.h>
#include <transform.h>
#include <vlrException.h>
#include <lltransform.h>
#include <passat_constants.h>
#include <applanix/ApplanixPose.h>
#include <localize/LocalizePose.h>
#include <perception/PerceptionObstacles.h>
#include <driving_common/Actuator.h>
#include <driving_common/CanStatus.h>
#include <driving_common/EStopStatus.h>
#include <driving_common/RadarSensor.h>
#include <driving_common/SimulatorGroundTruth.h>
#include <driving_common/SimulatorTag.h>
#include <driving_common/LdlrsData.h>
#include <driving_common/ErrorString.h>
#include <aw_roadNetwork.h>
#include <vehicle.h>
#include <lasersim.h>
#include <trafficlights.h>
#include <crosswalks.h>

using namespace vlr::rndf;
using namespace dgc;

namespace vlr {

typedef struct {
  double x, y, z;
  double roll, pitch, yaw;
} sensor_offset_t, *sensor_offset_p;

#define MAX_NUM_VEHICLES   40

int32_t num_vehicles = 0;
vehicle_state* vehicle = NULL;
bool bicycle_model, torque_mode;
std::string obstacle_map_filename;

double laser_max_range, track_max_range, radar_max_range;

sensor_offset_t ldlrs1_offset;
sensor_offset_t radar1_offset, radar2_offset, radar3_offset, radar4_offset, radar5_offset;

double ldlrs1_start_angle = -180.0;

std::string rndf_filename;
bool rndf_valid = false;
RoadNetwork rn;

TrafficLightSimulator *lights = NULL;
CrosswalkSimulator *crosswalks = NULL;

#define MIN(x,y) (x < y ? x : y)
#define MAX(x,y) (x > y ? x : y)

typedef struct {
  int id1, id2;
} collision_t;

double *start_lat, *start_lon, *start_theta;

collision_t *collision;
int num_collisions = 0, max_collisions = 0;

int *no_plan, *high_lateral_accel, *high_forward_accel;
double *last_plan;

double *first_forward_accel_ts;
int *bad_forward_accel;

double *first_lateral_accel_ts;
int *bad_lateral_accel;

std::vector<std::string> vehicle_name;

ros::NodeHandle* nh_ = NULL;
ros::Subscriber actuator_sub_;
ros::Subscriber estop_status_sub_;
ros::Publisher can_status_pub_;
ros::Publisher applanix_pub_;
ros::Publisher perception_pub_;
ros::Publisher localize_pub_;
ros::Publisher error_pub_;
ros::Publisher radar1_pub_, radar2_pub_, radar3_pub_, radar4_pub_, radar5_pub_;
ros::Publisher sim_ground_truth_pub_, sim_tag_pub_;

tf::TransformListener* tf_listener_=NULL;

driving_common::ErrorString err_msg_;

int32_t findOurVehicleId() {
  return 0;
}

void publishErrorStatus(const std::stringstream& s) {
  err_msg_.text = s.str();
  error_pub_.publish(err_msg_);
}

inline bool point_inside_poly(double *px, double *py, int N, double x, double y) {
  double p1x, p1y, p2x, p2y;
  int counter = 0;
  int i;
  double xinters;

  p1x = px[0];
  p1y = py[0];
  for (i = 1; i <= N; i++) {
    p2x = px[i % N];
    p2y = py[i % N];
    if (y > MIN(p1y, p2y)) if (y <= MAX(p1y, p2y)) if (x <= MAX(p1x, p2x)) if (p1y != p2y) {
      xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
      if (p1x == p2x || x <= xinters) counter++;
    }
    p1x = p2x;
    p1y = p2y;
  }
  if (counter % 2 != 0) return true;
  return false;
}

int car_car_intersection(double x1, double y1, double theta1, double x2, double y2, double theta2, double w, double l) {
  double xp1[4], yp1[4];
  double ctheta, stheta;
  double x, y, w2 = w / 2.0, l2 = l / 2.0;

  ctheta = cos(theta1);
  stheta = sin(theta1);

  xp1[0] = x1 + l2 * ctheta - w2 * stheta;
  yp1[0] = y1 + l2 * stheta + w2 * ctheta;
  xp1[1] = x1 + l2 * ctheta + w2 * stheta;
  yp1[1] = y1 + l2 * stheta - w2 * ctheta;
  xp1[2] = x1 - l2 * ctheta + w2 * stheta;
  yp1[2] = y1 - l2 * stheta - w2 * ctheta;
  xp1[3] = x1 - l2 * ctheta - w2 * stheta;
  yp1[3] = y1 - l2 * stheta + w2 * ctheta;

  ctheta = cos(theta2);
  stheta = sin(theta2);

  x = x2 + l2 * ctheta - w2 * stheta;
  y = y2 + l2 * stheta + w2 * ctheta;
  if (point_inside_poly(xp1, yp1, 4, x, y)) return true;

  x = x2 + l2 * ctheta + w2 * stheta;
  y = y2 + l2 * stheta - w2 * ctheta;
  if (point_inside_poly(xp1, yp1, 4, x, y)) return true;

  x = x2 - l2 * ctheta + w2 * stheta;
  y = y2 - l2 * stheta - w2 * ctheta;
  if (point_inside_poly(xp1, yp1, 4, x, y)) return true;

  x = x2 - l2 * ctheta - w2 * stheta;
  y = y2 - l2 * stheta + w2 * ctheta;
  if (point_inside_poly(xp1, yp1, 4, x, y)) return true;

  return false;
}

void compute_warnings(void) {
  double current_time;
  int i, j;

  /* first collision warnings */
  num_collisions = 0;
  for (i = 0; i < num_vehicles; i++)
    for (j = i + 1; j < num_vehicles; j++)
      if (car_car_intersection(vehicle[i].x + vehicle[i].origin_x, vehicle[i].y + vehicle[i].origin_y, vehicle[i].yaw, vehicle[j].x + vehicle[j].origin_x,
          vehicle[j].y + vehicle[j].origin_y, vehicle[j].yaw, DGC_PASSAT_WIDTH, DGC_PASSAT_LENGTH)) {
        if (num_collisions == max_collisions) {
          max_collisions += 100;
          collision = (collision_t *) realloc(collision, max_collisions * sizeof(collision_t));
          dgc_test_alloc(collision);
        }
        /*
         fprintf(stderr, "%d - %d : %.2f %.2f %.2f    %.2f %.2f %.2f\n",
         i, j, vehicle[i].x, vehicle[i].y, vehicle[i].yaw,
         vehicle[j].x, vehicle[j].y, vehicle[j].yaw);
         */
        collision[num_collisions].id1 = i;
        collision[num_collisions].id2 = j;
        num_collisions++;
      }

  /* forward accel */
  current_time = dgc_get_time();
  for (i = 0; i < num_vehicles; i++) {
    if (fabs(vehicle[i].actual_forward_accel) > 3.0) {
      if (!bad_forward_accel[i]) {
        if (i == 0)
        /*
         fprintf(stderr, "actual forward accel = %f\n",
         vehicle[i].actual_forward_accel);
         */

        first_forward_accel_ts[i] = current_time;
        bad_forward_accel[i] = 1;
      }
    }
    else bad_forward_accel[i] = 0;
  }

  /* lateral_accel */
  for (i = 0; i < num_vehicles; i++) {
    if (fabs(vehicle[i].lateral_accel) > 2.0) {
      if (!bad_lateral_accel[i]) {
        first_lateral_accel_ts[i] = current_time;
        bad_lateral_accel[i] = 1;
      }
    }
    else bad_lateral_accel[i] = 0;
  }

  for (i = 0; i < num_vehicles; i++)
    if (vehicle[i].paused) last_plan[i] = current_time;

}

void publish_collision_info() {
  int i;

  for (i = 0; i < num_collisions; i++) {
    std::stringstream s;
    s << "COLLISION!: cars  " << collision[i].id1 + 1 << " and " << collision[i].id2 + 1 << "crashed.";
    publishErrorStatus(s);
  }
}

void get_sensor_offset(dgc::dgc_transform_t t, sensor_offset_p sensor_offset) {
  dgc_transform_get_rotation(t, &sensor_offset->roll, &sensor_offset->pitch, &sensor_offset->yaw);
  dgc_transform_get_translation(t, &sensor_offset->x, &sensor_offset->y, &sensor_offset->z);
}

void publish_ldlrs1(vehicle_state* vehicle) {
  //  static int first = 1;
  //  static driving_common::LdlrsLaser ldlrs;
  //  double laser_x, laser_y;
  //
  //  if (first) {
  //    ldlrs.range = (float *) calloc(270 * 4, sizeof(float));
  //    dgc_test_alloc(ldlrs.range);
  //    strcpy(ldlrs.host, dgc_hostname());
  //    ldlrs.scan_count = 0;
  //    first = 0;
  //  }
  //
  //  laser_x = vehicle->x + vehicle->origin_x + (ldlrs1_offset.x - vehicle->param.imu_to_cg_dist) * cos(vehicle->yaw) + ldlrs1_offset.y * cos(vehicle->yaw + M_PI
  //      / 2.0);
  //  laser_y = vehicle->y + vehicle->origin_y + (ldlrs1_offset.x - vehicle->param.imu_to_cg_dist) * sin(vehicle->yaw) + ldlrs1_offset.y * sin(vehicle->yaw + M_PI
  //      / 2.0);
  //
  //  generate_ldlrs_laser_scan(&ldlrs, laser_x, laser_y, vehicle->yaw + ldlrs1_offset.yaw, dgc_d2r(ldlrs1_start_angle), laser_max_range);
  //
  //  ldlrs.timestamp = dgc_get_time();
  //  int err = ipc->Publish(dgc::LdlrsLaser1ID, &ldlrs);
  //  dgc::TestIpcExit(err, "Could not publish", dgc::LdlrsLaser1ID);
}

void generate_radar_message(driving_common::RadarSensor& radar, double sensor_x, double sensor_y, double sensor_yaw, double max_range, int vehicle_num) {
  double obstacle_x, obstacle_y, range, bearing, dx, dy;
  int i, mark;

  mark = 0;
  radar.target.clear();
  if (num_vehicles > 1) {
    radar.target.resize(num_vehicles - 1);
  }
  for (i = 0; i < num_vehicles; i++)
    if (i != vehicle_num) {
      obstacle_x = vehicle[i].x + vehicle[i].origin_x;
      obstacle_y = vehicle[i].y + vehicle[i].origin_y;

      range = hypot(obstacle_x - sensor_x, obstacle_y - sensor_y);
      bearing = vlr::normalizeAngle(atan2(obstacle_y - sensor_y, obstacle_x - sensor_x) - sensor_yaw);

      if (range > max_range || fabs(bearing) > dgc_d2r(7.0)) continue;

      dx = (obstacle_x - sensor_x) / range;
      dy = (obstacle_y - sensor_y) / range;

      radar.target[mark].id = i;
      radar.target[mark].measured = 1;
      radar.target[mark].historical = 0;
      radar.target[mark].distance = range * cos(bearing);
      radar.target[mark].lateral_offset = range * sin(bearing);
      radar.target[mark].lateral_offset_var = 1;
      radar.target[mark].relative_acceleration = 0;
      radar.target[mark].relative_velocity = (vehicle[i].v_x * cos(vehicle[i].yaw) - vehicle[i].v_y * sin(vehicle[i].yaw)) * dx + (vehicle[i].v_x * sin(
          vehicle[i].yaw) + vehicle[i].v_y * cos(vehicle[i].yaw)) * dy;
      mark++;
    }
}

void publish_radar(int which_vehicle, int radar_num) {
  static driving_common::RadarSensor radar;
  double radar_x, radar_y;
  sensor_offset_p offset;

  if (radar_num == 1) offset = &radar1_offset;
  else if (radar_num == 2) offset = &radar2_offset;
  else if (radar_num == 3) offset = &radar3_offset;
  else if (radar_num == 4) offset = &radar4_offset;
  else offset = &radar5_offset;

  radar_x = vehicle[which_vehicle].x + vehicle[which_vehicle].origin_x + (offset->x - vehicle[which_vehicle].param.imu_to_cg_dist) * cos(
      vehicle[which_vehicle].yaw) + offset->y * cos(vehicle[which_vehicle].yaw + M_PI / 2.0);
  radar_y = vehicle[which_vehicle].y + vehicle[which_vehicle].origin_y + (offset->x - vehicle[which_vehicle].param.imu_to_cg_dist) * sin(
      vehicle[which_vehicle].yaw) + offset->y * sin(vehicle[which_vehicle].yaw + M_PI / 2.0);

  generate_radar_message(radar, radar_x, radar_y, vehicle[which_vehicle].yaw + offset->yaw, radar_max_range, which_vehicle);
  radar.timestamp = dgc_get_time();

  if (radar_num == 1) {
    radar1_pub_.publish(radar);
  }
  else if (radar_num == 2) {
    radar2_pub_.publish(radar);
  }
  else if (radar_num == 3) {
    radar3_pub_.publish(radar);
  }
  else if (radar_num == 4) {
    radar4_pub_.publish(radar);
  }
  else if (radar_num == 5) {
    radar5_pub_.publish(radar);
  }
}

void passatActuatorHandler(const driving_common::Actuator& actuator) {
  int32_t i = findOurVehicleId();
  if (vehicle[i].param.torque_mode) {
    vehicle[i].set_torque_controls(actuator.steering_value, actuator.throttle_fraction, 100 * actuator.brake_pressure);
  }
  else {
    vehicle[i].set_controls(actuator.steering_value, actuator.throttle_fraction, 100 * actuator.brake_pressure);
  }

  vehicle[i].set_direction((actuator.direction == driving_common::Actuator::DIRECTION_FORWARD) ? 1 : 0);
  last_plan[i] = dgc_get_time();
}

void estopStatusHandler(const driving_common::EStopStatus& estop) {
  int32_t i = findOurVehicleId();
  vehicle[i].paused = (estop.estop_code != driving_common::EStopStatus::ESTOP_RUN);
}

inline int point_inside_rectangle(double point_x, double point_y, double rect_x, double rect_y, double rect_theta, double rect_w, double rect_l)

{
  double perp_dist, par_dist, x_match, y_match;
  double ctheta, stheta;
  double x1, y1, x2, y2;

  /* compute center axis vector */
  ctheta = cos(rect_theta);
  stheta = sin(rect_theta);
  x1 = rect_x - (rect_w / 2.0) * stheta;
  y1 = rect_y + (rect_w / 2.0) * ctheta;
  x2 = rect_x - (-rect_w / 2.0) * stheta;
  y2 = rect_y + (-rect_w / 2.0) * ctheta;
  dgc_point_to_line_distance(point_x, point_y, x1, y1, x2, y2, &par_dist, &perp_dist, &x_match, &y_match);
  if (par_dist >= 0.0 && par_dist <= 1.0 && perp_dist <= rect_l / 2.0) return 1;
  return 0;
}

inline int car_partly_inside_rectangle(double car_x, double car_y, double car_theta, double car_w, double car_l, double rect_x, double rect_y,
    double rect_theta, double rect_w, double rect_l) {
  double ctheta, stheta, x, y;

  ctheta = cos(car_theta);
  stheta = sin(car_theta);

  x = car_x + (car_l / 2.0) * ctheta - (car_w / 2.0) * stheta;
  y = car_y + (car_l / 2.0) * stheta + (car_w / 2.0) * ctheta;
  if (point_inside_rectangle(x, y, rect_x, rect_y, rect_theta, rect_w, rect_l)) return 1;

  x = car_x + (car_l / 2.0) * ctheta - (-car_w / 2.0) * stheta;
  y = car_y + (car_l / 2.0) * stheta + (-car_w / 2.0) * ctheta;
  if (point_inside_rectangle(x, y, rect_x, rect_y, rect_theta, rect_w, rect_l)) return 1;

  x = car_x + (-car_l / 2.0) * ctheta - (-car_w / 2.0) * stheta;
  y = car_y + (-car_l / 2.0) * stheta + (-car_w / 2.0) * ctheta;
  if (point_inside_rectangle(x, y, rect_x, rect_y, rect_theta, rect_w, rect_l)) return 1;

  x = car_x + (-car_l / 2.0) * ctheta - (car_w / 2.0) * stheta;
  y = car_y + (-car_l / 2.0) * stheta + (car_w / 2.0) * ctheta;
  if (point_inside_rectangle(x, y, rect_x, rect_y, rect_theta, rect_w, rect_l)) return 1;
  return 0;
}

void publish_traffic_lights() {
  lights->update(dgc_get_time());
}

void makeStaticPoints(std::vector<perception::StaticObstaclePoint>::iterator pit, int n, double x1, double y1, double x2, double y2, int id) {
  int i;

  for (i = 0; i < n; i++) {
    (*pit).x = x1 + (x2 - x1) * i / (double) n;
    (*pit).y = y1 + (y2 - y1) * i / (double) n;
    (*pit).z_min = -1.6;
    (*pit).z_max = 0;
    (*pit).type = id;
    pit++;
  }
}

static perception::PerceptionObstacles all_obstacles;

void precompute_fake_obstacles() {
  double obstacle_x, obstacle_y;
  double ctheta, stheta;
  int i, id, mark;

  mark = 0;
  all_obstacles.static_point.resize(num_vehicles*40);
  std::vector<perception::StaticObstaclePoint>::iterator pit=all_obstacles.static_point.begin();
  for (i = 0; i < num_vehicles; i++) {
    obstacle_x = vehicle[i].x;
    obstacle_y = vehicle[i].y;
    stheta = sin(vehicle[i].yaw);
    ctheta = cos(vehicle[i].yaw);

    if (hypot(vehicle[i].v_x, vehicle[i].v_y) > dgc_mph2ms(2.0)) id = 1;
    else id = 0;

    makeStaticPoints(pit, 10, obstacle_x + 0.5 * (DGC_PASSAT_LENGTH * ctheta - DGC_PASSAT_WIDTH * stheta), obstacle_y + 0.5
        * (DGC_PASSAT_LENGTH * stheta + DGC_PASSAT_WIDTH * ctheta), obstacle_x + 0.5 * (DGC_PASSAT_LENGTH * ctheta + DGC_PASSAT_WIDTH * stheta), obstacle_y
        + 0.5 * (DGC_PASSAT_LENGTH * stheta - DGC_PASSAT_WIDTH * ctheta), id);
    pit+=10;
    makeStaticPoints(pit, 10, obstacle_x + 0.5 * (DGC_PASSAT_LENGTH * ctheta + DGC_PASSAT_WIDTH * stheta), obstacle_y + 0.5
        * (DGC_PASSAT_LENGTH * stheta - DGC_PASSAT_WIDTH * ctheta), obstacle_x + 0.5 * (-DGC_PASSAT_LENGTH * ctheta + DGC_PASSAT_WIDTH * stheta), obstacle_y
        + 0.5 * (-DGC_PASSAT_LENGTH * stheta - DGC_PASSAT_WIDTH * ctheta), id);
    pit+=10;
    makeStaticPoints(pit, 10, obstacle_x + 0.5 * (-DGC_PASSAT_LENGTH * ctheta + DGC_PASSAT_WIDTH * stheta), obstacle_y + 0.5
        * (-DGC_PASSAT_LENGTH * stheta - DGC_PASSAT_WIDTH * ctheta), obstacle_x + 0.5 * (-DGC_PASSAT_LENGTH * ctheta - DGC_PASSAT_WIDTH * stheta), obstacle_y
        + 0.5 * (-DGC_PASSAT_LENGTH * stheta + DGC_PASSAT_WIDTH * ctheta), id);
    pit+=10;
    makeStaticPoints(pit, 10, obstacle_x + 0.5 * (-DGC_PASSAT_LENGTH * ctheta - DGC_PASSAT_WIDTH * stheta), obstacle_y + 0.5
        * (-DGC_PASSAT_LENGTH * stheta + DGC_PASSAT_WIDTH * ctheta), obstacle_x + 0.5 * (DGC_PASSAT_LENGTH * ctheta - DGC_PASSAT_WIDTH * stheta), obstacle_y
        + 0.5 * (DGC_PASSAT_LENGTH * stheta + DGC_PASSAT_WIDTH * ctheta), id);
    pit+=10;
  }
}

void publish_dirk_obstacles(int which_vehicle) {
  static perception::PerceptionObstacles obstacles;
  static const int32_t samples_per_vehicle = 40;

  obstacles.timestamp = dgc_get_time();

  double applanix_x = vehicle[which_vehicle].x + vehicle[which_vehicle].origin_x - vehicle[which_vehicle].param.imu_to_cg_dist * cos(vehicle[which_vehicle].yaw);
  double applanix_y = vehicle[which_vehicle].y + vehicle[which_vehicle].origin_y - vehicle[which_vehicle].param.imu_to_cg_dist * sin(vehicle[which_vehicle].yaw);

  int32_t mark = 0;
  obstacles.static_point.clear();
  obstacles.dynamic_obstacle.clear();
  if(num_vehicles > 1) {
    obstacles.static_point.resize(samples_per_vehicle*(num_vehicles-1));
  }

  for (int32_t i = 0; i < num_vehicles; i++) {
    if (i != which_vehicle) {
      double obstacle_x = vehicle[i].x + vehicle[i].origin_x;
      double obstacle_y = vehicle[i].y + vehicle[i].origin_y;
      double r = hypot(obstacle_x - applanix_x, obstacle_y - applanix_y);

      if (r < laser_max_range) {
        double add_x = vehicle[i].origin_x - applanix_x + vehicle[which_vehicle].x - vehicle[which_vehicle].param.imu_to_cg_dist * cos(vehicle[which_vehicle].yaw);
        double add_y = vehicle[i].origin_y - applanix_y + vehicle[which_vehicle].y - vehicle[which_vehicle].param.imu_to_cg_dist * sin(vehicle[which_vehicle].yaw);

        memcpy(&obstacles.static_point[mark], &all_obstacles.static_point[i * samples_per_vehicle], samples_per_vehicle * sizeof(perception::StaticObstaclePoint));
        for (int32_t j = 0; j < samples_per_vehicle; j++) {
          obstacles.static_point[mark + j].type = 0;
          obstacles.static_point[mark + j].x += add_x;
          obstacles.static_point[mark + j].y += add_y;
        }
        mark += samples_per_vehicle;

        perception::DynamicObstacle obstacle;
        obstacle.id = i;
        obstacle.confidence = 255;
        obstacle.x = obstacle_x;
        obstacle.y = obstacle_y;
        obstacle.direction = vehicle[i].yaw;
        obstacle.velocity = sqrt(vehicle[i].v_x * vehicle[i].v_x + vehicle[i].v_y * vehicle[i].v_y);
        obstacle.length = DGC_PASSAT_LENGTH;
        obstacle.width = DGC_PASSAT_WIDTH;
        obstacle.type = OBSTACLE_CAR;

        obstacles.dynamic_obstacle.push_back(obstacle);
      }
    }
  }

  // TODO: completely remove static obstacles purposefully makred to be dynamic
  //  if (crosswalks->getObstacles().num_points > 0) {
  //    memcpy(obstacles.point + mark, crosswalks->getObstacles().point, crosswalks->getObstacles().num_points
  //        * sizeof(PerceptionObstaclePoint));
  //    obstacles.num_points += crosswalks->getObstacles().num_points;
  //
  //    for (j = 0; j < crosswalks->getObstacles().num_points; j++) {
  //      obstacles.point[mark + j].x -= vehicle[which_vehicle].origin_x;
  //      obstacles.point[mark + j].y -= vehicle[which_vehicle].origin_y;
  //    }
  //  }

  if (crosswalks->getObstacles().dynamic_obstacle.size() > 0) {
    if(num_vehicles > 1) {
      obstacles.static_point.resize(samples_per_vehicle*(num_vehicles-1));
    }
//    memcpy(&obstacles.dynamic_obstacle[obstacles.dynamic_obstacle.size()], crosswalks->getObstacles().dynamic_obstacle,
//        crosswalks->getObstacles().dynamic_obstacle.size() * sizeof(PerceptionDynamicObstacle));
    for (int32_t j = 0; j < (int32_t)crosswalks->getObstacles().dynamic_obstacle.size(); j++) {
      obstacles.dynamic_obstacle.push_back(crosswalks->getObstacles().dynamic_obstacle[j]);
      obstacles.dynamic_obstacle[mark + j].x -= vehicle[which_vehicle].origin_x;    // TODO: ugly
      obstacles.dynamic_obstacle[mark + j].y -= vehicle[which_vehicle].origin_y;
    }
  }

  if (obstacles.dynamic_obstacle.size() > 0 || obstacles.static_point.size() > 0) {
    perception_pub_.publish(obstacles);
  }
}

void publish_localize(vehicle_state* vehicle) {
  static localize::LocalizePose pose;

  pose.x_offset = vehicle->origin_x + vehicle->added_error_x;
  pose.y_offset = vehicle->origin_y + vehicle->added_error_y;
  pose.timestamp = dgc_get_time();
//  printf("%s: %f, %f\n", __PRETTY_FUNCTION__, pose.x_offset, pose.y_offset);
  localize_pub_.publish(pose);
}

void publish_applanix(vehicle_state* vehicle) {
  static applanix::ApplanixPose applanix_pose;

  vehicle->fill_applanix_message(applanix_pose);

//  printf("%s: %f, %f\n", __PRETTY_FUNCTION__, applanix_pose.latitude, applanix_pose.longitude);
  applanix_pub_.publish(applanix_pose);
}

void publish_groundtruth() {
  static driving_common::SimulatorGroundTruth truth;
  int i, j, mark = 0;
  double current_time;

  truth.vehicle.resize(num_vehicles);
  for (j = 0; j < num_vehicles; j++) {
    truth.vehicle[mark].x = vehicle[j].x + vehicle[j].origin_x;
    truth.vehicle[mark].y = vehicle[j].y + vehicle[j].origin_y;
    truth.vehicle[mark].theta = vehicle[j].yaw;
    truth.vehicle[mark].alpha = vehicle[j].wheel_angle;
    truth.vehicle[mark].v = vehicle[j].v_x;
    truth.vehicle[mark].forward_accel = vehicle[j].actual_forward_accel;
    truth.vehicle[mark].lateral_accel = vehicle[j].lateral_accel;

    // fill us in later
    truth.vehicle[mark].plan_warning = 0;
    truth.vehicle[mark].collision_warning = 0;
    truth.vehicle[mark].forward_accel_warning = 0;
    truth.vehicle[mark].lateral_accel_warning = 0;
    mark++;
  }

  /* mark collisions */
  for (i = 0; i < num_collisions; i++) {
    truth.vehicle[collision[i].id1].collision_warning = 1;
    truth.vehicle[collision[i].id2].collision_warning = 1;
  }

  current_time = dgc_get_time();

  /* then plan warnings */
  for (i = 0; i < num_vehicles; i++)
    truth.vehicle[i].plan_warning = (current_time - last_plan[i] > 2.0);

  /* then acceleration warnings */
  for (i = 0; i < num_vehicles; i++) {
    truth.vehicle[i].forward_accel_warning = (bad_forward_accel[i] && current_time - first_forward_accel_ts[i] >= 0.25);
    truth.vehicle[i].lateral_accel_warning = (bad_lateral_accel[i] && current_time - first_lateral_accel_ts[i] >= 0.25);
  }

  for (i = 0; i < num_vehicles; i++) {
    if (truth.vehicle[i].forward_accel_warning) {
      std::stringstream s;
      s << "HARD BRAKE: car " << i + 1;
      publishErrorStatus(s);
    }
    if (truth.vehicle[i].lateral_accel_warning) {
      std::stringstream s;
      s << "TURN TOO FAST: car " << i + 1;
      publishErrorStatus(s);
    }
    if (truth.vehicle[i].plan_warning) {
      std::stringstream s;
      s << "NO PLAN: car " << i + 1;
      publishErrorStatus(s);
    }
  }

  truth.our_vehicle_num = findOurVehicleId();

  truth.timestamp = dgc_get_time();
  sim_ground_truth_pub_.publish(truth);
}

void publish_tags() {
  static driving_common::SimulatorTag tag;

  int32_t numv = std::min(num_vehicles, MAX_NUM_VEHICLES);

  for (int32_t i = 0; i < numv; i++) {
    tag.tag.push_back(vehicle_name[i]);
  }

  tag.timestamp = dgc_get_time();
  sim_tag_pub_.publish(tag);
}

void publish_can(vehicle_state* vehicle) {
  static driving_common::CanStatus can;

  vehicle->fill_can_message(can);
  can.timestamp = dgc_get_time();
  can_status_pub_.publish(can);
}

template <class T> void getParam(std::string key, T& var) {
  if(!nh_->getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + std::string("."));
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

void get_start_state(double& lat, double& lon, double& theta, double& vel, double& max_steering,
    double& max_throttle, double& max_brake, bool& sim_laser, bool& fake_perception) {

  dgc_transform_t ldlrs1_transform;
  dgc_transform_t radar1_transform, radar2_transform, radar3_transform, radar4_transform, radar5_transform;
  getParam("rndf_file", rndf_filename);
  getParam("simulator/vehicle_start_latitude", lat);
  getParam("simulator/vehicle_start_longitude", lon);
  getParam("simulator/vehicle_start_theta", theta);
  getParam("simulator/vehicle_start_velocity", vel);
  getParam("simulator/bicycle_model", bicycle_model);
  getParam("simulator/torque_mode", torque_mode);
  getParam("simulator/obstacle_map", obstacle_map_filename);
  getParam("simulator/simulate_laser", sim_laser);
  getParam("simulator/laser_max_range", laser_max_range);
  getParam("simulator/track_max_range", track_max_range);
  getParam("simulator/radar_max_range", radar_max_range);
  getParam("simulator/fake_perception", fake_perception);
  getParam("vehicle/max_steering", max_steering);
  getParam("vehicle/max_throttle", max_throttle);
  getParam("vehicle/max_brake", max_brake);
//  getParamTransform("ldlrs_laser1", ldlrs1_transform);
  getParamTransform("RadarSensor1", radar1_transform);
  getParamTransform("RadarSensor2", radar2_transform);
  getParamTransform("RadarSensor3", radar3_transform);
  getParamTransform("RadarSensor4", radar4_transform);
  getParamTransform("RadarSensor5", radar5_transform);

  get_sensor_offset(ldlrs1_transform, &ldlrs1_offset);

  get_sensor_offset(radar1_transform, &radar1_offset);
  get_sensor_offset(radar2_transform, &radar2_offset);
  get_sensor_offset(radar3_transform, &radar3_offset);
  get_sensor_offset(radar4_transform, &radar4_offset);
  get_sensor_offset(radar5_transform, &radar5_offset);

  printf("%s: %f, %f\n", __PRETTY_FUNCTION__, lat, lon);
// ?!?
//  Param vtag_params[MAX_NUM_VEHICLES];
//  char temp[100];
//  for (i = 0; i < MAX_NUM_VEHICLES; i++) {
//    vtag_params[i].module = "sim";
//    snprintf(temp, 50, "vehicle_name%02d", i);
//    vtag_params[i].variable = temp;
//    vtag_params[i].type = DGC_PARAM_STRING;
//    vtag_params[i].user_variable = &(vehicle_name[i]);
//    vtag_params[i].subscribe = 0;
//    vtag_params[i].cb = NULL;
//  }
//
//  pint->AllowUnfoundVariables(true);
//  pint->InstallParams(argc, argv, vtag_params, sizeof(vtag_params) / sizeof(vtag_params[0]));
}

} // namespace vlr

using namespace vlr;

int main(int argc, char **argv) {

  ros::init(argc, argv, argv[0]);
  nh_ = new ros::NodeHandle("/driving");
  tf_listener_ = new tf::TransformListener;

  int i, j, iter = 0;
  double left, start_time, current_time, last_printout = 0;
  double vel;
  double max_steering, max_brake, max_throttle;
  int tenhz = 0;
  int load_map = 0;
  bool* sim_laser;
  bool* fake_perception;

  std::stringstream s;
  for (i = 0; i < MAX_NUM_VEHICLES; i++) {
    s.str("");
    s << "VEHICLE_" << i+1;
    vehicle_name.push_back(s.str());
  }

  no_plan = (int *) calloc(MAX_NUM_VEHICLES, sizeof(int));
  dgc_test_alloc(no_plan);
  high_lateral_accel = (int *) calloc(MAX_NUM_VEHICLES, sizeof(int));
  dgc_test_alloc(high_lateral_accel);
  high_forward_accel = (int *) calloc(MAX_NUM_VEHICLES, sizeof(int));
  dgc_test_alloc(high_forward_accel);
  last_plan = (double *) calloc(MAX_NUM_VEHICLES, sizeof(double));
  dgc_test_alloc(last_plan);

  first_forward_accel_ts = (double *) calloc(MAX_NUM_VEHICLES, sizeof(double));
  dgc_test_alloc(first_forward_accel_ts);
  bad_forward_accel = (int *) calloc(MAX_NUM_VEHICLES, sizeof(int));
  dgc_test_alloc(bad_forward_accel);

  first_lateral_accel_ts = (double *) calloc(MAX_NUM_VEHICLES, sizeof(double));
  dgc_test_alloc(first_lateral_accel_ts);
  bad_lateral_accel = (int *) calloc(MAX_NUM_VEHICLES, sizeof(int));
  dgc_test_alloc(bad_lateral_accel);

  for (i = 0; i < MAX_NUM_VEHICLES; i++) {
    last_plan[i] = dgc_get_time();
    bad_forward_accel[i] = 0;
    bad_lateral_accel[i] = 0;
  }

  // Subscribers
  actuator_sub_ = nh_->subscribe("Actuator", 5, vlr::passatActuatorHandler);
  estop_status_sub_ = nh_->subscribe("EStopStatus", 5, vlr::estopStatusHandler);

  // Publishers
  can_status_pub_ = nh_->advertise<driving_common::CanStatus> ("CanStatus", 5);
  applanix_pub_ = nh_->advertise<applanix::ApplanixPose> ("ApplanixPose", 5);
  localize_pub_ = nh_->advertise<localize::LocalizePose> ("LocalizePose", 5);
  perception_pub_ = nh_->advertise<perception::PerceptionObstacles> ("PerceptionObstacles", 5);
  radar1_pub_ = nh_->advertise<driving_common::RadarSensor> ("RadarSensor1", 5);
  radar2_pub_ = nh_->advertise<driving_common::RadarSensor> ("RadarSensor2", 5);
  radar3_pub_ = nh_->advertise<driving_common::RadarSensor> ("RadarSensor3", 5);
  radar4_pub_ = nh_->advertise<driving_common::RadarSensor> ("RadarSensor4", 5);
  radar5_pub_ = nh_->advertise<driving_common::RadarSensor> ("RadarSensor5", 5);
  sim_ground_truth_pub_ = nh_->advertise<driving_common::SimulatorGroundTruth> ("SimulatorGroundTruth", 5);
  sim_tag_pub_ = nh_->advertise<driving_common::SimulatorTag> ("SimulatorTag", 5);
  error_pub_ = nh_->advertise<driving_common::ErrorString> ("ErrorString", 5);
  err_msg_.host = dgc::dgc_hostname();

  num_vehicles = 1; // TODO: fix namespaces to make other cars available

  /* initialize the vehicles */
  fprintf(stderr, "Simulation includes %d vehicles.\n", num_vehicles);
  vehicle = new vehicle_state[num_vehicles];
  sim_laser = new bool[num_vehicles];
  fake_perception = new bool[num_vehicles];
  start_lat = new double[num_vehicles];
  start_lon = new double[num_vehicles];
  start_theta = new double[num_vehicles];

  for (i = 0; i < num_vehicles; i++) {
    try {
    vehicle[i].set_passat_params();
    get_start_state(start_lat[i], start_lon[i], start_theta[i], vel, max_steering, max_throttle, max_brake, sim_laser[i], fake_perception[i]);
    if (sim_laser[i]) load_map = 1;
    vehicle[i].reset();
    vehicle[i].param.max_steering = max_steering;
    vehicle[i].param.max_throttle = max_throttle;
    vehicle[i].param.max_brake = max_brake;
    vehicle[i].set_position(start_lat[i], start_lon[i], start_theta[i]);
    vehicle[i].set_velocity(vel, 0);
    vehicle[i].param.bicycle_model = bicycle_model;
    vehicle[i].param.torque_mode = torque_mode;
    }
    catch(vlr::Ex<>& e) {
      std::cout << e.what() << "\n";
      exit(-5);
    }
  }

  /* load the RNDF file, if available */
  //  rndf = new rndf_file;
  if (rn.loadRNDF(rndf_filename)) {
    rndf_valid = 1;
  }
  if (rndf_valid) {
    lights = new TrafficLightSimulator(rn, *nh_);
    crosswalks = new CrosswalkSimulator(rn, *nh_);
  }

 //  add_waypoint_data(rndf);

  if (load_map) vlr::load_obstacle_map(obstacle_map_filename);

  start_time = driving_common::Time::current();

  ros::Rate r(100); // 100 hz

  while (ros::ok()) {

    tenhz = !tenhz;

    // update the vehicle positions
    for (i = 0; i < num_vehicles; i++) {
      for (j = 0; j < 5; j++)
        vehicle[i].update(1.0 / 100.0);
    }

    if (load_map) {
      vlr::fill_car_segments(num_vehicles, vehicle);
    }

    if (tenhz) {
      precompute_fake_obstacles();
      crosswalks->update(dgc_get_time());
      compute_warnings();
    }

    for (i = 0; i < num_vehicles; i++) {
      publish_applanix(&vehicle[i]);
      publish_can(&vehicle[i]);
      publish_groundtruth();

      if (tenhz) {
        publish_collision_info();
        publish_localize(&vehicle[i]);

        if (fake_perception[i]) {
          publish_dirk_obstacles(i);
          publish_traffic_lights();
        }

        else if (sim_laser[i]) {
          publish_ldlrs1(&vehicle[i]);
        }
        publish_radar(i, 1);
        publish_radar(i, 2);
        publish_radar(i, 3);
        publish_radar(i, 4);
        publish_radar(i, 5);
      }

      if (iter % 20 == 0) publish_tags();
    }
    iter++;
    current_time = dgc_get_time();
    left = (start_time + iter * 0.05) - current_time;
    if (left > 0) usleep((int) (left * 1e6));
    if (current_time - last_printout > 0.5 && tenhz) {
      //fprintf(stderr, "\rSIM:   %.1f%% idle      \n", left / 0.05 * 100.0);
      last_printout = current_time;
    }

    ros::spinOnce();
    r.sleep();
  }

  delete lights;
  delete nh_;

  return 0;
}
