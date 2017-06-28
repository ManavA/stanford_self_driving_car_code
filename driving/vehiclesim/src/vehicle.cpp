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


#define _GLIBCXX_USE_C99_MATH 1
#define _GLIBCXX_USE_C99_FP_MACROS_DYNAMIC 1
#include <cmath>
#include <global.h>
#include <lltransform.h>
#include <passat_constants.h>
#include <vehicle.h>

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

using namespace dgc;

namespace vlr {

vehicle_state::vehicle_state() : added_error_x(0), added_error_y(0) {
}

void vehicle_state::set_passat_params() {
  param.a = 1.37;
  param.b = 1.47;
  param.tire_stiffness = 145000.0;
  param.mass = 2943.0;
  param.iz = 5533.4531;
  param.tau = 0.2;
  param.steering_ratio = DGC_PASSAT_STEERING_RATIO;
  param.wheel_base = DGC_PASSAT_WHEEL_BASE;
  param.imu_to_cg_dist = DGC_PASSAT_WHEEL_BASE + DGC_PASSAT_FA_TO_BUMPER_DIST -
    DGC_PASSAT_LENGTH / 2.0;
  param.torque_mode = 0;
  param.max_steering_rate = 650;
  param.max_steering = 525.0;
  
  param.max_wheel_angle = param.max_steering*(M_PI/180)/param.steering_ratio;
  param.max_wheel_rate = param.max_steering_rate*(M_PI/180)/
    param.steering_ratio;
  
  param.max_torque = 1.0;
  param.max_brake = 100.0;
  param.max_throttle = 1.0;
  param.steer_inertia = 0.1;
  param.brake_decel_coef = 0.08125;
  param.throttle_accel_coef = 3.0;
}

void vehicle_state::reset(void)
{
  origin_x = 0;
  origin_y = 0;
  utmzone[0] = '\0';
  x = 0;
  y = 0;
  yaw = 0;
  v_x = 0;
  v_y = 0;
  yaw_rate = 0;
  wheel_angle = 0;
  wheel_angle_rate = 0;
  commanded_wheel_angle = 0;
  actual_forward_accel = 0;
  commanded_forward_accel = 0;
  torque = 0;
  added_error_x = 0;
  added_error_y = 0;
  direction = 1;
  commanded_direction = 1;
  shift_timer = -1;
  paused = 1;
}

void vehicle_state::set_position(double lat, double lon, double yaw)
{
  vlr::latLongToUtm(lat, lon, &origin_x, &origin_y, utmzone);
  x = 0;
  y = 0;
  this->yaw = yaw;
}

void vehicle_state::set_position(double x, double y, char *utmzone, double yaw)
{
  origin_x = x;
  origin_y = y;
  this->x = 0;
  this->y = 0;
  strcpy(this->utmzone, utmzone);
  this->yaw = yaw;

  this->wheel_angle = 0;
  this->commanded_wheel_angle = 0;
  this->commanded_forward_accel = 0;
}

void vehicle_state::set_direction(int direction)
{
  commanded_direction = direction;
}

void vehicle_state::set_velocity(double forward_vel, double yaw_rate)
{
  v_x = forward_vel;
  v_y = 0;
  this->yaw_rate = yaw_rate;

  this->wheel_angle = 0;
  this->commanded_wheel_angle = 0;
  this->commanded_forward_accel = 0;
}

bool my_isnormal(double x)
{
  int i = fpclassify(x);
  if(i == FP_NORMAL || i == FP_ZERO)
    return true;
  else 
    return false;
}

const char* why_not_normal(double x)
{
  int i = fpclassify(x);
  
  if(i == FP_NAN)
    return "NAN";
  else if(i == FP_INFINITE)
    return "INFINITE";
  else if(i == FP_ZERO)
    return "ZERO";
  else if(i == FP_SUBNORMAL)
    return "SUBNORMAL";
  else
    return "NORMAL";
}

void vehicle_state::update(double dt)
{
  double alpha_f, alpha_r, Fxf, Fxr, Fyf, Fyr;
  double x_dot, y_dot, yaw_dot;
  double v_x_dot, v_y_dot, yaw_rate_dot, wheel_angle_dot;
  double wheel_angle_rate_dot;


  if(shifting) {
    shift_timer += dt;
    if(shift_timer > 2.0) {
      direction = commanded_direction;
      shifting = 0;
    }
  }
  
 
  /* if we get a change of direction command, max brake to zero speed */
  if (direction != commanded_direction)
    actual_forward_accel = -param.max_brake * param.brake_decel_coef;
  else {
    if(commanded_forward_accel < 0) 
      actual_forward_accel += 1 / 0.3 * (commanded_forward_accel - 
					 actual_forward_accel) * dt;
    else
      actual_forward_accel += 1 / 0.4 * (commanded_forward_accel - 
					 actual_forward_accel) * dt;

    actual_forward_accel = commanded_forward_accel;
    //    if(paused && v_x > 0)
    if(paused)
      actual_forward_accel = -3.0;
    //    if(paused && v_x < 0)
    //      actual_forward_accel = 3.0;
  }

  if(param.bicycle_model && v_x > 0) {
    /* front wheel drive car - input force */
    Fxr = 0;
    if(direction)
      Fxf = param.mass * actual_forward_accel;
    else
      Fxf = param.mass * -actual_forward_accel;
      
    
    /* steering dynamics */
    if(param.torque_mode) {
      wheel_angle_rate_dot = (torque - param.steering_ratio / 
			      dgc_d2r(param.max_steering_rate) * 
			      wheel_angle_rate) / param.steer_inertia;
      wheel_angle_dot = wheel_angle_rate;
    }
    else {
      wheel_angle_dot = 1 / param.tau * (commanded_wheel_angle - wheel_angle);
      wheel_angle_rate = wheel_angle_dot;
      wheel_angle_rate_dot = 0;
    }
    
    /* full bicycle model with limited steering dynamics */
    if(v_x < dgc_mph2ms(5)) {
      alpha_f = atan2(v_y + yaw_rate * param.a, dgc_mph2ms(5)) - wheel_angle;
      alpha_r = atan2(v_y - yaw_rate * param.b, dgc_mph2ms(5));
    }
    else {
      alpha_f = atan2(v_y + yaw_rate * param.a, v_x) - wheel_angle;
      alpha_r = atan2(v_y - yaw_rate * param.b, v_x);
    }
    
    Fyf = -param.tire_stiffness * alpha_f;
    Fyr = -param.tire_stiffness * alpha_r;
    x_dot = v_x * cos(yaw) - v_y * sin(yaw);
    y_dot = v_x * sin(yaw) + v_y * cos(yaw);
    yaw_dot = yaw_rate;
    v_x_dot = 1 / param.mass * (Fxr + Fxf * cos(wheel_angle) - 
				Fyf * sin(wheel_angle)) + yaw_rate * v_y;
    v_y_dot = 1 / param.mass * (Fyr + Fxf * sin(wheel_angle) +
				Fyf * cos(wheel_angle)) - yaw_rate * v_x;
    yaw_rate_dot = 1 / param.iz * (param.a * Fxf * sin(wheel_angle) + 
				   param.a * Fyf * cos(wheel_angle) -
				   param.b * Fyr);
  }
  else {
    /* steering dynamics */
    if (param.torque_mode) {
      wheel_angle_rate_dot = (torque - param.steering_ratio / 
			      dgc_d2r(param.max_steering_rate) * 
			      wheel_angle_rate) / param.steer_inertia;
      wheel_angle_dot = wheel_angle_rate;

      //fprintf( stderr, "(%f)\n", torque );

    }
    else {
      wheel_angle_dot = 1 / param.tau * (commanded_wheel_angle - wheel_angle);
      wheel_angle_rate = wheel_angle_dot;
      wheel_angle_rate_dot = 0;
    }

    x_dot = v_x * cos(yaw);
    y_dot = v_x * sin(yaw);
    yaw_dot = v_x * tan(wheel_angle) / param.wheel_base;
    
    if (!direction) {
      actual_forward_accel *= -1;
    }

    /*    if(direction)
      v_x_dot = actual_forward_accel;
    else
    v_x_dot = -actual_forward_accel;*/
    v_x_dot = actual_forward_accel;
    //fprintf( stderr, "[%f]", actual_forward_accel );
    v_y_dot = 0;
    yaw_rate_dot = 0;
  }

  if(!my_isnormal(x)) {
    fprintf(stderr, "S before something wrong with pos x %f %s\n", x,
	    why_not_normal(x));
  }
  
  // state (x,y) tracks center of gravity;
  // if using simple bicycle model, need to convert to rear axle for update
  double cg_to_ra_dist = -DGC_PASSAT_IMU_TO_CG_DIST + DGC_PASSAT_IMU_TO_FA_DIST
    - DGC_PASSAT_WHEEL_BASE;
  if (!param.bicycle_model) {
    x += cg_to_ra_dist * cos(yaw);
    y += cg_to_ra_dist * sin(yaw);
  }

  /* first order integration */
  x += x_dot * dt;
  y += y_dot * dt;
  yaw += yaw_dot * dt;
  v_x += v_x_dot * dt;
  if(direction && v_x < 0) {
    actual_forward_accel = 0;
    v_x = 0;
  }
  else if(!direction && v_x > 0) {
    actual_forward_accel = 0;
    v_x = 0;
  }
  v_y += v_y_dot * dt;
  yaw_rate += yaw_rate_dot * dt;
  if(v_x == 0) {
    v_y = 0;
    yaw_rate = 0;
  }

  /* convert back to cg coordinates */
  if (!param.bicycle_model) {
    x -= cg_to_ra_dist * cos(yaw);
    y -= cg_to_ra_dist * sin(yaw);
  }

  if(!my_isnormal(x)) {
    fprintf(stderr, "S after something wrong with pos x %f %s\n", x,
	    why_not_normal(x));		       
    fprintf(stderr, "commanded steering %f accel %f dt %f\n", commanded_wheel_angle,
	    commanded_forward_accel, dt);
    exit(0);
  }
  

  if(wheel_angle > dgc_d2r(param.max_steering) / param.steering_ratio) {
    wheel_angle = dgc_d2r(param.max_steering) / param.steering_ratio;
    if(wheel_angle_rate > 0)
      wheel_angle_rate = 0;
    if(wheel_angle_dot > 0)
      wheel_angle_dot = 0;
    if(wheel_angle_rate_dot > 0)
      wheel_angle_rate_dot = 0;
  }
  else if(wheel_angle < -dgc_d2r(param.max_steering) / param.steering_ratio) {
    wheel_angle = -dgc_d2r(param.max_steering) / param.steering_ratio;
    if(wheel_angle_rate < 0)
      wheel_angle_rate = 0;
    if(wheel_angle_dot < 0)
      wheel_angle_dot = 0;
    if(wheel_angle_rate_dot < 0)
      wheel_angle_rate_dot = 0;
  }

  wheel_angle += wheel_angle_dot * dt;
  wheel_angle_rate += wheel_angle_rate_dot * dt;

  if(param.bicycle_model) {
    lateral_accel = v_y_dot;
  }
  else {
    lateral_accel = dgc_square(v_x) * tan(wheel_angle) / param.wheel_base;
    yaw_rate = v_x * tan(wheel_angle) / param.wheel_base;
  }

  /* after coming to a stop, start timer for shift */
  if(direction != commanded_direction && !shifting && v_x == 0) {
    shift_timer = 0;
    shifting = 1;
  }

  //fprintf (stderr, "\r  v_x = %4.3f  wheel_angle = %4.3f %4.3f %4.3f  accel = %4.3f ", v_x,
  //         commanded_wheel_angle, wheel_angle, wheel_angle_dot, v_x_dot) ;

}

void vehicle_state::set_controls(double steering_angle,
				 double throttle_fraction,
				 double brake_pressure)
{
  if(throttle_fraction > param.max_throttle)
    throttle_fraction = param.max_throttle;
  if(brake_pressure > param.max_brake)
    brake_pressure = param.max_brake;
  if(steering_angle > dgc_d2r(param.max_steering))
    steering_angle = dgc_d2r(param.max_steering);
  if(steering_angle < -dgc_d2r(param.max_steering))
    steering_angle = -dgc_d2r(param.max_steering);

  throttle = throttle_fraction - brake_pressure/100.0;

  commanded_wheel_angle = steering_angle / param.steering_ratio;
  if(brake_pressure > 0)
    commanded_forward_accel = -brake_pressure * param.brake_decel_coef;
  else
    commanded_forward_accel = throttle_fraction * param.throttle_accel_coef;
}

void vehicle_state::set_torque_controls(double torque_in,
					double throttle_fraction,
					double brake_pressure)
{
  if(throttle_fraction > param.max_throttle)
    throttle_fraction = param.max_throttle;
  if(brake_pressure > param.max_brake)
    brake_pressure = param.max_brake;
  if (torque_in > param.max_torque) torque_in = param.max_torque;
  if (torque_in < -param.max_torque) torque_in = -param.max_torque;
  torque = torque_in;
  
  throttle = throttle_fraction - brake_pressure/100.0;

  if(brake_pressure > 0)
    commanded_forward_accel = -brake_pressure * param.brake_decel_coef;
  else
    commanded_forward_accel = throttle_fraction * param.throttle_accel_coef;
}

void vehicle_state::fill_can_message(driving_common::CanStatus& can)
{
  //can.throttle_position = commanded_forward_accel / 3.8;
  can.throttle_position = throttle;
  can.steering_angle = dgc_r2d(wheel_angle * param.steering_ratio);
  can.steering_angle = (int)rint(can.steering_angle / 1.4) * 1.4;
  can.steering_rate = dgc_r2d(wheel_angle_rate * param.steering_ratio);
  can.wheel_speed_fl = std::abs(dgc::dgc_ms2kph(v_x));
  can.wheel_speed_fr = std::abs(dgc::dgc_ms2kph(v_x));
  can.wheel_speed_rl = std::abs(dgc::dgc_ms2kph(v_x));
  can.wheel_speed_rr = std::abs(dgc::dgc_ms2kph(v_x));
}

void vehicle_state::fill_applanix_message(applanix::ApplanixPose& pose) {
  double x, y;
  static int iter = 0;

  pose.id = 0;
  pose.wander = 0;
  pose.timestamp = dgc_get_time();
  pose.hardware_timestamp = pose.timestamp;

  //#define ADD_NOISE
#ifdef ADD_NOISE  
  if(iter % 20 == 0) {
    if(((iter / 20) % 2) == 1) {
      added_error_x = 1;
      added_error_y = 1;
    }
    else {
      added_error_x = 0;
      added_error_y = 0;
    }
  }
#endif
  iter++;

  // vehicle model tracks position of the CG - move back to the IMU
  x = this->x + origin_x - param.imu_to_cg_dist * cos(yaw) + added_error_x;
  y = this->y + origin_y - param.imu_to_cg_dist * sin(yaw) + added_error_y;
  
//  printf("%s: %f, %f, %f, %f, %f, %f\n", __PRETTY_FUNCTION__, x, this->x, origin_x, param.imu_to_cg_dist, yaw, added_error_x);
//  printf("%s: %f, %f, %f, %f, %f, %f\n", __PRETTY_FUNCTION__, y, this->y, origin_y, param.imu_to_cg_dist, yaw, added_error_y);
  pose.smooth_x = this->x - param.imu_to_cg_dist * cos(yaw);
  pose.smooth_y = this->y - param.imu_to_cg_dist * sin(yaw);
  
  pose.vel_east = v_x * cos(yaw) - v_y * sin(yaw) + param.imu_to_cg_dist *  yaw_rate * sin(yaw);
  pose.vel_north = v_x * sin(yaw) + v_y * cos(yaw) - param.imu_to_cg_dist * yaw_rate * cos(yaw);
  
  vlr::utmToLatLong(x, y, utmzone, &pose.latitude, &pose.longitude);
  pose.altitude = 0;
  pose.smooth_z = 0;
  
  pose.vel_up = 0;
  pose.speed = hypot(pose.vel_north, pose.vel_east);
  pose.track = atan2(pose.vel_north, pose.vel_east);
  pose.roll = 0;
  pose.pitch = 0;
  pose.yaw = yaw;
  pose.rate_roll = 0;
  pose.rate_pitch = 0;
  pose.rate_yaw = yaw_rate;

  static double last_vel_east = pose.vel_east;
  static double last_vel_north = pose.vel_north;
  static double last_timestamp = pose.timestamp;
  double delta_t = pose.timestamp - last_timestamp;
  pose.accel_x = 0;//(pose.v_east-last_v_east)/delta_t;
  pose.accel_y = 0;//(pose.v_north-last_v_north)/delta_t;
  pose.accel_z = 0;
  last_vel_east = pose.vel_east;
  last_vel_north = pose.vel_north;
  last_timestamp = pose.timestamp;
}

} // namespace vlr
