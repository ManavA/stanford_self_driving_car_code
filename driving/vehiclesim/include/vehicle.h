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


#ifndef VLR_VEHICLE_H_
#define VLR_VEHICLE_H_

#include <applanix/ApplanixPose.h>
#include <driving_common/CanStatus.h>

namespace vlr {

  typedef struct {
  int torque_mode;
  double max_steering, max_throttle, max_brake, max_torque;
  double max_wheel_angle, max_wheel_rate;

  double steering_ratio, wheel_base, imu_to_cg_dist;
  double a, b, tire_stiffness;
  double mass, iz, tau;
  double max_steering_rate;
  double steer_inertia;

  double brake_decel_coef, throttle_accel_coef;
  int bicycle_model;
} vehicle_params;

class vehicle_state {
public:
  vehicle_state();
  void set_passat_params(void);
  void reset(void);
  void update(double dt);
  void set_direction(int forward);
  void set_position(double lat, double lon, double yaw);
  void set_position(double x, double y, char *utmzone, double yaw);
  void set_velocity(double forward_vel, double yaw_rate);
  void set_controls(double steering_angle, double throttle_fraction,
		    double brake_pressure);
  void set_torque_controls(double torque, double throttle_fraction,
			   double brake_pressure);
  void fill_applanix_message(applanix::ApplanixPose& pose);
  void fill_can_message(driving_common::CanStatus& can);

  vehicle_params param;

  /* offset to real world coordinates */
  double origin_x, origin_y;
  char utmzone[5];

  int paused;

  double x, y, yaw;
  double v_x, v_y, yaw_rate;
  double wheel_angle, wheel_angle_rate;
  double commanded_wheel_angle, commanded_forward_accel;
  double actual_forward_accel;
  double lateral_accel;

  double added_error_x, added_error_y;

  double torque, throttle;

  int shifting;
  double shift_timer;
  int commanded_direction, direction;
};

} // namespace vlr
#endif
