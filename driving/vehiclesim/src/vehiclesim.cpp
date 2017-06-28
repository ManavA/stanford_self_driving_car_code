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
#include <global.h>
#include <transform.h>
#include <lltransform.h>
#include <passat_constants.h>
#include <applanix/ApplanixPose.h>
#include <driving_common/Actuator.h>
#include <driving_common/CanStatus.h>
#include <driving_common/EStopStatus.h>
#include <vehicle.h>

ros::NodeHandle* nh_=NULL;
ros::Publisher can_status_pub_;
ros::Publisher applanix_pub_;
ros::Subscriber actuator_sub_;
ros::Subscriber estop_status_sub_;

vlr::vehicle_state vehicle;
int torque_mode;
int debug = 0;

double initial_lat, initial_lon, initial_yaw;

namespace vlr {

void passatActuatorHandler(const driving_common::Actuator& actuator)
{
	fprintf( stderr, "%f %f %f\n", actuator.steering_value, actuator.throttle_fraction,
		actuator.brake_pressure );
  if(vehicle.param.torque_mode) {
    vehicle.set_torque_controls(actuator.steering_value,
                                actuator.throttle_fraction,
                                actuator.brake_pressure);

    fprintf(stderr, "\rTHROTTLE %3.0f%% - BRAKE %3.1f - STEERING %5.1f     ", 
            actuator.throttle_fraction * 100, actuator.brake_pressure,
            actuator.steering_value);
  } else {
    vehicle.set_controls(actuator.steering_value,
                         actuator.throttle_fraction,
                         actuator.brake_pressure);

    fprintf(stderr, "\rTHROTTLE %3.0f%% - BRAKE %3.1f - STEERING %5.1f     ", 
            actuator.throttle_fraction * 100, actuator.brake_pressure,
            actuator.steering_value);
  }
}

void estopStatusHandler(const driving_common::EStopStatus& estop) {
  vehicle.paused = (estop.estop_code != driving_common::EStopStatus::ESTOP_RUN);
}

void publish_applanix(vehicle_state *vehicle)
{
  static applanix::ApplanixPose pose;
  
  vehicle->fill_applanix_message(pose);

  pose.timestamp = driving_common::Time::current();
  pose.hardware_timestamp = pose.timestamp;
  applanix_pub_.publish(pose);
}

void publish_can(vehicle_state *vehicle)
{
  static driving_common::CanStatus can;

  vehicle->fill_can_message(can);
  can.timestamp = driving_common::Time::current();
  can_status_pub_.publish(can);
}

void simulator_timer() {
  vehicle.update(1.0 / 100.0);
  publish_applanix(&vehicle);
  publish_can(&vehicle);
}

template <class T> void getParam(std::string key, T& var) {
  if(!nh_->getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + std::string("."));
  }
}

void getParamTransform(std::string key, dgc::dgc_transform_t& t) {
  XmlRpc::XmlRpcValue list;
  getParam(key, list);
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw VLRException("Parameter " + key + std::string(" is not an Array (and therefore not a transform)."));
  }

  for (int32_t i = 0; i < list.size(); ++i) {
    if (list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
      throw VLRException("Array " + key + std::string(" contains non-double values."));
    }
    int32_t x = i%4;
    int32_t y = i/4;
    t[y][x] = list[i];
  }
}

void read_parameters()
{
//  dgc::Param params[] = {
//    {"sim", "vehicle_start_latitude", dgc::DGC_PARAM_DOUBLE, &initial_lat, 0, NULL},
//    {"sim", "vehicle_start_longitude", dgc::DGC_PARAM_DOUBLE, &initial_lon, 0, NULL},
//    {"sim", "vehicle_start_theta", dgc::DGC_PARAM_DOUBLE, &initial_yaw, 0, NULL},
//    {"vehiclesim", "torque_mode", dgc::DGC_PARAM_ONOFF, &torque_mode, 0, NULL},
//    {"passat", "max_throttle", dgc::DGC_PARAM_DOUBLE, &vehicle.param.max_throttle, 1, NULL},
//    {"passat", "max_steering", dgc::DGC_PARAM_DOUBLE, &vehicle.param.max_steering, 1, NULL},
//    {"passat", "max_brake", dgc::DGC_PARAM_DOUBLE, &vehicle.param.max_brake, 1, NULL},
//    {"passat", "max_torque", dgc::DGC_PARAM_DOUBLE, &vehicle.param.max_torque, 1, NULL},
//  };

  getParam("vehicle_start_latitude", initial_lat);
  getParam("vehicle_start_longitude", initial_lon);
  getParam("vehicle_start_theta", initial_yaw);
  getParam("/driving/passat/max_throttle", vehicle.param.max_throttle);
  getParam("/driving/passat/max_steering", vehicle.param.max_steering);
  getParam("/driving/passat/max_brake", vehicle.param.max_brake);
  getParam("/driving/passat/max_torque", vehicle.param.max_torque);
  getParam("torque_mode", torque_mode);
}

} // namespace vlr

int main(int argc, char **argv) {

  ros::init(argc, argv, "vehiclesim");
  nh_ = new ros::NodeHandle("/driving");
  vehicle.reset();
  vehicle.set_passat_params();

  vlr::read_parameters();
  vehicle.param.torque_mode = torque_mode;
  vehicle.set_position(initial_lat, initial_lon, initial_yaw);
  vehicle.set_velocity(dgc::dgc_mph2ms(0), 0);
  vehicle.param.bicycle_model = 1;

  //  dgc::IpcMessageID messages[] = {
  //      dgc::CanStatusID, dgc::PassatActuatorID, dgc::ApplanixPoseID, dgc::EstopStatusID
  //  };

    // Subscribers
  estop_status_sub_ = nh_->subscribe("estop_status", 5, vlr::estopStatusHandler);
  actuator_sub_ = nh_->subscribe("actuator", 5, vlr::passatActuatorHandler);

    // Publishers
  applanix_pub_ = nh_->advertise<applanix::ApplanixPose>("applanix", 5);
  can_status_pub_ = nh_->advertise<driving_common::CanStatus>("can_status", 5);

  ros::Rate r(100); // 100 hz
  bool run_sim = true;

  while (run_sim) {
    vlr::simulator_timer();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
