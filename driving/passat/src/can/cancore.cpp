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


#include <global.h>
#include <serial.h>
#include <driving_common/CanStatus.h>
#include <driving_common/Heartbeat.h>
#include <usbfind.h>
#include <cancore.h>

namespace drc = driving_common;

namespace vlr {

CanServer::CanServer() :
  nh_("/driving"), fd_(-1), last_update_(0), last_publish_(0) {

  heartbeat_.modulename="can";

  readParameters();

  can_status_pub_ = nh_.advertise<driving_common::CanStatus> ("CanStatus", 1);
  heartbeat_pub_ = nh_.advertise<driving_common::Heartbeat> ("Heartbeat", 1);

  connectToCan();
}

CanServer::~CanServer() {
  disconnectFromCan();
}

void CanServer::readParameters() {
  if (!nh_.getParam("vehicle/can_device", can_device_)) {
    throw VLRException("Cannot read parameter vehicle/can_device.");
  }
}

void CanServer::connectToCan() {
  std::string port;

  if(!usbFindLookupParamString(can_device_, port)) {
    throw VLRException("Could not find port for " + can_device_ + std::string("."));
  }

  if (dgc::dgc_serial_connect(&fd_, port.c_str(), 115200) < 0) {
    throw VLRException("Could not connect to passat CAN bus.");
  }

  dgc::dgc_serial_setparams(fd_, 115200, 'N');
}

bool CanServer::readMessage() {
  static uint8_t buffer[1000];
  static int buffer_length = 0;
  int nread, i;
  uint16_t temp;
  uint8_t *data;

    // fill the buffer
  if (buffer_length < 47) {
    nread = dgc::dgc_serial_readn(fd_, buffer + buffer_length, 47 - buffer_length, -1);
    if (nread > 0) {buffer_length += nread;}
  }

  if (buffer_length >= 47 && buffer[45] == 0x55 && buffer[46] == 0xAA) {
    data = buffer;
    can_status_.esp_status = (data[0] >> 4) & 0x01;
    can_status_.abs_status = (data[0] >> 2) & 0x01;

    can_status_.engine_rpm = (data[2] * 256 + data[1]) * 0.25;

    can_status_.rpm_error = (data[2] == 0xFF);

    can_status_.throttle_position = data[3] * 0.4;
    can_status_.throttle_error = (data[3] == 0xFF);

    can_status_.steering_angle = (data[5] & 0x7F) * 256 + data[4];
    if (data[5] & 0x80) {can_status_.steering_angle *= -1;}
    can_status_.steering_angle *= 0.04375;

    can_status_.steering_rate = (data[7] & 0x7F) * 256 + data[6];
    if (data[7] & 0x80) {can_status_.steering_rate *= -1;}
    can_status_.steering_rate *= 0.04375;

    //    can_status_.clutch_status = (data[8] & 0x18) >> 3;

    can_status_.target_gear = (data[9] & 0x0F);
    can_status_.gear_position = ((data[9] & 0xF0) >> 4);

    temp = (data[11] << 7) | (data[10] >> 1);
    can_status_.wheel_speed_fl = temp * 0.01;
    if (data[10] & 0x01) {can_status_.wheel_speed_fl *= -1;}

    temp = (data[13] << 7) | (data[12] >> 1);
    can_status_.wheel_speed_fr = temp * 0.01;
    if (data[12] & 0x01) {can_status_.wheel_speed_fr *= -1;}

    temp = (data[15] << 7) | (data[14] >> 1);
    can_status_.wheel_speed_rl = temp * 0.01;
    if (data[14] & 0x01) {can_status_.wheel_speed_rl *= -1;}

    temp = (data[17] << 7) | (data[16] >> 1);
    can_status_.wheel_speed_rr = temp * 0.01;
    if (data[16] & 0x01) {can_status_.wheel_speed_rr *= -1;}

    can_status_.brake_pressure = (double) ((data[19] & 0x0F) * 256 + data[18]) * 0.1;

    can_status_.wheel_height_fl = data[20] - 127;
    can_status_.wheel_height_fr = data[21] - 127;
    can_status_.wheel_height_rl = data[22] - 127;
    can_status_.wheel_height_rr = data[23] - 127;

    can_status_.wheel_height_fl_error = (data[20] == 0xFE || data[20] == 0xFF);
    can_status_.wheel_height_fr_error = (data[21] == 0xFE || data[21] == 0xFF);
    can_status_.wheel_height_rl_error = (data[22] == 0xFE || data[22] == 0xFF);
    can_status_.wheel_height_rr_error = (data[23] == 0xFE || data[23] == 0xFF);

    can_status_.parking_brake = ((data[24] >> 6) & 0x01);

    can_status_.steering_status = data[25];

    can_status_.avg_wheel_revolutions = (double) (data[27] * 256 + data[26]) * 0.002;

    can_status_.distance_pulses_front_axle = data[29] * 256 + data[28];

    can_status_.yaw_rate = (double) ((data[31] & 0x3F) * 256 + data[30]) * 0.01;
    if (data[31] & 0x80) {can_status_.yaw_rate *= -1;} // highest bit
    if (data[31] & 0x40) {can_status_.yaw_rate = 200;} // second highest bit     // ?!?

    can_status_.backing_up_light = (data[32] & 0x01);

    can_status_.wheel_impulses_fl = ((data[35] & 0x03) << 8) | data[34];
    if (data[33] & 16) {can_status_.wheel_impulses_fl = 1030;} // Qualification bit wheel impulses indicates invalid value


      // wheel rolls backwards?
    if (data[39] & 16) {can_status_.wheel_direction_fl = -1;} else {can_status_.wheel_direction_fl = 1;}
    if (data[39] & 1) {can_status_.wheel_direction_fl = 0;} // Qualification bit wheel direction indicates invalid value


    can_status_.wheel_impulses_fr = ((data[36] & 0x0F) << 6) | (data[35] >> 2);
    if (data[33] & 32) {can_status_.wheel_impulses_fr = 1030;} // Qualification bit wheel impulses indicates invalid value


      // wheel rolls backwards?
    if (data[39] & 32) {can_status_.wheel_direction_fr = -1;} else {can_status_.wheel_direction_fr = 1;}
    if (data[39] & 2) {can_status_.wheel_direction_fr = 0;} // Qualification bit wheel direction indicates invalid value


    can_status_.wheel_impulses_rl = ((data[37] & 0x3F) << 4) | (data[36] >> 4);
    if (data[33] & 64) {can_status_.wheel_impulses_rl = 1030;} // Qualification bit wheel impulses indicates invalid value

      // wheel rolls backwards ?
    if (data[39] & 64) {can_status_.wheel_direction_rl = -1;}  else {can_status_.wheel_direction_rl = 1;}
    if (data[39] & 4) {can_status_.wheel_direction_rl = 0;} // Qualification bit wheel direction indicates invalid value


    can_status_.wheel_impulses_rr = (data[38] << 2) | (data[37] >> 6);
    if (data[33] & 128) {can_status_.wheel_impulses_rr = 1030;} // Qualification bit wheel impulses indicates invalid value


      // wheel rolls backwards?
    if (data[39] & 128) {can_status_.wheel_direction_rr = -1;} // works just in Passat newer than 2008
    else {can_status_.wheel_direction_rr = 1;} // works just in Passat newer than 2008
    if (data[39] & 8) // Qualification bit wheel direction indicates invalid value
    {can_status_.wheel_direction_rr = 0;} // works just in Passat newer than 2008

      // wheel direction rear right, from additional sensor/uC from VW Germany
    if (data[40] & 128) {can_status_.wheel_direction_rr_added = 1;} // forwards //?!?
    else {can_status_.wheel_direction_rr_added = -1;}               // backwards

    can_status_.steer_angleCalculated = (double) ((data[42] & 0x0F) * 256 + data[41]) * 0.15;
    if (data[42] & 0x10)  {can_status_.steer_angleCalculated *= -1;} //sign
    if (data[42] & 0x20) //Qualification bit angleCalculated    0=valid
    {can_status_.steer_angleCalculated = 1000;} //angleCalculated invalid

    can_status_.steer_handTorque = (double) ((data[44] & 0x03) * 256 + data[43]) * 0.0147;
    if (data[44] & 0x04) //sign
    {can_status_.steer_handTorque *= -1;}
    if (data[44] & 0x08) //Qualification bit handTorque     0=valid
    {can_status_.steer_handTorque = 1000;} //handTorque  invalid
    can_status_.steer_statusEPS = data[44] >> 4; // according Lastenheft PLA/EPS

    can_status_.timestamp = drc::Time::current();

    // move any remaining bytes to beginning of the buffer if necessary
    if (buffer_length == 47) {buffer_length = 0;}
    else {
      memmove(buffer, buffer + 47, buffer_length - 47);
      buffer_length -= 47;
    }
    return true;
  }
  else {
    if (buffer_length > 0) {
        // shift everything by one byte and try again
      for (i = 0; i < buffer_length - 1; i++) {buffer[i] = buffer[i + 1];}
      buffer_length--;
    }
    return false;
  }

  return false;
}

void CanServer::disconnectFromCan() {
  if (fd_ != -1) {
    dgc::dgc_serial_close( fd_);
  }
}

void CanServer::run() {
  ros::Rate r(100); // 100 Hz
  while (ros::ok()) {
    double current_time = drc::Time::current();

    if (readMessage()) {
      can_status_pub_.publish(can_status_);
      last_publish_ = current_time;
    }
    else {
      fprintf(stderr, "X");
    }

    if (current_time - last_update_ > 1.0) {
      heartbeat_pub_.publish(heartbeat_);
      last_update_ = drc::Time::current();
      if (current_time - last_publish_ < 1.0) fprintf(stderr, ".");
    }

    r.sleep();
  }
}

} //namespace vlr
