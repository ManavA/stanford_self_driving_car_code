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


#include <serial.h>
#include <usbfind.h>
#include <time.h>
#include <global.h>
#include <driving_common/Actuator.h>
#include <passatcore.h>

using namespace dgc;

namespace drc = driving_common;

namespace vlr {

PassatCore::PassatCore() : nh_("/driving"), first_command_(true),
                           requested_direction_(driving_common::Actuator::DIRECTION_FORWARD),
                           requested_steering_(0.0), requested_brake_(0.0), requested_throttle_(0.0),
                           requested_torque_(0.0), requested_gear_(GEAR_NEUTRAL),
                           requested_signal_(driving_common::TurnSignal::NONE), requested_ebrake_(false),
                           steering_commands_allowed_(true), engine_commands_allowed_(true),
                           steering_autonomous_(true), brake_autonomous_(true), throttle_autonomous_(true), gear_shift_autonomous_(true),
                           max_steering_(360.0), max_torque_(1.0), max_brake_(1e6), max_throttle_(1.0),
                           message_counter_(0), num_bytes_(0), command_log_fp_(NULL), start_time_(0) {


  readParameters();

    // this will apply limits to the read parameters
  updateControlLimits(max_steering_, max_torque_, max_throttle_, max_brake_);

  passat_status_pub_ = nh_.advertise<passat::PassatStatus>("PassatStatus", 5);
  passat_output_pub_ = nh_.advertise<passat::PassatOutput>("PassatOutput", 5);
  passat_ext_output_pub_ = nh_.advertise<passat::PassatExtOutput>("PassatExtOutput", 5);

  if(!usbFindLookupParamString(device_, port_)) {
    throw VLRException("Could not connect to " + device_ + std::string("."));
  }

  if(dgc_serial_connect(&fd_, port_.c_str(), PASSAT_BAUD_RATE) < 0) {
    throw VLRException("Could not connect to passat at port " + port_);
  }

  time_t clock_time = time(NULL);
  struct tm* local_time = localtime(&clock_time);

  char filename[200];
  sprintf(filename, "/tmp/passat-%02d-%02d-%04d_%02d-%02d-%02d.log", local_time->tm_mon + 1, local_time->tm_mday, local_time->tm_year + 1900,
      local_time->tm_hour, local_time->tm_min, local_time->tm_sec);
  fprintf(stderr, "\nStarting passat logfile %s\n", filename);

  command_log_fp_ = fopen(filename, "w");
  if (!command_log_fp_) {
    throw VLRException("Could not open log file " + std::string(filename) + std::string("for writing."));
  }

  start_time_ = drc::Time::current();
}

PassatCore::~PassatCore() {
  updateControlLimits(0, 0, 0, 0);
  fclose( command_log_fp_);
  steering_autonomous_ = false;
  brake_autonomous_ = false;
  throttle_autonomous_ = false;
  gear_shift_autonomous_ = false;
  sendCachedTorqueCommand();
  sendCachedExtendedCommand();
}

template <class T> void PassatCore::getParam(std::string key, T& var) {
  if(!nh_.getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + std::string("."));
  }
}

void PassatCore::readParameters() {
  getParam("vehicle/device", device_);
  getParam("vehicle/torque_control", torque_control_); // TODO: move to Passat?!?
  getParam("vehicle/belt_steering", belt_steering_);
  getParam("vehicle/max_throttle", max_throttle_);
  getParam("vehicle/max_brake", max_brake_);
  getParam("vehicle/max_steering", max_steering_);
  getParam("vehicle/max_torque", max_torque_);
  getParam("vehicle/steering_auto", steering_autonomous_);  // TODO: this does more than just steering :-(
}

void PassatCore::updateControlLimits(double max_steering, double max_torque, double max_throttle, double max_brake) {
  max_steering_ = max_steering;
  max_torque_ = std::max(0.0, std::min(max_torque, 1.0));;
  max_throttle_ = std::max(0.0, std::min(max_throttle, 1.0));
  max_brake_ = std::max(0.0, max_brake);
}

void PassatCore::readStatus() {
    // read only the bytes that are currently available on the port
  int n = dgc_serial_bytes_available(fd_);
  printf("%d bytes\n", n);
  if (num_bytes_ + n > READ_BUFFER_SIZE) n = READ_BUFFER_SIZE - num_bytes_;
  if (n <= 0) {return;}

  int nread = dgc_serial_readn(fd_, read_buffer_ + num_bytes_, n, 0);
  num_bytes_ += nread;

  int64_t i, start_i = 0, end_i = 0;

  bool found_start = false, found_end = false;
  do {
      // search for message header
    found_start = false;
    for (i = 0; i < (int64_t)num_bytes_ - 1; i++) {
      if (read_buffer_[i] == 0xAA && read_buffer_[i + 1] == 0x55) {
        found_start = true;
        start_i = i;
        break;
      }
    }

      // search for message terminator
    found_end = false;
    if (found_start) {
      for (i = start_i + 2; i < (int64_t)num_bytes_ - 1; i++) {
        if (read_buffer_[i] == 0xBB && read_buffer_[i + 1] == 0x66) {
          found_end = 1;
          end_i = i + 1;
          break;
        }
      }
    }

    if (found_start && found_end) {
//      int packet_length = read_buffer_[start_i + 2];
      uint8_t* data = read_buffer_ + start_i + 3;
      int packet_id = ((data[1] << 8) | data[0]);

      if (packet_id == 0x0702) {
        status_.brake_error = (data[3] & 1);
        status_.prc_warning_lamp = ((data[3] & (1 << 4)) != 0);
        status_.can_failure = ((data[3] & (1 << 6)) != 0);
        status_.actual_brake_pressure1 = ((data[5] << 8) | data[6]) * 0.01;
        status_.actual_brake_pressure2 = ((data[10] << 8) | data[11]) * 0.01;
        status_.requested_brake_pressure = (data[7]) * 0.5;
        status_.brake_travel = data[12] * 0.169;
        status_.no_brake_200ms = (data[18] & 1);
        status_.brake_initializing = ((data[18] & (1 << 1)) != 0);
        status_.no_steering_400ms = ((data[18] & (1 << 4)) != 0);
        status_.no_gear_200ms = ((data[18] & (1 << 5)) != 0);
        status_.motor_temp = data[21];
        status_.motor_error_code = (data[24] >> 5);
        status_.rs232_buffer_overruns = ((data[28] << 8) | (data[27]));
        status_.rs232_framing_error = ((data[30] << 8) | (data[29]));
        status_.data_buffer_overrun = ((data[32] << 8) | (data[31]));
        status_.broken_packet = ((data[34] << 8) | (data[33]));
        status_.missed_messages = ((data[36] << 8) | (data[35]));

        /*	fprintf(stderr, "brake error %d warning lamp %d can_failure %d\n", 
         status_.brake_error,
         status_.prc_warning_lamp,
         status_.can_failure);
         fprintf(stderr, "brake pressure 1 %f 2 %f req %f trav %f\n",
         status_.actual_brake_pressure1,
         status_.actual_brake_pressure2,
         status_.requested_brake_pressure,
         status_.brake_travel);
         fprintf(stderr, "no brake %d brake init %d no steer %d no gear %d\n",
         status_.no_brake_200ms,
         status_.brake_initializing,
         status_.no_steering_400ms,
         status_.no_gear_200ms);
         fprintf(stderr, "motor temp %f code %d\n",
         status_.motor_temp,
         status_.motor_error_code);
         fprintf(stderr, "rs232 buffer overruns %d framing error %d buffer overrun %d broken packet %d missed messages %d\n",
         status_.rs232_buffer_overruns,
         status_.rs232_framing_error,
         status_.data_buffer_overrun,
         status_.broken_packet,
         status_.missed_messages);*/

        status_.timestamp = drc::Time::current();
        passat_status_pub_.publish(status_);
      }

      if (end_i + 1 == (int64_t)num_bytes_) {num_bytes_ = 0;}
      else {
        memmove(read_buffer_, read_buffer_ + end_i + 1, num_bytes_ - (end_i + 1));
        num_bytes_ -= (end_i + 1);
      }
    }
    else if (num_bytes_ == READ_BUFFER_SIZE) {
      fprintf(stderr, "Warning: throwing away read buffer data\n");
      num_bytes_ = 0;
    }
  } while (found_start && found_end);
}

void PassatCore::sendExtendedCommand(uint8_t turn_signal, bool engine_kill, bool honk, bool parking_brake) {
  uint8_t messagedata[20];
  int err, i;
  double current_time;

  requested_ebrake_ = parking_brake;
  requested_signal_ = turn_signal;

  messagedata[0] = 0xAA;
  messagedata[1] = 0x55;
  messagedata[2] = 10;

  messagedata[3] = (message_counter_ & 0xFF);
  messagedata[4] = ((message_counter_ >> 8) & 0xFF);

  messagedata[5] = 0xE1;
  messagedata[6] = 0x06;
  messagedata[7] = 0;
  if (turn_signal == driving_common::TurnSignal::LEFT || turn_signal == driving_common::TurnSignal::BOTH) messagedata[7] |= 1;
  if (turn_signal == driving_common::TurnSignal::RIGHT || turn_signal == driving_common::TurnSignal::BOTH) messagedata[7] |= (1 << 1);
  if (engine_kill) messagedata[7] |= (1 << 2);
  if (honk) messagedata[7] |= (1 << 3);
  if (parking_brake) messagedata[7] |= (1 << 4);
  for (i = 5; i < 12; i++)
    messagedata[i + 3] = 0;
  messagedata[14] = (1 | (1 << 3)); // allow turn signals to be autonomous
  messagedata[15] = 0xBB;
  messagedata[16] = 0x66;

  err = dgc_serial_writen(fd_, messagedata, 17, 0.05);
  if (err < 0) dgc_warning("Could not send packet to passat.\n");

  current_time = drc::Time::current();
  fprintf(command_log_fp_, "E %d ", 17);
  for (i = 0; i < 17; i++)
    fprintf(command_log_fp_, "%d ", messagedata[i]);
  fprintf(command_log_fp_, "%d %d %lf %lf\n", err, message_counter_, current_time, current_time - start_time_);

    // send extended output info for logging
  publishExtOutput(requested_signal_, engine_kill, honk, requested_ebrake_, err);

  message_counter_++;
  if (message_counter_ > 32000) {message_counter_ = 0;}
}

void PassatCore::sendTorqueControlCommand(double steering_torque, double brake_pressure, double throttle_fraction, GearState gear_position) {
  uint8_t messagedata[20];
  int16_t steering_int;
  uint16_t brake_int, throttle_int;
  int err, i;
  double current_time;

  last_command_steering_ = 0;

    // take commands either from parameters, or cached controls
  if (steering_commands_allowed_) {
    requested_torque_ = steering_torque;
  }
  if (engine_commands_allowed_) {
    requested_brake_ = brake_pressure;
    requested_throttle_ = throttle_fraction;
  }
  requested_gear_ = gear_position;

    // limit throttle, brake, and steering commands
  requested_throttle_ = std::max(0.0, std::min(requested_throttle_,max_throttle_));
  requested_brake_    = std::max(0.0, std::min(requested_brake_, max_brake_));
  requested_torque_   = std::max(-max_torque_, std::min(requested_torque_, max_torque_));

  messagedata[0] = 0xAA;
  messagedata[1] = 0x55;
  messagedata[2] = 10;

  messagedata[3] = (message_counter_ & 0xFF);
  messagedata[4] = ((message_counter_ >> 8) & 0xFF);

  // construct binary message
  messagedata[5] = 0xE0;
  messagedata[6] = 0x06;

    // steering
  steering_int = (int16_t) rint(requested_torque_ * 32760.0);
  messagedata[7] = (steering_int & 0xFF);
  messagedata[8] = (steering_int >> 8);

    // brake
  brake_int = (uint16_t) rint(requested_brake_ * 20.0);
  messagedata[9] = (brake_int & 0xFF);
  messagedata[10] = (brake_int >> 8);

    // throttle
  throttle_int = (uint16_t) rint(200 + 600 * requested_throttle_);
  messagedata[11] = (throttle_int & 0xFF);
  messagedata[12] = (throttle_int >> 8);

    // gear
  if (requested_gear_ == GEAR_DRIVE) {messagedata[13] = 'D';}
  else if (requested_gear_ == GEAR_REVERSE) {messagedata[13] = 'R';}
  else if (requested_gear_ == GEAR_NEUTRAL) {messagedata[13] = 'N';}
  else {
    throw VLRException("Requested an illegal gear.");
  }

    // flags
  messagedata[14] = 0;
  if (steering_autonomous_) messagedata[14] |= 1;
  if (brake_autonomous_) messagedata[14] |= (1 << 1);
  if (throttle_autonomous_) messagedata[14] |= (1 << 2);
  if (gear_shift_autonomous_) messagedata[14] |= (1 << 3);

    // set to steering torque
  if (belt_steering_) {
    messagedata[14] |= (3 << 5);
  }
  else {
    // use EPS (electric power steering)
    messagedata[14] |= (2 << 5);
  }

  messagedata[15] = 0xBB;
  messagedata[16] = 0x66;

  fprintf(stderr, "\rTHR %3.0f%% - BRA %3.1f - TRQE %.2f %d   ", requested_throttle_ * 100.0, requested_brake_, requested_torque_, requested_ebrake_);

  err = dgc_serial_writen(fd_, messagedata, 17, 0.05);
  if (err < 17) {
    std::cout << "Could not send packet to passat.\n";
  }
  else {
    first_command_ = false;
  }

  current_time = drc::Time::current();
  fprintf(command_log_fp_, "T %d ", 17);
  for (i = 0; i < 17; i++)
    fprintf(command_log_fp_, "%d ", messagedata[i]);
  fprintf(command_log_fp_, "%d %d %lf %lf\n", err, message_counter_, current_time, current_time - start_time_);

  // send output for logging
  publishOutput(requested_torque_, requested_brake_, requested_throttle_, requested_gear_, steering_int, brake_int, throttle_int, err);

  message_counter_++;
  if (message_counter_ > 32000) {message_counter_ = 0;}
}

void PassatCore::sendCachedTorqueCommand() {
  sendTorqueControlCommand(requested_torque_, requested_brake_, requested_throttle_, requested_gear_);
}

void PassatCore::sendCachedExtendedCommand() {
  sendExtendedCommand(requested_signal_, 0, 0, requested_ebrake_);
}

void PassatCore::sendCompleteCommand(double steering_torque, double throttle_command, double brake_command) {
  requested_torque_ = steering_torque;
  requested_brake_ = brake_command;
  requested_throttle_ = throttle_command;
  sendCachedTorqueCommand();
}

void PassatCore::sendSteeringCommand(double steering_angle) {
  requested_steering_ = steering_angle;
  sendCachedTorqueCommand();
}

void PassatCore::sendThrottleCommand(double throttle_fraction) {
  requested_throttle_ = throttle_fraction;
  sendCachedTorqueCommand();
}

void PassatCore::sendBrakeCommand(double brake_pressure) {
  requested_brake_ = brake_pressure;
  sendCachedTorqueCommand();
}

void PassatCore::sendParkingBrakeCommand(bool brake) {
  requested_ebrake_ = brake;
  sendCachedExtendedCommand();
}

void PassatCore::sendEngineCommand(double throttle_fraction, double brake_pressure) {
  requested_throttle_ = throttle_fraction;
  requested_brake_ = brake_pressure;
  sendCachedTorqueCommand();
}

void PassatCore::sendGearShiftCommand(GearState gear_position) {
  requested_gear_ = gear_position;
  sendCachedTorqueCommand();
}

void PassatCore::publishOutput(double steering_torque, double brake_pressure, double throttle_fraction, GearState gear,
                          int16_t steering_int, uint16_t brake_int, uint16_t throttle_int, uint64_t bytes_written) {

  output_.steering_torque = steering_torque;
  output_.brake_pressure = brake_pressure;
  output_.throttle_fraction = throttle_fraction;
  output_.gear = (uint8_t)gear;
  output_.steering_int = steering_int;
  output_.brake_int = brake_int;
  output_.throttle_int = throttle_int;
  output_.bytes_written = bytes_written;
  output_.timestamp = drc::Time::current();

  passat_output_pub_.publish(output_);
}

void PassatCore::publishExtOutput(uint8_t turn_signal_state, bool engine_kill, bool honk, bool parking_brake, uint64_t bytes_written) {

  ext_output_.turn_signal_state = turn_signal_state;
  ext_output_.engine_kill = engine_kill;
  ext_output_.honk = honk;
  ext_output_.parking_brake = parking_brake;
  ext_output_.bytes_written = bytes_written;
  ext_output_.timestamp = drc::Time::current();

  passat_ext_output_pub_.publish(ext_output_);
}

} // namespace vlr

