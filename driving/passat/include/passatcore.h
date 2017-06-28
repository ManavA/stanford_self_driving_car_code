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


#ifndef PASSAT_CORE_H_
#define PASSAT_CORE_H_

#include <sys/ioctl.h>
#include <netinet/in.h> 
#include <netdb.h> 
#include <sys/socket.h> 
#include <sys/wait.h> 
#include <arpa/inet.h>
#include <ros/ros.h>
#include <passat/PassatStatus.h>
#include <passat/PassatOutput.h>
#include <passat/PassatExtOutput.h>
#include <driving_common/TurnSignal.h>

#define      READ_BUFFER_SIZE    10000
#define      PASSAT_BAUD_RATE    115200

namespace vlr {

class PassatCore {
public:
  typedef enum {GEAR_DRIVE, GEAR_NEUTRAL, GEAR_REVERSE} GearState;

public:
   PassatCore();
   virtual ~PassatCore();
   void readStatus();
   void sendSteeringCommand(double steering_angle);
   void sendExtendedCommand(uint8_t turn_signal, bool engine_kill, bool honk, bool parking_brake);
   void sendAngleControlCommand(double steering_angle, double brake_pressure, double throttle_fraction, GearState gear_position);
   void sendTorqueControlCommand(double steering_torque, double brake_pressure, double throttle_fraction, GearState gear_position);
   void sendThrottleCommand(double throttle_fraction);
   void sendBrakeCommand(double brake_pressure);
   void sendParkingBrakeCommand(bool brake);
   void sendEngineCommand(double throttle_fraction, double brake_pressure);
   void sendGearShiftCommand(GearState gear_position);
   void sendCompleteCommand(double steering_torque, double throttle_command, double brake_command);

 protected:
   template <class T> void getParam(std::string key, T& var);

 private:
   void readParameters();
   void updateControlLimits(double max_steering, double max_torque, double max_throttle, double max_brake);
   void sendCachedTorqueCommand();
   void sendCachedExtendedCommand();
   void publishOutput(double steering_torque, double brake_pressure, double throttle_fraction, GearState gear,
                      int16_t steering_int, uint16_t brake_int, uint16_t throttle_int, uint64_t bytes_written);
   void publishExtOutput(uint8_t turn_signal_state, bool engine_kill, bool honk, bool parking_brake, uint64_t bytes_written);

 protected:
  ros::NodeHandle nh_;
  ros::Publisher passat_status_pub_;
  ros::Publisher passat_output_pub_, passat_ext_output_pub_;
  std::string device_;
  std::string port_;

  int fd_;

  bool first_command_;
  int last_command_steering_;

  bool torque_control_;
  bool belt_steering_;

  uint8_t requested_direction_;
  double requested_steering_, requested_brake_, requested_throttle_;
  double requested_torque_;
  GearState requested_gear_;
  uint8_t requested_signal_;
  bool requested_ebrake_;

  bool steering_commands_allowed_, engine_commands_allowed_;
  bool steering_autonomous_, brake_autonomous_, throttle_autonomous_;
  bool gear_shift_autonomous_;
  double max_steering_, max_torque_, max_brake_, max_throttle_;

  unsigned char read_buffer_[READ_BUFFER_SIZE];
  int message_counter_;
  uint64_t num_bytes_;

  passat::PassatStatus status_;
  passat::PassatOutput output_;
  passat::PassatExtOutput ext_output_;

  FILE* command_log_fp_;
  double start_time_;
};

} // namespace vlr

#endif
