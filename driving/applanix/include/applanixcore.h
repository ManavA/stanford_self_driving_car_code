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


#ifndef APPLANIX_CORE_H_
#define APPLANIX_CORE_H_

#include <netdb.h>
#include <errno.h>
#include <ros/ros.h>
#include <global.h>
#include <applanix/ApplanixPose.h>
#include <applanix/ApplanixRMS.h>
#include <applanix/ApplanixDMI.h>
#include <applanix/ApplanixGPS.h>

namespace vlr {

  // constants
const int kInternalBufferSize = 30000;
const double kDisplayRefresh = .25;
const double kWarningRefresh = .25;

const int kParseError      = -1;
const int kParseUnfinished  = 0;

class ApplanixServer {
 public:
  ApplanixServer();
  virtual ~ApplanixServer();
  void run();

 private:
  template <class T> void getParam(std::string key, T& var);
  void readParameters();

  void processInput();

  void connectToApplanix();
  void connectToLoggingPort();
  void disconnectFromApplanix();
  int readData(char *buf, int buf_size);

  void publishPoseMessage();
  void publishRMSMessage();
  void publishDMIMessage();
  void publishGPSMessage();

  void applanixCalculateSmoothedPose(applanix::ApplanixPose& pose);

  int connectToSocket(const std::string& address, int port);
  int validateMessage(char *buffer, int buffer_length, int expected_length, const char* name);
  int validateGenericMessage(char *buffer, int buffer_length);
  int parsePoseMessage(char *buffer, int buffer_length, applanix::ApplanixPose& pose);
  int parseRMSMessage(char *buffer, int buffer_length, applanix::ApplanixRMS& rms);
  int parseDMIMessage(char *buffer, int buffer_length, applanix::ApplanixDMI& dmi);
  int parseGPSMessage(char *buffer, int buffer_length, int *sats);
  int parseTimeMessage(char *buffer, int buffer_length, int *sync_mode);
  int parseGamsMessage(char *buffer, int buffer_length, int *code);

  int findNextIndex(const char *process_buffer, int process_buffer_bytes, int index);

private:
  ros::NodeHandle nh_;
  ros::Publisher applanix_pose_pub_;
  ros::Publisher applanix_rms_pub_;
  ros::Publisher applanix_dmi_pub_;
  ros::Publisher applanix_gps_pub_;

  int sockfd_, log_sockfd_;

  unsigned int global_id_;
  double time_sync_timestamp_;
  int time_sync_mode_;

  /* Parameters from param server. */
  std::string ip_address_;
  int32_t port_;
  int32_t logging_port_;
  bool publish_dmi_;
  double network_panic_timeout_;
  double pose_panic_timeout_;
  double rms_panic_timeout_;
  double gps_panic_timeout_;
  double time_panic_timeout_;
  double gams_panic_timeout_;
  double dmi_panic_timeout_;
  
  /* Buffer for reading network messages. */
  char process_buffer_[kInternalBufferSize];
  
  /* Variables we populate and publish. */
  applanix::ApplanixPose pose_;
  applanix::ApplanixRMS rms_;
  applanix::ApplanixGPS gps_;
  applanix::ApplanixDMI dmi_;
  
  /* Time of last warning message */
  double last_pose_warning_;
  double last_rms_warning_;
  double last_primary_warning_;
  double last_secondary_warning_;
  double last_time_synchronization_warning_;
  double last_gams_warning_;
  double last_dmi_warning_;
};

}

#endif
