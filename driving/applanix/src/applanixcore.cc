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


#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <lltransform.h>

#include <applanixcore.h>

namespace drc = driving_common;

namespace vlr {

ApplanixServer::ApplanixServer() :
  nh_("/driving"), sockfd_(-1), log_sockfd_(-1), global_id_(0), time_sync_timestamp_(0), time_sync_mode_(0), last_pose_warning_(0), last_rms_warning_(0),
      last_primary_warning_(0), last_secondary_warning_(0), last_time_synchronization_warning_(0), last_gams_warning_(0), last_dmi_warning_(0) {

  readParameters();

  applanix_pose_pub_ = nh_.advertise<applanix::ApplanixPose> ("ApplanixPose", 200);
  applanix_rms_pub_ = nh_.advertise<applanix::ApplanixRMS> ("ApplanixRMS", 5);
  applanix_dmi_pub_ = nh_.advertise<applanix::ApplanixDMI> ("ApplanixDMI", 5);
  applanix_gps_pub_ = nh_.advertise<applanix::ApplanixGPS> ("ApplanixGPS", 5);

  connectToApplanix();
}

ApplanixServer::~ApplanixServer() {
  disconnectFromApplanix();
}

template<class T> void ApplanixServer::getParam(std::string key, T& var) {
  if (!nh_.getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + ".");
  }
}

void ApplanixServer::readParameters() {
  getParam("applanix/ip_address", ip_address_);
  getParam("applanix/remote_port", port_);
  getParam("applanix/logging_port", logging_port_);
  getParam("applanix/publish_dmi", publish_dmi_);
  getParam("applanix/network_panic_in_seconds", network_panic_timeout_);
  getParam("applanix/pose_panic_in_seconds", pose_panic_timeout_);
  getParam("applanix/rms_panic_in_seconds", rms_panic_timeout_);
  getParam("applanix/dmi_panic_in_seconds", dmi_panic_timeout_);
  getParam("applanix/gps_panic_in_seconds", gps_panic_timeout_);
  getParam("applanix/time_panic_in_seconds", time_panic_timeout_);
  getParam("applanix/gams_panic_in_seconds", gams_panic_timeout_);
}

void ApplanixServer::run() {
  ros::Rate r(200); // 200 Hz

  while (ros::ok()) {
    processInput();
    ros::spinOnce();
//    r.sleep();  // processInput() is blocking
  }
}

void ApplanixServer::disconnectFromApplanix() {
  if (sockfd_ != -1) {
    std::cout << __PRETTY_FUNCTION__ << "Closing connection to Applanix POS LV...\n";
    shutdown(sockfd_, SHUT_RDWR);
    close(sockfd_);
    sockfd_ = -1;
  }
}

int ApplanixServer::connectToSocket(const std::string& address, int port) {
  const unsigned short arbitrary_local_port = 5000;
  struct sockaddr_in local_address, remote_address;
  struct hostent *phostent;
  int ret;
  long opt;
  int sock;

  std::cout << "Opening connection to Applanix POS LV [" << address << ":" << port << "]...";

  /* The arbitrary local port is a "hint".  If the port is unavailable, 
   another port will be selected automatically.  Note that the port
   is NOT privileged. */

  /* Initialize Local Port for Outbound Communication. */
  memset(&local_address, 0, sizeof(local_address));
  local_address.sin_family = AF_INET;
  local_address.sin_addr.s_addr = INADDR_ANY;
  local_address.sin_port = htons(arbitrary_local_port);

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock == -1) {
    throw VLRException("Socket could not be created.");
  }

  bind(sock, (struct sockaddr *) &local_address, sizeof(local_address));

  /* Socket is blocking. */
  opt = 0;
  ret = ioctl(sock, FIONBIO, &opt);
  if (ret == -1) {
    throw VLRException("Socket could not be set to blocking mode.");
  }

  /* Socket will not receive broadcasts. */
  opt = 0;
  ret = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
  if (ret == -1) {
    throw VLRException("Socket could not be set to block broadcasts.");
  }

  /* Initialize Remote Address for Outbound Communication */
  phostent = gethostbyname(address.c_str());
  if (phostent == NULL) {
    throw VLRException("Could not resolve hostname/IP address.");
  }
  memset(&remote_address, 0, sizeof(remote_address));
  remote_address.sin_family = AF_INET;
  remote_address.sin_port = htons(port);
  remote_address.sin_addr = *((struct in_addr *) phostent->h_addr);

  /* Connect to Applanix Hardware */
  ret = connect(sock, (struct sockaddr *) &remote_address, sizeof(remote_address));
  if (ret == -1) {
    throw VLRException("Could not connect to Applanix hardware.");
  }

  std::cout << "connected. Listening for messages...\n";
  return sock;
}

void ApplanixServer::connectToApplanix() {
  disconnectFromApplanix();
  sockfd_ = connectToSocket(ip_address_, port_);
}

void ApplanixServer::connectToLoggingPort() {
  disconnectFromApplanix();

  sockfd_ = connectToSocket(ip_address_, logging_port_);
  if (sockfd_ == -1) exit(1);
  fprintf(stderr, "...SUCCESS!\nListening for messages...\n");
}

int ApplanixServer::readData(char *buf, int buf_size) {
  int bytes_received;

  if (sockfd_ == -1) {
    fprintf(stderr, "\nAPPLANIX: applanix_read() called before "
      "applanix_connect().\n");
    return -1;
  }

  bytes_received = recv(sockfd_, buf, buf_size, 0);

  if (bytes_received == 0) {
    fprintf(stderr, "\nAPPLANIX: Applanix device performed "
      "orderly shutdown.\n");
    return -1;
  }

  if (bytes_received == -1) {
    if (errno == EAGAIN) /* No data right now.  Try again later. */
    return 0;

    fprintf(stderr, "\nAPPLANIX: Error while reading data.\n");
    fprintf(stderr, "APPLANIX: Error message is: %s\n", strerror(errno));
    return -1;
  }
  return bytes_received;
}

void ApplanixServer::publishPoseMessage() {
  static double last_timestamp = -1;

  pose_.timestamp = drc::Time::current();

  pose_.header.frame_id = "0";
//  pose_.header.stamp.fromSec(pose_.timestamp);
  pose_.header.seq += 1;
  pose_.postprocess_code = 0;

  if (last_timestamp != -1 && last_timestamp > pose_.timestamp) {
    std::cout << __PRETTY_FUNCTION__ << "Time going backwards!!  Bug!!  Find me.\n";
  }

  last_timestamp = pose_.timestamp;

  applanix_pose_pub_.publish(pose_);
}

void ApplanixServer::publishRMSMessage() {
  rms_.postprocess_code = 0;
  rms_.timestamp = drc::Time::current();
  applanix_rms_pub_.publish(rms_);
}

void ApplanixServer::publishGPSMessage() {
  gps_.timestamp = drc::Time::current();
  applanix_gps_pub_.publish(gps_);
}

void ApplanixServer::publishDMIMessage() {
  dmi_.timestamp = drc::Time::current();
  applanix_dmi_pub_.publish(dmi_);
}

int ApplanixServer::validateGenericMessage(char *buffer, int buffer_length) {
  unsigned short message_length;
  int i;
  signed short checksum = 0;

  if (buffer_length < 8) return kParseUnfinished;

  message_length = *(unsigned short *) (buffer + 6) + 8;
  if (message_length > buffer_length) return kParseUnfinished;

  if (memcmp(buffer + message_length - 2, "$#", 2) != 0) {
    fprintf(stderr, "\nAPPLANIX: A generic message is malformed (termination). "
      "ID = %u\n", global_id_);
    return kParseError;
  }

  for (i = 0; i < message_length; i += 2)
    checksum += *(signed short *) (buffer + i);
  if (checksum != 0) {
    fprintf(stderr, "\nAPPLANIX: Checksum failed on a generic message. "
      "ID=%u\n", global_id_);
    return kParseError;
  }
  return message_length;
}

int ApplanixServer::validateMessage(char *buffer, int buffer_length, int expected_length, const char* name) {
  unsigned short message_length;
  signed short checksum = 0;
  int i;

  if (buffer_length < 8) return kParseUnfinished;

  message_length = *(unsigned short *) (buffer + 6) + 8;
  if (message_length > buffer_length) return kParseUnfinished;

  if (message_length != 8 + expected_length) {
    fprintf(stderr, "\nAPPLANIX: %s is malformed (length). ID=%u\n", name, global_id_);
    return kParseError;
  }

  if (memcmp(buffer + message_length - 2, "$#", 2) != 0) {
    fprintf(stderr, "\nAPPLANIX: %s is malformed (termination). ID=%u\n", name, global_id_);
    return kParseError;
  }

  for (i = 0; i < message_length; i += 2)
    checksum += *(signed short *) (buffer + i);
  if (checksum != 0) {
    fprintf(stderr, "\nAPPLANIX: Checksum failed on %s. ID=%u\n", name, global_id_);
    return kParseError;
  }
  return message_length;
}

int ApplanixServer::parsePoseMessage(char *buffer, int buffer_length, applanix::ApplanixPose& pose) {
  unsigned short message_length;
  unsigned char time_type;

  message_length = validateMessage(buffer, buffer_length, 132, "navigation solution message");
  if (message_length <= 0) return message_length;

  time_type = *(unsigned char *) (buffer + 32);
  time_type = time_type & ((unsigned char) 15);
  if (time_type != 2) {
    fprintf(stderr, "\nAPPLANIX: Navigation solution time format is not "
      "UTC. Skipping. ID=%u\n", global_id_);
    return kParseError;
  }

  pose.latitude = *(double *) (buffer + 34);
  pose.longitude = *(double *) (buffer + 42);
  pose.altitude = *(double *) (buffer + 50);

  pose.vel_east = *(float *) (buffer + 62);
  pose.vel_north = *(float *) (buffer + 58);
  pose.vel_up = *(float *) (buffer + 66);

  /* check for velocity jumps */
  static double old_ve = 0, old_vn = 0, old_vu = 0;
  if (old_ve != 0) {
    if (fabs(old_ve - pose.vel_east) > 0.1 || fabs(old_vn - pose.vel_north) > 0.1 || fabs(old_vu - pose.vel_up) > 0.1) fprintf(stderr,
        "\nAPPLANIX: Velocity jump: (%6.4f,%6.4f,%6.4f) to"
          " (%6.4f,%6.4f,%6.4f) \n", old_ve, old_vn, old_vu, pose.vel_east, pose.vel_north, pose.vel_up);
  }
  old_ve = pose.vel_east;
  old_vn = pose.vel_north;
  old_vu = pose.vel_up;

  pose.roll = *(double *) (buffer + 70);
  pose.pitch = *(double *) (buffer + 78);
  pose.yaw = *(double *) (buffer + 86);

  pose.wander = *(double *) (buffer + 94);

  pose.track = *(float *) (buffer + 102);
  pose.speed = *(float *) (buffer + 106);

  pose.rate_roll = *(float *) (buffer + 110);
  pose.rate_pitch = *(float *) (buffer + 114);
  pose.rate_yaw = *(float *) (buffer + 118);

  pose.accel_x = *(float *) (buffer + 122);
  pose.accel_y = *(float *) (buffer + 126);
  pose.accel_z = *(float *) (buffer + 130);

  pose.hardware_timestamp = *(double *) (buffer + 8);

  /* Change coordinate system and convert to radians 
   for Stanley compatibility. */
  pose.vel_up *= -1.;

  pose.roll = dgc::dgc_d2r(pose.roll);
  pose.pitch = dgc::dgc_d2r(-1. * pose.pitch);
  pose.yaw = vlr::normalizeAngle(dgc::dgc_d2r(-1. * pose.yaw) + dgc::dgc_d2r(90.));

  pose.wander = dgc::dgc_d2r(pose.wander);

  pose.track = vlr::normalizeAngle(dgc::dgc_d2r(-1. * pose.track) + dgc::dgc_d2r(90.));

  pose.rate_roll = dgc::dgc_d2r(pose.rate_roll);
  pose.rate_pitch = dgc::dgc_d2r(-1. * pose.rate_pitch);
  pose.rate_yaw = dgc::dgc_d2r(-1. * pose.rate_yaw);

  pose.accel_y *= -1.;
  pose.accel_z *= -1.;

  // Toyota applanix fix
#if 1
  /**
   * fix for metric coordinate integrations --MS
   * Applanix reports velocities and headings in terms of geographic North
   * coordinate system. Depending on location of the sensor, this may be
   * more or less not aligned with a UTM system imposed onto the coordinates.
   * This code finds a basis in UTM for the geographic coordinates, and
   * recalculates v_east, v_north, and yaw in terms of that basis.
   *
   * In Ann Arbor, this correction factor is approximately 1.8 degrees, and
   * is extremely noticable when driving loops.
   *
   * */
  double utm_x, utm_y;
  char zone[5];
  vlr::latLongToUtm(pose.latitude, pose.longitude, &utm_x, &utm_y, zone);
  double geographic_delta = 0.0002; /* ~ 15m long basis vectors*/
  double utm_n_x, utm_n_y, utm_e_x, utm_e_y;
  vlr::latLongToUtm(pose.latitude + geographic_delta, pose.longitude, &utm_n_x, &utm_n_y, zone);
  vlr::latLongToUtm(pose.latitude, pose.longitude + geographic_delta, &utm_e_x, &utm_e_y, zone);
  utm_n_y -= utm_y;
  utm_n_x -= utm_x;
  utm_e_y -= utm_y;
  utm_e_x -= utm_x;
  double l_n = hypot(utm_n_x, utm_n_y);
  double l_e = hypot(utm_e_x, utm_e_y);
  utm_n_y /= l_n;
  utm_n_x /= l_n;
  utm_e_y /= l_e;
  utm_e_x /= l_e;

  pose.vel_east = pose.vel_north * utm_n_x + pose.vel_east * utm_e_x;
  pose.vel_north = pose.vel_north * utm_n_y + pose.vel_east * utm_e_y;

  double yaw_delta = -atan2(utm_n_x, utm_n_y);
  pose.yaw += yaw_delta;
#endif

  applanixCalculateSmoothedPose(pose);
  return message_length;
}

int ApplanixServer::parseRMSMessage(char *buffer, int buffer_length, applanix::ApplanixRMS& rms) {
  unsigned short message_length;
  unsigned char time_type;

  message_length = validateMessage(buffer, buffer_length, 80, "RMS message");
  if (message_length <= 0) return message_length;

  time_type = *(unsigned char *) (buffer + 32);
  time_type = time_type & ((unsigned char) 15);
  if (time_type != 2) {
    fprintf(stderr, "\nAPPLANIX: RMS message time format is not UTC. "
      "Skipping. ID=%u\n", global_id_);
    return kParseError;
  }

  rms.hardware_timestamp = *(double *) (buffer + 8);

  rms.rms_north = *(float *) (buffer + 34);
  rms.rms_east = *(float *) (buffer + 38);
  rms.rms_up = *(float *) (buffer + 42);

  rms.rms_v_north = *(float *) (buffer + 46);
  rms.rms_v_east = *(float *) (buffer + 50);
  rms.rms_v_up = *(float *) (buffer + 54);

  rms.rms_roll = *(float *) (buffer + 58);
  rms.rms_pitch = *(float *) (buffer + 62);
  rms.rms_yaw = *(float *) (buffer + 66);

  rms.semi_major = *(float *) (buffer + 70);
  rms.semi_minor = *(float *) (buffer + 74);
  rms.orientation = *(float *) (buffer + 78);

  /* Change coordinate system and convert to radians
   for Stanley compatibility. */

  rms.rms_roll = dgc::dgc_d2r(rms.rms_roll);
  rms.rms_pitch = dgc::dgc_d2r(rms.rms_pitch);
  rms.rms_yaw = dgc::dgc_d2r(rms.rms_yaw);

  rms.orientation = vlr::normalizeAngle(dgc::dgc_d2r(-1. * rms.orientation) + dgc::dgc_d2r(90.));
  return message_length;
}

int ApplanixServer::parseGPSMessage(char *buffer, int buffer_length, int *sats) {
  unsigned short message_length;
  unsigned char internal_sats;
  signed char solution_status;
  signed short checksum = 0;
  int i;

  if (buffer_length < 36) return kParseUnfinished;

  message_length = *(unsigned short *) (buffer + 6) + 8;
  internal_sats = *(unsigned char *) (buffer + 35);
  if (internal_sats > 12) {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (sats). ID=%u\n", global_id_);
    return kParseError;
  }

  if (message_length != 76 + internal_sats * 20 + 8) {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (length). ID=%u\n", global_id_);
    return kParseError;
  }

  if (message_length > buffer_length) return kParseUnfinished;

  if (memcmp(buffer + message_length - 2, "$#", 2) != 0) {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (termination)."
      " ID=%u\n", global_id_);
    return kParseError;
  }

  solution_status = *(signed char *) (buffer + 34);
  if (solution_status < -1 || solution_status > 8) {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (solution). ID=%u\n", global_id_);
    return kParseError;
  }

  for (i = 0; i < message_length; i += 2)
    checksum += *(signed short *) (buffer + i);
  if (checksum != 0) {
    fprintf(stderr, "\nAPPLANIX: Checksum failed on GPS message. ID=%u\n", global_id_);
    return kParseError;
  }
  *sats = internal_sats;
  return message_length;
}

int ApplanixServer::parseTimeMessage(char *buffer, int buffer_length, int *sync_mode) {
  short message_length;
  unsigned char internal_sync_status;

  message_length = validateMessage(buffer, buffer_length, 36, "time synchronization message");
  if (message_length < 0) return message_length;

  internal_sync_status = *(unsigned char *) (buffer + 38);
  if (internal_sync_status > 3) {
    fprintf(stderr, "\nAPPLANIX: Time synchronization message is "
      "malformed (sync status). ID=%u\n", global_id_);
    return kParseError;
  }

  if (internal_sync_status == 0) *sync_mode = 0;
  else if (internal_sync_status == 1 || internal_sync_status == 3) *sync_mode = 1;
  else if (internal_sync_status == 2) *sync_mode = 2;
  else {
    fprintf(stderr, "\nAPPLANIX: Internal consistency error in parsing "
      "time sync message. BUG! Find me. ID=%u\n", global_id_);
    return kParseError;
  }
  return message_length;
}

int ApplanixServer::parseGamsMessage(char *buffer, int buffer_length, int *code) {
  unsigned short message_length;
  signed char solution_status;

  message_length = validateMessage(buffer, buffer_length, 72, "GAMS message");
  if (message_length <= 0) return message_length;

  solution_status = *(signed char *) (buffer + 43);
  if (solution_status < 0 || solution_status > 7) {
    fprintf(stderr, "\nAPPLANIX: GAMS message is malformed (solution code)."
      " ID=%u\n", global_id_);
    return kParseError;
  }
  *code = 7 - solution_status;
  return message_length;
}

int ApplanixServer::parseDMIMessage(char *buffer, int buffer_length, applanix::ApplanixDMI& dmi) {
  unsigned short message_length;
  unsigned char is_valid, time_type;

  message_length = validateMessage(buffer, buffer_length, 52, "DMI message");
  if (message_length <= 0) return message_length;

  time_type = *(unsigned char *) (buffer + 32);
  time_type = time_type & ((unsigned char) 15);
  if (time_type != 2) {
    fprintf(stderr, "\nAPPLANIX: DMI message time format is not UTC. "
      "Skipping. ID=%u\n", global_id_);
    return kParseError;
  }

  is_valid = *(unsigned char *) (buffer + 52);
  if (is_valid != 1) {
    fprintf(stderr, "\nAPPLANIX: DMI message indicates data is invalid. "
      "Skipping. ID=%u\n", global_id_);
    return kParseError;
  }

  dmi.hardware_timestamp = *(double *) (buffer + 8);
  dmi.signed_odometer = *(double *) (buffer + 34);
  dmi.unsigned_odometer = *(double *) (buffer + 42);
  return message_length;
}

int ApplanixServer::findNextIndex(const char *process_buffer, int process_buffer_bytes, int index) {
  char *needle_grp, *needle_msg;

  needle_grp = (char *) memmem(process_buffer + index, process_buffer_bytes - index, "$GRP", 4);
  needle_msg = (char *) memmem(process_buffer + index, process_buffer_bytes - index, "$MSG", 4);

  if (needle_grp == NULL && needle_msg == NULL) return -1;

  int new_index;

  if (needle_grp == NULL) new_index = needle_msg - process_buffer;
  else if (needle_msg == NULL) new_index = needle_grp - process_buffer;
  else if (needle_grp < needle_msg) new_index = needle_grp - process_buffer;
  else new_index = needle_msg - process_buffer;

  if (new_index < 0) {
    fprintf(stderr, "\nAPPLANIX: Got negative index value.  BUG!  Find me."
      " ID=%u\n", global_id_);
    new_index = process_buffer_bytes;
  }
  return new_index;
}

inline void printDelayWarning(double timestamp, double *last_warning, double timeout, const char *name) {
  double current_time = drc::Time::current();
  double delay = current_time - timestamp;

  if (timestamp != 0 && delay > timeout && current_time - *last_warning >= kWarningRefresh) {
    fprintf(stderr, "\nAPPLANIX: Warning: Have not received %s in %f "
      "seconds.\n", name, delay);
    *last_warning = current_time;
  }
}

void ApplanixServer::processInput() {
  double current_time, start_time, delay;
  int index = 0, bytes_read = 0;

  static int unfinished_parse_size, unfinished_parse = 0;
  static double last_printout = -1;

  if (unfinished_parse) {
    memmove(process_buffer_, process_buffer_ + index, bytes_read - index);
    unfinished_parse_size = bytes_read - index;
    unfinished_parse = 0;
  }
  else {
    unfinished_parse_size = 0;
  }

  /* Add data to the buffer.  This blocks until data is available. */
  start_time = drc::Time::current();
  bytes_read = readData(process_buffer_ + unfinished_parse_size, kInternalBufferSize - unfinished_parse_size);
  bytes_read += unfinished_parse_size;

  if ((delay = drc::Time::current() - start_time) > network_panic_timeout_) {
    std::cout << __PRETTY_FUNCTION__ << "Warning: Network read blocked for " << delay << "seconds.\n";
  }
    
  if (bytes_read == -1) {
    connectToApplanix();
    return;
  }

  if (bytes_read <= 0) {
    std::cout << __PRETTY_FUNCTION__ << "Warning: Got unknown error from network code.  BUG!  Find me.\n";
    return;
  }

  for (index = 0; index < bytes_read;) {
    int bytes_parsed;

    if (index + 4 > bytes_read) {
      unfinished_parse = 1;
      break;
    }

    global_id_++;

    char *start;
    start = process_buffer_ + index;

    if (memcmp(start, "$MSG", 4) == 0) {
      bytes_parsed = validateGenericMessage(start, bytes_read - index);
      if (bytes_parsed > 0) {
        index += bytes_parsed;
        continue;
      }
      else if (bytes_parsed == kParseError) {
        index = findNextIndex(process_buffer_, bytes_read, index + 1);
        if (index < 0) break;
        continue;
      }
      else if (bytes_parsed == kParseUnfinished) {
        unfinished_parse = 1;
        break;
      }
    }

    if (memcmp(start, "$GRP", 4) != 0) {
      std::cout << __PRETTY_FUNCTION__ << "Warning: Applanix message is malformed (no header): id=" << global_id_ << "index=" << index << "\n";
      std::cout << "Got " << start[0] << start[1] << start[2] << start[3] << "instead of $GRP.\n";
      index = findNextIndex(process_buffer_, bytes_read, index + 1);
      if (index < 0) {break;}
      continue;
    }

    if (index + 6 > bytes_read) {
      unfinished_parse = 1;
      break;
    }

    unsigned short GroupID;
    GroupID = *(unsigned short *) &start[4];

    switch (GroupID) {
      case 1:
        bytes_parsed = parsePoseMessage(start, bytes_read - index, pose_);
        if (bytes_parsed > 0) {
          pose_.id = global_id_;
          pose_.hardware_time_mode = time_sync_mode_;
          publishPoseMessage();
          index += bytes_parsed;
        }
        break;
      case 2:
        bytes_parsed = parseRMSMessage(start, bytes_read - index, rms_);
        if (bytes_parsed > 0) {
          rms_.id = global_id_;
          rms_.hardware_time_mode = time_sync_mode_;
          publishRMSMessage();
          index += bytes_parsed;
        }
        break;
      case 3:
        bytes_parsed = parseGPSMessage(start, bytes_read - index, &gps_.primary_sats);
        if (bytes_parsed > 0) {
          gps_.primary_id = global_id_;
          gps_.primary_timestamp = drc::Time::current();
          publishGPSMessage();
          index += bytes_parsed;
        }
        break;
      case 11:
        bytes_parsed = parseGPSMessage(start, bytes_read - index, &gps_.secondary_sats);
        if (bytes_parsed > 0) {
          gps_.secondary_id = global_id_;
          gps_.secondary_timestamp = drc::Time::current();
          publishGPSMessage();
          index += bytes_parsed;
        }
        break;
      case 7:
        bytes_parsed = parseTimeMessage(start, bytes_read - index, &time_sync_mode_);
        if (bytes_parsed > 0) {
          time_sync_timestamp_ = drc::Time::current();
          index += bytes_parsed;
        }
        break;
      case 9:
        bytes_parsed = parseGamsMessage(start, bytes_read - index, &gps_.gams_solution_code);
        if (bytes_parsed > 0) {
          gps_.gams_id = global_id_;
          gps_.gams_timestamp = drc::Time::current();
          publishGPSMessage();
          index += bytes_parsed;
        }
        break;
      case 15:
        bytes_parsed = parseDMIMessage(start, bytes_read - index, dmi_);
        if (bytes_parsed > 0) {
          dmi_.id = global_id_;
          dmi_.hardware_time_mode = time_sync_mode_;
          if (publish_dmi_) {publishDMIMessage();}
          index += bytes_parsed;
        }
        break;
      default:
        std::cout << __PRETTY_FUNCTION__ << "Warning: Hardware is publishing unused group id (" << GroupID << ").\n";
        bytes_parsed = validateGenericMessage(start, bytes_read - index);
        if (bytes_parsed > 0) index += bytes_parsed;
        break;
    }

    if (bytes_parsed == kParseError) {
      index = findNextIndex(process_buffer_, bytes_read, index + 1);
      if (index < 0) break;
    }
    else if (bytes_parsed == kParseUnfinished) {
      unfinished_parse = 1;
      break;
    }
  }

  printDelayWarning(pose_.timestamp, &last_pose_warning_, pose_panic_timeout_, "pose data");
  printDelayWarning(rms_.timestamp, &last_rms_warning_, rms_panic_timeout_, "RMS data");
  printDelayWarning(gps_.primary_timestamp, &last_primary_warning_, gps_panic_timeout_, "primary GPS data");
  printDelayWarning(gps_.secondary_timestamp, &last_secondary_warning_, gps_panic_timeout_, "secondary GPS data");
  printDelayWarning(time_sync_timestamp_, &last_time_synchronization_warning_, time_panic_timeout_, "time synchronization data");
  printDelayWarning(gps_.gams_timestamp, &last_gams_warning_, gams_panic_timeout_, "GAMS data");
  printDelayWarning(dmi_.timestamp, &last_dmi_warning_, dmi_panic_timeout_, "DMI data");

  current_time = drc::Time::current();
  if (current_time - last_printout >= kDisplayRefresh) {
    if (pose_.timestamp != 0 && rms_.timestamp != 0 && gps_.primary_timestamp != 0 && gps_.secondary_timestamp != 0 && time_sync_timestamp_ != 0
        && gps_.gams_timestamp != 0) {
      printf("\rRMS:[%.2f,%.2f,%.2f | %.2f,%.2f,%.2f] "
        "GPS:[%d,%d] GAMS:[%d] TIME:[%d] %c     ", rms_.rms_north, rms_.rms_east, rms_.rms_up, dgc::dgc_r2d(rms_.rms_roll), dgc::dgc_r2d(rms_.rms_pitch), dgc::dgc_r2d(
          rms_.rms_yaw), gps_.primary_sats, gps_.secondary_sats, gps_.gams_solution_code, time_sync_mode_, dgc::dgc_ascii_rotor());
    }
    else {
      printf("\rWaiting for data %c", dgc::dgc_ascii_rotor());
    }
    last_printout = current_time;
  }
}

void ApplanixServer::applanixCalculateSmoothedPose(applanix::ApplanixPose& pose) {
  static double old_hw_timestamp = 0;

  if(old_hw_timestamp != 0) {
    double dt = pose.hardware_timestamp - old_hw_timestamp;
    if(dt < -600000) {
      dt = 0.005;
      fprintf(stderr, "\nAPPLANIX: GPS week rollover handled \n");
    }
    if(dt <= 0) {
      fprintf(stderr, "\nAPPLANIX: Hardware TS difference = %.6f (<=0) \n", dt);
    }
    else {
      if(dt >= 0.1) {
        fprintf(stderr, "\nAPPLANIX: WARNING: Large hardware TS jump = %.6f (>=0.1) \n", dt);
      }
      pose.smooth_x += pose.vel_east * dt;
      pose.smooth_y += pose.vel_north * dt;
      pose.smooth_z += pose.vel_up * dt;
    }
  }
  else {
    pose.smooth_x = 0;
    pose.smooth_y = 0;
    pose.smooth_z = 0;
  }

  old_hw_timestamp = pose.hardware_timestamp;
}

} // namespace vlr
