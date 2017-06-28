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


#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fnmatch.h>
#include <global.h>
#include <passat_constants.h>
#include <velodyne.h>

#define FILE_VLF_EXT            ".vlf"
#define FILE_PCAP_EXT           ".pcap"

#define MAX_NAME_LENGTH       256

#define MAX_NUM_SCANS        20000
#define MAXRECVLEN            4096
#define MAX_SENSOR_TIMEOUT    0.5

namespace drc = driving_common;

namespace velodyne {

Velodyne::Velodyne(const std::string logfile_name) :
  nh_("/driving"), port_(2368), calibrate_intensities_(false), received_applanix_pose_(false), applanix_pose_mutex_(PTHREAD_MUTEX_INITIALIZER),
  logfile_name_(logfile_name), last_encoder_(0), bytes_(0), num_scans_(0), num_spins_(0), projected_packet_time_(0) {
  readParameters();
  if (!config_.readCalibration(cal_filename_)) {
    throw VLRException("Could not open calibration file " + cal_filename_ + std::string("."));
  }

  if (!config_.readIntensity(int_filename_)) {
    throw VLRException("Could not open intensity calibration file " + int_filename_ + std::string("."));
  }

  if (!logfile_name_.empty()) {
    if (writer_.Open(logfile_name_.c_str()) < 0) {
      std::cout << "Could not open log file " << logfile_name_ << " for writing.\n";
    }
  }

  config_.integrateOffset(offset_);

  applanix_sub_ = nh_.subscribe("ApplanixPose", 5, &Velodyne::applanixHandler, this);

  projected_packet_pub_ = nh_.advertise<velodyne::Projected> ("velodyne/projected", MAX_NUM_SCANS);
  stat_pub_ = nh_.advertise<velodyne::Stat> ("velodyne/Stat", MAX_NUM_SCANS);
  error_pub_ = nh_.advertise<driving_common::ErrorString> ("ErrorString", 5);
}

Velodyne::~Velodyne() {
}

template<class T> void Velodyne::getParam(std::string key, T& var) {
  if (!nh_.getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + std::string("."));
  }
}

void Velodyne::getParamTransform(std::string key, dgc::dgc_transform_t& tr) {
  ros::Time now = ros::Time::now();
  tf_listener_.waitForTransform("Applanix", key, now, ros::Duration(3.0));

  tf::StampedTransform transform;
  try {
    tf_listener_.lookupTransform("Applanix", key, ros::Time(0), transform);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }
  
  tr[3][0] = 0; tr[3][1] = 0; tr[3][2] = 0; tr[3][3] = 1;
  btMatrix3x3 R = transform.getBasis();
  for(int32_t  r=0; r<3; r++) {
    for(int32_t  c=0; c<3; c++) {tr[r][c] = R[r][c];}
  }
  btVector3 t = transform.getOrigin();
  tr[0][3] = t[0];
  tr[1][3] = t[1];
  tr[2][3] = t[2];
}

//void read_parameters(ParamInterface *pint, int argc, char **argv)
void Velodyne::readParameters() {
  getParam("velodyne/port", port_);
  getParamTransform("Velodyne", offset_);
  getParam("velodyne/cal_file", cal_filename_);
  getParam("velodyne/int_file", int_filename_);
  getParam("velodyne/calibrate_intensities", calibrate_intensities_);

  if (!calibrate_intensities_) int_filename_ = "";
}

void Velodyne::applanixHandler(const applanix::ApplanixPose& applanix_pose) {
  pthread_mutex_lock(&applanix_pose_mutex_);
  received_applanix_pose_ = true;
  applanix_pose_msg_ = applanix_pose;
  pthread_mutex_unlock(&applanix_pose_mutex_);
}

void Velodyne::run() {
  unsigned char data[MAXRECVLEN + 11];
  int len, err;
  struct timeval t;
  fd_set set;

  UDPPacket* p = new UDPPacket;
  data[0] = VLF_START_BYTE;
  uint16_t l = VELO_PACKET_SIZE;
  memcpy(&(data[9]), &l, 2);

  std::cout << "Opened udp socket " << port_ << " to velodyne\n";
  int sock = openSocket(port_);

  std::cout << "Starting UDP main loop... ";

  double last_spin_publish_time = drc::Time::current();
  uint64_t last_spin_num = 0;

  uint64_t last_byte_count = 0;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok()) {
    FD_ZERO(&set);
    FD_SET(sock, &set);
    t.tv_sec = 0;
    t.tv_usec = 100000;
    err = select(sock + 1, &set, NULL, NULL, &t);

    if (err == 1) {
      len = recv(sock, &(data[11]), MAXRECVLEN, MSG_WAITALL);
      if (len < 0) {
        throw VLRException("recvfrom() failed");
      }
      projected_packet_time_ = drc::Time::current();
      p->timestamp = projected_packet_time_;
      if (parsePacket(&(data[11]), len, p)) {
        addPacket(p);
        memcpy(&(data[1]), &(p->timestamp), 8);
        data[len + 11] = computeChecksum(&(data[11]), len);
//        writeRaw(len + 12, data);
        if (!logfile_name_.empty()) {writer_.Write(len + 12, data);}
      }
      bytes_ += len;
    }
    if(num_spins_ > last_spin_num ) {
      double  time = drc::Time::current();
      stat_.turn_rate = (time != last_spin_publish_time ? 1/(time-last_spin_publish_time) : 0);
      stat_.byte_rate = bytes_ - last_byte_count;
      stat_pub_.publish(stat_);
      last_spin_num = num_spins_;
      last_spin_publish_time = time;
      last_byte_count = bytes_;
    }
  }

  spinner.stop();
  close(sock);
  writer_.Close();
  delete p;
}

void Velodyne::addPacket(UDPPacket* pkt) {
  driving_common::Pose cached_pose;

  pthread_mutex_lock(&applanix_pose_mutex_);
  cached_pose.x = applanix_pose_msg_.smooth_x;
  cached_pose.y = applanix_pose_msg_.smooth_y;
  cached_pose.z = applanix_pose_msg_.smooth_z;
  cached_pose.roll = applanix_pose_msg_.roll;
  cached_pose.pitch = applanix_pose_msg_.pitch;
  cached_pose.yaw = applanix_pose_msg_.yaw;
  pthread_mutex_unlock(&applanix_pose_mutex_);

  projected_packet_.header.stamp = ros::Time(pkt->timestamp);
  for (int32_t i = 0; i < velodyne::Packet::NUM_BLOCKS; i++) {
    projected_packet_.block[i].encoder = pkt->scan[i].encoder;
    projected_packet_.block[i].block = pkt->scan[i].block;
    for (int32_t j = 0; j < velodyne::Block::NUM_BEAMS; j++) {
      projected_packet_.block[i].laser[j].distance = pkt->scan[i].range[j];
      projected_packet_.block[i].laser[j].intensity = pkt->scan[i].intensity[j];
    }
    projectMeasurement(pkt->scan[i], block_, cached_pose);
    block_.timestamp = pkt->timestamp;
    block_.encoder = (pkt->scan[i].encoder + 18000) % 36000;
    if (block_.encoder < last_encoder_) {
      num_spins_++;
      num_scans_ = 0;
    }
    projected_packet_pub_.publish(projected_packet_);
    last_encoder_ = block_.encoder;
  }
}

int Velodyne::openSocket(uint16_t port) {
  int sock_rmem_size = 2097152;
  int sock;
  struct sockaddr_in broadcastAddr; /* Broadcast Address */

  /* Create a best-effort datagram socket using UDP */
  if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    throw VLRException("socket() failed");
  }

  /* Request a larger receive buffer window */
  if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &sock_rmem_size, sizeof(sock_rmem_size)) == -1) {
    std::cout << "Could not increase socket receive buffer to " << sock_rmem_size << ". This may cause dropped veloydne data\n";
  }

  /* Zero out structure */
  memset(&broadcastAddr, 0, sizeof(broadcastAddr));

  /* Internet address family */
  broadcastAddr.sin_family = AF_INET;

  /* Any incoming interface */
  broadcastAddr.sin_addr.s_addr = htonl(INADDR_ANY);

  /* Broadcast port */
  broadcastAddr.sin_port = htons(port);

  /* Bind to the broadcast port */
  if (bind(sock, (struct sockaddr *) &broadcastAddr, sizeof(broadcastAddr)) < 0) {
    throw VLRException("bind() failed");
  }

  return sock;
}

int Velodyne::writeRaw(int len, uint8_t* data) {
  return 0; // raw_shm->write(len, data);
}

int Velodyne::parsePacket(uint8_t* pkt, uint16_t len, UDPPacket* p) {
  int i, j, ptr = 0;

  if (len != VELO_PACKET_SIZE) {
    return 0;
  }
  else {
    for (i = 0; i < velodyne::Packet::NUM_BLOCKS; i++) {
      memcpy(&(p->scan[i].block), &(pkt[ptr]), sizeof(uint16_t));
      ptr += sizeof(uint16_t);
      switch (p->scan[i].block) {
        case 0xeeff: // upper block
          p->scan[i].block = 0;
          break;
        case 0xddff: // lower block
          p->scan[i].block = 1;
          break;
        default:
          std::cout << __PRETTY_FUNCTION__ << ": Unknown block id (" << p->scan[i].block << ") in the file.\n";
          return 0;
      }

      memcpy(&(p->scan[i].encoder), &(pkt[ptr]), sizeof(uint16_t));
      ptr += sizeof(uint16_t);

      for (j = 0; j < velodyne::Block::NUM_BEAMS; j++) {
        memcpy(&(p->scan[i].range[j]), &(pkt[ptr]), sizeof(uint16_t));
        ptr += sizeof(uint16_t);
        p->scan[i].intensity[j] = pkt[ptr];
        ptr++;
      }
    }

    memcpy(p->status, &(pkt[ptr]), 6);
    ptr += 6;
  }
  return 1;
}

void Velodyne::openEtherealFile(const std::string& filename, VelodyneFile*& velodyne) {
  uint8_t header[24];
  uint32_t magic;
  uint16_t major, minor;
  uint32_t sigfigs, snaplen, network;
  int zone, n;

  if(velodyne) {delete velodyne;}
  velodyne = new VelodyneFile;

  velodyne->filename = filename;

  velodyne->fp = vlr::cio::fopen(filename.c_str(), "r");
  if (velodyne->fp == NULL) {
    delete velodyne;
    throw VLRException("Could not open file " + filename + std::string(" for reading."));
  }

  n = vlr::cio::fread(header, 24, 1, velodyne->fp);
  if (n != 1) {
    delete velodyne;
    vlr::cio::fclose(velodyne->fp);
    throw VLRException("Could not read header of file " + filename + std::string("."));
  }

  magic = *((uint32_t *) header);
  if (magic != 0xa1b2c3d4) {
    delete velodyne;
    vlr::cio::fclose(velodyne->fp);
    throw VLRException("File " + filename + std::string("is not an ethereal file."));
  }

  major = *((uint16_t*) (header + 4));
  minor = *((uint16_t*) (header + 6));
  zone = *((int *) (header + 8));
  sigfigs = *((uint32_t *) (header + 12));
  snaplen = *((uint32_t *) (header + 16));
  network = *((uint32_t *) (header + 20));

  velodyne->msg_buffer = (uint8_t *) calloc(snaplen + 1, 1);
  dgc::dgc_test_alloc(velodyne->msg_buffer);

  velodyne->sweep_number = 0;
}

void Velodyne::etherealReadPacket(VelodyneFile* velodyne, UDPPacket* pkt) {
  uint8_t header[16];
  uint32_t sec, usec;
  uint32_t n, len1, len2;
  uint8_t * data;

  /* read pcap packet header */
  n = vlr::cio::fread(header, 16, 1, velodyne->fp);
  if (n != 1) {
    throw VLRException("Could not read log message.");
  }
  sec = *((uint32_t*) header);
  usec = *((uint32_t*) (header + 4));
  pkt->timestamp = sec + usec / 1e6;
  len1 = *((uint32_t*) (header + 8));
  len2 = *((uint32_t*) (header + 12));

  /* read pcap packet data */
  n = vlr::cio::fread(velodyne->msg_buffer, len1, 1, velodyne->fp);
  if (n != 1) {
    throw VLRException("Could not read log message.");
  }

  /* skip over the IP and UDP header and checksum*/
  data = velodyne->msg_buffer + 40 + 4;

  parsePacket(data, n - 44, pkt);
}

void Velodyne::openVlfFile(const std::string& filename, VelodyneFile*& velodyne) {

  if(velodyne) {delete velodyne;}
  velodyne = new VelodyneFile;

  velodyne->filename = filename;

  velodyne->fp = vlr::cio::fopen(filename.c_str(), "r");
  if (velodyne->fp == NULL) {
    delete velodyne;
    velodyne=NULL;
    throw VLRException("Could not open file " + filename + std::string(" for reading."));
  }

  velodyne->buffer_len = 2 * VELO_PACKET_SIZE;
  velodyne->msg_buffer = (uint8_t *) malloc(velodyne->buffer_len * sizeof(uint8_t));
  dgc::dgc_test_alloc(velodyne->msg_buffer);

  velodyne->sweep_number = 0;
}

void Velodyne::vlfReadPacket(VelodyneFile* velodyne, UDPPacket* pkt) {
  uint8_t data[16];
  int n;
  uint16_t len;

  n = vlr::cio::fgetc(velodyne->fp);
  if (n != VLF_START_BYTE) {
    throw VLRException("# ERROR: wrong start byte.");
  }

  n = vlr::cio::fread(data, 8, 1, velodyne->fp);
  if (n != 1) {
    throw VLRException("Could not read time stamp.");
  }
  memcpy(&(pkt->timestamp), data, 8);

  n = vlr::cio::fread(data, 2, 1, velodyne->fp);
  if (n != 1) {
    throw VLRException("Could not read packet length.");
  }
  memcpy(&(len), data, 2);

  if (len != VELO_PACKET_SIZE) {
    throw VLRException("# ERROR: packet has wrong size.");
  }

  n = vlr::cio::fread(velodyne->msg_buffer, len + 1, 1, velodyne->fp);
  if (n != 1) {
    throw VLRException("Read error.");
  }

  parsePacket(velodyne->msg_buffer, len, pkt);
}

Velodyne::VelodyneFile* Velodyne::openFile(const std::string& filename) {
  VELODYNE_FILE_TYPE inp_type = UNKNOWN;
  char fname[4*MAX_NAME_LENGTH];
  char *completed_filename = NULL;

  //  dgc_complete_filename(filename, &completed_filename);
  if (!fnmatch("pcap:*", filename.c_str(), 0)) {
    fprintf(stderr, "# INFO: use pcap file type!\n");
    strncpy(fname, &(filename[5]), MAX_NAME_LENGTH);
    inp_type = PCAP;
  }
  else if (!fnmatch("vlf:*", filename.c_str(), 0)) {
    fprintf(stderr, "# INFO: read vlf file type!\n");
    strncpy(fname, &(filename[4]), MAX_NAME_LENGTH);
    inp_type = VLF;
  }
  else if (!fnmatch("*" FILE_PCAP_EXT, filename.c_str(), 0)) {
    fprintf(stderr, "# INFO: read pcap file type!\n");
    strncpy(fname, filename.c_str(), MAX_NAME_LENGTH);
    inp_type = PCAP;
  }
  else if (!fnmatch("*" FILE_VLF_EXT, filename.c_str(), 0)) {
    fprintf(stderr, "# INFO: read vlf file type!\n");
    strncpy(fname, filename.c_str(), MAX_NAME_LENGTH);
    inp_type = VLF;
  }
  else if (dgc::dgc_complete_filename(filename.c_str(), FILE_VLF_EXT, &completed_filename)) {
    strncpy(fname, completed_filename, MAX_NAME_LENGTH);
    free(completed_filename);
    inp_type = VLF;
  }
  else if (dgc::dgc_complete_filename(filename.c_str(), FILE_PCAP_EXT, &completed_filename)) {
    strncpy(fname, completed_filename, MAX_NAME_LENGTH);
    free(completed_filename);
    inp_type = PCAP;
  }

  VelodyneFile* velodyne = NULL;

  switch (inp_type) {
    case PCAP:
      openEtherealFile(fname, velodyne);
      velodyne->format = PCAP;
      return velodyne;

    case VLF:
      openVlfFile(fname, velodyne);
      velodyne->format = VLF;
      return velodyne;

    default:
      fprintf(stderr, "# ERROR: unknown file type!\n");
      return NULL;
  }
}

// bool -> uint8_t!?
bool Velodyne::beamInsideCar(double x, double y) {
  if (fabs(y) < ((DGC_PASSAT_WIDTH / 2) + 1.2) && x > -(DGC_PASSAT_IMU_TO_R_BUMPER + 1.0) && x < DGC_PASSAT_LENGTH - DGC_PASSAT_IMU_TO_R_BUMPER) {
    return true;
  }

  return false;
}

void Velodyne::projectMeasurement(const Config& config, const Measurement& msrm, velodyne::Block& block, const driving_common::Pose& robot) {
  double x, y, z;
  dgc::dgc_transform_t t;
  int j, n;

  float distance, distance1, cosVertAngle, sinVertAngle, cosRotAngle, sinRotAngle, hOffsetCorr, vOffsetCorr;
  float xyDistance; //, distanceCorr, shortOffset, longOffset;

  dgc::dgc_transform_rpy(t, config.offset_, robot.roll, robot.pitch, robot.yaw);

  for (j = 0; j < velodyne::Block::NUM_BEAMS; j++) {
    n = j + velodyne::Block::NUM_BEAMS * msrm.block;
    // use calibrated intensity!
    block.laser[j].intensity = config.intensity_map[config.inv_beam_order[n]][msrm.intensity[j]];
    if ((msrm.range[j] == 0 || msrm.range[j] * VELODYNE_TICKS_TO_METER > 110 || isnan(config.sin_rot_angle[msrm.encoder][n]))) {
      block.point[j].x = 0;
      block.point[j].y = 0;
      block.point[j].z = 0;
      block.laser[j].distance = 0;
    }
    else {
      distance1 = msrm.range[j] * VELODYNE_TICKS_TO_METER;
      distance = config.range_offsetX[n] * distance1 + config.range_offset[n];

      cosVertAngle = config.cos_vert_angle[n];
      sinVertAngle = config.sin_vert_angle[n];
      cosRotAngle = config.cos_rot_angle[msrm.encoder][n];
      sinRotAngle = config.sin_rot_angle[msrm.encoder][n];
      hOffsetCorr = config.h_offset[n];
      vOffsetCorr = config.v_offset[n];

      xyDistance = distance * cosVertAngle;

      x = xyDistance * cosRotAngle - hOffsetCorr * sinRotAngle;
      y = xyDistance * sinRotAngle + hOffsetCorr * cosRotAngle;
      z = (xyDistance / cosVertAngle) * sinVertAngle + vOffsetCorr;
#ifdef CHECK_INSIDE_CAR
      if(beam_inside_car(x, y)) {
        block.point[j].x = 0;
        block.point[j].y = 0;
        block.point[j].z = 0;
        block.point[j].range = 0;
      }
      else {
#endif
      dgc::dgc_transform_point(&x, &y, &z, t);
      block.point[j].x = (short) (x * 100 + .0);
      block.point[j].y = (short) (y * 100 + .0);
      block.point[j].z = (short) (z * 100 + .0);
      block.laser[j].distance = (uint16_t) (xyDistance * 100);
#ifdef CHECK_INSIDE_CAR
    }
#endif
    }
  }
//  block.robot = robot; TODO: pose?
  block.block = msrm.block;
  block.encoder = msrm.encoder;
//  block.timestamp   = velodyne->ts;
}

char Velodyne::computeChecksum(uint8_t* bytes, int32_t numbytes) {
  char c = 0;
  for (int32_t i = 0; i < numbytes; i++) {
    c += bytes[i];
  }

  return c;
}

} // namespace velodyne
