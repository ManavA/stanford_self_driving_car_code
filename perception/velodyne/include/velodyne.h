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


#ifndef VELODYNE_H_
#define VELODYNE_H_

#include <stdint.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <comp_stdio.h>
#include <transform.h>
#include <driving_common/Pose.h>
#include <driving_common/ErrorString.h>
#include <applanix/ApplanixPose.h>
#include <async_writer.h>
#include <velodyneConfig.h>
#include <velodyne/Packet.h>
#include <velodyne/Projected.h>
#include <velodyne/Stat.h>

namespace velodyne {

typedef enum { UNKNOWN, PCAP, VLF } VELODYNE_FILE_TYPE;

#define VELODYNE_TICKS_TO_METER 0.002

#define VELO_NUM_LASERS          64

#define VELO_PACKET_SIZE         1206

#define VLF_START_BYTE           0x1b
#define VELO_PACKET_FSIZE        1226         /* 1 + 16 + 2 + 1206 + 1 */

/*
   one packet is

   one scan =  velodyne::Block::NUM_BEAMS * 3 + 4 [enc+block+n*3] = 100 bytes
   packet = velodyne::Packet::NUM_BLOCKS * 100 + status (6 bytes) = 1206 bytes

   file:

   startbyte  timestamp length  [... DATA ...] checksum

   => additional 1+16+2+1 = 20 bytes

     startbyte = 0x1b
     timestamp = 2 x <unsigned long>  [tv_sec and tv_usec]
     length    = <uint16_t>
     ...
     checksum  = <uint8_t>
*/

class Velodyne {
  public:
  struct UDPPacket;
  struct Measurement;

  public:
  struct VelodyneFile;

  public:
  Velodyne(const std::string logfile_name="");
  virtual ~Velodyne();

  void run();

  int openSocket(uint16_t port);

  int createServer();
  int writeRaw(int len, uint8_t* data);

  static int parsePacket(uint8_t* pkt, uint16_t len, UDPPacket* p);
  static VelodyneFile* openFile(const std::string& filename);

  inline static void readPacket(VelodyneFile* velodyne, UDPPacket* pkt) {
    if (velodyne->format == PCAP) {
      etherealReadPacket(velodyne, pkt);
    }
    else {
      vlfReadPacket(velodyne, pkt);
    }
  }

  inline void projectMeasurement(const Measurement& msrm, velodyne::Block& block, const driving_common::Pose& robot) {
    projectMeasurement(config_, msrm, block, robot);
  }
  static void projectMeasurement(const Config& config, const Measurement& msrm, velodyne::Block& block, const driving_common::Pose& robot);
  const Config& config() const {return config_;}

 public: // TODO: for now ....
  struct Measurement {
    uint16_t      encoder;
    uint16_t      block;
    uint16_t      range[velodyne::Block::NUM_BEAMS];
    uint8_t       intensity[velodyne::Block::NUM_BEAMS];
  };

  struct UDPPacket {
    double        timestamp;
    Measurement   scan[velodyne::Packet::NUM_BLOCKS];
    uint8_t       status[6];
  };

  struct VelodyneFile {
    VELODYNE_FILE_TYPE    format;
    vlr::cio::FILE*       fp;
    std::string           filename;
    int                   buffer_len;
    uint8_t*               msg_buffer;
    // variables that change infrequently
    int                   sweep_number;
  };

private:
  template <class T> void getParam(std::string key, T& var);
  void getParamTransform(std::string key, dgc::dgc_transform_t& t);
  void readParameters();
  void applanixHandler(const applanix::ApplanixPose& applanix_pose);
  static bool beamInsideCar(double x, double y);
  char computeChecksum(uint8_t* bytes, int32_t numbytes);
  static void openEtherealFile(const std::string& filename, VelodyneFile*& velodyne);
  static void etherealReadPacket(VelodyneFile* velodyne, UDPPacket* pkt);
  static void openVlfFile(const std::string& filename, VelodyneFile*& velodyne);
  static void vlfReadPacket(VelodyneFile* velodyne, UDPPacket* pkt);
  void addPacket(UDPPacket*  pkt);

private:
  Config config_;
  ros::NodeHandle nh_;
  ros::Subscriber applanix_sub_;
  ros::Publisher projected_packet_pub_, stat_pub_, error_pub_;
  tf::TransformListener tf_listener_;
  driving_common::ErrorString err_msg_;
  int32_t port_;
  std::string  cal_filename_;
  std::string  int_filename_;
  bool calibrate_intensities_;
  dgc::dgc_transform_t offset_;

  velodyne::Projected projected_packet_;
  velodyne::Block block_;
  velodyne::Stat stat_;
  bool received_applanix_pose_;
  pthread_mutex_t applanix_pose_mutex_;
  applanix::ApplanixPose applanix_pose_msg_;
  std::string logfile_name_;
  dgc::AsyncWriter writer_;
  uint16_t last_encoder_;

  uint64_t bytes_;
  uint64_t num_scans_;
  uint64_t num_spins_;
  double projected_packet_time_;
};

} // namespace velodyne

#endif
