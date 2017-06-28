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


#include <sstream>
#include <global.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <velodyne_interface.h>
//#include <playback_interface.h>
#include <velodyne_playback_server.h>

namespace drc = driving_common;

namespace vlr {

#define       MAX_NUM_SCANS                    20000
#define       VELODYNE_BYTES_PER_SECOND        3100000.0

VelodynePlaybackServer::VelodynePlaybackServer() : nh_("/driving"), last_timestamp_(0) {


//  cv::namedWindow("Map",1);
  playback_speed_ = 1.0;
  packet_pub_ = nh_.advertise<velodyne::Packet> ("/driving/velodyne/packet", 100);
  projected_packet_pub_ = nh_.advertise<velodyne::Projected> ("/driving/velodyne/projected", 100);
}

VelodynePlaybackServer::~VelodynePlaybackServer() {
  disable_playback();
}

void VelodynePlaybackServer::run() {
  ros::spin();
  //  ros::Rate r(20); // 20 Hz
//  while (ros::ok()) {
//    double current_time = vlr::Time::current();
//
//    if (readMessage()) {
//      can_status_pub_.publish(can_status_);
//      last_publish_ = current_time;
//    }
//    else {
//      fprintf(stderr, "X");
//    }
//
//    if (current_time - last_update_ > 1.0) {
//      heartbeat_pub_.publish(heartbeat_);
//      last_update_ = vlr::Time::current();
//      if (current_time - last_publish_ < 1.0) fprintf(stderr, ".");
//    }
//
//    r.sleep();
//  }
}

int vlf_find_start(vlr::cio::FILE* fp, double* timestamp, off64_t* pos) {
  unsigned char data[16];
  unsigned short len;
  int n;

  while (ros::ok()) {
    do {
      *pos = vlr::cio::ftell(fp);
      n = vlr::cio::fgetc(fp);
      if (n == EOF) return -1;
    } while (n != VLF_START_BYTE);

    n = vlr::cio::fread(data, 8, 1, fp);
    if (n != 1) return -1;
    memcpy(timestamp, data, 8);

    n = vlr::cio::fread(data, 2, 1, fp);
    if (n != 1) return -1;
    memcpy(&len, data, 2);

    if (len == VELO_PACKET_SIZE) {
      vlr::cio::fseek(fp, *pos, SEEK_SET);
      return 0;
    }
  }

return 0;
}

void VelodynePlaybackServer::applanixPoseHandler(const applanix::ApplanixPose& pose) {
  set_pose(pose.smooth_x, pose.smooth_y, pose.smooth_z, pose.roll, pose.pitch, pose.yaw, pose.timestamp);
}

//void VelodynePlaybackServer::playbackCommandHandler(PlaybackCommand *command)
//{
//  if(command->cmd == DGC_PLAYBACK_COMMAND_RESET ||
//     command->cmd == DGC_PLAYBACK_COMMAND_SEEK)
//    player_->reset();
//  playback_speed_ = command->speed;
//}


void VelodynePlaybackServer::setup(const std::string& vlf_filename) {

  velodyne_file_ = velodyne_.openFile(vlf_filename);
  if (velodyne_file_ == NULL) {
    throw VLRException("Could not open velodyne file " + vlf_filename);
  }

    // find first valid file offset and timestamp
  if (vlf_find_start(velodyne_file_->fp, &first_packet_ts_, &last_packet_fpos_) == -1) {
    throw VLRException("Could not find valid packet in VLF file.");
  }


  enable_playback();
}

void VelodynePlaybackServer::seek(double t) {
  off64_t new_pos;
  double next_ts;

  t -= first_packet_ts_;

  if (t < 0) new_pos = 0;
  else new_pos = (off64_t) (t * VELODYNE_BYTES_PER_SECOND);

  if (vlr::cio::fseek(velodyne_file_->fp, new_pos, SEEK_SET) != 0) {
    std::stringstream s;
    s << "# ERROR: skipping " << t << " seconds - fseek failed.";
    throw VLRException(s.str());
  }
  vlf_find_start(velodyne_file_->fp, &next_ts, &new_pos);

  set_last_packet_time(next_ts);
}

void data2Img(const velodyne::Projected& packet, double z_thresh, double res, double cx, double cy, cv::Mat& img) {
  memset(img.data, 0, img.cols * img.rows * sizeof(uint8_t));

  //  static_obstacle_map_cx_ = cx;
  //  static_obstacle_map_cy_ = cy;

  for (uint32_t i = 0; i < velodyne::Projected::NUM_BLOCKS; i++) {
    for (uint32_t j = 0; j < velodyne::Block::NUM_BEAMS; j++) {
      int32_t xi = img.cols / 2 - (int32_t)((packet.block[i].point[j].x - cx) / res + .5);
      int32_t yi = img.rows / 2 - (int32_t)((packet.block[i].point[j].y - cy) / res + .5);
      if (xi >= 0 && xi < int32_t(img.cols) && yi >= 0 && yi < int32_t(img.rows)) {
        if(packet.block[i].point[j].z > z_thresh) {img.data[yi * img.cols + xi] = 255;}
      }
    }
  }

}

void VelodynePlaybackServer::readPacket(double t, double max_age) {
  // read a new packet
  velodyne::Velodyne::UDPPacket pkt;
  try {
    velodyne_.readPacket(velodyne_file_, &pkt);
  }
  catch (vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
    set_eof(true);
    return;
  }

  if (std::abs(last_timestamp_ - t) > max_age) {
    last_timestamp_ = t;
    return;
  }

  set_last_packet_time(pkt.timestamp);
  off64_t fpos = vlr::cio::ftell(velodyne_file_->fp);
  add_input_bytes(fpos - last_packet_fpos_);
  last_packet_fpos_ = fpos;

  // project the beams, and publish
  for (int i = 0; i < velodyne::Packet::NUM_BLOCKS; i++) {
    packet_.block[i].encoder   = pkt.scan[i].encoder;
    packet_.block[i].block     = pkt.scan[i].block;
    packet_.block[i].timestamp = pkt.timestamp;
    for (int32_t b = 0; b < velodyne::BlockRaw::NUM_BEAMS; b++) {
      packet_.block[i].laser[b].distance = pkt.scan[i].range[b];
      packet_.block[i].laser[b].intensity = pkt.scan[i].intensity[b];
    }
//    drc::GlobalPose tpose = pose_queue_.pose(t);
    drc::GlobalPose tpose = pose_queue_.pose(pkt.timestamp);
    driving_common::Pose pose;
    pose.x = tpose.x();
    pose.y = tpose.y();
    pose.z = tpose.z();
    pose.yaw = tpose.yaw();
    pose.pitch = tpose.pitch();
    pose.roll = tpose.roll();
    velodyne_.projectMeasurement(velodyne_.config(), pkt.scan[i], projected_packet_.block[i], pose);
    projected_packet_.block[i].timestamp = pkt.timestamp;
    add_output_bytes(sizeof(velodyne::Packet));
    add_frames(1);
  }

  packet_.header.stamp = ros::Time(pkt.timestamp);

  last_timestamp_ = pkt.timestamp;

  packet_pub_.publish(packet_);
  projected_packet_pub_.publish(projected_packet_);
  packet_.spin_count++;
  packet_.header.seq++;

//    static uint32_t width=1000, height=1000;
//    static cv::Mat img(height, width, CV_8UC1);
//    static double z_thresh = 0.2;
//    static double res = 0.1;
//    static uint32_t fnum=0;
//    data2Img(projected_packet_, z_thresh, res, 0, 0, img);
//    std::stringstream filename;
//    filename << "plbmap" << fnum << ".png";
//    cv::imwrite(filename.str(), img);
//    cv::imshow("Map", img);
//    cv::waitKey(50);
//    fnum++;
//    usleep(50000);
}

} // namespace vlr
