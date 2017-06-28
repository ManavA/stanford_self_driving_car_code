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


#ifndef VELODYNE_PLAYBACK_SERVER_H
#define VELODYNE_PLAYBACK_SERVER_H

#include <ros/ros.h>
#include <velodyne.h>
#include <velodyne_interface.h>
#include <velodyne/Packet.h>
#include <applanix/ApplanixPose.h>
//#include <playback_interface.h>
#include <data_player.h>

namespace vlr {

class VelodynePlaybackServer : public DataPlayer {
public:
  VelodynePlaybackServer();
  ~VelodynePlaybackServer();
  void setup(const std::string& filename);
  void run();
  void applanixPoseHandler(const applanix::ApplanixPose& pose);

private:
  void seek(double time);
  void readPacket(double t, double max_age);
//  void playbackCommandHandler(PlaybackCommand *command);

private:
  ros::NodeHandle nh_;
  ros::Publisher packet_pub_, projected_packet_pub_;
  applanix::ApplanixPose applanix_pose_;
//  driving_common::Heartbeat heartbeat_;

  double playback_speed_;

  velodyne::Velodyne velodyne_;
  velodyne::Velodyne::VelodyneFile* velodyne_file_;
  velodyne::Packet packet_;
  velodyne::Projected projected_packet_;

  double first_packet_ts_;
  off64_t last_packet_fpos_;
  int last_encoder_;
  double last_timestamp_;
};

} // namespace vlr

#endif
