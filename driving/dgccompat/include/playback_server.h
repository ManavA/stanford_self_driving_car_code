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


#ifndef PLAYBACK_SERVER_H_
#define PLAYBACK_SERVER_H_

#include <ros/ros.h>
//#include <velodyne_interface.h>
//#include <playback_interface.h>
#include <applanix/ApplanixPose.h>
#include <applanix_interface.h>
#include <lineBuffer.h>
#include <logio.h>
#include <velodyne_playback_server.h>

namespace vlr {

class CombinedPlaybackServer {
public:
  CombinedPlaybackServer(bool basic, bool use_velodyne, VelodynePlaybackServer* velo_server);
  ~CombinedPlaybackServer();
  void setup(const std::string& ipc_filename);
  void processData();
  void shutdown();
  bool paused() {return paused_;}


private:
//  void playbackCommandHandler(PlaybackCommand *command);
//  void printPlaybackStats();
  int readMessage(int message_num, int publish);
  void waitForTimestamp(double ts);
  void convertApplanixPose(const dgc::ApplanixPose& inpose, applanix::ApplanixPose& outpose);

  ros::NodeHandle nh_;
  ros::Publisher applanix_pub_;

  vlr::cio::FILE* infile_;
  bool valid_index_;
  dgc::LogfileIndex logfile_index_;

  bool basic_, paused_, hit_eof_;
  bool use_velodyne_;
  VelodynePlaybackServer* velo_server_;


  dgc::LogReaderCallbackList callbacks_;

  double playback_starttime_, playback_timestamp_, playback_speed_;
  int current_position_, offset_, advance_frame_;
  driving_common::LineBuffer line_buffer_;
  int last_message_num_;
  double last_print_stats_;
  uint32_t seq_id_;
};

}

#endif
