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
//#include <heartbeat_interface.h>
//#include <can_interface.h>
#include <applanix_interface.h>
//#include <controller_interface.h>
//#include <planner_interface.h>
//#include <estop_interface.h>
//#include <localize_interface.h>
//#include <perception_interface.h>
//#include <radar_interface.h>
//#include <passat_interface.h>
//#include <error_interface.h>
//#include <playback_interface.h>
//#include <simulator_interface.h>
//#include <healthmon_interface.h>
//#include <timesync_interface.h>
//#include <velodyne_shm_interface.h>
//#include <ladybug_shm_interface.h>
//#include <signal_handler.h>
//#include <dgc_curses.h>
#include <logio.h>
#include <velodyne_playback_server.h>
#include <playback_server.h>

using namespace dgc;

namespace drc = driving_common;

namespace vlr {

CombinedPlaybackServer::CombinedPlaybackServer(bool basic, bool use_velodyne, VelodynePlaybackServer* velo_server) :
    nh_("/driving"), infile_(NULL), valid_index_(false), basic_(basic), paused_(false), hit_eof_(false),
    use_velodyne_(use_velodyne), velo_server_(velo_server),
    playback_starttime_(0), playback_timestamp_(0), playback_speed_(1.0), current_position_(0), offset_(0),
    advance_frame_(0), last_message_num_(-1), last_print_stats_(0), seq_id_(0) {

  applanix_pub_ = nh_.advertise<applanix::ApplanixPose> ("ApplanixPose", 200);
}

//void CombinedPlaybackServer::PlaybackCommandHandler(PlaybackCommand *command) {
//  switch (command->cmd) {
//    case DGC_PLAYBACK_COMMAND_PLAY:
//      if (paused_ && !hit_eof_) {
//        playback_starttime_ = 0.0;
//        paused_ = false;
//      }
//      break;
//    case DGC_PLAYBACK_COMMAND_STOP:
//      if (!paused_) {
//        paused_ = true;
//      }
//      break;
//    case DGC_PLAYBACK_COMMAND_RESET:
//      if (!paused_) paused_ = true;
//      current_position_ = 0;
//      playback_starttime_ = 0.0;
//      playback_timestamp_ = 0;
//      line_buffer_.reset();
//      hit_eof_ = false;
//      break;
//    case DGC_PLAYBACK_COMMAND_FORWARD:
//      if (!hit_eof_) {
//        line_buffer_.reset();
//        offset_ = command->arg;
//        if (offset_ > 0 && paused_) advance_frame_ = 1;
//      }
//      break;
//    case DGC_PLAYBACK_COMMAND_REWIND:
//      if (!hit_eof_) {
//        line_buffer_.Reset();
//        offset_ = -1 * command->arg;
//        if (offset_ < 0 && paused_) advance_frame_ = 1;
//      }
//      break;
//    case DGC_PLAYBACK_COMMAND_SET_SPEED:
//      break;
//    case DGC_PLAYBACK_COMMAND_SEEK:
//      line_buffer_.Reset();
//      if (!paused_) paused_ = true;
//      advance_frame_ = 1;
//      current_position_ = command->arg;
//      hit_eof_ = false;
//      break;
//  }
//  if (command->speed > 0 && fabs(command->speed - playback_speed_) > 0.001) {
//    playback_starttime_ = 0.0;
//    playback_speed_ = command->speed;
//  }
//}

void CombinedPlaybackServer::setup(const std::string& ipc_filename) {
  infile_ = vlr::cio::fopen(ipc_filename.c_str(), "r");
  if (infile_ == NULL) {
    throw VLRException("Could not open file " + ipc_filename + " for reading.");
  }

  int err = logfile_index_.Load(infile_->filename);
  valid_index_ = (err == 0);
  fprintf(stderr, "valid index = %d\n", valid_index_);

  seq_id_ = 0;

  /* register the message callbacks */
  //  HeartbeatAddLogReaderCallbacks(&callbacks_);
  //  CanAddLogReaderCallbacks(&callbacks_);
  dgc::ApplanixAddLogReaderCallbacks(&callbacks_);
  //  ControllerAddLogReaderCallbacks(&callbacks_);
  //  PlannerAddLogReaderCallbacks(&callbacks_);
  //  EstopAddLogReaderCallbacks(&callbacks_);
  //  PassatAddLogReaderCallbacks(&callbacks_);
  //  LocalizeAddLogReaderCallbacks(&callbacks_);
  //  RadarAddLogReaderCallbacks(&callbacks_);
  //  RadarLRR3AddLogReaderCallbacks(&callbacks_);
  //  ErrorAddLogReaderCallbacks(&callbacks_);
  //  SimulatorAddLogReaderCallbacks(&callbacks_);
  //  HealthmonAddLogReaderCallbacks(&callbacks_);
  //  TimesyncAddLogReaderCallbacks(&callbacks_);
  //  vlr::TrajectoryPointsAddLogReaderCallbacks(&callbacks_);

//  callbacks_.DefineIpcMessages(ipc_);

//  ipc_->Subscribe(PlaybackCommandID, this, &CombinedPlaybackServer::PlaybackCommandHandler, DGC_SUBSCRIBE_ALL);
}

void CombinedPlaybackServer::waitForTimestamp(double ts) {
  double current_time, towait;
  struct timeval tv;

  if (playback_starttime_ == 0.0) {playback_starttime_ = (drc::Time::current() - ts / playback_speed_);}
  current_time = (drc::Time::current() - playback_starttime_) * playback_speed_;
  if (!paused_ && ts > current_time) {
    towait = (ts - current_time) / playback_speed_;
    tv.tv_sec = (int) floor(towait);
    tv.tv_usec = (int) ((towait - tv.tv_sec) * 1e6);
    select(0, NULL, NULL, NULL, &tv);
  }
}

void CombinedPlaybackServer::convertApplanixPose(const dgc::ApplanixPose& inpose, applanix::ApplanixPose& outpose) {
  outpose.timestamp = inpose.timestamp;
  outpose.smooth_x = inpose.smooth_x;
  outpose.smooth_y = inpose.smooth_y;
  outpose.smooth_z = inpose.smooth_z;
  outpose.latitude = inpose.latitude;
  outpose.longitude = inpose.longitude;
  outpose.altitude = inpose.altitude;
  outpose.vel_north = inpose.v_north;
  outpose.vel_east = inpose.v_east;
  outpose.vel_up = inpose.v_up;
  outpose.speed = inpose.speed;
  outpose.track = inpose.track;
  outpose.roll = inpose.roll;
  outpose.pitch = inpose.pitch;
  outpose.yaw = inpose.yaw;
  outpose.rate_roll = inpose.ar_roll;
  outpose.rate_pitch = inpose.ar_pitch;
  outpose.rate_yaw = inpose.ar_yaw;
  outpose.accel_x = inpose.a_x;
  outpose.accel_y = inpose.a_y;
  outpose.accel_z = inpose.a_z;
  outpose.wander = inpose.wander;
  outpose.id = inpose.ID;
  outpose.postprocess_code = inpose.postprocess_code;
  outpose.hardware_timestamp = inpose.timestamp;
  outpose.hardware_time_mode = 0; // TODO: ?!?
}

int CombinedPlaybackServer::readMessage(int message_num, int publish) {
  char *line = NULL;
  char *mark = NULL, command[100];
  double current_time;
  bool found_pose = false;
  LogReaderCallback* cb;

  if (message_num <= last_message_num_) {
    vlr::cio::fseek(infile_, 0, SEEK_SET);
    last_message_num_ = -1;
  }

  if (message_num != last_message_num_ + 1 && valid_index_) {
    vlr::cio::fseek(infile_, logfile_index_.Offset(message_num), SEEK_SET);
    last_message_num_ = message_num - 1;
  }

  while (message_num > last_message_num_) {
    /* read the next line */
    line = line_buffer_.readLine(infile_);
    if (line == NULL) {return -1;}
    mark = dgc::dgc_next_word(line);
    last_message_num_++;
  }

  /* copy the command over */
  int j = 0;
  while (line[j] != ' ') {
    command[j] = line[j];
    j++;
  }
  command[j] = '\0';

//  printf("command: %s\n", command);
  cb = callbacks_.FindCallback(command);
  if (cb == NULL) return false;

  if (!basic_ || !cb->interpreted) {
    mark = cb->conv_func(mark, cb->message_data);
    playback_timestamp_ = atof(mark);

    /* if we read an applanix message, use it to sync the velodyne
     playback */
    if (strcmp(cb->logger_message_name, "APPLANIX_POSE_V2") == 0) {
      applanix::ApplanixPose pose;
      found_pose = true;
      convertApplanixPose(*(dgc::ApplanixPose*)cb->message_data, pose);
      if (velo_server_) {
        velo_server_->applanixPoseHandler(pose);
      }
    }

    if (publish) {
      current_time = drc::Time::current();
      waitForTimestamp( playback_timestamp_);
      if (strcmp(cb->logger_message_name, "APPLANIX_POSE_V2") == 0) {
        applanix::ApplanixPose pose;
        convertApplanixPose(*(dgc::ApplanixPose*)cb->message_data, pose);
        applanix_pub_.publish(pose);
      }
    }
    return found_pose;
  }
  return found_pose;
}

void CombinedPlaybackServer::processData() {
  double current_time;
  int err;

  current_time = drc::Time::current();
  if (current_time - last_print_stats_ > 0.25) {
//    printPlaybackStats();
    last_print_stats_ = current_time;
  }

  if (offset_ != 0) {
    playback_starttime_ = 0.0;
    current_position_ += offset_;
    if (current_position_ < 0) current_position_ = 0;
    offset_ = 0;
  }

  if (!hit_eof_) {
    if (!paused_) {
      err = readMessage(current_position_, 1);
      if (err < 0) {
        paused_ = 1;
        hit_eof_ = 1;
      }
      else current_position_++;
    }
    else if (paused_ && advance_frame_) {
      do {
        err = readMessage(current_position_, 1);
        if (err < 0) {
          paused_ = 1;
          hit_eof_ = 1;
        }
        else current_position_++;
      } while (err == 0);
      advance_frame_ = 0;
    }
  }
}

void CombinedPlaybackServer::shutdown() {
}

}
