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


#ifndef DATA_PLAYER_H_
#define DATA_PLAYER_H_

#include <PoseQueue.h>
#include <driving_common/Pose.h>

class DataPlayer {
public:
  DataPlayer();
  virtual ~DataPlayer();

  void play();
  void reset();

  void enable_playback();
  void disable_playback();

  void set_pose(double x, double y, double z, double roll, double pitch, double yaw, double timestamp);

  void throughput_stats(double *disk_rate, double *shm_rate, double *freq);

protected:
  void set_eof(bool eof);
  void set_last_packet_time(double t);
  void add_input_bytes(int input_bytes);
  void add_output_bytes(int output_bytes);
  void add_frames(int frames);
  virtual void seek(double t) = 0;
  virtual void readPacket(double t, double max_age) = 0;

protected:
  pthread_t thread;
  pthread_mutex_t input_mutex;
  pthread_cond_t input_cond;

  bool found_eof_, reset_playback_, stop_playback_;
  driving_common::PoseQueue pose_queue_;
  double last_packet_time_;

  int i_bytes;
  int o_bytes;
  int frame_count;

  double last_stat_check;
  double i_speed;
  double o_speed;
  double hz;
  int first;
};

#endif
