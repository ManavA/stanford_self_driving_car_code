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
#include <data_player.h>

#define    DATA_MIN_SKIP            1.0
#define    DATA_MAX_AGE             0.2

DataPlayer::DataPlayer() : found_eof_(false), reset_playback_(true), stop_playback_(false), pose_queue_(200),
                           last_packet_time_(0), i_bytes(0), o_bytes(0), frame_count(0),
                           last_stat_check(0), i_speed(0), o_speed(0), hz(0.0), first(true) {

  pthread_mutex_init(&input_mutex, NULL);
  pthread_cond_init(&input_cond, NULL);
}

DataPlayer::~DataPlayer() {
  pthread_mutex_destroy(&input_mutex);
  pthread_cond_destroy(&input_cond);
}

void DataPlayer::reset() {
  pthread_mutex_lock(&input_mutex);
  reset_playback_ = true;
  pthread_cond_signal(&input_cond);
  pthread_mutex_unlock(&input_mutex);
}

void DataPlayer::disable_playback() {
  pthread_mutex_lock(&input_mutex);
  stop_playback_ = true;
  pthread_cond_signal(&input_cond);
  pthread_mutex_unlock(&input_mutex);
  pthread_join(thread, NULL);
}

static void* player_thread(void *ptr) {
  DataPlayer* player = (DataPlayer *) ptr;

  player->play();
  return NULL;
}

void DataPlayer::enable_playback() {
  pthread_create(&thread, NULL, player_thread, this);
}

void DataPlayer::play() {

  stop_playback_ = false;
  reset_playback_ = true;
  while (!stop_playback_) {
      // wait for some input
    pthread_mutex_lock(&input_mutex);
    pthread_cond_wait(&input_cond, &input_mutex);

    // if we get reset signal, go back to first packet
    if (reset_playback_) {
      seek(0);
      pose_queue_.clear();
      found_eof_ = false;
      reset_playback_ = false;
    }

      // get current time
    double current_time;
    try {
      current_time = pose_queue_.latestPoseTimestamp();
    }
    catch(vlr::Ex<>& e) {
      std::cout << e.what() << std::endl;
      pthread_mutex_unlock(&input_mutex);
      continue;
    }

    pthread_mutex_unlock(&input_mutex);


      // skip to right place
    if (std::abs(current_time - last_packet_time_) > DATA_MIN_SKIP && !found_eof_) {
      seek(std::max(current_time - 0.001, 0.0));
    }

   while (last_packet_time_ < current_time && !found_eof_) {
     try {
       readPacket(current_time, DATA_MAX_AGE);
     }
     catch(vlr::Ex<>& e) {
       std::cout << e.what() << std::endl;
     }
   }
  }
}

void DataPlayer::set_eof(bool eof) {
  found_eof_ = eof;
}

void DataPlayer::set_last_packet_time(double t) {
  last_packet_time_ = t;
}

void DataPlayer::set_pose(double x, double y, double z, double roll, double pitch, double yaw, double timestamp) {
  pthread_mutex_lock(&input_mutex);
  if (!found_eof_) {
    pose_queue_.push(driving_common::GlobalPose(x, y, z, yaw, pitch, roll), timestamp);
  }
  pthread_cond_signal(&input_cond);
  pthread_mutex_unlock(&input_mutex);
}

void DataPlayer::throughput_stats(double *disk_rate, double *shm_rate, double *freq) {
  double t, dt;

  t = last_packet_time_;
  if (first) {
    i_bytes = 0;
    o_bytes = 0;
    frame_count = 0;
    last_stat_check = t;
    first = 0;
    dt = 0;
  }
  else dt = t - last_stat_check;

  if (dt > 1.0) {
    i_speed = i_bytes / dt / (1024.0 * 1024.0);
    o_speed = o_bytes / dt / (1024.0 * 1024.0);
    hz = frame_count / dt;
    i_bytes = 0;
    o_bytes = 0;
    frame_count = 0;
    last_stat_check = t;
  }

  *disk_rate = i_speed;
  *shm_rate = o_speed;
  *freq = hz;
}

void DataPlayer::add_input_bytes(int input_bytes) {
  i_bytes += input_bytes;
}

void DataPlayer::add_output_bytes(int output_bytes) {
  o_bytes += output_bytes;
}

void DataPlayer::add_frames(int frames) {
  frame_count += frames;
}
