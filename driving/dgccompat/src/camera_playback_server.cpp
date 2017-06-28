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


#include <roadrunner.h>
#include <ipc_interface.h>
#include <applanix/applanix_interface.h>
#include <camera_interface.h>
#include <playback_interface.h>
#include <param_interface.h>
#include <dgc_curses.h>
#include <camplayer.h>
#include "camera_playback_server.h"

namespace vlr {

CameraPlaybackServer::CameraPlaybackServer(IpcInterface *ipc,
					   CameraInterface *cam_int)
{
  ipc_ = ipc;
  camera_interface_ = cam_int;
  player = new cam_player(camera_interface_);
  playback_rate_ = 1.0;
}

CameraPlaybackServer::~CameraPlaybackServer()
{
  delete player;
}

void CameraPlaybackServer::ApplanixPoseHandler(ApplanixPose *pose)
{
  player->set_pose(pose->smooth_x, pose->smooth_y, pose->smooth_z,
		   pose->roll, pose->pitch, pose->yaw, pose->timestamp);
}

void CameraPlaybackServer::PlaybackCommandHandler(PlaybackCommand *command)
{
  if(command->cmd == DGC_PLAYBACK_COMMAND_RESET ||
     command->cmd == DGC_PLAYBACK_COMMAND_SEEK)
    player->reset();
  playback_rate_ = command->speed;
}

void CameraPlaybackServer::PrintPlaybackStats(int line_num, bool paused)
{
  double disk_rate, shm_rate, freq;

  dgc_curses_black();
  mvprintw(line_num, 0, "CAMERA");

  player->throughput_stats(&disk_rate, &shm_rate, &freq);
  
  dgc_curses_black();
  mvprintw(line_num, 11, "DISK:");
  dgc_curses_blue();
  mvprintw(line_num, 18, "%.1f MB/s", paused ? 0 : disk_rate * playback_rate_);
  
  dgc_curses_black();
  mvprintw(line_num, 28, "SHM:");
  dgc_curses_blue();
  mvprintw(line_num, 34, "%.1f MB/s", paused ? 0 : shm_rate * playback_rate_);
  
  dgc_curses_black();
  mvprintw(line_num, 45, "FREQ:");
  dgc_curses_blue();
  mvprintw(line_num, 51, "%.2f Hz", paused ? 0 : freq * playback_rate_);
}

void *camera_thread(void *ptr)
{
  cam_player *player = (cam_player *)ptr;
  player->play();
  return NULL;
}

void CameraPlaybackServer::Setup(char *filename)
{
  pthread_t thread;

  if(player->initialize(filename) < 0)
    dgc_die("Error: could not initialize camera playback.\n");

  /* start camera thread */
  pthread_create(&thread, NULL, camera_thread, player);
}

}
