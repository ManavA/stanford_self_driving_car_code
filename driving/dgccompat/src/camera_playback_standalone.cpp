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
#include <ipc_std_interface.h>
#include <camera_shm_interface.h>
#include <applanix/applanix_interface.h>
#include <playback_interface.h>
#include <param_interface.h>
#include <signal_handler.h>
#include <dgc_curses.h>
#include <camplayer.h>
#include "camera_playback_server.h"

using namespace vlr;

int main(int argc, char **argv)
{
  int err, camera_num = 0;
  char *cam_filename;

  if(argc < 2)
    dgc_fatal_error("Usage: %s <camlog-filename> [camera_num]\n", argv[0]);

  cam_filename = argv[1];
  if(argc >= 3)
    camera_num = atoi(argv[2]);

  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  CameraInterface *camera_interface = new CameraShmInterface;
  if(camera_interface->CreateServer(camera_num) < 0) 
    dgc_die("Error: could not open camera interface.\n");

  CameraPlaybackServer *server = 
    new CameraPlaybackServer(ipc, camera_interface);
  server->Setup(cam_filename);

  err = ipc->Subscribe(PlaybackCommandID, server,
		       &CameraPlaybackServer::PlaybackCommandHandler);
  TestIpcExit(err, "Could not subscribe,", PlaybackCommandID);
  err = ipc->Subscribe(ApplanixPoseID, server,
		       &CameraPlaybackServer::ApplanixPoseHandler);
  TestIpcExit(err, "Could not subscribe,", ApplanixPoseID);

  SignalHandler signal_handler;
  signal_handler.Start();

  while(!signal_handler.ReceivedSignal(SIGINT)) {
    ipc->Sleep(0.1);
  }

  fprintf(stderr, "\n# INFO: quit program with CTRL-C\n");
  delete server;
  delete camera_interface;
  delete ipc;
  return 0;
}
