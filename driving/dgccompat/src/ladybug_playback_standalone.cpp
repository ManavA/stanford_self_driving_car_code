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
#include <ipc_std_interface.h>
#include <ladybug_shm_interface.h>
#include <applanix/applanix_interface.h>
#include <playback_interface.h>
#include <signal_handler.h>
#include <dgc_curses.h>
#include "ladybug_playback_server.h"

using namespace vlr;

static void PrintStats(LadybugPlaybackServer *server)
{
  clear();
  server->PrintPlaybackStats(0, 1.0);
  move(1, 0);
  refresh();
}

int main(int argc, char **argv)
{
  int err;

  if(argc != 2)
    dgc_die("Usage: %s llf-filename\n", argv[0]);

  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  LadybugInterface *lbug_int = new LadybugShmInterface;
  if (lbug_int->CreateServer() < 0) 
    dgc_die("Error: could not open velodyne interface.\n");

  LadybugPlaybackServer *server = new LadybugPlaybackServer(ipc, lbug_int);
  server->Setup(argv[1]);

  err = ipc->Subscribe(PlaybackCommandID, server,
		       &LadybugPlaybackServer::PlaybackCommandHandler);
  TestIpcExit(err, "Could not subscribe", PlaybackCommandID);
  err = ipc->Subscribe(ApplanixPoseID, server,
		       &LadybugPlaybackServer::ApplanixPoseHandler);
  TestIpcExit(err, "Could not subscribe", ApplanixPoseID);

  dgc_curses_initialize();
  ipc->AddTimer(0.25, PrintStats, server);

  SignalHandler signal_handler;
  signal_handler.Start();

  while(!signal_handler.ReceivedSignal(SIGINT)) {
    ipc->Sleep(0.1);
  }

  fprintf(stderr, "\n# INFO: quit program with CTRL-C\n");
  dgc_curses_close();
  delete server;
  delete lbug_int;
  delete ipc;
  return 0;
}
