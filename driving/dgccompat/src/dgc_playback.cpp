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


#include <stdint.h>
#include <stdarg.h>
#include <ros/ros.h>
#include <global.h>
#include <velodyne_playback_server.h>
#include <playback_server.h>
#include <velodyne_playback_server.h>

using namespace dgc;
using namespace vlr;

void usage(const char* fmt, ...) {
  va_list args;

  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);

  fprintf(stderr, "Usage: playback filename <args>\n");
  fprintf(stderr, "\t-basic        - no interpreted messages.\n");
  fprintf(stderr, "\t-novelo       - don't playback velodyne.\n");
  exit(-1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "playback");

  try {
    char *completed_filename = NULL, *ipc_filename = NULL, *vlf_filename = NULL;

    bool basic = false, use_velodyne = true;
    VelodynePlaybackServer* velo_server = NULL;

    if (argc < 2) usage("Error: Not enough arguments\n\n");

    // try to find IPC, and velodyne filenames
    for (int i = 1; i < argc; i++) {
      printf("%s\n", argv[i]);
    if (dgc_complete_filename(argv[i], ".log.gz", &completed_filename) || dgc_complete_filename(argv[i], ".log", &completed_filename)) {
        if (ipc_filename != NULL) dgc_die("Error: %s only accepts one .log or .log.gz file as input.\n", argv[0]);
        ipc_filename = completed_filename;
      }
      else if (dgc_complete_filename(argv[i], ".vlf", &completed_filename)) {
        if (vlf_filename != NULL) dgc_die("Error: %s only accepts one .vlf file as input.\n", argv[0]);
        vlf_filename = completed_filename;
      }
      else if (strcmp(argv[i], "-basic") == 0) basic = true;
      else if (strcmp(argv[i], "-novelo") == 0) use_velodyne = false;
      else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) usage(NULL);
    }
    if (ipc_filename == NULL) {
      std::cout << "Error: could not find unique .log.gz file in commandline parameters.";
      exit(5);
    }

    if (use_velodyne) {
      /* if we don't have a vlf filename, see if we can find one */
      if (vlf_filename == NULL) {
        vlf_filename = find_matching_logfile(ipc_filename, ".vlf", 30);
        if (vlf_filename == NULL) fprintf(stderr, "Error: could not find vlf file matching %s\n", ipc_filename);
        else fprintf(stderr, "Found matching vlf file %s\n", vlf_filename);
      }

      if (vlf_filename != NULL) {
        velo_server = new VelodynePlaybackServer();
        velo_server->setup(vlf_filename);

        //      err = ipc->Subscribe(PlaybackCommandID, velo_server, &VelodynePlaybackServer::PlaybackCommandHandler);
        //      TestIpcExit(err, "Could not subscribe", PlaybackCommandID);
      }
    }

    CombinedPlaybackServer* playback_server = new CombinedPlaybackServer(basic, use_velodyne, velo_server);

    playback_server->setup(ipc_filename);
    ros::Rate active_rate(200); // 200 Hz
    ros::Rate paused_rate(10); // 10 Hz
    while (ros::ok()) {
      playback_server->processData();
      if (playback_server->paused()) {
        paused_rate.sleep();
      }
      else {
        active_rate.sleep();
      }
    }
    playback_server->shutdown();

    if (velo_server) {
      delete velo_server;
    }
  }
  catch (vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
  }
  return 0;
}
