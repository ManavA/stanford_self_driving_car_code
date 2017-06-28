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


#include <velodyne_shm_interface.h>
#include <async_writer.h>

namespace dgc {

#define MAX_DATA_LENGTH   (5*1024*1024)

bool quit_flag = false;

void shutdownModule(int sig)
{
  if(sig == SIGINT)
    quit_flag = true;
}

} // namespace dgc

using namespace dgc;

int main(int argc, char **argv)
{
  VelodyneInterface *velo_interface = NULL;
  unsigned char data[MAX_DATA_LENGTH];
  double start_time, current_time;
  int duration, bytes = 0, n;
  dgc::AsyncWriter writer;
  double last_print = 0;
  char *fname;

  if (argc != 2) 
    dgc_die("usage: %s <VELO-LOGNAME>\n\n", argv[0]);

  velo_interface = new VelodyneShmInterface;
  if (velo_interface->CreateClient() < 0)
    dgc_fatal_error("Could not connect to velodyne interface.\n");

  signal(SIGINT, shutdownModule);

  // FIXME (mmde): 64MB write buffer size (1000 * 64K) copied from
  // previous version of velodyne_logger. I think this could be reduced.
  fname = dgc_timestamped_filename(argv[1], ".vlf");
  if (writer.Open(fname, 1000) < 0)
    dgc_fatal_error("Could not open file %s for writing.", fname);
  free(fname);
  
  start_time = dgc_get_time();
  while (!quit_flag) {
    while (velo_interface->RawDataWaiting()) {
      n = velo_interface->ReadRaw(data);
      if (writer.Write(n, data) < 0)
	dgc_warning("Asyncronous writing error.");
      bytes += n;
    }

    current_time = dgc_get_time();
    if (current_time - last_print > 0.5) {
      last_print = current_time;
      duration = (int)floor(current_time - start_time);
      fprintf(stderr, 
	      "\r# INFO: record time: %02d:%02d:%02d (%5.2f MB)    ", 
	      (duration / 3600), (duration / 60) % 60,
	      duration % 60, bytes / 1048576.0);
    }
    usleep(10000);
  }
  writer.Close();
  return 0;
}

