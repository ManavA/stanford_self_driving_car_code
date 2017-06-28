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


#include <ipc_std_interface.h>
#include <param_interface.h>
#include <signal_handler.h>
#include <applanixcore.h>
#include <signal.h>

using namespace dgc;

static char process_buffer[kInternalBufferSize];
static const int process_buffer_size = kInternalBufferSize;

static double bytes_to_megabytes(double bytes)
{
  return bytes / 1048576.;
}

static void LogMessages(ApplanixServer *applanix, FILE *fp)
{
  double last_printout = -1, start, now;
  int bytes_read, total_bytes_read = 0;
  
  start = dgc_get_time();
  while(1) {
    bytes_read = applanix->ReadData(process_buffer, process_buffer_size);
    
    if(bytes_read > 0) {
      int i;
      for(i = 0; i < bytes_read; i++)
        fprintf(fp, "%c", process_buffer[i]);
      
      total_bytes_read += bytes_read;
    }

    now = dgc_get_time();
    if(now - last_printout > 0.25) {
      int hours, minutes, seconds;
      
      hours = (int)floor((now - start) / 60. / 60.);
      minutes = (int)floor((now - start) / 60. - (hours * 60.));
      seconds = (int)floor((now - start) - (hours * 60. * 60.) - 
			   (minutes * 60.));
      
      fprintf(stderr, "\r%02d Hrs %02d Min %02d Sec    %.3f MB Read    %c", 
	      hours, minutes, seconds, bytes_to_megabytes(total_bytes_read), 
	      dgc_ascii_rotor());
      
      last_printout = now;
    }
  }
}

int main(int argc, char **argv)
{
  IpcInterface *ipc = new IpcStandardInterface();
  ParamInterface *pint = new ParamInterface(ipc);
  char basefilename[256], *filename;
  SignalHandler signal_handler;
  FILE *output_file = NULL;

  if (argc != 2) {
    fprintf(stderr, "usage: %s base_file_name\n", argv[0]);
    return -1;
  }

  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ApplanixServer *applanix = new ApplanixServer(ipc);
  applanix->ReadParameters(pint, argc, argv);
  applanix->ConnectToLoggingPort();
  
  sprintf(basefilename, "%s.applanix_raw", argv[1]);
  filename = dgc_unique_filename(basefilename);
  output_file = fopen(filename, "w");
  if(output_file == NULL) 
    dgc_fatal_error("Could not open file %s", filename);

  signal_handler.Start();
  while(!signal_handler.ReceivedSignal(SIGINT)) 
    LogMessages(applanix, output_file);

  fprintf(stderr, "\nAPPLANIX LOGGER: Caught SIGINT.\n");
  applanix->Shutdown();
  
  delete applanix;
  delete pint;
  delete ipc;

  fclose(output_file);
  fprintf(stderr, "APPLANIX LOGGER: Closing %s.\n", filename);
  return 0;
}
