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


#include <logio.h>
#include <applanix_interface.h>
#include <velo_support.h>

namespace dgc {

int
dgc_velodyne_write_packet(dgc_FILE* fp, double timestamp, unsigned char* msg_buffer)
{
  int             n;
  unsigned short  len;

  n = dgc_fputc(VLF_START_BYTE, fp);
  if(n != VLF_START_BYTE) {
    fprintf(stderr, "# ERROR: could not write start byte.\n");
    perror("Could not write start byte");
    return -1;
  }

  n = dgc_fwrite(&timestamp, 8, 1, fp);
  if(n != 1) {
    fprintf(stderr, "Error: could not write time stamp.\n");
    return -1;
  }

  len = VELO_PACKET_SIZE;
  n = dgc_fwrite(&len, 2, 1, fp);
  if(n != 1) {
    fprintf(stderr, "Error: could not write packet length.\n");
    return -1;
  }

  n = dgc_fwrite(msg_buffer, len+1, 1, fp);
  if(n != 1) {
    fprintf(stderr, "Error: could not write message buffer.\n");
    return -1;
  }

  return 0;
}

} // namespace dgc

using namespace dgc;

int main(int argc, char **argv)
{
  double start_time, end_time = 1e6;
  char velodyne_in_file[200];
  char velodyne_out_file[200];
  int err;

  dgc_velodyne_packet_t pkt;
  dgc_velodyne_file_p velodyne = NULL;
  dgc_FILE* fout;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
        "Usage: %s logfile start-second [end-second]\n"
        "       seconds since beginning of logfile\n", argv[0]);

  start_time = atof(argv[2]);
  if(argc >= 4)
    end_time = atof(argv[3]);

  strcpy(velodyne_in_file, argv[1]);
  strcpy(velodyne_out_file, argv[1]);

  if(strcmp(velodyne_out_file + strlen(velodyne_out_file) - 4, ".vlf") == 0)
    strcpy(velodyne_out_file + strlen(velodyne_out_file) - 4, "-cut.vlf");
  else
    dgc_die("Error: input logfile must end in .vlf\n");

  fout = dgc_fopen(velodyne_out_file, "w");
  if(fout == NULL)
    dgc_die("Error: could not open file %s for writing.\n", velodyne_out_file);

  fprintf(stderr, "Opening logfile... ");
  /* prepare velodyne */
  velodyne = dgc_velodyne_open_file(velodyne_in_file);
  if(velodyne == NULL)
    dgc_die("Error: could not open velodyne file %s\n", velodyne_in_file);

  err = dgc_velodyne_read_packet(velodyne, &pkt);
  if (err)
    dgc_die("Error: could not read velodyne file %s\n", velodyne_in_file);

  start_time += pkt.timestamp;
  end_time += pkt.timestamp;

  int count = 0;
  while (pkt.timestamp < start_time) {
    count++;
    int err = dgc_velodyne_read_packet(velodyne, &pkt);
    if (err) break;
  }

  fprintf(stderr, "skipped %d packets\n", count);

  count = 0;
  while (pkt.timestamp <= end_time) {
    count++;
    err = dgc_velodyne_write_packet(fout, pkt.timestamp, velodyne->msg_buffer);
    if (err)
      dgc_die("Error: could not write to file %s\n", velodyne_out_file);
    err = dgc_velodyne_read_packet(velodyne, &pkt);
    if (err) break;
  }
  fprintf(stderr, "wrote %d packets about %f seconds\n", count, count / 10.0);
  fprintf(stderr, "done.\n");

  dgc_fclose(velodyne->fp);
  dgc_fclose(fout);
  return 0;
}
