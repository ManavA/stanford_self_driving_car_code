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
#include <param_interface.h>
#include <velo_support.h>

namespace dgc {

void vlf_index(char *velo_log_filename, char *ipc_log_filename,
    char *index_filename)
{
  int i, err, encoder, last_encoder = 0, spin_num_packets = 0;
  int scan_num[MAX_NUM_VELOSCANS], percent, last_percent = -1;
  ApplanixPose applanix_pose, next_applanix_pose;
  LocalizePose localize_pose;
  ApplanixPose scan_pose[MAX_NUM_VELOSCANS];
  LocalizePose scan_localize_pose[MAX_NUM_VELOSCANS];
  int num_poses = 0, num_scans = 0, spin_start_i;
  bool new_applanix = true, incomplete = false;
  dgc_FILE *log_fp = NULL, *output_fp = NULL;
  off64_t packet_offset, spin_start_offset;
  LineBuffer *line_buffer = NULL;
  dgc_velodyne_file_p velodyne = NULL;
  dgc_velodyne_packet_t pkt;
  dgc_velodyne_index_pose p;
  off64_t velodyne_size;

  localize_pose.x_offset = 0.0;
  localize_pose.y_offset = 0.0;

  /* prepare for velodyne */
  velodyne = dgc_velodyne_open_file(velo_log_filename);
  if(velodyne == NULL)
    dgc_die("Error: could not open velodyne file %s\n", velo_log_filename);

  /* prepare IPC logfile */
  log_fp = dgc_fopen(ipc_log_filename, "r");
  if(log_fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", ipc_log_filename);
  line_buffer = new LineBuffer;

  output_fp = dgc_fopen(index_filename, "w");
  if(output_fp == NULL)
    dgc_die("Error: could not open file %s for writing.\n", index_filename);

  /* read two applanix messages */
  err = read_applanix_message(log_fp, line_buffer, &applanix_pose, &localize_pose);
  if(err < 0) {
    dgc_fclose(output_fp);
    return;
  }
  err = read_applanix_message(log_fp, line_buffer, &next_applanix_pose, &localize_pose);
  if(err < 0) {
    dgc_fclose(output_fp);
    return;
  }

  velodyne_size = dgc_file_size(velo_log_filename);

  spin_start_offset = dgc_ftell(velodyne->fp);
  spin_num_packets = 0;
  spin_start_i = 0;

  while(1) {
    /* read a velodyne packet */
    packet_offset = dgc_ftell(velodyne->fp);
    do {
      err = dgc_velodyne_read_packet(velodyne, &pkt);
      if(dgc_feof(velodyne->fp)) {
        dgc_fclose(output_fp);
        return;
      }
    } while(err < 0);
    spin_num_packets++;

    percent = (int)rint((double)packet_offset / (double)velodyne_size * 100);
    if(percent != last_percent) {
      fprintf(stderr, "\rCreating index (%d%%)   ", percent);
      last_percent = percent;
    }

    /* remember if we got a packet before the 1st pose */
    if(pkt.timestamp < applanix_pose.timestamp)
      incomplete = true;
    /* read applanix messages until the velodyne data falls
       between the timestamps */
    else
      while(next_applanix_pose.timestamp < pkt.timestamp) {
        new_applanix = true;
        memcpy(&applanix_pose, &next_applanix_pose, sizeof(ApplanixPose));
        err = read_applanix_message(log_fp, line_buffer, &next_applanix_pose, &localize_pose);
        if(err < 0) {
          dgc_fclose(output_fp);
          fprintf(stderr, "\n");
          return;
        }
      }

    /* project the scan according to the current pose */
    for(i = 0; i < VELO_SCANS_IN_PACKET; i++) {
      encoder = (pkt.scan[i].encoder + VELO_SPIN_START) % VELO_NUM_TICKS;
      if(encoder < last_encoder) {    
        if(!incomplete) {
          dgc_fwrite(&spin_start_offset, sizeof(off64_t), 1, output_fp);
          dgc_fwrite(&spin_start_i, sizeof(int), 1, output_fp);
          dgc_fwrite(&num_scans, sizeof(int), 1, output_fp);
          dgc_fwrite(&num_poses, sizeof(int), 1, output_fp);
          for(int j = 0; j < num_poses; j++) {
            p.scan_num = scan_num[j];
            p.smooth_x = scan_pose[j].smooth_x;
            p.smooth_y = scan_pose[j].smooth_y;
            p.smooth_z = scan_pose[j].smooth_z;
            p.longitude = scan_pose[j].longitude;
            p.latitude = scan_pose[j].latitude;
            p.altitude = scan_pose[j].altitude;
            p.v_east = scan_pose[j].v_east;
            p.v_north = scan_pose[j].v_north;
            p.v_up = scan_pose[j].v_up;
            p.roll = scan_pose[j].roll;
            p.pitch = scan_pose[j].pitch;
            p.yaw = scan_pose[j].yaw;
            p.timestamp = scan_pose[j].timestamp;
            p.x_offset = scan_localize_pose[j].x_offset;
            p.y_offset = scan_localize_pose[j].y_offset;
            dgc_fwrite(&p, sizeof(dgc_velodyne_index_pose), 1, output_fp);
          }
        }
        spin_start_offset = packet_offset;
        spin_num_packets = 0;
        spin_start_i = i;
        num_scans = 0;
        num_poses = 0;
        incomplete = false;
      }

      if(num_scans == 0 || new_applanix) {
        memcpy(&(scan_pose[num_poses]), &applanix_pose, sizeof(ApplanixPose));
        memcpy(&(scan_localize_pose[num_poses]), &localize_pose, sizeof(LocalizePose));
        scan_num[num_poses] = num_scans;
        num_poses++;
      }

      if(num_scans < MAX_NUM_VELOSCANS) 
        num_scans++;

      last_encoder = encoder;
      new_applanix = false;
    }
  }
  dgc_fclose(output_fp);
}

} // namespace dgc

using namespace dgc;

int main(int argc, char **argv)
{
  char output_filename[200];

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
        "Usage: %s log-file velodyne-file\n", argv[0]);

  strcpy(output_filename, argv[2]);
  if(strcmp(output_filename + strlen(output_filename) - 4, ".vlf") != 0)
    dgc_die("Velodyne file must end in .vlf\n");
  strcat(output_filename, ".index.gz");
  vlf_index(argv[2], argv[1], output_filename);
  return 0;
}
