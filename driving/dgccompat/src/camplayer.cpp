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
#include <camplayer.h>
#include <blf_id.h>

namespace vlr {

cam_player::cam_player(CameraInterface* camint)
{
  camera_interface = camint;
  blf = new blf_t;
  blf_index = NULL;
  image = NULL;
}

cam_player::~cam_player()
{
  if(image != NULL)
    delete image;
  delete camera_interface;
  delete blf;
}

int cam_player::initialize(char *cam_filename)
{
  image = new CameraImage;

  if(blf->open(cam_filename, "r") != BLF_OK) {
    dgc_error("Could not open file %s for reading.", cam_filename);
    return -1;
  }
  blf_index = blf_index_load(cam_filename);
  if(blf_index == NULL) 
    dgc_warning("Could not open index file %s.idx.gz", cam_filename);
  return 0; 
}

void cam_player::seek(double t)
{
  int i = 0;

  /* skip to right place */
  if(blf_index != NULL) {
    while(i < blf_index->num_blocks && 
	  blf_index->block[i].timestamp < t)
      i++;
    if(i >= blf_index->num_blocks)
      i = blf_index->num_blocks - 1;
    blf->seek(blf_index->block[i].offset, SEEK_SET);
  }
  else {
    blf->rewind();
    blf->seek_timestamp(t);
  }

  set_last_packet_time(t);
}

void cam_player::read_packet(double t, 
			     dgc_pose_p pose __attribute__ ((unused)), 
			     double max_age)
{
  unsigned short pkt_id;
  unsigned int pkt_len;
  double pkt_timestamp;
  int ret;
  unsigned int old_num_bytes;
  unsigned int dummy;

  /* get the packet info without reading the packet */
  ret = blf->start_partial_read(&pkt_id, &pkt_timestamp, &pkt_len);
  if(ret == BLF_EOF) 
    set_eof(true);
  if(ret != BLF_OK)
    return;

  /* skip it if it isn't a camera packet, or it is too old */
  if(pkt_id != BLF_CAMERA_ID || fabs(pkt_timestamp - t) > max_age ||
     pkt_len <= sizeof(CameraImage)) {
    set_last_packet_time(pkt_timestamp);
    blf->abort_partial_read();
    return;
  }

  old_num_bytes = image->num_bytes;
  /* read the camera struct */
  ret = blf->partial_read((unsigned char *)&(image->info.width),
      sizeof(unsigned int));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&(image->info.height),
      sizeof(unsigned int));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&(image->info.padded_width),
      sizeof(unsigned int));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&(image->info.channels),
      sizeof(unsigned char));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&(image->info.depth),
      sizeof(unsigned char));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&(image->info.format),
      sizeof(unsigned short));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&(image->info.camera_number),
      sizeof(unsigned short));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&(image->info.guid),
      sizeof(unsigned long long));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&dummy,
      sizeof(unsigned short));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&(image->num_bytes),
      sizeof(unsigned int));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&dummy, // This write
      sizeof(unsigned int));  // is a dummy to read the space where the data
  if(ret != BLF_OK) {         // data pointer lives
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&(image->frame_num),
      sizeof(unsigned int));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }
  ret = blf->partial_read((unsigned char *)&(image->timestamp),
      sizeof(double));
  if(ret != BLF_OK) {
    image->num_bytes = old_num_bytes;
    blf->abort_partial_read();
    return;
  }

  /* resize the image, if necessary */
  if(image->num_bytes != old_num_bytes) {
    if(image->data != NULL)
      delete image->data;
    image->data = new unsigned char[image->num_bytes];
  }

  /* read the image data */
  ret = blf->partial_read(image->data, image->num_bytes);
  if(ret != BLF_OK) {
    blf->abort_partial_read();
    return;
  }

  /* finish reading packet, make sure chksum is ok */
  ret = blf->finish_partial_read();
  if(ret != BLF_OK)
    return;

  camera_interface->WriteImage(image);
  add_output_bytes(pkt_len);
  add_frames(1);
  set_last_packet_time(image->timestamp);
  add_input_bytes(pkt_len);
}

}
