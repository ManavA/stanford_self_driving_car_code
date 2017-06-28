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


#include <applanix_interface.h>
#include <applanix.h>
#include <applanix_parse.h>

int applanix_parse_generic_message(char *buffer, int buffer_length)
{
  unsigned short message_length;
  int i;
  signed short checksum = 0;

  if(buffer_length < 8)
    return APPLANIX_PARSE_UNFINISHED;

  message_length = *(unsigned short *)(buffer + 6) + 8;
  if(message_length > buffer_length)
    return APPLANIX_PARSE_UNFINISHED;
        
  if(memcmp(buffer + message_length - 2, "$#", 2) != 0) {
    fprintf(stderr, 
            "\nAPPLANIX: A generic message is malformed (termination). "
            "ID = %u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
        
  for(i = 0; i < message_length; i += 2)
    checksum += *(signed short *)(buffer + i);
  if(checksum != 0) {
    fprintf(stderr,
            "\nAPPLANIX: Checksum failed on a generic message. ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
  return message_length;
}

int applanix_parse_pose_message(char *buffer, int buffer_length, 
                                ApplanixPose *pose)
{
  int i;
  signed short checksum = 0;
  unsigned short message_length;
  unsigned char time_type;
        
  if(buffer_length < 8)
    return APPLANIX_PARSE_UNFINISHED;

  message_length = *(unsigned short *)(buffer + 6) + 8;
        
  if(message_length != 132 + 8) {
    fprintf(stderr, "\nAPPLANIX: Navigation solution message is malformed"
            " (length). ID=%u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  if(message_length > buffer_length)
    return APPLANIX_PARSE_UNFINISHED;

  if(memcmp(buffer + message_length - 2, "$#", 2) != 0)        {
    fprintf(stderr, "\nAPPLANIX: Navigation solution message is malformed"
            " (termination). ID=%u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
        
  for(i = 0; i < message_length; i += 2)
    checksum += *(signed short *)(buffer + i);
  if(checksum != 0) {
    fprintf(stderr, "\nAPPLANIX: Checksum failed on navigation solution"
            " message. ID=%u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  time_type = * (unsigned char *) (buffer + 32);
  time_type = time_type & ((unsigned char) 15);
  if (time_type != 2) {
    fprintf(stderr, "\nAPPLANIX: Navigation solution time format is not UTC. Skipping. ID=%u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  pose->latitude = *(double *)(buffer + 34);
  pose->longitude = *(double *)(buffer + 42);
  pose->altitude = *(double *)(buffer + 50);
  
  pose->v_east = *(float *)(buffer + 62);
  pose->v_north = *(float *)(buffer + 58);
  pose->v_up = *(float *)(buffer + 66);

  /* check for velocity jumps */
  static double old_ve = 0, old_vn = 0, old_vu = 0;
  if(old_ve != 0) {
    if(fabs(old_ve - pose->v_east) > 0.1 ||
       fabs(old_vn - pose->v_north) > 0.1 ||
       fabs(old_vu - pose->v_up) > 0.1)
      fprintf(stderr, "\nAPPLANIX: Velocity jump: (%6.4f,%6.4f,%6.4f) to (%6.4f,%6.4f,%6.4f) \n", 
              old_ve, old_vn, old_vu, pose->v_east, pose->v_north, pose->v_up );
  }
  old_ve = pose->v_east;
  old_vn = pose->v_north;
  old_vu = pose->v_up;
    
  pose->roll = *(double *)(buffer + 70);
  pose->pitch = *(double *)(buffer + 78);
  pose->yaw = *(double *)(buffer + 86);
  
  pose->wander = *(double *)(buffer + 94);
  
  pose->track = *(float *)(buffer + 102);
  pose->speed = *(float *)(buffer + 106);
  
  pose->ar_roll = *(float *)(buffer + 110);
  pose->ar_pitch = *(float *)(buffer + 114);
  pose->ar_yaw = *(float *)(buffer + 118);
  
  pose->a_x = *(float *)(buffer + 122);
  pose->a_y = *(float *)(buffer + 126);
  pose->a_z = *(float *)(buffer + 130);
  
  pose->hardware_timestamp = * (double *) (buffer + 8);

  /* Change coordinate system and convert to radians 
     for Stanley compatibility. */
  pose->v_up *= -1.;        
  
  pose->roll = dgc_d2r(pose->roll);
  pose->pitch = dgc_d2r(-1. * pose->pitch);
  pose->yaw = vlr::normalizeAngle(dgc_d2r(-1. * pose->yaw) + dgc_d2r(90.));
  
  pose->wander = dgc_d2r(pose->wander);
  
  pose->track = vlr::normalizeAngle(dgc_d2r(-1. * pose->track) + dgc_d2r(90.));
  
  pose->ar_roll = dgc_d2r(pose->ar_roll);
  pose->ar_pitch = dgc_d2r(-1. * pose->ar_pitch);
  pose->ar_yaw = dgc_d2r(-1. * pose->ar_yaw);
        
  pose->a_y *= -1.;
  pose->a_z *= -1.;

  ApplanixCalculateSmoothedPose(pose);

  return message_length;
}

int applanix_parse_rms_message(char *buffer, int buffer_length, 
                               ApplanixRms *rms)
{
  unsigned short message_length;
  int i;
  signed short checksum = 0;
  unsigned char time_type;

  if(buffer_length < 8)
    return APPLANIX_PARSE_UNFINISHED;

  message_length = * (unsigned short *) (buffer + 6) + 8;
  
  if(message_length != 80 + 8) {
    fprintf(stderr, "\nAPPLANIX: RMS message is malformed (length). ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  if(message_length > buffer_length)
    return APPLANIX_PARSE_UNFINISHED;

  if(memcmp(buffer + message_length - 2, "$#", 2) != 0) {
    fprintf(stderr, "\nAPPLANIX: RMS message is malformed (termination)."
            " ID=%u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
        
  for(i = 0; i < message_length; i += 2)
    checksum += * (signed short *) (buffer + i);
  if(checksum != 0) {
    fprintf(stderr, "\nAPPLANIX: Checksum failed on RMS message. ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  time_type = * (unsigned char *) (buffer + 32);
  time_type = time_type & ((unsigned char) 15);
  if (time_type != 2) {
    fprintf(stderr, "\nAPPLANIX: RMS message time format is not UTC. Skipping. ID=%u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  rms->hardware_timestamp = *(double *)(buffer + 8);
  
  rms->rms_north = *(float *)(buffer + 34);
  rms->rms_east = *(float *)(buffer + 38);
  rms->rms_up = *(float *)(buffer + 42);
        
  rms->rms_v_north = *(float *)(buffer + 46);
  rms->rms_v_east = *(float *)(buffer + 50);
  rms->rms_v_up = *(float *)(buffer + 54);
  
  rms->rms_roll = *(float *)(buffer + 58);
  rms->rms_pitch = *(float *)(buffer + 62);
  rms->rms_yaw = *(float *)(buffer + 66);
  
  rms->semi_major = *(float *)(buffer + 70);
  rms->semi_minor = *(float *)(buffer + 74);
  rms->orientation = *(float *)(buffer + 78);
  
    /* Change coordinate system and convert to radians 
     for Stanley compatibility. */

  rms->rms_roll = dgc_d2r(rms->rms_roll);
  rms->rms_pitch = dgc_d2r(rms->rms_pitch);
  rms->rms_yaw = dgc_d2r(rms->rms_yaw);
  
  rms->orientation = vlr::normalizeAngle(dgc_d2r(-1. * rms->orientation) +
                                         dgc_d2r(90.));
  return message_length;
}

int applanix_parse_gps_message(char *buffer, int buffer_length, int *sats)
{
  unsigned short message_length;
  unsigned char internal_sats;
  signed char solution_status;
  int i;
  signed short checksum = 0;
  
  if(buffer_length < 36)
    return APPLANIX_PARSE_UNFINISHED;
  
  message_length = *(unsigned short *)(buffer + 6) + 8;
  internal_sats = *(unsigned char *)(buffer + 35);
  if(internal_sats > 12) {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (sats). ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  if(message_length != 76 + internal_sats * 20 + 8) {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (length). ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  if(message_length > buffer_length)
    return APPLANIX_PARSE_UNFINISHED;
        
  if(memcmp(buffer + message_length - 2, "$#", 2) != 0)        {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (termination)."
            " ID=%u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
        
  solution_status = *(signed char *)(buffer + 34);
  if(solution_status < -1 || solution_status > 8) {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (solution). ID=%u\n",
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
        
  for(i = 0; i < message_length; i += 2)
    checksum += *(signed short *)(buffer + i);
  if(checksum != 0) {
    fprintf(stderr, "\nAPPLANIX: Checksum failed on GPS message. ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
  *sats = internal_sats;
  return message_length;
}

int applanix_parse_time_message(char *buffer, int buffer_length, int *sync_mode)
{
  unsigned short message_length;
  int i;
  signed short checksum = 0;
  unsigned char internal_sync_status;

  if(buffer_length < 8)
    return APPLANIX_PARSE_UNFINISHED;

  message_length = *(unsigned short *)(buffer + 6) + 8;

  if(message_length != 36 + 8) {
    fprintf(stderr, "\nAPPLANIX: Time synchronization message is malformed (length). ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  if(message_length > buffer_length)
    return APPLANIX_PARSE_UNFINISHED;
        
  if(memcmp(buffer + message_length - 2, "$#", 2) != 0) {
    fprintf(stderr, 
            "\nAPPLANIX: Time synchronization status message is malformed (termination). "
            "ID = %u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
        
  for(i = 0; i < message_length; i += 2)
    checksum += *(signed short *)(buffer + i);
  if(checksum != 0) {
    fprintf(stderr,
            "\nAPPLANIX: Checksum failed on time synchronization message. ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  internal_sync_status = * (unsigned char *) (buffer + 38);
  if (internal_sync_status > 3) {
    fprintf(stderr, "\nAPPLANIX: Time synchronization message is malformed (sync status). ID=%u\n",
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  if (internal_sync_status == 0)
    *sync_mode = 0;
  else if (internal_sync_status == 1 || internal_sync_status == 3)
    *sync_mode = 1;
  else if (internal_sync_status == 2)
    *sync_mode = 2;
  else {
    fprintf(stderr, "\nAPPLANIX: Internal consistency error in parsing time sync message. BUG! Find me. ID=%u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  return message_length;
}

int applanix_parse_gams_message(char *buffer, int buffer_length, int *code)
{
  unsigned short message_length;
  signed char solution_status;
  int i;
  signed short checksum = 0;

  if(buffer_length < 8)
    return APPLANIX_PARSE_UNFINISHED;
        
  message_length = *(unsigned short *)(buffer + 6) + 8;
  
  if(message_length != 72 + 8) {
    fprintf(stderr, "\nAPPLANIX: GAMS message is malformed (length) ID=%u.\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  if(message_length > buffer_length)
    return APPLANIX_PARSE_UNFINISHED;

  if(memcmp(buffer + message_length - 2, "$#", 2) != 0)        {
    fprintf(stderr,
            "\nAPPLANIX: GAMS message is malformed (termination) ID=%u.\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
        
  solution_status = *(signed char *)(buffer + 43);
  if(solution_status < 0 || solution_status > 7) {
    fprintf(stderr, 
            "\nAPPLANIX: GAMS message is malformed (solution code). ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
        
  for(i = 0; i < message_length; i += 2)
    checksum += *(signed short *)(buffer + i);
  if(checksum != 0) {
    fprintf(stderr, "\nAPPLANIX: Checksum failed on GAMS message. ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
  *code = 7 - solution_status;
  return message_length;
}

int applanix_parse_dmi_message(char *buffer, int buffer_length, 
                               ApplanixDmi *dmi)
{
  unsigned short message_length;
  int i;
  signed short checksum = 0;
  unsigned char is_valid, time_type;

  if(buffer_length < 8)
    return APPLANIX_PARSE_UNFINISHED;

  message_length = * (unsigned short *) (buffer + 6) + 8;
  
  if(message_length != 52 + 8) {
    fprintf(stderr, "\nAPPLANIX: DMI message is malformed (length). ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  if(message_length > buffer_length)
    return APPLANIX_PARSE_UNFINISHED;

  if(memcmp(buffer + message_length - 2, "$#", 2) != 0) {
    fprintf(stderr, "\nAPPLANIX: DMI message is malformed (termination)."
            " ID=%u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }
        
  for(i = 0; i < message_length; i += 2)
    checksum += * (signed short *) (buffer + i);
  if(checksum != 0) {
    fprintf(stderr, "\nAPPLANIX: Checksum failed on DMI message. ID=%u\n", 
            applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  is_valid = * (unsigned char *) (buffer + 52);
  if (is_valid != 1) {
    fprintf(stderr, "\nAPPLANIX: DMI message indicates data is invalid. Skipping. ID=%u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  time_type = * (unsigned char *) (buffer + 32);
  time_type = time_type & ((unsigned char) 15);
  if (time_type != 2) {
    fprintf(stderr, "\nAPPLANIX: DMI message time format is not UTC. Skipping. ID=%u\n", applanix_global_unique_id);
    return APPLANIX_PARSE_ERROR;
  }

  dmi->hardware_timestamp = *(double *)(buffer + 8);
  
  dmi->signed_odometer = *(double *)(buffer + 34);
  dmi->unsigned_odometer = *(double *)(buffer + 42);

  return message_length;
}
