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


#ifndef DGC_APPLANIX_PARSE_H
#define DGC_APPLANIX_PARSE_H

#include <applanix_messages.h>

#define      APPLANIX_PARSE_ERROR         (-1)
#define      APPLANIX_PARSE_UNFINISHED    (0)

int 
applanix_parse_generic_message(char *buffer, int buffer_length);

int
applanix_parse_pose_message(char *buffer, int buffer_len, 
                            dgc::ApplanixPose *pose);
int
applanix_parse_rms_message(char *buffer, int buffer_len, 
                           dgc::ApplanixRms *rms);

int
applanix_parse_gps_message(char *buffer, int buffer_len, int *sats);

int
applanix_parse_time_message(char *buffer, int buffer_length, int *sync_mode);

int
applanix_parse_gams_message(char *buffer, int buffer_len, int *code);

int
applanix_parse_dmi_message(char *buffer, int buffer_length, 
                           dgc::ApplanixDmi *dmi);

#endif
