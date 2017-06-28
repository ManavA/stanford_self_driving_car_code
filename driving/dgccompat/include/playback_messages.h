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


#ifndef    DGC_PLAYBACK_MESSAGES_H
#define    DGC_PLAYBACK_MESSAGES_H

#include <ipc_interface.h>

namespace vlr {

#define    DGC_PLAYBACK_COMMAND_PLAY         0
#define    DGC_PLAYBACK_COMMAND_STOP         1
#define    DGC_PLAYBACK_COMMAND_RESET        2
#define    DGC_PLAYBACK_COMMAND_FORWARD      3
#define    DGC_PLAYBACK_COMMAND_REWIND       4
#define    DGC_PLAYBACK_COMMAND_FWD_SINGLE   5
#define    DGC_PLAYBACK_COMMAND_RWD_SINGLE   6
#define    DGC_PLAYBACK_COMMAND_SET_SPEED    7
#define    DGC_PLAYBACK_COMMAND_SEEK         8

typedef struct {
  int cmd;
  long int arg;
  float speed;
} PlaybackCommand;

#define DGC_PLAYBACK_COMMAND_NAME     "dgc_playback_command"
#define DGC_PLAYBACK_COMMAND_FMT      "{int,long,float}"

const IpcMessageID PlaybackCommandID = { DGC_PLAYBACK_COMMAND_NAME, 
					 DGC_PLAYBACK_COMMAND_FMT };

#define DGC_PLAYBACK_STATUS_PLAYING          0
#define DGC_PLAYBACK_STATUS_PAUSED           1

typedef struct {
  int mode;
  float percent_read;
  float speed;
} PlaybackStatus;


#define DGC_PLAYBACK_STATUS_NAME     "dgc_playback_status"
#define DGC_PLAYBACK_STATUS_FMT      "{int, float, float}"

const IpcMessageID PlaybackStatusID = {  DGC_PLAYBACK_STATUS_NAME, 
					 DGC_PLAYBACK_STATUS_FMT };

}

#endif
