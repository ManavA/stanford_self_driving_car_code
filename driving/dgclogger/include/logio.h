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


#ifndef LOGIO_H_
#define LOGIO_H_

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <global.h>
#include <vector>

namespace dgc {

struct IpcMessageID {
  const char *name;
  const char *fmt;
};

typedef char *(*LogConverterFunc)(char*, void*);

typedef struct {
  char *logger_message_name;
  IpcMessageID messageID;
  //  char *output_string;
  LogConverterFunc conv_func;
  void *message_data;
  int interpreted;
} LogReaderCallback;

class LogReaderCallbackList {
public:
  void AddCallback(char* logger_message_name, const IpcMessageID id, LogConverterFunc converter, int message_size, int interpreted);
//  void DefineIpcMessages(IpcInterface *ipc);
  LogReaderCallback* FindCallback(char* name);

private:
  std::vector<LogReaderCallback> callback_;
};

off64_t LogfileUncompressedLength(vlr::cio::FILE* infile);

class LogfileIndex {
public:
  LogfileIndex();
  void IndexFile(vlr::cio::FILE* infile);

  int Load(char* logfile_name);
  void Save(char* logfile_name);
  float PercentRead();
  long int MessageLength(int message_num);
  off64_t Offset(int message_num);
  int num_messages();
  inline int numMessages() {
    return num_messages_;
  }

private:
  int num_messages_;
  off64_t *offset_;
  long int current_position_;
  off64_t total_bytes_;
  int corrupted_;
};

#define READ_FLOAT(pos) strtof(*(pos), (pos))
#define READ_DOUBLE(pos) strtod(*(pos), (pos))
#define READ_UINT(pos) strtoul(*(pos), (pos), 10)
#define READ_INT(pos) strtol(*(pos), (pos), 10)
#define READ_HOST(host, pos) { if(*(*(pos)) == ' ') (*pos)++; \
                               if(*(*(pos)) == ' ') strcpy(host,"unknown"); \
                               else sscanf(*(pos), "%s", host); \
                               *(pos) = dgc_next_word(*(pos)); }

} // namespace dgc

#endif

