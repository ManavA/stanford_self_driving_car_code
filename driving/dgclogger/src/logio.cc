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


#include <sys/stat.h>
#include <logio.h>

namespace dgc {

//void LogReaderCallbackList::DefineIpcMessages(IpcInterface *ipc)
//{
//  unsigned int i;
//  int err;
//
//  for (i = 0; i < callback_.size(); i++) {
//    err = ipc->DefineMessage(callback_[i].messageID);
//    TestIpcExit(err, "Could not define", callback_[i].messageID);
//  }
//}

void LogReaderCallbackList::AddCallback(char* logger_message_name, const IpcMessageID id, LogConverterFunc converter, int message_size, int interpreted) {
  LogReaderCallback c;

  c.logger_message_name = strdup(logger_message_name);
  c.messageID.name = strdup(id.name);
  c.messageID.fmt = strdup(id.fmt);
  c.conv_func = converter;
  c.message_data = (void *) calloc(1, message_size);
  c.interpreted = interpreted;
  callback_.push_back(c);
}

LogReaderCallback* LogReaderCallbackList::FindCallback(char* name) {
  unsigned int i, n;

  n = strlen(name);
  for (i = 0; i < callback_.size(); i++)
    if (strncmp(name, callback_[i].logger_message_name, n) == 0) return &(callback_[i]);
  return NULL;
}

off64_t LogfileUncompressedLength(vlr::cio::FILE* infile) {
  unsigned char buffer[10000];
  off64_t log_bytes = 0;
  int nread;
  struct stat64 stat_buf;

  if (!infile->compressed) {
    // compute total length of logfile
    vlr::cio::fseek(infile, 0, SEEK_SET);
    log_bytes = 0;
    do {
      nread = vlr::cio::fread(buffer, 1, 10000, infile);
      log_bytes += nread;
    } while (nread > 0);
    vlr::cio::fseek(infile, 0, SEEK_SET);
    return log_bytes;
  }
  else {
    // report compressed size for compressed files
    fstat64(fileno(infile->fp), &stat_buf);
    return stat_buf.st_size;
  }
}

LogfileIndex::LogfileIndex() {
  num_messages_ = 0;
  offset_ = NULL;
  current_position_ = 0;
  total_bytes_ = 0;
  corrupted_ = false;
}

void LogfileIndex::IndexFile(vlr::cio::FILE* infile) {
  int i, found_linebreak = 1, nread, percent, last_percent = -1, err;
  off64_t file_length = 0, file_position = 0, total_bytes;
  int max_messages, read_count = 0;
  unsigned char buffer[100000];

  /* compute the total length of the uncompressed logfile. */
  fprintf(stderr, "\n\rIndexing messages (0%%)    ");
  file_length = LogfileUncompressedLength(infile);

  /* mark the start of all messages */
  num_messages_ = 0;
  max_messages = 10000;
  offset_ = (off64_t *) calloc(max_messages, sizeof(off64_t));
  dgc::dgc_test_alloc(offset_);

  vlr::cio::fseek(infile, 0, SEEK_SET);

  total_bytes = 0;
  do {
    nread = vlr::cio::fread(buffer, 1, 100000, infile);
    read_count++;
    if (read_count % 100 == 0) {
      if (!infile->compressed) file_position = total_bytes + nread;
      else file_position = lseek64(fileno(infile->fp), 0, SEEK_CUR);
      percent = (int) rint((double) file_position / (double) file_length * 100.0);
      if (percent != last_percent) {
        fprintf(stderr, "\rIndexing messages (%d%%)      ", percent);
        last_percent = percent;
      }
      read_count = 0;
    }

    if (nread > 0) {
      for (i = 0; i < nread; i++) {
        if (found_linebreak && buffer[i] != '\r') {
          found_linebreak = 0;
          if (num_messages_ == max_messages) {
            max_messages += 10000;
            offset_ = (off64_t *) realloc(offset_, max_messages * sizeof(off64_t));
            dgc::dgc_test_alloc(offset_);
          }
          offset_[num_messages_] = total_bytes + i;
          num_messages_++;
        }
        if (buffer[i] == '\n') found_linebreak = 1;
      }
      total_bytes += nread;
    }
  } while (nread > 0);
  fprintf(stderr, "\rIndexing messages (100%%) - %d messages found.      \n", num_messages_);
  err = vlr::cio::fseek(infile, 0L, SEEK_SET);
  corrupted_ = ((err < 0) || !found_linebreak);

  current_position_ = 0;
  total_bytes_ = total_bytes;
}

int LogfileIndex::Load(char* logfile_name) {
  char index_name[300], *err, line[1000];
  vlr::cio::FILE* fp;
  int i;

  if (strlen(logfile_name) > 4 && strcmp(logfile_name + strlen(logfile_name) - 4, ".log") == 0) {
    strcpy(index_name, logfile_name);
    strcpy(index_name + strlen(index_name) - 4, ".index.gz");
  }
  else if (strlen(logfile_name) > 7 && strcmp(logfile_name + strlen(logfile_name) - 7, ".log.gz") == 0) {
    strcpy(index_name, logfile_name);
    strcpy(index_name + strlen(index_name) - 7, ".index.gz");
  }
  else fprintf(stderr, "dgc_logfile_index_load: Unrecognized file extension.\n");

  fp = vlr::cio::fopen(index_name, "r");
  if (fp == NULL) {
    fprintf(stderr, "Error: could not open index %s for reading.\n", index_name);
    return -1;
  }

  fprintf(stderr, "Loading logfile index %s ... ", index_name);

  err = vlr::cio::fgets(line, 1000, fp);
  sscanf(line, "%lld", (long long int *) &total_bytes_);

  err = vlr::cio::fgets(line, 1000, fp);
  sscanf(line, "%d", &num_messages_);

  offset_ = (off64_t *) calloc(num_messages_, sizeof(off64_t));
  dgc::dgc_test_alloc(offset_);

  for (i = 0; i < num_messages_; i++) {
    err = vlr::cio::fgets(line, 1000, fp);
    sscanf(line, "%lld", (long long int *) &offset_[i]);
  }

  current_position_ = 0;
  corrupted_ = 0;
  fprintf(stderr, "done.\n");
  return 0;
}

void LogfileIndex::Save(char* logfile_name) {
  char index_name[300];
  vlr::cio::FILE* fp;
  int i;

  if (strlen(logfile_name) > 4 && strcmp(logfile_name + strlen(logfile_name) - 4, ".log") == 0) {
    strcpy(index_name, logfile_name);
    strcpy(index_name + strlen(index_name) - 4, ".index.gz");
  }
  else if (strlen(logfile_name) > 7 && strcmp(logfile_name + strlen(logfile_name) - 7, ".log.gz") == 0) {
    strcpy(index_name, logfile_name);
    strcpy(index_name + strlen(index_name) - 7, ".index.gz");
  }
  else fprintf(stderr, "Unrecognized file extension.\n");

  fprintf(stderr, "Writing logfile index %s ... ", index_name);

  fp = vlr::cio::fopen(index_name, "w");
  if (fp == NULL) {
    throw VLRException("Could not save logfile index " + std::string(index_name));
  }

  vlr::cio::fprintf(fp, "%lld\n", total_bytes_);
  vlr::cio::fprintf(fp, "%d\n", num_messages_);
  for (i = 0; i < num_messages_; i++)
    vlr::cio::fprintf(fp, "%lld\n", offset_[i]);

  vlr::cio::fclose(fp);

  fprintf(stderr, "done.\n");
}

float LogfileIndex::PercentRead() {
  return current_position_ / (float) num_messages_;
}

long int LogfileIndex::MessageLength(int message_num) {
  if (message_num == num_messages_ - 1) return total_bytes_ - offset_[message_num];
  else return offset_[message_num + 1] - offset_[message_num];
}

off64_t LogfileIndex::Offset(int message_num) {
  return offset_[message_num];
}

int LogfileIndex::num_messages() {
  return num_messages_;
}

} // namespace dgc
