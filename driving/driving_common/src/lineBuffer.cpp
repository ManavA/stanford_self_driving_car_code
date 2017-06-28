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


#include <stdio.h>
#include <string.h>
#include <lineBuffer.h>

using namespace vlr;

namespace driving_common {

LineBuffer::LineBuffer() {
  max_bytes_ = 100000;
  current_position_ = 0;
  buffer_.reserve(max_bytes_);
}

LineBuffer::~LineBuffer() {
}

char* LineBuffer::findExistingLine() {
  bool found = false, found_space = false;
  int end_mark = 0;
  char* line;

  // look for a complete line
  for (size_t i = current_position_; i < buffer_.size(); i++) {
    if (buffer_[i] == '\n') {
      buffer_[i] = '\0';
      end_mark = i;
      found = true;
      break;
    }
    else if (buffer_[i] == ' ') {found_space = true;}
  }

  if (found) {
    line = &buffer_[0] + current_position_;
    current_position_ = end_mark + 1;
    if (current_position_ == buffer_.size()) {
      current_position_ = 0;
      buffer_.clear();
    }
    return line;
  }
 return NULL;
}

char* LineBuffer::readLine(cio::FILE* fp) {
  int left, nread, increased_size = 0;
  char* line = NULL;

  /* try to find a line with the existing data */
  line = findExistingLine();
  if (line != NULL) return line;

  /* if we can't find one, move the extra bytes to the beginning 
   of the buffer */
  if (buffer_.size() > current_position_) memmove(&buffer_[0], &buffer_[0] + current_position_, buffer_.size() - current_position_);
  buffer_.resize(buffer_.size()-current_position_);
  current_position_ = 0;

  do {
    /* fill the rest of the buffer */
    left = max_bytes_ - buffer_.size();
    nread = 0;
    if (left > 0) {
      size_t old_size = buffer_.size();
      buffer_.resize(buffer_.size() + left);
      nread = cio::fread(&buffer_[old_size], 1, left, fp);
      if (nread > 0) {
        if(nread != left) {
          buffer_.resize(buffer_.size() - (left-nread));
        }
      }
    }

    /* try to find a line with the existing data */
    line = findExistingLine();
    if (line != NULL) return line;

    /* if the buffer is full, double its size */
    increased_size = 0;
    if (buffer_.size() == max_bytes_) {
      max_bytes_ *= 2;
      buffer_.reserve(max_bytes_);
      increased_size = 1;
    }
  } while (nread > 0 || increased_size);
  return NULL;
}

void LineBuffer::reset() {
  current_position_ = 0;
  buffer_.clear();
}

} // namespace driving_common
