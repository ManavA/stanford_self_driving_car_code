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
#include "conout.h"
#include "bufferedStringBuf.h"

namespace vlr {
BufferedStringBuf::BufferedStringBuf(uint32_t bufSize) {
  if (bufSize) {
    char *ptr = new char[bufSize];
    setp(ptr, ptr + bufSize);
  }
  else {
    setp(0, 0);
  }
}

BufferedStringBuf::~BufferedStringBuf() {
  sync();
  delete[] pbase();
}

void BufferedStringBuf::writeString(const std::string &str) {
  conout(str.c_str());
}

int32_t BufferedStringBuf::overflow(int32_t c) {
  sync();

  if (c != EOF) {
    if (pbase() == epptr()) {
      std::string temp;

      temp += char(c);
      writeString(temp);
    }
    else {
      sputc(c);
    }
  }

  return 0;
}

int32_t BufferedStringBuf::sync() {
  if (pbase() != pptr()) {
    int32_t len = int32_t(pptr() - pbase());
    std::string temp(pbase(), len);
    writeString(temp);
    setp(pbase(), epptr());
  }
  return 0;
}

} // namespace vlr
