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


#ifndef DGC_STDIO_H_
#define DGC_STDIO_H_

#include <stdio.h>
#include <stdlib.h>
#include <zlib.h>

namespace vlr {
namespace cio { // c-style io functions that transparently handle compressed files

struct FILE {
  int compressed;
  char *filename, *mode;
  ::FILE* fp;
  gzFile *comp_fp;
};

cio::FILE* fopen(const char* filename, const char* mode);

int fgetc(cio::FILE* fp);

int feof(cio::FILE* fp);

int fseek(cio::FILE* fp, off64_t offset, int whence);

off64_t ftell(cio::FILE* fp);

int fclose(cio::FILE* fp);

size_t fread(void* ptr, size_t size, size_t nmemb, cio::FILE* fp);

size_t fwrite(const void* ptr, size_t size, size_t nmemb, cio::FILE* fp);

char *fgets(char* s, int size, cio::FILE* fp);

int fputc(int c, cio::FILE* fp);

void fprintf(cio::FILE* fp, const char* fmt, ...);

int fflush(cio::FILE* fp);

} // namespace cio
} // namespace vlr
#endif
