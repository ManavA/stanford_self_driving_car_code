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


#include <inttypes.h>
#include <string.h>
#include <stdarg.h>
#include <comp_stdio.h>

namespace vlr {
namespace cio {

cio::FILE* fopen(const char* filename, const char* mode)
{
  cio::FILE* fp=NULL;

    // allocate a new file pointer
  fp = new cio::FILE;

    // TODO: buffers have to be deleted manually :-(
    // copy the filename & mode
  fp->filename = new char[strlen(filename)+1];
  strcpy(fp->filename, filename);

  fp->mode = new char[strlen(mode)+1];
  strcpy(fp->mode, mode);

  /* look at filename extension to determine if file is compressed */
  if(strcmp(filename + strlen(filename) - 3, ".gz") == 0)
    fp->compressed = 1;
  else
    fp->compressed = 0;

  if(!fp->compressed) {
    fp->fp = fopen64(filename, mode);
    if(fp->fp == NULL) {
      delete fp;
      return NULL;
    }
  }
  else {
    fp->fp = fopen64(filename, mode);
    if(fp->fp == NULL) {
      delete fp;
      return NULL;
    }
    fp->comp_fp = (void**)gzdopen(fileno(fp->fp), mode);
    if(fp->comp_fp == NULL) {
      fclose(fp->fp);
      delete fp;
      return NULL;
    }
  }
  return fp;
}

int fgetc(cio::FILE* fp)
{
  if(!fp->compressed)
    return fgetc(fp->fp);
  else
    return gzgetc(fp->comp_fp);
}

int feof(cio::FILE* fp)
{
  if(!fp->compressed)
    return feof(fp->fp);
  else
    return gzeof(fp->comp_fp);
}

int fseek(cio::FILE* fp, off64_t offset, int whence)
{
  int err;

  if(!fp->compressed)
    return fseeko64(fp->fp, offset, whence);
  else {
    err = gzseek(fp->comp_fp, offset, whence);

    if(err < 0 && whence == SEEK_SET && offset == 0) {
      gzclose(fp->comp_fp);

      fp->fp = fopen64(fp->filename, fp->mode);
      if(fp->fp == NULL) {
	delete fp;
	return -1;
      }
      fp->comp_fp = (void**)gzdopen(fileno(fp->fp), fp->mode);
      if(fp->comp_fp == NULL) {
	fclose(fp->fp);
	delete fp;
	return -1;
      }

      return 0;
    }
    else
      return err;
  }
}

off64_t ftell(cio::FILE* fp)
{
  if(!fp->compressed)
    return ftello64(fp->fp);
  else
    return gztell(fp->comp_fp);
}

int fclose(cio::FILE* fp)
{
  if(!fp->compressed)
    return fclose(fp->fp);
  else
    return gzclose(fp->comp_fp);
}

size_t fread(void *ptr, size_t size, size_t nmemb, cio::FILE* fp)
{
  if(!fp->compressed)
    return fread(ptr, size, nmemb, fp->fp);
  else
    return gzread(fp->comp_fp, ptr, size * nmemb) / size;
}

size_t fwrite(const void *ptr, size_t size, size_t nmemb, cio::FILE* fp)
{
  if(!fp->compressed)
    return fwrite(ptr, size, nmemb, fp->fp);
  else
    return gzwrite(fp->comp_fp, (void *)ptr, size * nmemb) / size;
}

char* fgets(char* s, int size, cio::FILE* fp)
{
  if(!fp->compressed)
    return fgets(s, size, fp->fp);
  else
    return gzgets(fp->comp_fp, s, size);
}

int fputc(int c, cio::FILE* fp)
{
  if(!fp->compressed)
    return fputc(c, fp->fp);
  else
    return gzputc(fp->comp_fp, c);
}

void fprintf(cio::FILE* fp, const char* fmt, ...)
{
  /* Guess we need no more than 100 bytes. */
  int n, size = 100;
  char* p;
  va_list ap;

  if((p = (char* )malloc(size)) == NULL)
    return;
  while(1) {
    /* Try to print in the allocated space. */
    va_start(ap, fmt);
    n = vsnprintf(p, size, fmt, ap);
    va_end(ap);
    /* If that worked, return the string. */
    if(n > -1 && n < size) {
      fwrite(p, strlen(p), 1, fp);
      free(p);
      return;
    }
    /* Else try again with more space. */
    if(n > -1)    /* glibc 2.1 */
      size = n + 1; /* precisely what is needed */
    else           /* glibc 2.0 */
      size *= 2;  /* twice the old size */
    if((p = (char*)realloc(p, size)) == NULL)
      return;
  }
}

int fflush(cio::FILE* fp)
{
  if(!fp->compressed)
    return fflush(fp->fp);
  else
    return gzflush(fp->comp_fp, Z_SYNC_FLUSH);
}

} // namespace cio
} // namespace vlr
