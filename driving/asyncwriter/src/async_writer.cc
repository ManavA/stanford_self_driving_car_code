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


#include <async_writer.h>
#include <global.h>
#include <string.h>

namespace dgc {

AsyncWriter::AsyncWriter()
{
  fd_ = -1;
}

AsyncWriter::~AsyncWriter()
{
  if (fd_ >= 0) 
    Close();
}

int AsyncWriter::Open(const char* filename, unsigned int num_buffers,
		      unsigned int buffer_size)
{
  unsigned int i;

  if (fd_ != -1) {
    dgc_fwarning(__PRETTY_FUNCTION__, "Only one file can be opened at a time.");
    return -1;
  }

  fd_ = ::open64(filename, 
		 O_CREAT | O_TRUNC | O_RDWR | O_LARGEFILE | O_APPEND,
		 S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH);
  if (fd_ < 0) 
    return -1;

  num_buffers_ = num_buffers;
  buffer_size_ = buffer_size;

  current_buffer_ = 0;
  unwritten_bytes_ = 0;
  
  aio_.resize(num_buffers_);
  occupied_.resize(num_buffers_);
  buffer_.resize(num_buffers_);
  for (i = 0; i < num_buffers_; i++) {
    buffer_[i] = new unsigned char[buffer_size_];
    occupied_[i] = false;
  }

  wait_count_ = 0;
  write_count_ = 0;
  return 0;
}

int AsyncWriter::WriteBlock(int n, unsigned int len)
{
  aio_[n].aio_offset     = 0;
  aio_[n].aio_fildes     = fd_;
  aio_[n].aio_nbytes     = len;
  aio_[n].aio_buf        = buffer_[n];
  aio_[n].aio_lio_opcode = 0;
  aio_[n].aio_reqprio    = 0;
  aio_[n].aio_sigevent.sigev_notify = SIGEV_NONE;
  aio_[n].aio_sigevent.sigev_signo = 0;
  aio_[n].aio_sigevent.sigev_value.sival_int = 0;
  occupied_[n]           = true;
  return aio_write64(&(aio_[n]));
}

void AsyncWriter::Close(void)
{
  aiocb64 *cblist[1];
  unsigned int i;

  // write any leftover bytes
  if (unwritten_bytes_ > 0) {
    WriteBlock(current_buffer_, unwritten_bytes_);
    current_buffer_ = (current_buffer_ + 1) % num_buffers_;
    unwritten_bytes_ = 0;
  }

  // wait until each active write is complete
  for (i = 0; i < num_buffers_; i++)
    if (occupied_[i]) {
      cblist[0] = &(aio_[i]);
      aio_suspend64(cblist, 1, NULL);
    }
  ::close(fd_);
  fd_ = -1;
  
  for (i = 0; i < num_buffers_; i++)
    delete [] buffer_[i];
}

int AsyncWriter::Write(unsigned int len, unsigned char *data) 
{
  unsigned char *ptr = data;
  unsigned int qlen = len;
  aiocb64 *cblist[1];
  int sz;

  write_count_++;
  if (fd_ == -1) {
    dgc_warning("File has not been opened.");
    return -1;
  }

  while (qlen > 0) {
    if (occupied_[current_buffer_]) {
      // check to see if the current buffer has completed its last write
      // if it has, mark it as unoccupied and use it
      if (aio_error64(&(aio_[current_buffer_])) == 0) {
	if (aio_return64(&(aio_[current_buffer_])) < 0) {
	  dgc_ferror(__PRETTY_FUNCTION__, "Previous async write returned error.\n");
	}
	occupied_[current_buffer_] = false;
      } else {
	// otherwise, wait for this read to complete and then use the buffer
	cblist[0] = &(aio_[current_buffer_]);
	aio_suspend64(cblist, 1, NULL);
	wait_count_++;
	if (aio_error64(&(aio_[current_buffer_])) == 0) {
	  if (aio_return64(&(aio_[current_buffer_])) < 0) {
	    dgc_ferror(__PRETTY_FUNCTION__, "Previous async write returned error.\n");
	  }
	  occupied_[current_buffer_] = false;
	} else {
	  dgc_error("aio_suspend64 returned, but the buffer still isn't"
		    " ready.  This should never happen.");
	}
      }
    } 

    // break larger writes into buffer chunk size writes 
    if (unwritten_bytes_ + qlen >= buffer_size_) {
      sz = buffer_size_ - unwritten_bytes_;
      memcpy(buffer_[current_buffer_] + unwritten_bytes_, ptr, sz); 
      ptr  += sz;
      qlen -= sz;
      WriteBlock(current_buffer_, buffer_size_);
      current_buffer_ = (current_buffer_ + 1) % num_buffers_;
      unwritten_bytes_ = 0;
    } else {
      // wait to write leftover until next complete buffer
      memcpy(buffer_[current_buffer_] + unwritten_bytes_, ptr, qlen); 
      unwritten_bytes_ += qlen;
      qlen = 0;
    }
  }
  return 0;
}

}
