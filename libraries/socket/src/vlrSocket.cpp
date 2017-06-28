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
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <arpa/inet.h>

#include <cmath>

#include <vlrSocket.h>
#include <vlrException.h>

extern "C" {
void bzero(void *s, size_t n);
void bcopy(const void *src, void *dest, size_t n);
}

namespace vlr {

Socket::Socket() :
  connected_(false) {
}

Socket::~Socket() {
  if(connected_) {disconnect();}
}

bool Socket::connect(const std::string& host, int32_t port) {
  struct hostent *addr;
  unsigned long addr_tmp;
  struct sockaddr_in servaddr;

  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_ < 0) {return false;}
  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  if (atoi(host.c_str()) > 0) {servaddr.sin_addr.s_addr = inet_addr(host.c_str());}
  else {
    if ((addr = gethostbyname(host.c_str())) == NULL) {return -1;}
    bcopy(addr->h_addr, (char *) &addr_tmp, addr->h_length);
    servaddr.sin_addr.s_addr = addr_tmp;
  }
  servaddr.sin_port = htons(port);
  if (::connect(socket_, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0) {
    close(socket_);
    return false;
  }

  return true;
}

void Socket::disconnect() {
  if(connected_) {
    close(socket_);
    connected_ = false;
  }
}

int Socket::writen(const void* vptr, int n, double timeout) {

  if (!connected_) {
    throw VLRException("Not connected.");
  }

  ssize_t nwritten;
  fd_set writesock, errsock;
  struct timeval t;
  int result;

  const char* ptr = (const char*) vptr;
  size_t nleft = (size_t) n;

  while (nleft > 0) {
    FD_ZERO(&writesock);
    FD_ZERO(&errsock);
    FD_SET(socket_, &writesock);
    FD_SET(socket_, &errsock);
    if (timeout == -1.0) {
      if (select(socket_ + 1, NULL, &writesock, &errsock, NULL) < 0) {
        if (errno == EINTR) nwritten = 0;
        else return -1;
      }
    }
    else {
      t.tv_sec = (int) floor(timeout);
      t.tv_usec = (int) floor((timeout - t.tv_sec) * 1e6);
      result = select(socket_ + 1, NULL, &writesock, &errsock, &t);
      if (result < 0) {
        if (errno == EINTR) {
          nwritten = 0;
        }
        else {
          return -1;
        }
      }
      else if (result == 0) {
        return n - nleft;
      }
    }
    if (FD_ISSET(socket_, &errsock)) {
      return -1;
    }
    if ((nwritten = write(socket_, ptr, nleft)) <= 0) {
      if (errno == EINTR) nwritten = 0;
      else return -1;
    }
    nleft -= nwritten;
    ptr += nwritten;
  }
  return n;
}

int Socket::readn(void *vptr, int n, double timeout) {

  if (!connected_) {
    throw VLRException("not connected.");
  }

  ssize_t nread;
  fd_set readsock, errsock;
  struct timeval t;
  int result;

  char* ptr = (char*) vptr;
  size_t nleft = (size_t) n;
  while (nleft > 0) {
    FD_ZERO(&readsock);
    FD_ZERO(&errsock);
    FD_SET(socket_, &readsock);
    FD_SET(socket_, &errsock);
    if (timeout == -1.0) {
      if (select(socket_ + 1, &readsock, NULL, &errsock, NULL) < 0) {
        if (errno == EINTR) nread = 0;
        else return -1;
      }
    }
    else {
      t.tv_sec = (int) floor(timeout);
      t.tv_usec = (int) floor((timeout - t.tv_sec) * 1e6);
      result = select(socket_ + 1, &readsock, NULL, &errsock, &t);
      if (result < 0) {
        if (errno == EINTR) {
          nread = 0;
        }
        else {
          return -1;
        }
      }
      else if (result == 0) {
        return n - nleft;
      }
    }
    if (FD_ISSET(socket_, &errsock)) return -1;
    if ((nread = read(socket_, ptr, nleft)) < 0) {
      if (errno == EINTR) nread = 0;
      else return -1;
    }
    else if (nread == 0) return -1;
    nleft -= nread;
    ptr += nread;
  }
  return n;
}

int Socket::writeString(const char* s) {

  if (!connected_) {
    throw VLRException("Not connected.");
  }

  size_t length = strlen(s);

  while (length > 0 && (s[length - 1] == '\r' || s[length - 1] == '\n')) {
    length--;
  }

  int n = writen(s, length, -1);
  if (n < 0) {return n;}

  char c = '\r';
  int return_val = writen(&c, 1, -1);
  if (return_val < 0) {return return_val;}

  c = '\n';
  return_val = writen(&c, 1, -1);
  if (return_val < 0) {return return_val;}

  return n;
}

int Socket::printf(const char* fmt, ...) {

  if (!connected_) {
    throw VLRException("Not connected.");
  }

  va_list args;

  va_start(args, fmt);
  int64_t n = vsnprintf(printf_buf_, 4096, fmt, args);
  va_end(args);
  if (n > -1 && n < 4096) {
    return writeString(printf_buf_);
  }
  return -1;
}

int64_t Socket::bytesAvailable() {
  if (!connected_) {
    throw VLRException("Not connected.");
  }

  int64_t available = 0;

  if (ioctl(socket_, FIONREAD, &available) == 0) {return available;}
  else {return -1;}
}

void Socket::clearInputBuffer() {

  if (!connected_) {
    throw VLRException("Not connected.");
  }

  int64_t val = bytesAvailable();
  if (val > 0) {
    uint8_t* buffer = new uint8_t[val];
    ssize_t dummy = read(socket_, buffer, val);
    delete[] buffer;
  }
}

} // namespace vlr
