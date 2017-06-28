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


#include <global.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <vlrException.h>
#include <vlrSocket.h>
#include <imagery.h>

#include <imageryServer.h>

namespace vlr {

ImageryServer::ImageryServer() {

}

ImageryServer::~ImageryServer() {
  socket_.disconnect();
}

bool ImageryServer::connect(const std::string& host, int32_t port) {
  return socket_.connect(host, port);
}

void ImageryServer::disconnect() {
  socket_.disconnect();
}

void ImageryServer::sendImageryCommand(int imagery_type, int imagery_resolution, int imagery_x, int imagery_y, int imagery_zone) {
  uint8_t command[20];

  command[0] = imagery_type;
  *((int *) (command + 1)) = imagery_x;
  *((int *) (command + 5)) = imagery_y;
  *((int *) (command + 9)) = imagery_resolution;
  command[13] = imagery_zone;
  command[14] = 0;
  if(socket_.writen(command, 15, -1) < 15) {
    throw VLRException("Failed to send command.");
  }
}

bool ImageryServer::hasJPGHeader(uint8_t* buffer, int buffer_length) {
  if(buffer_length > 4 && buffer[0] == 0xFF && buffer[1] == 0xD8 &&
      buffer[2] == 0xFF && buffer[3] == 0xE0) {
    return true;
  }
  return false;
}

bool ImageryServer::hasGIFHeader(uint8_t* buffer, int buffer_length) {
  if(buffer_length > 3 && buffer[0] == 'G' && buffer[1] == 'I' &&
      buffer[2] == 'F') {
    return true;
  }
  return false;
}

cv::Mat* ImageryServer::getImage(const std::string& server_name, int32_t server_port, TileId id) {

  fprintf(stderr, "img %d %d %d %d %c\n", id.type, id.x, id.y, id.res, id.zone);

  if (!socket_.connected()) {
    socket_.connect(server_name, server_port);
  }

  sendImageryCommand(id.type, id.res, id.x, id.y, id.zone);

  // get image length
  off64_t size;
  int err = socket_.readn(&size, sizeof(off64_t), -1);
  if (err <= 0) {
    disconnect();
    return NULL;
  }

  if (size > 0) {
    // get image
    std::vector<uint8_t> buffer;
    buffer.resize(size + 1);

    err = socket_.readn(&buffer[0], size, -1);
    if (err < 0) {
      disconnect();
      return NULL;
    }

    cv::Mat* res = new cv::Mat;
    *res = cv::imdecode(cv::Mat(buffer, false), CV_LOAD_IMAGE_ANYCOLOR);
    return res;
  }

  return NULL;
}

} // namespace vlr
