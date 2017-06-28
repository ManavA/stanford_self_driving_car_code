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


#ifndef VLR_IMAGERY_SERVER_H_
#define VLR_IMAGERY_SERVER_H_

#include <opencv2/core/core.hpp>
#include <imageryStructs.h>
//#include <imagery.h>
#include <vlrSocket.h>

namespace vlr {

class ImageryServer {
 public:
  ImageryServer();
  virtual ~ImageryServer();
  bool connect(const std::string& host, int32_t port);
  void disconnect();
  cv::Mat* getImage(const std::string& server_name, int32_t server_port, TileId id);

private:
  void sendImageryCommand(int imagery_type, int imagery_resolution, int imagery_x, int imagery_y, int imagery_zone);
  bool hasJPGHeader(uint8_t* buffer, int buffer_length);
  bool hasGIFHeader(uint8_t* buffer, int buffer_length);

private:
  vlr::Socket socket_;
};

} // namespace vlr

#endif // VLR_IMAGERY_SERVER_H_
