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


#ifndef IMAGERY_STRUCTS_H_
#define IMAGERY_STRUCTS_H_

#include <inttypes.h>
#include <textures.h>
#include <opencv2/core/core.hpp>

namespace vlr {

typedef enum { UNINITIALIZED, REQUESTED, READY } ImageState_t;

typedef struct {
  int32_t type, x, y, res, zone;
  char zone_letter;
} TileId;

class TimedImage {
public:
  TimedImage() : state(UNINITIALIZED), image(NULL),
                 exists(false), needs_sync(false), grayscale(false),
                 texture(32, 32, 1024, 0), last_reference(0) {
  }

  virtual ~TimedImage() {}

//private:
//  TimedImage(const TimedImage&) {}
//  TimedImage& operator = (const TimedImage& other) {return *this;}

public:
  TileId id;
  ImageState_t state;
  cv::Mat* image;
  bool exists, needs_sync, grayscale;
  Texture texture;
  uint64_t last_reference;
};

}  // namespace vlr

#endif
