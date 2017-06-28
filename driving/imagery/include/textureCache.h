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


#ifndef IMAGE_CACHE_H_
#define IMAGE_CACHE_H_

#include <textures.h>
#include <imageryStructs.h>
#include <imageryServer.h>
//#include "imagery.h"

namespace vlr {

class TextureCache {
public:
  TextureCache(const std::string& imagery_root, uint64_t max_images = 1000); // size is limited by glGenTextures()
  virtual ~TextureCache();

  uint64_t lookup(TileId id);
  void resetCounters();

  bool syncCache();
  Texture* get(const std::string& detected_subdir, TileId& id, int priority, ImageState_t& state);

  void stopCaching() {stop_thread_ = true;}

  bool lastGrayscale() {return last_grayscale_;}
  void setVersion(int version_num) {version_num_ = version_num_;}
  int& version() {return version_num_;}

private:
  bool sameId(TileId& id1, TileId& id2);
  void syncTexture(uint64_t i);
  void* loaderThread();

private:
  class CompareReferences {
  public:
    CompareReferences(std::vector<TimedImage*>& images) : images_(images) {}
    bool operator() (uint64_t idx1, uint64_t idx2) {
      return images_[idx1]->last_reference < images_[idx2]->last_reference;
    }
    std::vector<TimedImage*>& images_;
  } comp_refs_;

//  int compare_refs(const void *a, const void *b) {
//    int ai, bi, ref1, ref2;
//
//    ai = *((int *) a);
//    bi = *((int *) b);
//    ref1 = image_[ai].last_reference;
//    ref2 = image_[bi].last_reference;
//
//    if (ref1 < ref2) return -1;
//    else if (ref1 > ref2) return 1;
//    else return 0;
//  }

  int version_num_;
  uint64_t max_images_;
  std::string imagery_root_;
  std::string detected_subdir_;

  bool last_grayscale_;
  uint64_t current_reference_;
  std::vector<TimedImage*> image_;

  pthread_t loader_thread_id_;
  bool stop_thread_;

  std::vector<uint8_t> curl_buf_;
  ImageryServer img_server_;
};

} // namespace vlr

#endif
