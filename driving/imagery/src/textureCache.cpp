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


#include <iostream>
#include <global.h>
#include <textureCache.h>
#include <imageryServer.h>
#include <imagery_proj.h>

namespace vlr {

TextureCache::TextureCache(const std::string& imagery_root, uint64_t max_images) :
                       comp_refs_(image_), version_num_(0), max_images_(max_images), imagery_root_(imagery_root),
                       last_grayscale_(false), current_reference_(0), stop_thread_(false) {

  for (uint64_t i = 0; i < max_images_; i++) {
    image_.push_back(new TimedImage);
  }

  pthread_create(&loader_thread_id_, NULL, threadCBWrapper<TextureCache, &TextureCache::loaderThread>, this);
}

TextureCache::~TextureCache() {
  stop_thread_ = true;
  pthread_join(loader_thread_id_, NULL);

  for (uint64_t i = 0; i < max_images_; i++) {
    delete image_[i];
  }
}

bool TextureCache::sameId(TileId& id1, TileId& id2) {
  if (id1.type != id2.type) return false;
  if (id1.x != id2.x) return false;
  if (id1.y != id2.y) return false;
  if (id1.res != id2.res) return false;
  if (id1.zone != id2.zone) return false;
  return true;
}

uint64_t TextureCache::lookup(TileId id) {

  /* locate image in cache */
  bool found = false;
  uint64_t which = 0;
  for (uint64_t i = 0; i < max_images_; i++) {
    if (image_[i]->state != UNINITIALIZED && sameId(image_[i]->id, id)) {
      which = i;
      found = true;
      break;
    }
  }

  if (found) {
      // image is in cache
      image_[which]->last_reference = current_reference_;
      current_reference_++;
      return which;
    }

  // image is not in cache
  // find unused or oldest used image; TODO: optimize search
  for (uint64_t i = 0; i < max_images_; i++) {
    if (image_[i]->state == UNINITIALIZED) {
      found = true;
      which = i;
      break;
    }
    else if (image_[i]->state == READY && (!found || (found && image_[i]->last_reference < image_[which]->last_reference))) {
      found = true;
      which = i;
    }
  }

  if (found) {
    if (image_[which]->state == READY) {
      if (image_[which]->image != NULL) {
        delete image_[which]->image;
        image_[which]->image = NULL;
      }
    }
    image_[which]->exists = true;
    image_[which]->last_reference = current_reference_;
    image_[which]->id = id;
    image_[which]->state = REQUESTED;
    current_reference_++;
    return which;
  }
  else {
    throw VLRException("Image list has no empty spots. This shouldn't happen.");
  }
}

void TextureCache::resetCounters() {
  static std::vector<int64_t> id;

  if (id.size() != max_images_) {
    id.resize(max_images_);
  }

  for (uint64_t i = 0; i < max_images_; i++) {id[i] = i;}
  sort(id.begin(), id.end(), comp_refs_);
  for (uint64_t i = 0; i < max_images_; i++) {
    image_[id[i]]->last_reference = i;
  }
  current_reference_ = max_images_ + 1;
}

void TextureCache::syncTexture(uint64_t i) {
  TimedImage& image = *image_[i];

  if (image.state == READY && image.needs_sync) {
    image.needs_sync = false;
    if (image.image == NULL) return;
    image.texture.updateFromImage(*image.image);
    delete image.image;
    image.image = NULL;
  }
}

bool TextureCache::syncCache() {
  static int32_t count = 0;

  if (count > 10) { // ?!?
    resetCounters();
    count = 0;
  }
  count++;

  bool needs_sync = false;
  for (uint64_t i = 0; i < max_images_; i++)
    if (image_[i]->needs_sync) {
      needs_sync = true;
      break;
    }

  if (needs_sync) {
    for (uint64_t i = 0; i < max_images_; i++) {
      if (image_[i]->needs_sync) {
        syncTexture(i);
      }
    }
    return true;
  }
  return false;
}


Texture* TextureCache::get(const std::string& detected_subdir, TileId& id, int priority, ImageState_t& state) {

//    std:: cout << "detected_subdir: " << detected_subdir << std::endl;
//  std:: cout << "imagery_root_: " << imagery_root_ << std::endl;
//  std:: cout << "Requested tile id with type " << id.type << "; x, y: " << id.x << ", " << id.y << "; res: " << id.res << "; zone: "<< id.zone << std::endl;

  if(!detected_subdir.empty() && detected_subdir_ != detected_subdir) {
    detected_subdir_ = detected_subdir;
  }

  uint64_t i = 0;
  try {
    i = lookup(id);
  }
  catch(vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
    state = UNINITIALIZED;
    return NULL;
  }

  if (priority && image_[i]->state == READY) {syncTexture(i);}

  image_[i]->last_reference = current_reference_;
  current_reference_++;

  if (image_[i]->state != UNINITIALIZED && image_[i]->exists) {
    state = image_[i]->state;
    last_grayscale_ = image_[i]->grayscale;
    return &image_[i]->texture;
  }

  state = UNINITIALIZED;
  return NULL;
}

size_t curl_callback(void* ptr, size_t size, size_t nmemb, void* data) {
  std::vector<uint8_t>* mem = static_cast<std::vector<uint8_t>*>(data);
  size_t realsize = size * nmemb;
  mem->resize(mem->size() + realsize + 1);    // +1 to have a final zero element
  memcpy(&((*mem)[mem->size()]), ptr, realsize);
  return realsize;
}

void* TextureCache::loaderThread() {
  char server_name[200], filename[200], whole_filename[200];
  int server_port;
  char *mark, *mark2;

#ifdef HAVE_LIBCURL
  CURLcode curl_return;

  /* init the curl session */
  curl_global_init(CURL_GLOBAL_ALL);
  CURL* curl_handle = curl_easy_init();
  curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");
  curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, curl_callback);
  curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)&curl_buf_);
  curl_easy_setopt(curl_handle, CURLOPT_CONNECTTIMEOUT, 1);
  curl_easy_setopt(curl_handle, CURLOPT_FAILONERROR, 1);
  curl_easy_setopt(curl_handle, CURLOPT_NOSIGNAL, 1);
#endif

  sleep(1); // ?!?
  while (!stop_thread_) {
    for (uint64_t i = 0; i < max_images_; i++) {
      if (image_[i]->state == REQUESTED) {
        if (version_num_ != 2) {
          if (image_[i]->id.type == Imagery::COLOR)      dgc_terra_color_tile_filename(image_[i]->id, filename, version_num_);
          else if (image_[i]->id.type == Imagery::TOPO)  dgc_terra_topo_tile_filename(image_[i]->id, filename, version_num_);
          else if (image_[i]->id.type == Imagery::LASER) dgc_laser_tile_filename(image_[i]->id, filename, version_num_);
          else if (image_[i]->id.type == Imagery::GSAT)  dgc_gmaps_tile_filename(image_[i]->id, filename);
          else if (image_[i]->id.type == Imagery::DARPA) dgc_darpa_tile_filename(image_[i]->id, filename, version_num_);
          else if (image_[i]->id.type == Imagery::BW)    dgc_terra_bw_tile_filename(image_[i]->id, filename);

          if (version_num_ == 0) {
            sprintf(whole_filename, "%s/%s/%s", imagery_root_.c_str(), detected_subdir_.c_str(), filename);
          }
          else if (version_num_ == 1) {
            sprintf(whole_filename, "%s/%s", imagery_root_.c_str(), filename);
          }
        }

        switch (version_num_) {

          case 0:
            if (dgc::dgc_file_exists(whole_filename)) {
              image_[i]->image = new cv::Mat;
              *image_[i]->image = cv::imread(whole_filename, CV_LOAD_IMAGE_ANYCOLOR);
            }
            else {
              image_[i]->image = NULL;
            }
            break;

          case 1:
#ifdef HAVE_LIBCURL
            /* handle http queries with libcurl */
            curl_easy_setopt(curl_handle, CURLOPT_URL, whole_filename);
            curl_return = curl_easy_perform(curl_handle);

            if(stop_thread_) {return NULL;} // ?!?

            if(curl_return == 0) {

            image_[i]->image = cv::imdecode(cv::Mat(curl_buf_, false), CV_LOAD_IMAGE_ANYCOLOR);
            }
            curl_buf_.clear();
#else
            image_[i]->image = new cv::Mat;
            *image_[i]->image = cv::imread(whole_filename, CV_LOAD_IMAGE_ANYCOLOR);
#endif
            break;

          case 2:
            strcpy(server_name, imagery_root_.c_str() + 6);
            if ((mark = strchr(server_name, ':')) != NULL) {
              server_port = strtol(mark + 1, &mark2, 10);
              *mark = '\0';
            }
            else {
              server_port = 3000;
              if ((mark = strchr(server_name, '/')) != NULL) *mark = '\0';
            }

            image_[i]->image = img_server_.getImage(server_name, server_port, image_[i]->id);
            break;
        }

        if (!image_[i]->image) {
          image_[i]->exists = false;
        }
        else {
          image_[i]->grayscale = (image_[i]->image->channels() == 1);
        }

        image_[i]->needs_sync = true;
        image_[i]->state = READY;
      }
    }
    usleep(10000);
  }

  return NULL;
}


} // namespace vlr
