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


#include <cmath>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#include <cv.h>
#include <global.h>
#include <vlrException.h>
//#include <camera_shm_interface.h>
#include <highgui.h>

#include <paw2Camera.h>
#include <graphics.h>

namespace vlr {

Paw2Camera::Paw2Camera(int32_t camera_id, Paw2Gui& gui, uint32_t queue_size) :
                      gui_(gui), queue_size_(queue_size),
                      //camera_interface_(NULL),
                      camera_id_(camera_id),
                      subscribed_(false), update_data_(false) {

  if(queue_size == 0) {
    throw VLRException("Paw2Camera: Cannot create image queue with size 0.");
  }

//  camera_interface_ = new dgc::CameraShmInterface;
}

Paw2Camera::~Paw2Camera() {
//  while(!image_queue_.empty()) {
//    dgc::CameraImage* oldest_image = image_queue_.begin()->second;
//    image_queue_.erase(image_queue_.begin());
//    delete oldest_image;
//  }
}

void Paw2Camera::subscribe() {
//  if(camera_interface_->CreateClient(camera_id_) < 0) {
//    std::stringstream s;
//    s << "Could not connect to camera " << camera_id_ << " interface.";
//    throw Exception(s.str());
//  }

subscribed_=true;
update_thread_ = new boost::thread(boost::bind(&Paw2Camera::update, this));
}

void Paw2Camera::unsubscribe() {
  if(!subscribed_) {return;}
  subscribed_=false;
  if(update_thread_) {
    update_thread_->join();
    update_thread_=NULL;
  }
  // TODO: DeleteClient() doesn't exist :-(
}

void Paw2Camera::update() {
  return;
while (subscribed_) {
    if (update_data_) {
//      vlr::Lock lock(mutex_);
//      while (camera_interface_->ImagesWaiting()) {
//        dgc::CameraImage* big_image = new dgc::CameraImage;
//        if (camera_interface_->ReadCurrentImage(big_image) < 0) {
//          delete big_image;
//          throw Exception("Image read failed.");
//        }
//        dgc::CameraImage* image = new dgc::CameraImage;
//        *image=*big_image;
//        uint32_t depth = image->info.depth/8;
//        uint32_t channels = image->info.channels;
//        image->info.width/=2;
//        image->info.height/=2;
//        image->info.padded_width/=2;
//        image->data = new uint8_t[image->info.padded_width*image->info.height*channels*depth];
//        IppiRect src_roi = {0, 0, big_image->info.width, big_image->info.height};
//        IppiRect dst_roi = {0, 0, image->info.width, image->info.height};
//        IppiSize src_size = {big_image->info.width, big_image->info.height};
//        int32_t tbuf_size;
//        ippiResizeGetBufSize(src_roi, dst_roi, channels, IPPI_INTER_CUBIC, &tbuf_size);
//        uint8_t* tbuf = new uint8_t[tbuf_size];
//        switch(image->info.format) {
//        case dgc::DGC_RGB8_FMT:
//          ippiResizeSqrPixel_8u_C3R(big_image->data, src_size, 3*big_image->info.padded_width, src_roi,
//                                    image->data, 3*image->info.padded_width, dst_roi, 0.5, 0.5, 0, 0, IPPI_INTER_CUBIC, tbuf);
//          break;
//
//        case dgc::DGC_GRAY8_FMT:
//          ippiResizeSqrPixel_8u_C1R(big_image->data, src_size, big_image->info.padded_width, src_roi,
//                                    image->data, image->info.padded_width, dst_roi, 0.5, 0.5, 0, 0, IPPI_INTER_CUBIC, tbuf);
//          break;
//
//        case dgc::DGC_GRAY10L_FMT:
//        case dgc::DGC_GRAY10B_FMT:
//        case dgc::DGC_GRAY12L_FMT:
//        case dgc::DGC_GRAY12B_FMT:
//        case dgc::DGC_GRAY16L_FMT:
//        case dgc::DGC_GRAY16B_FMT:
//          ippiResizeSqrPixel_16s_C3R((int16_t*)big_image->data, src_size, 2*big_image->info.padded_width, src_roi,
//                                    (int16_t*)image->data, 2*image->info.padded_width, dst_roi, 0.5, 0.5, 0, 0, IPPI_INTER_CUBIC, tbuf);
//          break;
//        default:
//          throw Exception("Paw2Camera::update(): Unknown image format.");
//        }
//        delete [] tbuf;
//        if(image_queue_.size() == queue_size_) {
//          dgc::CameraImage* oldest_image = image_queue_.begin()->second;
//          image_queue_.erase(image_queue_.begin());
//          delete oldest_image;
//        }
//        image_queue_.insert(std::make_pair(image->timestamp, image));
//
//
//      lock.unlock();
//      draw();
//      //gui_.requestCameraGlRedraw();
//      }
    }
  usleep(1000); // TODO:...
  }
}

void Paw2Camera::draw() {
//  vlr::Lock lock(mutex_);
//
//  if(image_queue_.empty()) {
//    throw Exception("Paw2Camera::draw(): Cannot draw, image queue is empty.");
//  }
//
//  dgc::CameraImage* image = image_queue_.rbegin()->second;
//  if(image->info.format == dgc::DGC_RGB8_FMT) {
//    vlr::Image<uint8_t> timg(image->info.width, image->info.height, image->info.channels, vlr::ImageBase::CS_RGB);
//    vlr::cpReorganize<uint8_t, vlr::COLORORG_RGB> reorg;
//    reorg.chunky2Planar(image->data, image->info.padded_width, timg);
//    gui_.ui.cameraGlView->updateImage(timg);
//  }
//  else {
//    gui_.ui.cameraGlView->updateImage(image->data, image->info.width, image->info.height, image->info.channels, image->info.padded_width, vlr::ImageBase::CS_GRAY);
//  }
}

} // namespace vlr
