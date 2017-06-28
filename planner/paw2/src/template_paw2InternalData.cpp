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

#include <global.h>

#include "paw2InternalData.h"
#include "graphics.h"

namespace vlr {

Paw2InternalData::Paw2InternalData(Paw2Gui& gui) :
                      gui_(gui), subscribed_(false), update_data_(true) {

}

Paw2InternalData::~Paw2InternalData() {
}

void Paw2InternalData::subscribe() {
//subscribed_=true;
//update_thread_ = new boost::thread(boost::bind(&Paw2InternalData::update, this));
}

void Paw2InternalData::unsubscribe() {
//  if(!subscribed_) {return;}
//  subscribed_=false;
//  if(update_thread_) {
//    update_thread_->join();
//    update_thread_=NULL;
//  }
//  // TODO: DeleteClient() doesn't exist :-(
}

void Paw2InternalData::update() {
//  while (subscribed_) {
//    if (update_data_) {
//      vlr::Lock lock(mutex_);
//      lock.unlock();
//      draw();
//      //gui_.requestCameraGlRedraw();
//      }
//    }
//  usleep(1000); // TODO:...
//  }
}

void Paw2InternalData::initializeGL() {
  const_cast<vlr::DisplayGL*>(gui_.ui.internalDataGlView->glWidget())->makeCurrent();
  gui_.ui.internalDataGlView->setDisplayMode(vlr::MODE_3D);

}

void Paw2InternalData::draw() {
  vlr::Lock lock(mutex_);

  uint32_t width=500, height=500;
  vlr::Image<uint8_t> timg(width, height, 1, width, true, vlr::ImageBase::CS_GRAY);
  for(uint32_t j=0; j<height; j++) {
    for(uint32_t i=0; i<width; i++) {
      timg.data()[j*width+i]=200*sin(i/50.);
    }
  }
  gui_.ui.internalDataGlView->updateImage(timg);
}


} // namespace vlr
