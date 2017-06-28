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


#ifndef PAW2CAMERA_H_
#define PAW2CAMERA_H_

#include <boost/thread.hpp>

#include <global.h>
#include <transform.h>

//#include <camera_interface.h>

#include <vlrImage.h>

#include <paw2_gui.h>

namespace vlr {

class Paw2Camera {
public:
	Paw2Camera(int32_t camera_id, Paw2Gui& gui, uint32_t queue_size_=5);
	virtual ~Paw2Camera();

  void subscribe();
  void unsubscribe();
  boost::mutex& mutex() {return mutex_;}
  void initializeGL();
  void draw();
  inline bool updateData() const {return update_data_;}
  inline void updateData(bool update) {update_data_=update;}
//  inline ColorMode_t colorMode() const {return color_mode_;}
//  inline void colorMode(ColorMode_t mode) {color_mode_=mode;}
//  inline DisplayStyle_t displayStyle() const {return display_style_;}
//  inline void displayStyle(DisplayStyle_t style) {display_style_=style;}

private:
  void update();

private:
	Paw2Gui& gui_;
	uint32_t queue_size_;
	boost::mutex mutex_;
//  dgc::CameraInterface* camera_interface_;
//  std::map<double, dgc::CameraImage*> image_queue_;

  int32_t camera_id_;
  bool subscribed_;
  bool update_data_;

  boost::thread* update_thread_;
};

} // namespace vlr

#endif // PAW2CAMERA_H_
