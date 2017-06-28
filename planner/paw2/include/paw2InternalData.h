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


#ifndef PAW2_INTERNAL_DATA_
#define PAW2_INTERNAL_DATA_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <global.h>
#include <transform.h>

#include <boost/signals.hpp>
#include <tf/tf.h>

#include <display.h>

#include <paw2_gui.h>

namespace vlr {

class Paw2InternalData {
public:
	Paw2InternalData(Paw2Gui& gui);
	virtual ~Paw2InternalData();

  void subscribe();
  void unsubscribe();
  void update();
  boost::mutex& mutex() {return mutex_;}
  void initializeGL();
  void draw(const std::vector<CurvePoint>& center_line, int32_t width, int32_t height, double res, const driving_common::GlobalPose& pose, double timestamp);
  inline uint32_t width() {return fb_width_;}
  inline uint32_t height() {return fb_height_;}
  inline double resolution() {return 1.0/fb_scale_;}
  inline const vlr::Image<uint8_t>* roadMap() {return road_map_;}
  inline void roadMap(vlr::Image<uint8_t>*& road_map, driving_common::GlobalPose& pose, double& timestamp) {
    road_map = road_map_;
    pose = pose_;
    timestamp = timestamp;
  }

private:
  void initFrameBufferObject();
  void deleteFrameBufferObject();
  void centerLine2FrameBuffer(const std::vector<CurvePoint>& center_line, double center_x, double center_y, vlr::Image<uint8_t>& res);

    // Tesselator callbacks
  static void tessBeginCB(GLenum which);
  static void tessEndCB();
  static void tessVertexCB(const GLvoid* data);
  static void tessErrorCB(GLenum errorCode);
  static void tessCombineCB(const GLdouble newVertex[3], const GLdouble *neighborVertex[4],
                              const GLfloat neighborWeight[4], GLdouble **outData);

private:
	Paw2Gui& gui_;
  bool subscribed_;
  bool update_data_;
  boost::thread* update_thread_;
	boost::mutex mutex_;

  vlr::Image<uint8_t>* bev_image_;


    // offscreen rendering
  GLuint frame_buffer_;
  GLuint fb_texture_;

  uint32_t fb_width_, fb_height_;
  double fb_scale_;
  double world_width_, world_height_;
  bool fbo_init_done_;

  double* smoothed_mission_points_buf_;  // buffer for OpenGL Bezier visualization
  uint32_t smoothed_mission_points_buf_size_;
  vlr::Image<uint8_t>* road_map_;
  driving_common::GlobalPose pose_;
  double timestamp_;
};

} // namespace vlr

#endif // PAW2_INTERNAL_DATA_
