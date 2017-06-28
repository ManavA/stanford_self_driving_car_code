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

#include <paw2InternalData.h>
#include <graphics.h>

namespace drc = driving_common;

namespace vlr {

Paw2InternalData::Paw2InternalData(Paw2Gui& gui) :
                      gui_(gui), subscribed_(false), update_data_(true), update_thread_(NULL), frame_buffer_(0), fb_texture_(0),
                      fb_width_(0), fb_height_(0), fbo_init_done_(false), smoothed_mission_points_buf_(NULL),
                      smoothed_mission_points_buf_size_(0), road_map_(NULL) {

}

Paw2InternalData::~Paw2InternalData() {
  if(road_map_) {
    delete road_map_;
  }
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
//  gui_.ui.internalDataGlView->setDisplayMode(vlr::MODE_3D);

}


///////////////////////////////////////////////////////////////////////////////
// GLU_TESS CALLBACKS
///////////////////////////////////////////////////////////////////////////////
  void Paw2InternalData::tessBeginCB(GLenum which) {glBegin(which);}

  void Paw2InternalData::tessEndCB() {glEnd();}

  void Paw2InternalData::tessVertexCB(const GLvoid *data) {
      // cast back to double type
      const GLdouble *ptr = (const GLdouble*)data;

      glVertex3dv(ptr);
    }

/////////////////////////////////////////////////////////////////////
  void Paw2InternalData::tessCombineCB(const GLdouble newVertex[3], const GLdouble *neighborVertex[4],
                              const GLfloat neighborWeight[4], GLdouble **outData)
  {
  }

  void Paw2InternalData::tessErrorCB(GLenum errorCode) {
    const GLubyte *errorStr;

    errorStr = gluErrorString(errorCode);
    std::cout << "[ERROR]: " << errorStr << std::endl;
  }


void Paw2InternalData::draw(const std::vector<CurvePoint>& center_line, int32_t width, int32_t height,
                            double res, const drc::GlobalPose& pose, double timestamp) {

  if(center_line.empty()) {return;}

  boost::unique_lock < boost::mutex > lock(mutex_);

  const_cast<DisplayGL*>(gui_.ui.internalDataGlView->glWidget())->makeCurrent();

  if(!fbo_init_done_) {
    fb_scale_=1.0/res;
    fb_width_=width;//world_width_*fb_scale_;
    fb_height_=height;//world_height_*fb_scale_;

    road_map_ = new vlr::Image<uint8_t>(fb_width_, fb_height_, 1, vlr::ImageBase::CS_GRAY);

//    printf("w: %u, h: %u\n", fb_width_, fb_height_);
    initFrameBufferObject();
    fbo_init_done_=true;
    }

  timestamp_ = timestamp;
  pose_ = pose;
  centerLine2FrameBuffer(center_line, pose_.utmX(), pose_.utmY(), *road_map_);
  glClearColor(0.0, 0.2, 0.3, 1.0);
  gui_.ui.internalDataGlView->updateImage(*road_map_);
}

void Paw2InternalData::centerLine2FrameBuffer(const std::vector<CurvePoint>& center_line,
                                              double center_x, double center_y, vlr::Image<uint8_t>& res) {

  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, frame_buffer_);

  glPushAttrib(GL_TRANSFORM_BIT | GL_VIEWPORT_BIT);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix(); glLoadIdentity();
  glOrtho(0., fb_width_, 0., fb_height_, -1, 1);

  double x1=0, y1=0;
  double x2=(double)fb_width_;
  double y2=(double)fb_height_;

  glViewport(x1, y1, x2, y2);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix(); glLoadIdentity();

  glDisable(GL_DEPTH_TEST);
//  glEnable(GL_TEXTURE_2D);
//  glEnable(GL_TEXTURE_RECTANGLE_ARB);

//  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, fb_texture_);

  glClearColor(0.0, 0.0, 0.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  if(smoothed_mission_points_buf_size_<center_line.size()) {
    if(smoothed_mission_points_buf_) {delete[] smoothed_mission_points_buf_;}
    smoothed_mission_points_buf_ = new double[6*center_line.size()];
    smoothed_mission_points_buf_size_ = center_line.size();
  }


  uint32_t i3=0;
    for(uint32_t i=0; i<center_line.size(); i+=2) {
    double theta = center_line[i].theta;
    smoothed_mission_points_buf_[i3++]= 0.5*fb_width_ - fb_scale_*(center_line[i].x - center_x-1.7*sin(theta));
    smoothed_mission_points_buf_[i3++]= 0.5*fb_height_ - fb_scale_*(center_line[i].y - center_y+1.7*cos(theta));
    smoothed_mission_points_buf_[i3++]=0;
  }

    for(int32_t i=center_line.size()-1; i>=0; i-=2) {
    double theta = center_line[i].theta;
    smoothed_mission_points_buf_[i3++]= 0.5*fb_width_ - fb_scale_*(center_line[i].x - center_x+1.7*sin(theta));
    smoothed_mission_points_buf_[i3++]= 0.5*fb_height_ - fb_scale_*(center_line[i].y - center_y-1.7*cos(theta));
    smoothed_mission_points_buf_[i3++]=0;
  }

//  glLineWidth(3.0);
//  glEnable(GL_LINE_SMOOTH);
//  glColor3f(0.6, 0.0, 0.0);
//  glBegin(GL_LINE_STRIP);
//  std::vector<CurvePoint>::const_iterator clit = center_line.begin();
//  std::vector<CurvePoint>::const_iterator clit_end = --center_line.end();
//
//  for(; clit != clit_end; clit++) {
//    glVertex3f(fb_scale_*((*clit).x - center_x)+0.5*fb_width_, fb_scale_*((*clit).y - center_y)+0.5*fb_height_, 0.0);
//  }
//  glEnd();
//  glDisable(GL_LINE_SMOOTH);

  GLUtesselator * tess = gluNewTess(); // create a tessellator
  if(!tess) return;

  // register callback functions
  gluTessCallback(tess, GLU_TESS_BEGIN, (void (*)())Paw2InternalData::tessBeginCB);
  gluTessCallback(tess, GLU_TESS_END, (void (*)())Paw2InternalData::tessEndCB);
  gluTessCallback(tess, GLU_TESS_ERROR, (void (*)())Paw2InternalData::tessErrorCB);
  gluTessCallback(tess, GLU_TESS_VERTEX, (void (*)())Paw2InternalData::tessVertexCB);
  gluTessCallback(tess, GLU_TESS_COMBINE, (void (*)())Paw2InternalData::tessCombineCB);

  // tessellate and compile a concave quad into display list
  glColor3f(1,.1,1);
  gluTessBeginPolygon(tess, 0);                       // with NULL data
      gluTessBeginContour(tess);                      // outer quad
        for (uint32_t i = 0; i < center_line.size(); i++) {
        gluTessVertex(tess, &smoothed_mission_points_buf_[3*i], &smoothed_mission_points_buf_[3*i]);
      }
      gluTessEndContour(tess);
  gluTessEndPolygon(tess);

  gluDeleteTess(tess);        // delete after tessellation
  glEnable(GL_DEPTH_TEST);
////  glDisable(GL_TEXTURE_2D);
////  glDisable(GL_TEXTURE_RECTANGLE_ARB);

  glPopMatrix();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glPopAttrib();

  glReadPixels(0, 0, fb_width_, fb_height_, GL_LUMINANCE, GL_UNSIGNED_BYTE, res.data());

  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

void Paw2InternalData::initFrameBufferObject() {
glGenFramebuffersEXT(1, &frame_buffer_);
glGenTextures(1, &fb_texture_);

glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, frame_buffer_);
glPixelStorei(GL_PACK_ALIGNMENT, 1);
glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

glBindTexture(GL_TEXTURE_RECTANGLE_ARB, fb_texture_);
glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, fb_width_, fb_height_, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, NULL); // Needed?!?

glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, fb_texture_, 0);

GLenum glerr = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
if (glerr != GL_FRAMEBUFFER_COMPLETE_EXT) {
  std::stringstream s;
  s << "GL Error " << (uint32_t)glerr << ": Cannot create frame buffer object";
  throw VLRException(s.str());
  }

glViewport(0, 0, fb_width_, fb_height_);

glClearColor(0.0,0.0,0.0,0.0);
//  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT|GL_ACCUM_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);

glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

void Paw2InternalData::deleteFrameBufferObject() {
glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); // Make the window the target
glDeleteTextures(1, &fb_texture_);
glDeleteFramebuffersEXT(1, &frame_buffer_);
}

} // namespace vlr
