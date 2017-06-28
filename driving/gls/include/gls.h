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


#ifndef GLS_H_
#define GLS_H_

#include <inttypes.h>
#include <string>
#include <ros/ros.h>
#include <driving_common/GLSOverlay.h>

namespace vlr {

class GLS: public driving_common::GLSOverlay {

public:
  GLS(const std::string& gls_name);
  virtual ~GLS();

  void clear();
  void begin(uint8_t drawing_type);
  void end();
  void vertex3f(double x, double y, double z);
  void vertex2f(double x, double y);
  void color3f(double r, double g, double b);
  void color4f(double r, double g, double b, double a);

  void pushMatrix();

  void popMatrix();

  void rotatef(double angle, double x, double y, double z);
  void translatef(double x, double y, double z);
  void scalef(double x, double y, double z);

  void enable(int32_t what);
  void disable(int32_t what);

  void depthMask(int32_t what);

  void lineStipple(int32_t factor, uint16_t pattern);
  void lineWidth(double width);

  void pointSize(double size);

  void drawArrow(double x1, double y1, double x2, double y2, double head_width, double head_length);

  void renderStrokeString(std::string& str);

  void send();

  void circle(double x, double y, double r, int32_t n);
  void square(double x, double y, double w);
  void carpet(int32_t n, float* x, float* y);
  void colorCarpet(int32_t n, float* x, float* y, float* c);

private:
  void verifyMemory(int32_t extra);

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  uint8_t* buf_;
  uint32_t num_bytes_, max_bytes_;
};

} // namespace vlr

#endif
