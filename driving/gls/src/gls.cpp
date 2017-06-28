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


#include <string>
#include <global.h>
#include <gls.h>

namespace vlr {

GLS::GLS(const std::string& gls_name) : nh_("/driving"),
  buf_(NULL), num_bytes_(0), max_bytes_(1000) {

  pub_ = nh_.advertise<driving_common::GLSOverlay>("GLSOverlay", 5);

  name = gls_name;
  first_vertex = true;
  if (!(buf_ = (uint8_t*) calloc(max_bytes_, 1))) {
    throw VLRException("Cannot create temporary buffer.");
  }
}

GLS::~GLS() {
  if(buf_) {free(buf_);}
}

void GLS::clear() {
  cmd_buf.clear();
  num_bytes_ = 0;
  first_vertex = true;
}

void GLS::verifyMemory(int32_t extra) {
  if (num_bytes_ + extra > max_bytes_) {
    max_bytes_ = num_bytes_ + extra + 1000; // 2*(num_bytes_ + extra)

    if (!(buf_ = (uint8_t*) realloc(buf_, max_bytes_))) {
      throw VLRException("Cannot increase size of temporary buffer.");
    }
  }
}

void GLS::begin(uint8_t drawing_type) {
  verifyMemory(1);
  buf_[num_bytes_] = drawing_type;
  num_bytes_++;
}

void GLS::end() {
  verifyMemory(1);
  buf_[num_bytes_] = driving_common::GLSOverlay::END;
  num_bytes_++;
}

void GLS::vertex3f(double x, double y, double z) {
  float temp;

  verifyMemory(1 + 3 * sizeof(float));
  buf_[num_bytes_] = VERTEX3;
  num_bytes_++;
  temp = (float) (x);
  *((float *) (buf_ + num_bytes_)) = temp;
  num_bytes_ += sizeof(float);
  temp = (float) (y);
  *((float *) (buf_ + num_bytes_)) = temp;
  num_bytes_ += sizeof(float);
  temp = (float) (z);
  *((float *) (buf_ + num_bytes_)) = temp;
  num_bytes_ += sizeof(float);
}

void GLS::vertex2f(double x, double y) {
  float temp;

  verifyMemory(1 + 2 * sizeof(float));
  buf_[num_bytes_] = VERTEX2;
  num_bytes_++;
  temp = (float) (x);
  *((float *) (buf_ + num_bytes_)) = temp;
  num_bytes_ += sizeof(float);
  temp = (float) (y);
  *((float *) (buf_ + num_bytes_)) = temp;
  num_bytes_ += sizeof(float);
}

void GLS::color3f(double r, double g, double b) {
  verifyMemory(4);
  buf_[num_bytes_++] = COLOR;
  buf_[num_bytes_++] = (uint8_t) rint(r * 255);
  buf_[num_bytes_++] = (uint8_t) rint(g * 255);
  buf_[num_bytes_++] = (uint8_t) rint(b * 255);
}

void GLS::color4f(double r, double g, double b, double a) {
  verifyMemory(5);
  buf_[num_bytes_++] = COLOR4;
  buf_[num_bytes_++] = (uint8_t) rint(r * 255);
  buf_[num_bytes_++] = (uint8_t) rint(g * 255);
  buf_[num_bytes_++] = (uint8_t) rint(b * 255);
  buf_[num_bytes_++] = (uint8_t) rint(a * 255);
}

void GLS::pushMatrix() {
  verifyMemory(1);
  buf_[num_bytes_++] = PUSH_MATRIX;
}

void GLS::popMatrix() {
  verifyMemory(1);
  buf_[num_bytes_++] = POP_MATRIX;
}

void GLS::rotatef(double angle, double x, double y, double z) {
  verifyMemory(1 + 4 * sizeof(float));
  buf_[num_bytes_++] = ROTATEF;

  *((float *) (buf_ + num_bytes_)) = (float) angle;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) x;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) y;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) z;
  num_bytes_ += sizeof(float);
}

void GLS::translatef(double x, double y, double z) {
  verifyMemory(1 + 3 * sizeof(float));
  buf_[num_bytes_++] = TRANSLATEF;

  *((float *) (buf_ + num_bytes_)) = (float) x;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) y;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) z;
  num_bytes_ += sizeof(float);
}

void GLS::scalef(double x, double y, double z) {
  verifyMemory(1 + 3 * sizeof(float));
  buf_[num_bytes_++] = SCALEF;

  *((float *) (buf_ + num_bytes_)) = (float) x;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) y;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) z;
  num_bytes_ += sizeof(float);
}

void GLS::enable(int32_t what) {
  verifyMemory(1 + sizeof(what));
  buf_[num_bytes_++] = ENABLE;
  *((int32_t *) (buf_ + num_bytes_)) = what;
  num_bytes_ += sizeof(what);
}

void GLS::disable(int32_t what) {
  verifyMemory(1 + sizeof(what));
  buf_[num_bytes_++] = DISABLE;
  *((int32_t *) (buf_ + num_bytes_)) = what;
  num_bytes_ += sizeof(what);
}

void GLS::depthMask(int32_t what) {
  verifyMemory(1 + sizeof(what));
  buf_[num_bytes_++] = DEPTH_MASK;
  *((int32_t *) (buf_ + num_bytes_)) = what;
  num_bytes_ += sizeof(what);
}

void GLS::lineStipple(int32_t factor, uint16_t pattern) {
  verifyMemory(1 + sizeof(factor) + sizeof(pattern));
  buf_[num_bytes_++] = LINE_STIPPLE;
  *((int32_t *) (buf_ + num_bytes_)) = factor;
  num_bytes_ += sizeof(int32_t);
  *((uint16_t *) (buf_ + num_bytes_)) = pattern;
  num_bytes_ += sizeof(short);
}

void GLS::lineWidth(double width) {
  verifyMemory(1 + sizeof(float));
  buf_[num_bytes_++] = LINEWIDTH;
  *((float *) (buf_ + num_bytes_)) = (float) width;
  num_bytes_ += sizeof(float);
}

void GLS::pointSize(double size) {
  verifyMemory(1 + sizeof(float));
  buf_[num_bytes_++] = POINTSIZE;
  *((float *) (buf_ + num_bytes_)) = (float) size;
  num_bytes_ += sizeof(float);
}

void GLS::renderStrokeString(std::string& str) {
  verifyMemory(1 + sizeof(int32_t) + str.size() + 1);
  buf_[num_bytes_++] = RENDERSTROKESTRING;
  *((int32_t *) (buf_ + num_bytes_)) = (int32_t) str.size();
  num_bytes_ += sizeof(int32_t);
  memcpy(buf_ + num_bytes_, str.c_str(), str.size() + 1);
  num_bytes_ += str.size() + 1;
}

void GLS::send() {
//  pub_.publish(*this);
  if(cmd_buf.size() < num_bytes_) {cmd_buf.resize(num_bytes_);}
  memcpy(&cmd_buf[0], buf_, num_bytes_);
  pub_.publish(*((driving_common::GLSOverlay*)this));
}

void GLS::drawArrow(double x1, double y1, double x2, double y2, double head_width, double head_length) {
  verifyMemory(1 + 6 * sizeof(float));
  buf_[num_bytes_++] = DRAW_ARROW;

  *((float *) (buf_ + num_bytes_)) = (float) x1;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) y1;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) x2;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) y2;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) head_width;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) head_length;
  num_bytes_ += sizeof(float);
}

void GLS::circle(double x, double y, double r, int32_t n) {
  verifyMemory(1 + 3 * sizeof(float) + sizeof(n));
  buf_[num_bytes_++] = CIRCLE;
  *((float *) (buf_ + num_bytes_)) = (float) x;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) y;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) r;
  num_bytes_ += sizeof(float);
  *((int32_t *) (buf_ + num_bytes_)) = (int32_t) n;
  num_bytes_ += sizeof(n);
}

void GLS::square(double x, double y, double w) {
  verifyMemory(1 + 3 * sizeof(float));
  buf_[num_bytes_++] = SQUARE;
  *((float *) (buf_ + num_bytes_)) = (float) x;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) y;
  num_bytes_ += sizeof(float);
  *((float *) (buf_ + num_bytes_)) = (float) w;
  num_bytes_ += sizeof(float);
}

void GLS::carpet(int32_t n, float* x, float* y) {
  verifyMemory(1 + sizeof(n) + 2 * n * sizeof(float));
  buf_[num_bytes_++] = CARPET;
  *((int32_t *) (buf_ + num_bytes_)) = n;
  num_bytes_ += sizeof(n);
  for (int32_t i = 0; i < n; i++) {
    *((float *) (buf_ + num_bytes_)) = (float) x[i];
    num_bytes_ += sizeof(float);
    *((float *) (buf_ + num_bytes_)) = (float) y[i];
    num_bytes_ += sizeof(float);
  }
}

void GLS::colorCarpet(int32_t n, float* x, float* y, float* c) {
  verifyMemory(1 + sizeof(n) + 3 * n * sizeof(float));
  buf_[num_bytes_++] = COLOR_CARPET;
  *((int32_t *) (buf_ + num_bytes_)) = n;
  num_bytes_ += sizeof(n);
  for (int32_t i = 0; i < n; i++) {
    *((float *) (buf_ + num_bytes_)) = (float) x[i];
    num_bytes_ += sizeof(float);
    *((float *) (buf_ + num_bytes_)) = (float) y[i];
    num_bytes_ += sizeof(float);
    *((float *) (buf_ + num_bytes_)) = (float) c[i];
    num_bytes_ += sizeof(float);
  }
}

} // namespace vlr
