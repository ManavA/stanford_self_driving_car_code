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


#ifndef VLRIMAGING_IMAGETAGS_H_
#define VLRIMAGING_IMAGETAGS_H_

#include <string>

namespace vlr {

enum {tidPoint=1, tidLine, tidRect, tidEllipse};

class TagBase
	{
	protected:
    TagBase(const unsigned int id, const std::string& label,
               const float r, const float g, const float b, const float a) :
                   id_(id), label_(label), r_(r), g_(g), b_(b), a_(a) {}

    TagBase(const unsigned int id, const std::string& label) :
                   id_(id), label_(label), r_(1.0f), g_(0.0f), b_(0.0f), a_(1.0f) {}

    TagBase(const unsigned int id) :
                   id_(id), r_(1.0f), g_(0.0f), b_(0.0f), a_(1.0f) {}

	public:
		virtual ~TagBase() {}
    inline float r() {return r_;}
    inline void setR(const float r) {r_=r;}

    inline float g() {return g_;}
    inline void setG(const float g) {g_=g;}

    inline float b() {return b_;}
    inline void setB(const float b) {b_=b;}

    inline float a() {return a_;}
    inline void setA(const float a) {a_=a;}

    inline void setRGB(const float r, const float g, const float b) {
      r_=r; g_=g; b_=b;
    }

    inline void setRGBA(const float r, const float g, const float b, const float a) {
      r_=r; g_=g; b_=b; a_=a;
    }

    inline const std::string& label() {return label_;}
    inline void setLabel(const std::string& label) {label_=label;}

	protected:
		unsigned int id_;		// this is for efficient sorting only
    std::string label_;
    float r_, g_, b_, a_;
	};

class TagPoint : public TagBase {
 public:
  TagPoint(const float x, const float y, const std::string& label,
        const float r, const float g, const float b, const float a) :
        TagBase(tidPoint, label, r, g, b, a) {
    coords_[0]=x; coords_[1]=y;
  }

  TagPoint(const float x, const float y, const std::string& label) :
        TagBase(tidPoint, label) {
    coords_[0]=x; coords_[1]=y;
  }

  TagPoint(const float x, const float y) :
        TagBase(tidPoint) {
    coords_[0]=x; coords_[1]=y;
  }

  virtual ~TagPoint() {}

  inline float x() {return coords_[0];}
  inline void setX(const float x) {coords_[0]=x;}

  inline float y() {return coords_[1];}
  inline void setY(const float y) {coords_[1]=y;}

  inline void operator()(const float x, const float y) {coords_[0]=x; coords_[1]=y;}

 private:
  float coords_[2];
	};

class TagLine : public TagBase {
 public:
  TagLine(const float x0, const float y0, const float x1, const float y1, const std::string& label,
        const float r, const float g, const float b, const float a) :
        TagBase(tidLine, label, r, g, b, a) {
    coords_[0]=x0; coords_[1]=y0;
    coords_[2]=x1; coords_[3]=y1;
  }

  TagLine(const float x0, const float y0, const float x1, const float y1, const std::string& label) :
        TagBase(tidLine, label) {
    coords_[0]=x0; coords_[1]=y0;
    coords_[2]=x1; coords_[3]=y1;
  }

  TagLine(const float x0, const float y0, const float x1, const float y1) :
        TagBase(tidLine) {
    coords_[0]=x0; coords_[1]=y0;
    coords_[2]=x1; coords_[3]=y1;
  }

  virtual ~TagLine() {}

  inline float x0() {return coords_[0];}
  inline void setX0(const float x0) {coords_[0]=x0;}

  inline float y0() {return coords_[1];}
  inline void setY0(const float y0) {coords_[1]=y0;}

  inline float x1() {return coords_[2];}
  inline void setX1(const float x1) {coords_[2]=x1;}

  inline float y1() {return coords_[3];}
  inline void setY1(const float y1) {coords_[3]=y1;}

  inline void setStart(const float x0, const float y0) {
    coords_[0]=x0; coords_[1]=y0;
  }

  inline void setEnd(const float x1, const float y1) {
    coords_[2]=x1; coords_[3]=y1;
  }

  inline void operator()(const float x0, const float y0, const float x1, const float y1) {
    coords_[0]=x0; coords_[1]=y0;
    coords_[2]=x1; coords_[3]=y1;
  }

 private:
  float coords_[4];
  };

}	// namespace vlr

#endif // VLRIMAGING_IMAGETAGS_H_
