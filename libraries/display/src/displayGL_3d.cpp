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


#include <limits.h>
#include <iostream>
#include <iomanip>

#include <displayGL.h>

namespace vlr {

template<class T>
bool DisplayGL::internalPaint3d() {
  //	static const float third = 0.333333333333333334;

  T* data = static_cast<Image<T>*> (imgBuf)->data();
  for (uint32_t y = 0, y2 = imgBuf->height() - 1; y < imgBuf->height() - 1; y++, y2--) {
    glBegin(GL_TRIANGLE_STRIP);
    for (uint32_t x = 0; x < imgBuf->width() - 1; x += 2) {
      uint32_t pos = y2 * imgBuf->paddedWidth() + x;
      float xf = (float) x;
      float yf = (float) y;

      float val = (float) data[pos] * heightScale_;
      //			float rval=(float)data[pos];
      //			float gval=(float)data[pos];
      //			float bval=(float)data[pos];
      //			glColor3f(rval, gval, bval);
      glColor3f(val, val, val);
      glVertex3f(xf, yf, val);

      val = (float) data[pos - imgBuf->paddedWidth()] * heightScale_;
      glColor3f(val, val, val);
      glVertex3f(xf, yf + 1, val);

      val = (float) data[pos + 1] * heightScale_;
      glColor3f(val, val, val);
      glVertex3f(xf + 1, yf, val);

      val = (float) data[pos - imgBuf->paddedWidth() + 1] * heightScale_;
      glColor3f(val, val, val);
      glVertex3f(xf + 1, yf + 1, val);
    }
    glEnd();
  }

  return true;
}

template<>
bool DisplayGL::internalPaint3d<uint8_t>() {
  uint8_t* data = static_cast<Image<uint8_t>*> (imgBuf)->data();
  for (uint32_t y = 0, y2 = imgBuf->height() - 1; y < imgBuf->height() - 1; y++, y2--) {
    glBegin(GL_TRIANGLE_STRIP);
    for (uint32_t x = 0; x < imgBuf->width() - 1; x += 2) {
      uint32_t pos = slice_offset_ + y2 * imgBuf->paddedWidth() + x;

      uint8_t val = data[pos]*heightScale_;
      glColor3f(color_map_red_[val], color_map_green_[val], color_map_blue_[val]);
      glVertex3i((int32_t) x, (int32_t) y, (int32_t) val);

      val = data[pos - imgBuf->paddedWidth()]*heightScale_;
      glColor3f(color_map_red_[val], color_map_green_[val], color_map_blue_[val]);
      glVertex3i(x, y + 1, (int32_t) val);

      val = data[pos + 1]*heightScale_;
      glColor3f(color_map_red_[val], color_map_green_[val], color_map_blue_[val]);
      glVertex3i(x + 1, y, (int32_t) val);

      val = data[pos - imgBuf->paddedWidth() + 1]*heightScale_;
      glColor3f(color_map_red_[val], color_map_green_[val], color_map_blue_[val]);
      glVertex3i(x + 1, y + 1, (int32_t) val);
    }
    glEnd();
  }

  return true;
}

template<>
bool DisplayGL::internalPaint3d<uint16_t>() {
  uint16_t* data = static_cast<Image<uint16_t>*> (imgBuf)->data();
  for (uint32_t y = 0, y2 = imgBuf->height() - 1; y < imgBuf->height() - 1; y++, y2--) {
    glBegin(GL_TRIANGLE_STRIP);
    for (uint32_t x = 0; x < imgBuf->width() - 1; x += 2) {
      uint32_t pos = slice_offset_ + y2 * imgBuf->paddedWidth() + x;

      float xf = (float) x;
      float yf = (float) y;

      float val = (float) data[pos];
      uint8_t index = (uint8_t)(val/USHRT_MAX*UCHAR_MAX);

      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf, yf, val*heightScale_);

      val = data[pos - imgBuf->paddedWidth()];
      index = (uint8_t)(val/USHRT_MAX*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf, yf + 1, val*heightScale_);

      val = data[pos + 1];
      index = (uint8_t)(val/USHRT_MAX*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf + 1, yf, val*heightScale_);

      val = data[pos - imgBuf->paddedWidth() + 1];
      index = (uint8_t)(val/USHRT_MAX*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf + 1, yf + 1, val*heightScale_);
    }
    glEnd();
  }

  return true;
}

template<>
bool DisplayGL::internalPaint3d<int16_t>() {
  int16_t* data = static_cast<Image<int16_t>*> (imgBuf)->data();
  for (uint32_t y = 0, y2 = imgBuf->height() - 1; y < imgBuf->height() - 1; y++, y2--) {
    glBegin(GL_TRIANGLE_STRIP);
    for (uint32_t x = 0; x < imgBuf->width() - 1; x += 2) {
      uint32_t pos = slice_offset_ + y2 * imgBuf->paddedWidth() + x;

      float xf = (float) x;
      float yf = (float) y;

      float val = (float) data[pos];
      uint8_t index = (uint8_t)((val+32768)/USHRT_MAX*UCHAR_MAX);

      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf, yf, val*heightScale_);

      val = data[pos - imgBuf->paddedWidth()];
      index = (uint8_t)((val+32768)/USHRT_MAX*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf, yf + 1, val*heightScale_);

      val = data[pos + 1];
      index = (uint8_t)((val+32768)/USHRT_MAX*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf + 1, yf, val*heightScale_);

      val = data[pos - imgBuf->paddedWidth() + 1];
      index = (uint8_t)((val+32768)/USHRT_MAX*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf + 1, yf + 1, val*heightScale_);
    }
    glEnd();
  }

  return true;
}

template<>
bool DisplayGL::internalPaint3d<float>() {
  float* data = static_cast<Image<float>*> (imgBuf)->data();
  for (uint32_t y = 0, y2 = imgBuf->height() - 1; y < imgBuf->height() - 1; y++, y2--) {
    glBegin(GL_TRIANGLE_STRIP);
    for (uint32_t x = 0; x < imgBuf->width() - 1; x += 2) {
      uint32_t pos = y2 * imgBuf->paddedWidth() + x;
      float xf = (float) x;
      float yf = (float) y;

      float val = (float) data[pos];
      uint8_t index = (uint8_t)(val*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf, yf, val*heightScale_);

      val = (float) data[pos - imgBuf->paddedWidth()];
      index = (uint8_t)(val*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf, yf + 1, val*heightScale_);

      val = (float) data[pos + 1];
      index = (uint8_t)(val*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf + 1, yf, val*heightScale_);

      val = (float) data[pos - imgBuf->paddedWidth() + 1];
      index = (uint8_t)(val*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf + 1, yf + 1, val*heightScale_);
    }
    glEnd();
  }

  return true;
}

template<>
bool DisplayGL::internalPaint3d<double>() {
  double* data = static_cast<Image<double>*> (imgBuf)->data();
  for (uint32_t y = 0, y2 = imgBuf->height() - 1; y < imgBuf->height() - 1; y++, y2--) {
    glBegin(GL_TRIANGLE_STRIP);
    for (uint32_t x = 0; x < imgBuf->width() - 1; x += 2) {
      uint32_t pos = y2 * imgBuf->paddedWidth() + x;
      float xf = (float) x;
      float yf = (float) y;

      double val = (double) data[pos] * heightScale_;
      uint8_t index = (uint8_t)(val*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf, yf, val);

      val = (double) data[pos - imgBuf->paddedWidth()] * heightScale_;
      index = (uint8_t)(val*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf, yf + 1, val);

      val = (double) data[pos + 1] * heightScale_;
      index = (uint8_t)(val*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf + 1, yf, val);

      val = (double) data[pos - imgBuf->paddedWidth() + 1] * heightScale_;
      index = (uint8_t)(val*UCHAR_MAX);
      glColor3f(color_map_red_[index], color_map_green_[index], color_map_blue_[index]);
      glVertex3f(xf + 1, yf + 1, val);
    }
    glEnd();
  }

  return true;
}

template<class T>
bool DisplayGL::internalPaint3dRGB() {
  T* data = static_cast<Image<T>*> (imgBuf)->data();
  for (uint32_t y = 0, y2 = imgBuf->height() - 1; y < imgBuf->height() - 1; y++, y2--) {
    glBegin(GL_TRIANGLE_STRIP);
    for (uint32_t x = 0; x < imgBuf->width() - 1; x += 2) {
      uint32_t pos = 3 * (y2 * imgBuf->paddedWidth() + x);
      float xf = (float) x;
      float yf = (float) y;

      float rval = (float) data[pos];
      float gval = (float) data[pos + 1];
      float bval = (float) data[pos + 2];
      //				float val = rval;
      float val = third * (rval + gval + bval);
      glColor3f(rval, gval, bval);
      glVertex3f(xf, yf, val);

      rval = (float) data[pos - 3 * imgBuf->paddedWidth()];
      gval = (float) data[pos - 3 * imgBuf->paddedWidth() + 1];
      bval = (float) data[pos - 3 * imgBuf->paddedWidth() + 2];
      //				val = rval;
      val = third * (rval + gval + bval);
      glColor3f(rval, gval, bval);
      glVertex3f(xf, yf + 1, val);

      rval = (float) data[pos + 3];
      gval = (float) data[pos + 3 + 1];
      bval = (float) data[pos + 3 + 2];
      //				val = rval;
      val = third * (rval + gval + bval);
      glColor3f(rval, gval, bval);
      glVertex3f(xf + 1, yf, val);

      rval = (float) data[pos - 3 * imgBuf->paddedWidth() + 3];
      gval = (float) data[pos - 3 * imgBuf->paddedWidth() + 3 + 1];
      bval = (float) data[pos - 3 * imgBuf->paddedWidth() + 3 + 2];
      //				val = rval;
      val = third * (rval + gval + bval);
      glColor3f(rval, gval, bval);
      glVertex3f(xf + 1, yf + 1, val);
    }
    glEnd();
  }

  return true;
}

template<> bool DisplayGL::internalPaint3dRGB<uint8_t>() {
  uint8_t* data = static_cast<Image<uint8_t>*> (imgBuf)->data();
  for (uint32_t y = 0, y2 = imgBuf->height() - 1; y < imgBuf->height() - 1; y++, y2--) {
    glBegin(GL_TRIANGLE_STRIP);
    for (uint32_t x = 0; x < imgBuf->width() - 1; x += 2) {
      uint32_t pos = 3 * (y2 * imgBuf->paddedWidth() + x);

      uint8_t rval = data[pos];
      uint8_t gval = data[pos + 1];
      uint8_t bval = data[pos + 2];
      //				uint8_t val = rval;
      uint8_t val = third * (rval + gval + bval);
      glColor3ub(rval, gval, bval);
      glVertex3i((int32_t) x, (int32_t) y, (int32_t) val);

      rval = data[pos - 3 * imgBuf->paddedWidth()];
      gval = data[pos - 3 * imgBuf->paddedWidth() + 1];
      bval = data[pos - 3 * imgBuf->paddedWidth() + 2];
      //				val = rval;
      val = third * (rval + gval + bval);
      glColor3ub(rval, gval, bval);
      glVertex3i((int32_t) x, (int32_t) y + 1, (int32_t) val);

      rval = data[pos + 3];
      gval = data[pos + 3 + 1];
      bval = data[pos + 3 + 2];
      //				val = rval;
      val = third * (rval + gval + bval);
      glColor3ub(rval, gval, bval);
      glVertex3i((int32_t) x + 1, (int32_t) y, (int32_t) val);

      rval = data[pos - 3 * imgBuf->paddedWidth() + 3];
      gval = data[pos - 3 * imgBuf->paddedWidth() + 3 + 1];
      bval = data[pos - 3 * imgBuf->paddedWidth() + 3 + 2];
      //				val = rval;
      val = third * (rval + gval + bval);
      glColor3ub(rval, gval, bval);
      glVertex3i((int32_t) x + 1, (int32_t) y + 1, (int32_t) val);
    }
    glEnd();
  }

  return true;
}

template<class T, class TT>
bool DisplayGL::internalPaint3dTexture() {
  T* data = static_cast<Image<T>*> (imgBuf)->data();
  TT* texData = static_cast<Image<TT>*> (texBuf)->data();
  for (uint32_t y = 0, y2 = imgBuf->height() - 1; y < imgBuf->height() - 1; y++, y2--) {
    glBegin(GL_TRIANGLE_STRIP);
    for (uint32_t x = 0; x < imgBuf->width() - 1; x += 2) {
      uint32_t pos = y2 * imgBuf->paddedWidth() + x;
      float xf = (float) x;
      float yf = (float) y;

      float tval = (float) texData[pos];
      float val = (float) data[pos];
      glColor3f(tval, tval, tval);
      glVertex3f(xf, yf, val*heightScale_);

      tval = (float) texData[pos - imgBuf->paddedWidth()];
      val = (float) data[pos - imgBuf->paddedWidth()];
      glColor3f(tval, tval, tval);
      glVertex3f(xf, yf + 1, val*heightScale_);

      tval = (float) texData[pos + 1];
      val = (float) data[pos + 1];
      glColor3f(tval, tval, tval);
      glVertex3f(xf + 1, yf, val*heightScale_);

      tval = (float) texData[pos - imgBuf->paddedWidth() + 1];
      val = (float) data[pos - imgBuf->paddedWidth() + 1];
      glColor3f(tval, tval, tval);
      glVertex3f(xf + 1, yf + 1, val*heightScale_);
    }
    glEnd();
  }

  return true;
}

template<class T, class TT>
bool DisplayGL::internalPaint3dTextureRGB() {
  T* data = static_cast<Image<T>*> (imgBuf)->data();
  TT* texData = static_cast<Image<TT>*> (texBuf)->data();
  for (uint32_t y = 0, y2 = imgBuf->height() - 1; y < imgBuf->height() - 1; y++, y2--) {
    glBegin(GL_TRIANGLE_STRIP);
    for (uint32_t x = 0; x < imgBuf->width() - 1; x += 2) {
      uint32_t texPos = 3 * (y2 * texBuf->paddedWidth() + x);
      uint32_t pos = y2 * imgBuf->paddedWidth() + x;
      float xf = (float) x;
      float yf = (float) y;

      float rval = (float) texData[texPos];
      float gval = (float) texData[texPos + 1];
      float bval = (float) texData[texPos + 2];
      float val = (float) data[pos];
      glColor3f(rval, gval, bval);
      glVertex3f(xf, yf, val);

      rval = (float) texData[texPos - 3 * texBuf->paddedWidth()];
      gval = (float) texData[texPos - 3 * texBuf->paddedWidth() + 1];
      bval = (float) texData[texPos - 3 * texBuf->paddedWidth() + 2];
      val = (float) data[pos - imgBuf->paddedWidth()];
      glColor3f(rval, gval, bval);
      glVertex3f(xf, yf + 1, val);

      rval = (float) texData[texPos + 3];
      gval = (float) texData[texPos + 3 + 1];
      bval = (float) texData[texPos + 3 + 2];
      val = (float) data[pos + 1];
      glColor3f(rval, gval, bval);
      glVertex3f(xf + 1, yf, val);

      rval = (float) texData[texPos - 3 * texBuf->paddedWidth() + 3];
      gval = (float) texData[texPos - 3 * texBuf->paddedWidth() + 3 + 1];
      bval = (float) texData[texPos - 3 * texBuf->paddedWidth() + 3 + 2];
      val = (float) data[pos - imgBuf->paddedWidth() + 1];
      glColor3f(rval, gval, bval);
      glVertex3f(xf + 1, yf + 1, val);
    }
    glEnd();
  }

  return true;
}

template<>
bool DisplayGL::internalPaint3dTextureRGB<uint8_t, uint8_t>() {
  uint8_t* data = static_cast<Image<uint8_t>*> (imgBuf)->data();
  uint8_t* texData = static_cast<Image<uint8_t>*> (texBuf)->data();
  for (uint32_t y = 0, y2 = imgBuf->height() - 1; y < imgBuf->height() - 1; y++, y2--) {
    glBegin(GL_TRIANGLE_STRIP);
    for (uint32_t x = 0; x < imgBuf->width() - 1; x += 2) {
      uint32_t texPos = 3 * (y2 * texBuf->paddedWidth() + x);
      uint32_t pos = y2 * imgBuf->paddedWidth() + x;

      uint8_t rval = texData[texPos];
      uint8_t gval = texData[texPos + 1];
      uint8_t bval = texData[texPos + 2];
      uint8_t val = data[pos];
      glColor3ub(rval, gval, bval);
      glVertex3i((int32_t) x, (int32_t) y, (int32_t) val);

      rval = texData[texPos - 3 * texBuf->paddedWidth()];
      gval = texData[texPos - 3 * texBuf->paddedWidth() + 1];
      bval = texData[texPos - 3 * texBuf->paddedWidth() + 2];
      val = data[pos - imgBuf->paddedWidth()];
      glColor3ub(rval, gval, bval);
      glVertex3i((int32_t) x, (int32_t) y + 1, (int32_t) val);

      rval = texData[texPos + 3];
      gval = texData[texPos + 3 + 1];
      bval = texData[texPos + 3 + 2];
      val = data[pos + 1];
      glColor3ub(rval, gval, bval);
      glVertex3i((int32_t) x + 1, (int32_t) y, (int32_t) val);

      rval = texData[texPos - 3 * texBuf->paddedWidth() + 3];
      gval = texData[texPos - 3 * texBuf->paddedWidth() + 3 + 1];
      bval = texData[texPos - 3 * texBuf->paddedWidth() + 3 + 2];
      val = data[pos - imgBuf->paddedWidth() + 1];
      glColor3ub(rval, gval, bval);
      glVertex3i((int32_t) x + 1, (int32_t) y + 1, (int32_t) val);
    }
    glEnd();
  }

  return true;
}

//	template <> bool DisplayGL::internalPaint3d<ImageBase::complex>()
//		{
//		return false;
//		}
//	template <> bool DisplayGL::internalPaint3d<ImageBase::dcomplex>()
//		{
//		return false;
//		}


void DisplayGL::internalPaint3d() {
  activate3DMode();
  glTranslatef(-0.5 * ((float) width()), -0.5 * ((float) height()), 0); // unsigned sucks...

//  printf("- 1 - %s: EC: %i\n", __FUNCTION__, glGetError());
  showGridXY = true;
  showGridXZ = true;
  showGridYZ = true;
  float zPos = 0, yPos = 0, xPos = 0;

  if (showGridXY) {
    drawGridXY(zPos);
  }
  if (showGridXZ) {
    drawGridXZ(yPos);
  }
  if (showGridYZ) {
    drawGridYZ(xPos);
  }

  //	glColor4f(1, 0, 0, 1);//0.75);
  glColor4f(1.0, 1.0, 1.0, 1.0);
  if (imgBuf) {
      // TODO: Texture and data can have different types...
    if (useTexture && texBuf && texBuf->colorSpace() == ImageBase::CS_RGB_C) {
      glEnable(GL_TEXTURE_2D);
      //glEnable(GL_TEXTURE_RECTANGLE_ARB);
      glBindTexture(texType, imgTexture);
      switch(data_type_) {
        case TYPE_UCHAR:
          internalPaint3dTextureRGB<uint8_t, uint8_t> ();
        break;

        case TYPE_CHAR:
          internalPaint3dTextureRGB<int8_t, int8_t> ();
        break;

        case TYPE_USHORT:
          internalPaint3dTextureRGB<uint16_t, uint16_t> ();
        break;

        case TYPE_SHORT:
          internalPaint3dTextureRGB<int16_t, int16_t> ();
        break;

        case TYPE_UINT:
          internalPaint3dTextureRGB<uint32_t, uint32_t> ();
        break;

        case TYPE_INT:
          internalPaint3dTextureRGB<int32_t, int32_t> ();
        break;

        case TYPE_FLOAT:
          internalPaint3dTextureRGB<float, float> ();
        break;

        case TYPE_DOUBLE:
//          internalPaint3dTextureRGB<double, double> ();
          internalPaint3dTextureRGB<float, float> (); // TODO: Make double work?!?
        break;
      }

      glDisable(GL_TEXTURE_2D);
      glDisable(GL_TEXTURE_RECTANGLE_ARB);
    }
    else if (imgBuf->colorSpace() == ImageBase::CS_RGB_C) {
      switch(data_type_) {
        case TYPE_UCHAR:
          internalPaint3dRGB<uint8_t> ();
        break;

        case TYPE_CHAR:
          internalPaint3dRGB<int8_t> ();
        break;

        case TYPE_USHORT:
          internalPaint3dRGB<uint16_t> ();
        break;

        case TYPE_SHORT:
          internalPaint3dRGB<int16_t> ();
        break;

        case TYPE_UINT:
          internalPaint3dRGB<uint32_t> ();
        break;

        case TYPE_INT:
          internalPaint3dRGB<int32_t> ();
        break;

        case TYPE_FLOAT:
          internalPaint3dRGB<float> ();
        break;

        case TYPE_DOUBLE:
          internalPaint3dRGB<float> (); // TODO: Make double work?!?
//          internalPaint3dRGB<double> ();
        break;
      }
    }
    else if (useTexture && texBuf && texBuf->colorSpace() == ImageBase::CS_GRAY) {
      switch(data_type_) {
        case TYPE_UCHAR:
          internalPaint3dTexture<uint8_t, uint8_t> ();
        break;

        case TYPE_CHAR:
          internalPaint3dTexture<int8_t, int8_t> ();
        break;

        case TYPE_USHORT:
          internalPaint3dTexture<uint16_t, uint16_t> ();
        break;

        case TYPE_SHORT:
          internalPaint3dTexture<int16_t, int16_t> ();
        break;

        case TYPE_UINT:
          internalPaint3dTexture<uint32_t, uint32_t> ();
        break;

        case TYPE_INT:
          internalPaint3dTexture<int32_t, int32_t> ();
        break;

        case TYPE_FLOAT:
          internalPaint3dTexture<float, float> ();
        break;

        case TYPE_DOUBLE:
//          internalPaint3dTextureRGB<double, double> ();
          internalPaint3dTexture<float, float> (); // TODO: Make double work?!?
        break;
      }
    }
    else {
//      printf("- 3b - %s: EC: %i\n", __FUNCTION__, glGetError());
      switch(data_type_) {
         case TYPE_UCHAR:
           internalPaint3d<uint8_t> ();
         break;

         case TYPE_CHAR:
           internalPaint3d<int8_t> ();
         break;

         case TYPE_USHORT:
           internalPaint3d<uint16_t> ();
         break;

         case TYPE_SHORT:
           internalPaint3d<int16_t> ();
         break;

         case TYPE_UINT:
           internalPaint3d<uint32_t> ();
         break;

         case TYPE_INT:
           internalPaint3d<int32_t> ();
         break;

         case TYPE_FLOAT:
           internalPaint3d<float> ();
         break;

         case TYPE_DOUBLE:
//           internalPaint3d<double> ();
           internalPaint3d<float> (); // TODO: Make double work?!?
         break;
       }
    }
    // drawTags3D();
    drawCustomTags3d();
  }
}

void DisplayGL::drawGridXY(float zPos) {
  float grid_x, grid_y;
  float xMin = 0;
  float xMax = (imgBuf ? imgBuf->width() : width());
  float yMin = 0;
  float yMax = (imgBuf ? imgBuf->height() : height());
  float xStep = (xMax - xMin) / 10;
  float yStep = (yMax - yMin) / 10;

  glLineWidth(0.5);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glEnable(GL_LINE_SMOOTH);
  //	glColor3f(0.4, 0.4, 0.4);
  glColor3f(1.0, 0, 0);
  glBegin(GL_LINES);
  for (grid_x = xMin; grid_x <= xMax; grid_x += xStep) {
    glVertex3f(grid_x, yMin, zPos);
    glVertex3f(grid_x, yMax, zPos);
  }
  for (grid_y = yMin; grid_y <= yMax; grid_y += yStep) {
    glVertex3f(xMin, grid_y, zPos);
    glVertex3f(xMax, grid_y, zPos);
  }
  glEnd();

  std::stringstream label;
  double x_offset=10;
  double y_offset=16;
  for (grid_x = xMin; grid_x <= xMax; grid_x += xStep) {
    label.str("");
    label << std::fixed << std::setprecision(3) << grid_x;
    fr.drawString2D(label.str(), float(grid_x+x_offset), float(yMax + y_offset));
    }


  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
}

void DisplayGL::drawGridXZ(float yPos) {
  float grid_x, grid_z;
  float xMin = 0;
  float xMax = (imgBuf ? imgBuf->width() : width());
  float zMin = 0, zMax = 500;
  float xStep = (xMax - xMin) / 10;
  float zStep = (zMax - zMin) / 10;

  glLineWidth(0.5);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glEnable(GL_LINE_SMOOTH);
  //	glColor3f(0.4, 0.4, 0.4);
  glColor3f(1.0, 0, 0);
  glBegin(GL_LINES);
  for (grid_x = xMin; grid_x <= xMax; grid_x += xStep) {
    glVertex3f(grid_x, yPos, zMin);
    glVertex3f(grid_x, yPos, zMax);
  }
  for (grid_z = zMin; grid_z <= zMax; grid_z += zStep) {
    glVertex3f(xMin, yPos, grid_z);
    glVertex3f(xMax, yPos, grid_z);
  }
  glEnd();

  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
}

void DisplayGL::drawGridYZ(float xPos) {
  if (xPos) {
  }
}

} // namespace vlr
