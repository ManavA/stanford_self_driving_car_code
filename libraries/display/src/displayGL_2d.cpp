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

#include <displayGL.h>

namespace vlr {

template<class T> bool DisplayGL::internalPaint2d() {
  return false; // TODO: implement conversion for non-standard types
}

template<> bool DisplayGL::internalPaint2d<unsigned char>() {
  unsigned char* data = &static_cast<Image<unsigned char>*> (imgBuf)->data()[slice_offset_];
  glTexImage2D(texType, 0, GL_RGBA, imgBuf->width(), imgBuf->height(), 0, bufColorFormat, GL_UNSIGNED_BYTE, data);
//  printf("- 1 - %s: EC: %i\n", __FUNCTION__, glGetError());
  return true;
}

template<> bool DisplayGL::internalPaint2d<char>() {
  char* data = &static_cast<Image<char>*> (imgBuf)->data()[slice_offset_];
  glTexImage2D(texType, 0, GL_RGBA, imgBuf->width(), imgBuf->height(), 0, bufColorFormat, GL_BYTE, data);
  return true;
}

template<> bool DisplayGL::internalPaint2d<unsigned short>() {
  unsigned short* data = &static_cast<Image<unsigned short>*> (imgBuf)->data()[slice_offset_];
  glTexImage2D(texType, 0, GL_RGBA, imgBuf->width(), imgBuf->height(), 0, bufColorFormat, GL_UNSIGNED_SHORT, data);
  return true;
}

template<> bool DisplayGL::internalPaint2d<short>() {
  short* data = &static_cast<Image<short>*> (imgBuf)->data()[slice_offset_];
  glTexImage2D(texType, 0, GL_RGBA, imgBuf->width(), imgBuf->height(), 0, bufColorFormat, GL_SHORT, data);
  return true;
}

template<> bool DisplayGL::internalPaint2d<unsigned int>() {
  unsigned int* data = &static_cast<Image<unsigned int>*> (imgBuf)->data()[slice_offset_];
  glTexImage2D(texType, 0, GL_RGBA, imgBuf->width(), imgBuf->height(), 0, bufColorFormat, GL_UNSIGNED_INT, data);
  return true;
}

template<> bool DisplayGL::internalPaint2d<int>() {
  int* data = &static_cast<Image<int>*> (imgBuf)->data()[slice_offset_];
  glTexImage2D(texType, 0, GL_RGBA, imgBuf->width(), imgBuf->height(), 0, bufColorFormat, GL_INT, data);
  return true;
}

template<> bool DisplayGL::internalPaint2d<float>() {
  float* data = &static_cast<Image<float>*> (imgBuf)->data()[slice_offset_];
  glTexImage2D(texType, 0, GL_RGBA, imgBuf->width(), imgBuf->height(), 0, bufColorFormat, GL_FLOAT, data);
  return true;
}

template<> bool DisplayGL::internalPaint2d<double>() {
  double* data = &static_cast<Image<double>*> (imgBuf)->data()[slice_offset_];
  glTexImage2D(texType, 0, GL_RGBA, imgBuf->width(), imgBuf->height(), 0, bufColorFormat, GL_DOUBLE, data);
  return true;
}

void DisplayGL::internalPaint2d() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  float scaled_width = width()*scale_;
  float scaled_height = height()*scale_;
  glOrtho(0., scaled_width, 0., scaled_height, -1, 1);
  glViewport(0, 0, scaled_width, scaled_height);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  glEnable(GL_TEXTURE_RECTANGLE_ARB);

  glBindTexture(texType, imgTexture);

  //	glColor4f(1, 0, 0, 1);//0.75);
  glColor4f(1.0, 1.0, 1.0, 1.0);

  if (imgBuf) {
    switch(data_type_) {
      case TYPE_UCHAR:
      internalPaint2d<unsigned char> ();
      break;

      case TYPE_CHAR:
      internalPaint2d<char> ();
      break;

      case TYPE_USHORT:
      internalPaint2d<unsigned short> ();
      break;

      case TYPE_SHORT:
      internalPaint2d<short> ();
      break;

      case TYPE_UINT:
      internalPaint2d<unsigned int> ();
      break;

      case TYPE_INT:
      internalPaint2d<int> ();
      break;

      case TYPE_FLOAT:
      internalPaint2d<float> ();
      break;

      case TYPE_DOUBLE:
        internalPaint2d<float> ();  // TODO: Make double texture work?!?
//        internalPaint2d<double> ();
      break;
    }
//    if (dynamic_cast<Image<unsigned char>*> (imgBuf)) {
//      internalPaint2d<unsigned char> ();
//    }
//    else if (dynamic_cast<Image<char>*> (imgBuf)) {
//      internalPaint2d<char> ();
//    }
//    else if (dynamic_cast<Image<unsigned short>*> (imgBuf)) {
//      internalPaint2d<unsigned short> ();
//    }
//    else if (dynamic_cast<Image<short>*> (imgBuf)) {
//      internalPaint2d<short> ();
//    }
//    else if (dynamic_cast<Image<unsigned int>*> (imgBuf)) {
//      internalPaint2d<unsigned int> ();
//    }
//    else if (dynamic_cast<Image<int>*> (imgBuf)) {
//      internalPaint2d<int> ();
//    }
//    else if (dynamic_cast<Image<float>*> (imgBuf)) {
//      internalPaint2d<float> ();
//    }
//    else if (dynamic_cast<Image<double>*> (imgBuf)) {
//      internalPaint2d<double> ();
//    }
    //		else if(dynamic_cast<Image<ImageBase::complex>*>(imgBuf))
    //			{internalPaint2d<ImageBase::complex>();}
    //		else if(dynamic_cast<Image<ImageBase::dcomplex>*>(imgBuf))
    //			{internalPaint2d<ImageBase::complex>();}

    glEnable(GL_BLEND);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);             glVertex2f(0, scaled_height);
    glTexCoord2f(width(), 0);        glVertex2f(scaled_width, scaled_height);
    glTexCoord2f(width(), height());  glVertex2f(scaled_width, 0);
    glTexCoord2f(0, height());       glVertex2f(0, 0);
    glEnd();
    glDisable(GL_BLEND);

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_TEXTURE_RECTANGLE_ARB);

    drawTags2d();
    drawCustomTags2d();
  }
}

void DisplayGL::drawTags2d() {

  std::pair<ImageBase::tagCIter, ImageBase::tagCIter> range;
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glEnable(GL_POLYGON_SMOOTH);

  // draw lines
  imgBuf->lines(range);
  glBegin(GL_LINES);
  for (ImageBase::tagCIter pit = range.first; pit != range.second; ++pit) {
    TagLine* ln = static_cast<TagLine*> ((*pit).second);
    glColor4f(ln->r(), ln->g(), ln->b(), ln->a());
    glVertex2f(ln->x0()*scale_, height() - 1 - ln->y0()*scale_);
    glVertex2f(ln->x1()*scale_, height() - 1 - ln->y1()*scale_);
  }
  glEnd();

  for (ImageBase::tagCIter pit = range.first; pit != range.second; ++pit) {
    TagLine* ln = static_cast<TagLine*> ((*pit).second);
    glColor4f(ln->r(), ln->g(), ln->b(), ln->a());
    fr.drawString2D(ln->label(), ln->x1()*scale_ + crossSize, height() - 1 - ln->y1()*scale_ + crossSize);
  }

  // draw points
  imgBuf->points(range);

  glBegin(GL_LINES);
  for (ImageBase::tagCIter pit = range.first; pit != range.second; ++pit) {
    TagPoint* pt = static_cast<TagPoint*> ((*pit).second);
    glColor4f(pt->r(), pt->g(), pt->b(), pt->a());
    glVertex2f(pt->x()*scale_ - crossSize, (height() - 1 - pt->y())*scale_);
    glVertex2f(pt->x()*scale_ + crossSize, (height() - 1 - pt->y())*scale_);
    glVertex2f(pt->x()*scale_, (height() - 1 - pt->y())*scale_ - crossSize);
    glVertex2f(pt->x()*scale_, (height() - 1 - pt->y())*scale_ + crossSize);
  }
  glEnd();

  for (ImageBase::tagCIter pit = range.first; pit != range.second; ++pit) {
    TagPoint* pt = static_cast<TagPoint*> ((*pit).second);
    glColor4f(pt->r(), pt->g(), pt->b(), pt->a());
    fr.drawString2D(pt->label(), pt->x()*scale_ + crossSize, (height() - 1 - pt->y())*scale_ + crossSize);
  }

  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
}

} // namespace vlr
