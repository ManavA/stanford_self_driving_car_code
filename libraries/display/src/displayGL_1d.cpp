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
#include <iomanip>
#include <algorithm>

#include <displayGL.h>

namespace vlr {

template<class T>
bool DisplayGL::internalPaint1d() {
  T* data = &static_cast<Image<T>*> (imgBuf)->data()[slice_offset_];
  for(uint32_t c=0; c<std::min((uint32_t)6, imgBuf->channels()); c++) {
    glColor3f(colors_1d[3*c], colors_1d[3*c+1], colors_1d[3*c+2]);
    glBegin(GL_LINES);
    for (uint32_t i=1; i<imgBuf->width()*imgBuf->height(); i++) {
      // data is normalized to [0,1] already
      glVertex2f((float)i-1, (*data-minval_)/(maxval_-minval_)*GLWidget::height()); data++;
      glVertex2f((float)i, (*data-minval_)/(maxval_-minval_)*GLWidget::height());// data++;
    }
    glEnd();
  }
  return true;
}

template<>
bool DisplayGL::internalPaint1d<float>() {
  float* data = &static_cast<Image<float>*> (imgBuf)->data()[slice_offset_];
  for(uint32_t c=0; c<std::min((uint32_t)6, imgBuf->channels()); c++) {
    glColor3f(colors_1d[3*c], colors_1d[3*c+1], colors_1d[3*c+2]);
    glBegin(GL_LINES);
    for (uint32_t i=1; i<imgBuf->width()*imgBuf->height(); i++) {
      // data is normalized to [0,1] already
//      glVertex2f((float)i-1, (*data-minval_)/(maxval_-minval_)*GLWidget::height()*1); data++;
//      glVertex2f((float)i, (*data-minval_)/(maxval_-minval_)*GLWidget::height()*1);// data++;
      glVertex2f((float)i-1, (*data)*GLWidget::height()); data++;
      glVertex2f((float)i, (*data)*GLWidget::height());// data++;
    }
    glEnd();
  }
  return true;
}

template<>
bool DisplayGL::internalPaint1d<double>() {
  double* data = &static_cast<Image<double>*> (imgBuf)->data()[slice_offset_];
  for(uint32_t c=0; c<std::min((uint32_t)6, imgBuf->channels()); c++) {
    glColor3f(colors_1d[3*c], colors_1d[3*c+1], colors_1d[3*c+2]);
    glBegin(GL_LINES);
    for (uint32_t i=1; i<imgBuf->width()*imgBuf->height(); i++) {
      // data is normalized to [0,1] already
      glVertex2d((float)i-1, (*data)*GLWidget::height()); data++;
      glVertex2d((float)i, (*data)*GLWidget::height());
    }
    glEnd();
  }
  return true;
}

void DisplayGL::internalPaint1d() {

  if(imgBuf->width() == 1 && imgBuf->height() == 1) {return;}
  if(width() == 0 || height() == 0) {return;}

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  float scaled_width = width()*scale_;
  float scaled_height = height()*scale_;
  glOrtho(0., scaled_width, 0., scaled_height, -1, 1);
  glViewport(0, 0, scaled_width, scaled_height);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glLineWidth(1);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  glBlendFunc(GL_ONE, GL_ZERO);
  glEnable(GL_LINE_SMOOTH);
  glColor3f(0.8, 0.8, 0.2);

  if (imgBuf) {
    switch(data_type_) {
      case TYPE_UCHAR:
      internalPaint1d<unsigned char> ();
      break;

      case TYPE_CHAR:
      internalPaint1d<char> ();
      break;

      case TYPE_USHORT:
      internalPaint1d<unsigned short> ();
      break;

      case TYPE_SHORT:
      internalPaint1d<short> ();
      break;

      case TYPE_UINT:
      internalPaint1d<unsigned int> ();
      break;

      case TYPE_INT:
      internalPaint1d<int> ();
      break;

      case TYPE_FLOAT:
      internalPaint1d<float> ();
      break;

      case TYPE_DOUBLE:
      internalPaint1d<double> ();
      break;
    }

    glDisable(GL_BLEND);
    glDisable(GL_LINE_SMOOTH);

    double minx=0.0;
    double maxx=(double)imgBuf->width()*imgBuf->height();
    drawGrid(minx, minval_, maxx, maxval_);
    drawTags1d();
  }
}

void DisplayGL::drawTags1d() {

  std::pair<ImageBase::tagCIter, ImageBase::tagCIter> range;
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glEnable(GL_POLYGON_SMOOTH);

//  // draw lines
//  imgBuf->lines(range);
//  glBegin(GL_LINES);
//  for (ImageBase::tagCIter pit = range.first; pit != range.second; ++pit) {
//    TagLine* ln = dynamic_cast<TagLine*> ((*pit).second);
//    glColor4f(ln->r(), ln->g(), ln->b(), ln->a());
//    glVertex2f(ln->x0(), height() - 1 - ln->y0());
//    glVertex2f(ln->x1(), height() - 1 - ln->y1());
//  }
//  glEnd();
//
//  for (ImageBase::tagCIter pit = range.first; pit != range.second; ++pit) {
//    TagLine* ln = dynamic_cast<TagLine*> ((*pit).second);
//    glColor4f(ln->r(), ln->g(), ln->b(), ln->a());
//    fr.drawString2D(ln->label(), ln->x1() + crossSize, height() - 1 - ln->y1() + crossSize);
//  }

  // draw points
  imgBuf->points(range);

  glBegin(GL_LINES);
  for (ImageBase::tagCIter pit = range.first; pit != range.second; ++pit) {
    TagPoint* pt = static_cast<TagPoint*> ((*pit).second);
    glColor4f(pt->r(), pt->g(), pt->b(), pt->a());
    glVertex2f(pt->x() - crossSize, height() - 1 - pt->y());
    glVertex2f(pt->x() + crossSize, height() - 1 - pt->y());
    glVertex2f(pt->x(), height() - 1 - pt->y() - crossSize);
    glVertex2f(pt->x(), height() - 1 - pt->y() + crossSize);
  }
  glEnd();

  for (ImageBase::tagCIter pit = range.first; pit != range.second; ++pit) {
    TagPoint* pt = static_cast<TagPoint*> ((*pit).second);
    glColor4f(pt->r(), pt->g(), pt->b(), pt->a());
    fr.drawString2D(pt->label(), pt->x() + crossSize, height() - 1 - pt->y() + crossSize);
  }

  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
}

void DisplayGL::drawGrid(double& minx, double& miny, double& maxx, double& maxy) {
  double grid_x, grid_y;
  double xstep, ystep;
  double numsteps_x = 5, numsteps_y=5;

  if(minx == maxx || miny == maxy) {return;}

  xstep = rint((maxx-minx)/numsteps_x)*(double)GLWidget::width()/(maxx-minx);
//  ystep = rint((maxy-miny)/numsteps_y)*(double)height()/(maxy-miny);
  ystep = (double)GLWidget::height()/(numsteps_y);

  glLineWidth(0.5);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glEnable(GL_LINE_SMOOTH);
  glColor3f(0.4, 0.4, 0.4);
  glBegin(GL_LINES);

  for (grid_x = 0; grid_x <= width(); grid_x+=xstep) {
    glVertex3d(grid_x, 0, GRID_Z);
    glVertex3d(grid_x, height(), GRID_Z);
    }

  if(grid_x != width()) {
    glVertex3d(width(), 0, GRID_Z);
    glVertex3d(width(), height(), GRID_Z);
  }

  for (grid_y = 0; grid_y <= height(); grid_y+=ystep) {
    glVertex3d(0, grid_y, GRID_Z);
    glVertex3d(width(), grid_y, GRID_Z);
    }

  if(grid_y != height()) {
    glVertex3d(0, height(), GRID_Z);
    glVertex3d(width(), height(), GRID_Z);
  }

  glEnd();

  std::stringstream label;

//      for (unsigned int i = 0; i < keys->size(); i++) {
//        res->addPoint((*keys)[i].x, (*keys)[i].y, label.str());

  double x_offset=10;
  double y_offset=-16;
  for (grid_x = 0; grid_x <= GLWidget::width(); grid_x+=xstep) {
    label.str("");
    label << std::fixed << std::setprecision(3) << grid_x*(maxx-minx)/GLWidget::width()+minx;
    fr.drawString2D(label.str(), float(grid_x+x_offset), float(GLWidget::height() + y_offset));
    }

  if(grid_x != GLWidget::width()) {
    label.str("");
    label << std::fixed << std::setprecision(3) << grid_x*(maxx-minx)/GLWidget::width()+minx;
    fr.drawString2D(label.str(), float(grid_x-5*x_offset), float(GLWidget::height() + y_offset));
  }

//  x_offset=10;
//  y_offset=-16;
  for (grid_y = 0; grid_y <= GLWidget::height(); grid_y+=ystep) {
    label.str("");
    label << std::fixed << std::setprecision(3) << grid_y*(maxy-miny)/GLWidget::height()+miny;
    fr.drawString2D(label.str(), x_offset, float(grid_y + y_offset));
    }

  if(grid_y != GLWidget::height()) {
    label.str("");
    label << std::fixed << std::setprecision(3) << grid_y*(maxy-miny)/GLWidget::height()+miny;
    fr.drawString2D(label.str(), x_offset, float(grid_y + y_offset));
  }

  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
}

const float DisplayGL::colors_1d[18] = {1, 0, 0,  0, 1, 0,  0, 0, 1,  1, 1, 0,  1, 0, 1,  0, 1, 1};

} // namespace vlr
