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


#include <QtGui/QtGui>
#include <QtOpenGL/QGLWidget>

#include <display.h>

namespace vlr {

class ToolBar : public QToolBar {
public:
  ToolBar(QWidget* parent=NULL) : QToolBar(parent) {}
  ~ToolBar() {}

  void paintEvent(QPaintEvent *) {
       QPainter p(this);
//       p.setRenderHint(QPainter::Antialiasing);
       p.setBackgroundMode(Qt::TransparentMode);
//       p.fillRect(rect(), QColor(0x00FF00A0));

       p.save();

//       extern void render_qt_text(QPainter *, int32_t, int32_t, const QColor &);
//       render_qt_text(&p, width(), height(), fgColorForName(color));

       p.restore();
   }
};

class ToolButton : public QToolButton {
public:
  ToolButton(QWidget* parent=NULL) : QToolButton(parent) {}
  ~ToolButton() {}

  void paintEvent(QPaintEvent *) {
       QPainter p(this);
//       p.setRenderHint(QPainter::Antialiasing);
       p.fillRect(rect(), QColor(0x00FF00A0));

       p.save();

//       extern void render_qt_text(QPainter *, int32_t, int32_t, const QColor &);
//       render_qt_text(&p, width(), height(), fgColorForName(color));

       p.restore();
   }
};


Display::Display(QWidget* parent) : QWidget(parent), glWidget_(NULL) {
  create(DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_MODE, DEFAULT_HPOS, DEFAULT_VPOS, DEFAULT_FRAMERATE, DEFAULT_FORMAT);
  glWidget_->requestRedraw();
}

Display::Display(uint32_t width, uint32_t height) : QWidget(NULL), glWidget_(NULL) {
  create(width, height, DEFAULT_MODE, DEFAULT_HPOS, DEFAULT_VPOS, DEFAULT_FRAMERATE, DEFAULT_FORMAT);
  glWidget_->requestRedraw();
}

Display::Display(uint32_t width, uint32_t height, int32_t hPos, int32_t vPos) : QWidget(NULL), glWidget_(NULL) {
  create(width, height, DEFAULT_MODE, hPos, vPos, DEFAULT_FRAMERATE, DEFAULT_FORMAT);
  glWidget_->requestRedraw();
}

Display::Display(uint32_t width, uint32_t height, int32_t hPos, int32_t vPos, QWidget* parent) :
  QWidget(parent), glWidget_(NULL) {
  create(width, height, DEFAULT_MODE, hPos, vPos, DEFAULT_FRAMERATE, DEFAULT_FORMAT);
  glWidget_->requestRedraw();
}

Display::Display(uint32_t width, uint32_t height, displayMode_t mode, int32_t hPos, int32_t vPos,
                 double frameRate, QWidget* parent, QGLFormat glFormat) :
                   QWidget(parent), glWidget_(NULL) {
  create(width, height, mode, hPos, vPos, frameRate, glFormat);
  glWidget_->requestRedraw();
}

Display::Display(ImageBase& img) : QWidget(NULL), glWidget_(NULL) {
  create(img.width(), img.height(), DEFAULT_MODE, DEFAULT_HPOS, DEFAULT_VPOS, DEFAULT_FRAMERATE, DEFAULT_FORMAT);
  glWidget_->updateImage(img);
}

Display::Display(ImageBase& img, int32_t hPos, int32_t vPos) : QWidget(NULL), glWidget_(NULL) {
  create(img.width(), img.height(), DEFAULT_MODE, hPos, vPos, DEFAULT_FRAMERATE, DEFAULT_FORMAT);
  glWidget_->updateImage(img);
}

Display::Display(ImageBase& img, int32_t hPos, int32_t vPos, QWidget* parent) : QWidget(parent), glWidget_(NULL) {
  create(img.width(), img.height(), DEFAULT_MODE, hPos, vPos, DEFAULT_FRAMERATE, DEFAULT_FORMAT);
  glWidget_->updateImage(img);
}

Display::Display(ImageBase& img, displayMode_t mode, int32_t hPos, int32_t vPos, double frameRate,
                 QWidget* parent, QGLFormat glFormat) :
                   QWidget(parent), glWidget_(NULL) {
  create(img.width(), img.height(), mode, hPos, vPos, frameRate, glFormat);
  glWidget_->updateImage(img);
}

Display::Display(QWidget* parent, int32_t hPos, int32_t vPos) : QWidget(parent), glWidget_(NULL) {
  create(DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_MODE, DEFAULT_HPOS, DEFAULT_VPOS, DEFAULT_FRAMERATE, DEFAULT_FORMAT);
  glWidget_->requestRedraw();

  if (hPos >= 0 && vPos >= 0) {
    move(hPos, vPos);
  }
}

Display::~Display() {
  if(glWidget_) {delete glWidget_;}
}

void Display::create(uint32_t width, uint32_t height, displayMode_t mode, int32_t hPos, int32_t vPos,
                      double frameRate, QGLFormat glFormat) {

  if(thread() != QApplication::instance()->thread()) {
    throw VLRException("Display was not created from GUI thread.");
  }

  if (hPos >= 0 && vPos >= 0) {
    move(hPos, vPos);
  }

  if(width == 1 || height == 1) {
    width=DisplayGL::initial_1d_width;
    height=DisplayGL::initial_1d_height;
  }
  setBaseSize(width, height);
  QWidget::resize(width, height);

  gridLayout = NULL;
  //  gridLayout = new QGridLayout(this);

  glWidget_ = new DisplayGL(this, mode, frameRate, glFormat);

  glWidget_->resize(width, height);
//  gridLayout->setContentsMargins(0, 0, 0, 0);
//  gridLayout->addWidget(glWidget_);

//  QToolBar* toolbar = new ToolBar(this);
//
//  QAction* action_2d = new QAction("2D", NULL);
//  QAction* action_3d = new QAction("3D", NULL);

//  ToolButton* tbutton_2d = new ToolButton(NULL);
//  toolbar->addWidget(tbutton_2d);

//  toolbar->addAction(action_2d);
//  toolbar->addAction(action_3d);
//  toolbar->setFloatable(true);
//  toolbar->setMovable(true);
//  toolbar->setIconSize(QSize(16,12));
//  action_2d->connect(action_2d, SIGNAL(activated()), this, SLOT(on_action_2d_activated()));
//  action_3d->connect(action_3d, SIGNAL(activated()), this, SLOT(on_action_3d_activated()));

//  QButtonGroup* buttons_ = new QButtonGroup(gridLayout);
//  gridLayout->addWidget((QWidget*)buttons_);
//  addToolBar(Qt::TopToolBarArea, toolbar);
  glWidget_->setFocus();

//  raise();
//  show();
}

void Display::on_action_2d_activated() {
  setDisplayMode(MODE_2D);
  glWidget_->requestRedraw();
}

void Display::on_action_3d_activated() {
  setDisplayMode(MODE_3D);
  glWidget_->requestRedraw();
}

void Display::setCustomGLDisplay(DisplayGL* customGLWidget) {

  if(!customGLWidget)
    {throw("zero pointer to custom GL widget.");}

  if(glWidget_) {delete glWidget_;}
  if(gridLayout) {delete gridLayout;}

  glWidget_=customGLWidget;

  glWidget_->resize(width(), height());

  glWidget_->setFocus();
  glWidget_->setParent(this);
  glWidget_->show();
  setContentsMargins(0, 0, 0, 0);
}

void Display::show() {
  raise();
  QWidget::show();
}

} // namespace vlr
