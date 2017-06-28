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


#ifndef VLRDISPLAY_H_
#define VLRDISPLAY_H_

#include <QtGui/QtGui>
#include <QtOpenGL/QGLWidget>

#include <displayDefs.h>
#include <displayGL.h>

namespace vlr {

class Display : public QWidget  // TODO: needs to be public to allow classes derived from Display access to arbitrary QWidgets...
{

  Q_OBJECT

public:
  Display(QWidget* parent=NULL);
  Display(uint32_t width, uint32_t height);
  Display(uint32_t width, uint32_t height, int32_t hPos, int32_t vPos);
  Display(uint32_t width, uint32_t height, int32_t hPos, int32_t vPos, QWidget* parent);
  Display(uint32_t width, uint32_t height, displayMode_t mode,
          int32_t hPos, int32_t vPos, double frameRate, QWidget* parent, QGLFormat glFormat);

  Display(ImageBase& img);
  Display(ImageBase& img, int32_t hPos, int32_t vPos);
  Display(ImageBase& img, int32_t hPos, int32_t vPos, QWidget* parent);
  Display(ImageBase& img, displayMode_t mode, int32_t hPos, int32_t vPos, double frameRate,
          QWidget* parent, QGLFormat glFormat);


	~Display();

 protected:
//   Display(DisplayGL* customGLDisplay);
     // This constructor is used for custom GL widgets
   Display(QWidget* parent, int32_t hPos, int32_t vPos);

 public:
	inline bool updateImage(ImageBase& img) {
		if(!glWidget_->updateImage(img)) {return false;}

		if (!parent()) {
      uint32_t width, height;

      if (img.width() == 1 || img.height() == 1) {
        width = DisplayGL::initial_1d_width;
        height = DisplayGL::initial_1d_height;
      }
      else {
        width = img.width();
        height = img.height();
      }

      resize(width, height);
    }

    return true;
  }

  template<class T>
  inline bool updateImage(T* data, uint32_t width, uint32_t height, uint32_t channels, uint32_t padded_width, ImageBase::colorSpace_t cs) {
    if (!glWidget_->updateImage(data, width, height, channels, padded_width, cs)) {
      return false;
    }

    if (!parent()) {
      if (width == 1 || height == 1) {
        width = DisplayGL::initial_1d_width;
        height = DisplayGL::initial_1d_height;
      }
      resize(width, height);
    }

    return true;
  }

  inline bool updateTexture(ImageBase& img)
		{
		if(!glWidget_->updateTexture(img)) {return false;}

//		QResizeEvent* resizeEvent = new QResizeEvent(QSize(img.width(), img.height()), glWidget->size());
//		QCoreApplication::postEvent(glWidget, resizeEvent);

		return true;
		}

  template<class T>
  inline bool updateTexture(T* data, uint32_t width, uint32_t height, uint32_t channels, uint32_t padded_width, ImageBase::colorSpace_t cs) {
    if (!glWidget_->updateTexture(data, width, height, channels, padded_width, cs)) {
      return false;
    }

    return true;
  }

  inline void resizeEvent (QResizeEvent* e) {
		  // resize ourself only
	  QWidget::resize(e->size().width(), e->size().height());
	    // resize child gl widget
		glWidget_->resize(e->size().width(), e->size().height());
		}

	inline void resize(int width, int height) {
    QResizeEvent* resizeEvent = new QResizeEvent(QSize(width, height), glWidget_->size());
    QCoreApplication::postEvent(this, resizeEvent);
	}
	inline bool snapshot(Image<unsigned char>& res) {return glWidget_->snapshot(res);}
	inline displayMode_t displayMode() {return glWidget_->displayMode();}
	inline void setDisplayMode(displayMode_t mode) {glWidget_->setDisplayMode(mode);}
  inline bool textureMode() {return glWidget_->textureMode();}
  inline void setTextureMode(bool onoff) {glWidget_->setTextureMode(onoff);}
	inline void setKeyPressFunc(DisplayGL::keyPressFunc* func) {glWidget_->setKeyPressFunc(func);}
  inline void setMousePressFunc(DisplayGL::mousePressFunc* func) {glWidget_->setMousePressFunc(func);}
  inline void setMouseReleaseFunc(DisplayGL::mouseReleaseFunc* func) {glWidget_->setMouseReleaseFunc(func);}
	inline void setMouseMoveFunc(DisplayGL::mouseMoveFunc* func) {glWidget_->setMouseMoveFunc(func);}

  inline void setParent(QWidget* parent) {QWidget::setParent(parent);}

  inline float heightScale() {return glWidget_->heightScale();}
  inline void setHeightScale(float height_scale) {glWidget_->setHeightScale(height_scale);}

  inline const DisplayGL* glWidget() const {return glWidget_;}
  inline void requestRedraw() {glWidget_->requestRedraw();}

  void show();

  void setCustomGLDisplay(DisplayGL* customGLWidget);

 private:
  #define DEFAULT_WIDTH       640
  #define DEFAULT_HEIGHT      480
  #define DEFAULT_HPOS        -1
  #define DEFAULT_VPOS        -1
  #define DEFAULT_MODE        MODE_2D
  #define DEFAULT_FRAMERATE   30
  #define DEFAULT_FORMAT      QGLFormat(QGL::DoubleBuffer)

 private:
	void create(uint32_t width, uint32_t height, displayMode_t mode, int32_t hPos, int32_t vPos, double frameRate, QGLFormat glFormat);

 private slots:
  void on_action_2d_activated();
  void on_action_3d_activated();

private:
  QGridLayout* gridLayout;
	DisplayGL* glWidget_;
};

}	// namespace vlr

#endif // VLRDISPLAY_H_
