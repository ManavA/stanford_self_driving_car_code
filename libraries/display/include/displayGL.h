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


#ifndef VLRDISPLAY_DISPLAYGL_H_
#define VLRDISPLAY_DISPLAYGL_H_

#include <QtGui/QtGui>
#include <QtOpenGL/QGLWidget>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <vlrImaging.h>
#include <fontRenderer.h>

#include <displayDefs.h>
#include <glWidget.h>

namespace vlr {

class DisplayGL : public GLWidget
{
     Q_OBJECT

public:
	typedef void (keyPressFunc)(unsigned int);
  typedef void (mousePressFunc)(int x, int y, unsigned char buttons);
  typedef void (mouseReleaseFunc)(int x, int y, unsigned char buttons);
	typedef void (mouseMoveFunc)(int x, int y, unsigned char buttons);

public:
  DisplayGL();
  DisplayGL(QWidget *parent, displayMode_t mode, double frameRate, QGLFormat glFormat);
    ~DisplayGL();

    bool updateImage(ImageBase& img);

    template <class T>
    bool updateImage(T* data, uint32_t width, uint32_t height, uint32_t channels, uint32_t padded_width, ImageBase::colorSpace_t cs) {
      boost::unique_lock<boost::mutex> lock(mutex_);
      Image<T> img(width, height, channels, padded_width, false, cs);
      img.setData(data);
      return updateBuffer(img, &imgBuf, bufColorFormat);
    }

    bool updateTexture(ImageBase& img);
    template <class T>
    bool updateTexture(T* data, uint32_t width, uint32_t height, uint32_t channels, uint32_t padded_width, ImageBase::colorSpace_t cs) {
      boost::unique_lock<boost::mutex> lock(mutex_);
      Image<T> img(width, height, channels, padded_width, false, cs);

      img.setData(data);
      return updateBuffer(img, &texBuf, texColorFormat);
    }

    void removeTexture();

	displayMode_t displayMode() {return mode_;}
	void setDisplayMode(displayMode_t mode) {
    boost::unique_lock<boost::mutex> lock(mutex_);
		mode_=mode;
		}

  bool textureMode() {return useTexture;}
  void setTextureMode(bool onoff) {useTexture=onoff;}

  float scale() {return scale_;}
  float heightScale() {return heightScale_;}

  void setHeightScale(float heightScale) {
    heightScale_=heightScale;
    requestRedraw();
  }

  bool snapshot(Image<unsigned char>& res);
	void setKeyPressFunc(keyPressFunc* func) {userKeyPressFunc=func;}
	void setMousePressFunc(mousePressFunc* func) {userMousePressFunc=func;}
  void setMouseMoveFunc(mouseMoveFunc* func) {userMouseMoveFunc=func;}
  void setMouseReleaseFunc(mouseReleaseFunc* func) {userMouseReleaseFunc=func;}

  virtual void drawCustomTags1d() {}
  virtual void drawCustomTags2d() {}
  virtual void drawCustomTags3d() {}

public slots:

 protected:
     virtual void initializeGL();
     virtual void paintGL();
     virtual void resizeEvent ( QResizeEvent * event );
     virtual void mousePressEvent(QMouseEvent *event);
     virtual void mouseReleaseEvent(QMouseEvent *event);
     virtual void mouseMoveEvent(QMouseEvent *event);
     virtual void keyPressEvent(QKeyEvent* event);

 protected:
    void drawGrid(double& minx, double& miny, double& maxx, double& maxy); // used for 1d display flavor
    void drawGridXY(float zPos);
    void drawGridXZ(float yPos);
    void drawGridYZ(float xPos);

 private:
    void create(displayMode_t mode, double frame_rate);
    void initLights();
    void internalPaint1d();
    void internalPaint2d();
    void internalPaint3d();
    template <class T> bool internalPaint1d();
    template <class T> bool internalPaint2d();
    template <class T> bool internalPaint3d();
    template <class T> bool internalPaint3dRGB();
	  template <class T, class TT> bool internalPaint3dTextureRGB();
	  template <class T, class TT> bool internalPaint3dTexture();

    void drawTags1d();
    void drawTags2d();
//    void drawTags3d();
    bool updateBuffer(ImageBase& img, ImageBase** dest, int& destColorFormat);

    template <class T> bool makeImageBuffer(Image<T>& img, Image<T>** dest, int& destColorFormat);
    template <class T> bool createGammaMap(unsigned int mapSize, double gamma, T* map);

 public:
   static const unsigned int initial_1d_width=640, initial_1d_height=480;

 protected:
   GLuint texType;
   GLuint imgTexture;
   ImageBase* imgBuf;
   ImageBase* texBuf;

 private:
   typedef enum {TYPE_CHAR, TYPE_UCHAR, TYPE_SHORT, TYPE_USHORT,
                 TYPE_INT, TYPE_UINT, TYPE_FLOAT, TYPE_DOUBLE} data_type_t;

	displayMode_t mode_;
	double refreshTimeMS;
	bool showGridXY, showGridXZ, showGridYZ;
	int bufColorFormat;
	int texColorFormat;
	bool useColorMap;
	bool useTexture;
	bool normalize_data_;
	double gamma;
	keyPressFunc* userKeyPressFunc;
  mousePressFunc* userMousePressFunc;
  mouseReleaseFunc* userMouseReleaseFunc;
	mouseMoveFunc* userMouseMoveFunc;
  float gamma_map_red_[256];
  float gamma_map_green_[256];
  float gamma_map_blue_[256];
  float gamma_map_alpha_[256];
	float heightScale_;
	float scale_;
  unsigned int current_slice_;
  unsigned int slice_offset_;
  double minval_, maxval_;
  data_type_t data_type_;   // dynamic_cast doesn't work well across shared objects

protected:
  bool create_snapshot_;
	unsigned char* snapshot_buf_;
  boost::mutex mutex_;
  FontRenderer fr;

private:
  static const float cmap_rb1_red_[256], cmap_rb1_green_[256], cmap_rb1_blue_[256];
  static const double GRID_Z=0.001;
	const float* color_map_red_, *color_map_green_, *color_map_blue_, *color_map_alpha_;
	static const float colors_1d[18];

	static const int crossSize=7;
	static const double third = 0.333333333333333;
};

}	// namespace vlr

#endif // VLRDISPLAY_DISPLAYGL_H_
