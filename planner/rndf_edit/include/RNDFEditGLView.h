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


#ifndef RNDFEDITGLVIEW_H
#define RNDFEDITGLVIEW_H

#include <glwidget.h>
#include <imagery.h>
#include <gl_support.h>

namespace vlr {

class RNDFEditGLView : public GLWidget
 {
     Q_OBJECT

public:
	GLuint gridMapTexture;
	GLuint tex_type;

public:
	RNDFEditGLView(QWidget *parent = 0);
    ~RNDFEditGLView();

public slots:

protected:
     void initializeGL();
     void paintGL();
//     void resizeGL(int width, int height);

private:
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
  void keyPressEvent(QKeyEvent* event);
  bool wayPointKeys(uint32_t key, Qt::KeyboardModifiers modifiers);
  bool laneKeys(uint32_t key, Qt::KeyboardModifiers modifiers);
  bool segmentKeys(uint32_t key, Qt::KeyboardModifiers modifiers);
  bool exitKeys(uint32_t key, Qt::KeyboardModifiers modifiers);
  bool perimeterPointKeys(uint32_t key, Qt::KeyboardModifiers modifiers);
  bool perimeterKeys(uint32_t key, Qt::KeyboardModifiers modifiers);
  bool spotPointKeys(uint32_t key, Qt::KeyboardModifiers modifiers);
  bool spotKeys(uint32_t key, Qt::KeyboardModifiers modifiers);
  bool zoneKeys(uint32_t key, Qt::KeyboardModifiers modifiers);
  bool trafficLightKeys(uint32_t key, Qt::KeyboardModifiers modifiers);
  bool crosswalkKeys(uint32_t key, Qt::KeyboardModifiers modifiers);

	void drawGrid(double center_x, double center_y);
	void drawSelected(double center_x, double center_y);
	void drawTrajectory(double center_x, double center_y);
	void drawSmoothedLaneStack(double center_x, double center_y);

private:
	Imagery* imagery_;
	FontRenderer fr;
};

} // namespace vlr

#endif // RNDFEDITGLVIEW_H
