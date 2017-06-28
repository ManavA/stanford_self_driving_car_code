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


// needed by Visual C++
#define NOMINMAX

#include <QtGui/QtGui>
#include <QtOpenGL/QtOpenGL>

#include <algorithm>

#include <iostream>
#include <fstream>
#include <glwidget.h>

namespace vlr {

GLWidget::GLWidget(QWidget *parent) :
  QGLWidget(parent), window_width(width()), window_height(height()), fps(DEFAULT_FPS), user_display_func(NULL), user_keyboard_func(NULL),
      user_mouse_func(NULL), user_motion_func(NULL), zoom_sensitivity(DEFAULT_ZOOM_SENSITIVITY), rotate_sensitivity(DEFAULT_ROTATE_SENSITIVITY),
      move_sensitivity(DEFAULT_MOVE_SENSITIVITY), min_zoom_range(DEFAULT_MIN_ZOOM_RANGE), camera_fov(DEFAULT_CAMERA_FOV),
      min_clip_range(DEFAULT_MIN_CLIP_RANGE), max_clip_range(DEFAULT_MAX_CLIP_RANGE), refresh_required(true)

{
  camera_pose.state = IDLE;
  camera_pose.pan = 0;
  camera_pose.tilt = 0;
  camera_pose.distance = 10.0;
  camera_pose.x_offset = 0;
  camera_pose.y_offset = 0;
  camera_pose.z_offset = 0;

  camera_pose.zoom = 1;
  camera_pose.warp_x = 1;
  camera_pose.warp_y = 1;
}

GLWidget::~GLWidget() {
}

void GLWidget::resizeGL(int w, int h) {
  int tx = 0, ty = 0, tw = w, th = h;
  glViewport(tx, ty, tw, th);

  window_width = tw;
  window_height = th;

  init3DMode(tw, th, camera_fov, min_clip_range, max_clip_range);
  requestRedraw();
}

void GLWidget::paintGL(void) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  refresh_required = false;
}

void GLWidget::rotateCamera(double dx, double dy) {
  camera_pose.pan -= float(dx * rotate_sensitivity);
  camera_pose.tilt += float(dy * rotate_sensitivity);

  if (camera_pose.tilt < 0) {
    camera_pose.tilt = 0;
  }
  else if (camera_pose.tilt > 89.0) {
    camera_pose.tilt = 89.0;
  }
}

void GLWidget::zoomCamera(double dy) {
  camera_pose.distance -= float(dy * zoom_sensitivity * camera_pose.distance);
  camera_pose.distance = float(std::max(double(camera_pose.distance), min_zoom_range));
}

void GLWidget::moveCamera(double dx, double dy) {
  camera_pose.x_offset += float(-dy * cos(rad(camera_pose.pan)) * move_sensitivity * camera_pose.distance);
  camera_pose.y_offset += float(-dy * sin(rad(camera_pose.pan)) * move_sensitivity * camera_pose.distance);
  camera_pose.x_offset += float(dx * cos(rad(camera_pose.pan - 90.0)) * move_sensitivity * camera_pose.distance);
  camera_pose.y_offset += float(dx * sin(rad(camera_pose.pan - 90.0)) * move_sensitivity * camera_pose.distance);
}

void GLWidget::setCameraParams(double zoom_sensitivity_, double rotate_sensitivity_, double move_sensitivity_, double min_zoom_range_, double camera_fov_,
    double min_clip_range_, double max_clip_range_) {
  zoom_sensitivity = zoom_sensitivity_;
  rotate_sensitivity = rotate_sensitivity_;
  move_sensitivity = move_sensitivity_;
  min_zoom_range = min_zoom_range_;
  camera_fov = camera_fov_;
  min_clip_range = min_clip_range_;
  max_clip_range = max_clip_range_;

  requestRedraw();
}

void GLWidget::setInitialCameraPos(float pan, float tilt, float range, float x_offset, float y_offset, float z_offset) {
  camera_pose.pan = pan;
  camera_pose.tilt = tilt;
  camera_pose.distance = range;
  camera_pose.x_offset = x_offset;
  camera_pose.y_offset = y_offset;
  camera_pose.z_offset = z_offset;
}

void GLWidget::initializeGL() {
  float light_ambient[] = { 0, 0, 0, 0 };
  float light_diffuse[] = { 1, 1, 1, 1 };
  float light_specular[] = { 1, 1, 1, 1 };
  float light_position[] = { 0, 0, 100, 0 };

  glEnable( GL_DEPTH_TEST);
  glShadeModel( GL_SMOOTH);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glEnable( GL_LIGHT0);
  glDisable( GL_LIGHTING);
  glEnable( GL_NORMALIZE);

  connect(&timer, SIGNAL(timeout()), this, SLOT(redraw(void)));

  timer.start(DISPLAY_REFRESH_DELAY_MS);
}

void GLWidget::setDisplayFunc(display_func func) {
  user_display_func = func;
}

void GLWidget::setMouseFunc(mouse_func func) {
  user_mouse_func = func;
}

void GLWidget::setMotionFunc(motion_func func) {
  user_motion_func = func;
}

void GLWidget::requestRedraw() {
  refresh_required = true;
}

void GLWidget::redraw() {
  if (refresh_required) {
    updateGL();
  }
}

void GLWidget::init3DMode(int w, int h, double& fovy, double& zNear, double& zFar) {
  glEnable( GL_DEPTH_TEST);
  glMatrixMode( GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fovy, w / (float) h, zNear, zFar);
  glMatrixMode( GL_MODELVIEW);
  glLoadIdentity();
}

void GLWidget::activate3DMode() {
  float cpan, ctilt, camera_x, camera_y, camera_z;

  /* setup camera view */
  cpan = rad(camera_pose.pan);
  ctilt = rad(camera_pose.tilt);
  camera_x = camera_pose.distance * cos(cpan) * cos(ctilt);
  camera_y = camera_pose.distance * sin(cpan) * cos(ctilt);
  camera_z = camera_pose.distance * sin(ctilt);

  init3DMode(window_width, window_height, camera_fov, min_clip_range, max_clip_range);

  glViewport(0, 0, (GLsizei) window_width, (GLsizei) window_height);
  gluLookAt(camera_x + camera_pose.x_offset, camera_y + camera_pose.y_offset, camera_z + camera_pose.z_offset, camera_pose.x_offset, camera_pose.y_offset,
      camera_pose.z_offset, 0, 0, 1);
}

void GLWidget::recenter() {
  camera_pose.x_offset = 0;
  camera_pose.y_offset = 0;
  camera_pose.z_offset = 0;
}

void GLWidget::pickPoint(int mouse_x, int mouse_y, double *scene_x, double *scene_y) {
  double cx = window_width / 2.0;
  double cy = window_height / 2.0;
  double pan = rad(-90.0 - camera_pose.pan);
  double tilt = rad(90.0 - camera_pose.tilt);
  double d = camera_pose.distance;
  double f = cy / tan(rad(camera_fov / 2.0));

  double px = (mouse_x - cx) * cos(tilt) * d / (cos(tilt) * f + sin(tilt) * mouse_y - sin(tilt) * cy);
  double py = -(mouse_y - cy) * d / (cos(tilt) * f + sin(tilt) * mouse_y - sin(tilt) * cy);

  // rotate by pan, add offset
  *scene_x = px * cos(pan) + py * sin(pan) + camera_pose.x_offset;
  *scene_y = -px * sin(pan) + py * cos(pan) + camera_pose.y_offset;
}

// QSize GLWidget::minimumSizeHint() const
// {
//     return QSize(50, 50);
// }
//
// QSize GLWidget::sizeHint() const
// {
//     return QSize(400, 400);
// }

void GLWidget::mousePressEvent(QMouseEvent* event) {
  last_mouse_x = event->x();
  last_mouse_y = event->y();
}

void GLWidget::mouseReleaseEvent(QMouseEvent* event) {
}

void GLWidget::mouseMoveEvent(QMouseEvent *event) {
  int dx = event->x() - last_mouse_x;
  int dy = event->y() - last_mouse_y;

  if (event->buttons() & Qt::LeftButton) {
    rotateCamera(dx, dy);
  }
  else if (event->buttons() & Qt::MidButton) {
    moveCamera(dx, dy);
  }
  else if (event->buttons() & Qt::RightButton) {
    zoomCamera(dy);
  }

  if (user_motion_func) {
    user_motion_func(int(event->x()), int(event->y()));
  }
  last_mouse_x = event->x();
  last_mouse_y = event->y();

  requestRedraw();
}

} // namespace vlr
