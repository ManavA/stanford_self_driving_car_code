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
#include <QtOpenGL/QtOpenGL>

#include <algorithm>

#include <iostream>
#include <fstream>
#include <glWidget.h>

namespace vlr {

GLWidget::GLWidget(QGLFormat glFormat, QWidget *parent)
: QGLWidget(glFormat, parent),
  windowWidth(width()), windowHeight(height()), fps(DEFAULT_FPS),
  userDisplayFunction(NULL), userKeyboardFunction(NULL), userMouseFunction(NULL), userMotionFunction(NULL),
  zoomSensitivity(DEFAULT_ZOOM_SENSITIVITY),
  rotateSensitivity(DEFAULT_ROTATE_SENSITIVITY),
  moveSensitivity(DEFAULT_MOVE_SENSITIVITY),
  minZoomRange(DEFAULT_MIN_ZOOM_RANGE),
  cameraFov(DEFAULT_CAMERA_FOV),
  minClipRange(DEFAULT_MIN_CLIP_RANGE),
  maxClipRange(DEFAULT_MAX_CLIP_RANGE),
  refresh_required(true), gl_initialized_(false)
{
	memcpy(ambientLight, ambientLightDefault, sizeof(ambientLightDefault));
	memcpy(diffuseLight, diffuseLightDefault, sizeof(diffuseLightDefault));
	memcpy(specularLight, specularLightDefault, sizeof(specularLightDefault));
	memcpy(lightPosition, lightPositionDefault, sizeof(lightPositionDefault));

	cameraPose.state = IDLE;
	cameraPose.pan = 0;
	cameraPose.tilt = 0;
	cameraPose.distance = 10.0;
	cameraPose.xOffset = 0;
	cameraPose.yOffset = 0;
	cameraPose.zOffset = 0;

	cameraPose.zoom = 1;
	cameraPose.warp_x = 1;
	cameraPose.warp_y = 1;
}

GLWidget::~GLWidget() {}

QSize GLWidget::minimumSizeHint() const
{
	return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
	return QSize(400, 400);
}

void GLWidget::resizeGL(int w, int h)
{
	if(!gl_initialized_) {return;}

	makeCurrent();
	int tx = 0, ty = 0, tw = w, th = h;
	glViewport(tx, ty, tw, th);

	windowWidth = tw;
	windowHeight = th;

	init3DMode(tw, th, cameraFov, minClipRange, maxClipRange);
	requestRedraw();
}

void GLWidget::paintGL(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	refresh_required=false;
}

void GLWidget::rotateCamera(double dx, double dy)
{
	cameraPose.pan -= float(dx * rotateSensitivity);
	cameraPose.tilt += float(dy * rotateSensitivity);

	if (cameraPose.tilt < 0) {cameraPose.tilt = 0;}
	else if (cameraPose.tilt > 89.0) {cameraPose.tilt = 89.0;}
}

void GLWidget::zoomCamera(double dy)
{
	cameraPose.distance -= float(dy * zoomSensitivity* cameraPose.distance);
	cameraPose.distance = float(std::max(double(cameraPose.distance), minZoomRange));
}

void GLWidget::moveCamera(double dx, double dy)
{
	cameraPose.xOffset += float(-dy * cos(rad(cameraPose.pan)) *moveSensitivity* cameraPose.distance);
	cameraPose.yOffset += float(-dy * sin(rad(cameraPose.pan)) *moveSensitivity* cameraPose.distance);
	cameraPose.xOffset += float( dx * cos(rad(cameraPose.pan - 90.0)) *moveSensitivity
			* cameraPose.distance);
	cameraPose.yOffset += float(dx * sin(rad(cameraPose.pan - 90.0)) *moveSensitivity
			* cameraPose.distance);
}

void GLWidget::setCameraParams(double zoomSensitivity_, double rotateSensitivity_, double moveSensitivity_, double minZoomRange_,
		double cameraFov, double minClipRange_, double maxClipRange_)
{
	zoomSensitivity = zoomSensitivity_;
	rotateSensitivity = rotateSensitivity_;
	moveSensitivity = moveSensitivity_;
	minZoomRange = minZoomRange_;
	cameraFov = cameraFov;
	minClipRange = minClipRange_;
	maxClipRange = maxClipRange_;

	requestRedraw();
}

void GLWidget::setInitialCameraPos(float pan, float tilt, float range, float xOffset, float yOffset, float zOffset)
{
	cameraPose.pan = pan;
	cameraPose.tilt = tilt;
	cameraPose.distance = range;
	cameraPose.xOffset = xOffset;
	cameraPose.yOffset = yOffset;
	cameraPose.zOffset = zOffset;
}

void GLWidget::initializeGL(void)
{
//	create();
//	makeCurrent();

	if (QGLContext::currentContext()) {
		std::cout << "QGLContext is valid" << std::endl;
	}
//	if (QGLContext::valid()) {
//		std::cout << "QGLContext is valid" << std::endl;
//	}
//	if (QGLContext::isSharing()) {
//		std::cout << "QGLContext is sharing" << std::endl;
//	}
//	if (QGLContext::initialized()) {
//		std::cout << "QGLContext is initialized" << std::endl;
//	}
//	if (QGLContext::windowCreated()) {
//		std::cout << "QGLContext windowCreated" << std::endl;
//	}

	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glEnable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);

	connect(&timer, SIGNAL(timeout()), this, SLOT(redraw(void)));

	timer.start(DISPLAY_REFRESH_DELAY_MS);
	gl_initialized_=true;
}

void GLWidget::setDisplayFunction(DisplayFunction func) {userDisplayFunction = func;}

void GLWidget::setMouseFunction(MouseFunction func) {userMouseFunction = func;}

void GLWidget::setMotionFunction(MotionFunction func) {userMotionFunction = func;}

void GLWidget::requestRedraw() {refresh_required=true;}

void GLWidget::redraw()
{
	if (refresh_required)
	{
		updateGL();
	}
}

void GLWidget::init3DMode(int w, int h, double& fovY, double& zNear, double& zFar)
{
	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fovY, w / (float)h, zNear, zFar);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}


void GLWidget::activate3DMode()
{
	float cpan, ctilt, cameraX, cameraY, cameraZ;

	/* setup camera view */
	cpan = rad(cameraPose.pan);
	ctilt = rad(cameraPose.tilt);
	cameraX = cameraPose.distance * cos(cpan) * cos(ctilt);
	cameraY = cameraPose.distance * sin(cpan) * cos(ctilt);
	cameraZ = cameraPose.distance * sin(ctilt);

	init3DMode(windowWidth, windowHeight, cameraFov, minClipRange, maxClipRange);

	glViewport(0, 0, (GLsizei)windowWidth, (GLsizei)windowHeight);
	gluLookAt(cameraX + cameraPose.xOffset, cameraY + cameraPose.yOffset, cameraZ + cameraPose.zOffset, cameraPose.xOffset,
			cameraPose.yOffset, cameraPose.zOffset, 0, 0, 1);
}

void GLWidget::activate2DMode() {
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//  gluOrtho2D(0.0, (GLfloat)windowWidth, 0.0, (GLfloat)windowHeight);
	glOrtho(0.0, (GLfloat)windowWidth, 0.0, (GLfloat)windowHeight, -1000, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}
void GLWidget::recenter()
{
	cameraPose.xOffset = 0;
	cameraPose.yOffset = 0;
	cameraPose.zOffset = 0;
}

void GLWidget::pickPoint(int mouseX, int mouseY, double *scene_x, double *scene_y)
{
	double cx = windowWidth / 2.0;
	double cy = windowHeight / 2.0;
	double pan = rad(-90.0 - cameraPose.pan);
	double tilt = rad(90.0 - cameraPose.tilt);
	double d = cameraPose.distance;
	double f = cy / tan(rad(cameraFov / 2.0));

	double px = (mouseX - cx) * cos(tilt) * d /(cos(tilt) * f + sin(tilt) * mouseY - sin(tilt) * cy);
	double py = -(mouseY - cy) * d /(cos(tilt) * f + sin(tilt) * mouseY - sin(tilt) * cy);

	// rotate by pan, add offset
	*scene_x = px * cos(pan) + py * sin(pan) + cameraPose.xOffset;
	*scene_y = -px * sin(pan) + py * cos(pan) + cameraPose.yOffset;
}

void GLWidget::mousePressEvent(QMouseEvent* event) {
	lastMouseX = event->x();
	lastMouseY = event->y();
}

void GLWidget::mouseReleaseEvent(QMouseEvent*) {}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastMouseX;
	int dy = event->y() - lastMouseY;

	if (event->buttons() & Qt::LeftButton) {rotateCamera(dx, dy);}
	else if (event->buttons() & Qt::MidButton) {moveCamera(dx, dy);}
	else if (event->buttons() & Qt::RightButton) {zoomCamera(dy);}

	if (userMotionFunction) {userMotionFunction(int(event->x()), int(event->y()));}
	lastMouseX = event->x();
	lastMouseY = event->y();

	requestRedraw();
}

const float GLWidget::ambientLightDefault[4] = {0, 0, 0, 0};
const float GLWidget::diffuseLightDefault[4] = {1, 1, 1, 1};
const float GLWidget::specularLightDefault[4] = {1, 1, 1, 1};
const float GLWidget::lightPositionDefault[4] = {0, 0, 100, 0};


}	// namespace vlr
