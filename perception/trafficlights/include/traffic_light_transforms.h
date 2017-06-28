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

#ifndef TRAFFIC_LIGHT_TRANSFORMS
#define TRAFFIC_LIGHT_TRANSFORMS

#include <roadrunner.h>
#include <applanix_interface.h>
#include <transform.h>
#include <localize_messages.h>
#include <cv.h>
#include <camera_interface.h>

using namespace dgc;

struct CameraParams
{
	int32_t frame_rate;
	double  pixel_size;
	double  cx;
	double  cy;
	double  Fx;
	double  Fy;
	CameraInfo info;
	CvRect  roi;
	char image_fmt[10];
	dgc_transform_t trans; 
};

class TrafficLightTransform
{
private:
	double cx, cy, Fx, Fy;
	dgc_transform_t cameraTrans;

public:
	TrafficLightTransform()	{}

	void set_camera_trans_matrix(dgc_transform_t cameraT);
	void set_camera_params(CameraParams&);

	//TRANSFORM FUNCTIONS:

	//global->smooth
	void global_to_smooth(double gx, double gy, double gz, double x_offset, double y_offset, 
			double *sx, double *sy, double *sz);

	//smooth->robot
	void smooth_to_robot(double sx, double sy, double sz, const ApplanixPose& currPose, double *rx, 
			double *ry, double *rz);

	//robot->camera
	void robot_to_Cam(double rx, double ry, double rz, double *camX, double *camY, double *camZ);

	//camera->image plane (assuming X points out from camera, y to the left, z up
	void Cam_to_imagePlane(double camX, double camY, double camZ, unsigned int im_width, 
			unsigned int im_height, double *u, double *v);

	void BackProject(double u, double v, unsigned int im_width, unsigned int im_height, 
			const ApplanixPose& currPose, double *d[3]);

	void imagePlane_to_OpenGLObj(int u, int v, int im_width, int im_height, double *objX, double *objY);

	void globalToUV(int im_width, int im_height, double gxLight, double gyLight, double gzLight,
	                int *u, int *v, double *robotXp, const LocalizePose& loc_pose, const ApplanixPose& currPose);

};


#endif
