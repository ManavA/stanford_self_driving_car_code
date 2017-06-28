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

#ifndef TRAFFIC_LIGHT_IMAGE_DATA_H
#define TRAFFIC_LIGHT_IMAGE_DATA_H

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cv.h>
#include <highgui.h>
#include <cxtypes.h>
#include <cxcore.h>
#include <detection_grid.h>

#define MAX_ROI_SIZE MAX_GRID_SIZE_TL*MAX_GRID_SIZE_TL*2  

#include <ShapeTable.h>

struct Point
{
	int X;
	int Y;
};

class ImageData
{

public:
	ImageData(bool, ShapeTable<uint8_t>*, ShapeTable<uint8_t>*, ShapeTable<uint8_t>*, ShapeTable<uint8_t>*, 
			const char**);
	bool UpdateDataScores(DetectionGrid *dg, IplImage *curr_image, 
			double dist_to_light, bool downsample, IplImage* videoframe, CvRect subIm, double* p);
	
	// for debug viewer:
	int getLastTplWidth();
	int getLastTplHeight();
	
	// helper functions:
	static void FindMinMax(int a, int b, int c, int d, int *min, int *max);

private:
	//region of interest within the camera image
	//(usually set as min and max bounds of the projection of detection grid onto the image)
	int roi_u0;
	int roi_v0;
	int roi_u_len;
	int roi_v_len;

	int conv_roi_u0;
	int conv_roi_v0;
	int conv_roi_u1;
	int conv_roi_v1;

	//the dimensions of the most recently-used light template
	int last_tpl_width_;
	int last_tpl_height_;

	// shape tables
	ShapeTable<uint8_t> stRedHue_;
	ShapeTable<uint8_t> stYlwHue_;
	ShapeTable<uint8_t> stGrnHue_;
	ShapeTable<uint8_t> stSaturation_;
	ShapeTable<uint8_t> stFrameVal_;

	///// calibration values from param server
	
	// traffic light lens radius = (m_ / x) + b_
	//   where x is distance from camera to lens in meters.
	int m_;
	int b_;
	int n_;
	int c_;

	//major functions:
	IplImage* GetLightTemplate(double dist_to_light, bool downsample);

	void ConvolveHue
		(IplImage *tpl, IplImage *img, IplImage* diffs, int frame_weight, int lens_weight, 
		 ShapeTable<uint8_t>* h_table, ShapeTable<uint8_t>* s_table, ShapeTable<uint8_t>* v_table);

	void ConvolveCircleTemplate(IplImage* ssd, IplImage* diffs, double dist_to_light, bool downsample);
	
	void CalculateCellScore(int, int, IplImage *, IplImage *, IplImage *,
	                        std::vector<Point> *, float *, float *, float *);
	
	void CalculatePixelScore(int, int, IplImage *, IplImage *, IplImage *,
	                         float *, float *, float *);
	
	// aka normalize_circle_ssds
	void NormalizeFeatures(IplImage *, IplImage *, IplImage *); 

	void RecalcMinMax(IplImage* a, float& min, float& max);

	//time it takes to perform the weighted SSD on the ROI in this camera frame
	//the time is proportional to size of grid projection the frame and is set by ImageData class methods
	double cpu_time_;

	//for viewing SSDs, if in debug mode
	bool debug_mode_;
	const char** window;
};

#endif // TRAFFIC_LIGHT_IMAGE_DATA_H

