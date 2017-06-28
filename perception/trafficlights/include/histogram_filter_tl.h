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

#ifndef TRAFFIC_LIGHT_HISTOGRAM_FILTER_TL_H
#define TRAFFIC_LIGHT_HISTOGRAM_FILTER_TL_H

#include <cv.h>
#include <highgui.h>
#include <cxtypes.h>
#include <cxcore.h>
#include <detection_grid.h>

class HistogramFilterTL
{
public:
	HistogramFilterTL();
	HistogramFilterTL(int width, int height);
	void UpdateHistogramFilterTL(DetectionGrid *dg, double curr_speed, double detection_grid_spacing);
	~HistogramFilterTL();
	void setMeasGain(double exponent);
	//image access methods (mostly for debug viewer)
	IplImage *getGridPrior();
	IplImage *getWinnersImage();
	void Initialize();
private:
	void NormalizeGrid(double sum);
	void BlurPrior(double curr_speed, double detection_grid_spacing);
	void setWinnersImage(char state, float score, int x, int y);
	IplImage *grid_prior_;
	IplImage *winners_image_;
	int grid_prior_width_;
	int grid_prior_height_;
	int measurement_exponent_;

};

#endif //TRAFFIC_LIGHT_HISTOGRAM_FILTER_TL_H
