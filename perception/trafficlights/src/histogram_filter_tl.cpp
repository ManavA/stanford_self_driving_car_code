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

#include "histogram_filter_tl.h"

//default constructor should not be called, but is called initially by stl map
HistogramFilterTL::HistogramFilterTL()
{
	grid_prior_width_ = -1;
	grid_prior_height_ = -1;
	winners_image_ = NULL;
}

HistogramFilterTL::HistogramFilterTL(int grid_dim_width, int grid_dim_height)
{
	//printf("width %d, height %d\n", grid_dim_width, grid_dim_height);
	grid_prior_ = cvCreateImage( cvSize(grid_dim_width,grid_dim_height), IPL_DEPTH_32F, 1);
	winners_image_ = cvCreateImage( cvSize(grid_dim_width,grid_dim_height), IPL_DEPTH_8U, 3);
	// TODO: I don't remember adding these lines. figure out why they are there.
//	grid_prior_->origin = 1;
//	winners_image_->origin = 1;
	grid_prior_width_ = grid_dim_width;
	grid_prior_height_ = grid_dim_height;
	measurement_exponent_ = 4;
	Initialize();
}

HistogramFilterTL::~HistogramFilterTL()
{
	cvReleaseImage(&grid_prior_);
	if (winners_image_ != NULL)
	{
		cvReleaseImage(&winners_image_);
	}
}

void HistogramFilterTL::Initialize()
{
	//set all values of prior to 1.0
	for (int y = 0; y < grid_prior_height_; y++)
	{
		float *grid_row = ((float *)(grid_prior_->imageData + y*grid_prior_->widthStep));

		for (int x = 0; x < grid_prior_width_; x++)
			grid_row[x] = 1.0;
	}
}

void HistogramFilterTL::BlurPrior(double curr_speed, double detection_grid_spacing)
{
	//use opencv's optimized gaussian blur:
	//int blur_radius = (int)((0.3 + curr_speed/25.0)/detection_grid_spacing);
	//for VAIL:
	int blur_radius = (int)((0.3 + curr_speed/40.0)/detection_grid_spacing);
	if (blur_radius%2 == 0)
	{
		blur_radius++;
	}

	//the stdevs MUST be odd or opencv crashes
	cvSmooth(grid_prior_, grid_prior_, CV_GAUSSIAN, blur_radius, blur_radius); 
}

void HistogramFilterTL::NormalizeGrid(double sum)
{
	if (sum == 0.) return;
	for (int y = 0; y < grid_prior_->height; y++)
	{
		float *grid_row = ((float *)(grid_prior_->imageData + y*grid_prior_->widthStep));

		for (int x = 0; x < grid_prior_->width; x++)
		{
			float prob = grid_row[x];
			prob /=sum;
			grid_row[x] = prob;
		}
	}
}

void HistogramFilterTL::setMeasGain(double exponent)
{
	measurement_exponent_ = exponent;
}

void HistogramFilterTL::setWinnersImage(char state, float score, int x, int y)
{
	uchar R, G, B;
	R=G=B=0;
	switch (state)
	{
	case 'r':
		R = 255;
		G = 21;
		break;
	case 'g':
		G = 255;
		B = 170;
		break;
	case 'y':
		R = 255;
		G = 127;
		break;
	default:
		break;
	}

	R = (uchar)(R*score);
	G = (uchar)(G*score);
	B = (uchar)(B*score);

	uchar* image_row = ((uchar *)(winners_image_->imageData + y*winners_image_->widthStep));
	int image_col    = x*winners_image_->nChannels;

	image_row[image_col + 0] = R;
	image_row[image_col + 1] = G;
	image_row[image_col + 2] = B;
}

void HistogramFilterTL::UpdateHistogramFilterTL(DetectionGrid *dg, double curr_speed, double detection_grid_spacing)
{
	// capture uncertainty in sensors
	BlurPrior(curr_speed, detection_grid_spacing);

	// make new measurements take effect -- uniform random noise:
	double sum = 0.0;
	float  grid_prior_pixels = (grid_prior_width_*grid_prior_height_);
	
	for (int y = 0; y < grid_prior_height_; y++)
	{
		float* grid_row = ((float *)(grid_prior_->imageData + y*grid_prior_->widthStep));
	
		for (int x = 0; x < grid_prior_width_; x++)
		{
			float prob = grid_row[x];
			prob += 0.3 / grid_prior_pixels;
			sum  += grid_row[x] = prob;
		}
	}

	// normalize
	NormalizeGrid(sum);

	// adjust prior based on data score and populate winners image
	sum = 0.0;
	for (int y = 0; y < grid_prior_height_; y++)
	{
		float* grid_row = ((float *)(grid_prior_->imageData + y*grid_prior_->widthStep));

		for (int x = 0; x < grid_prior_width_; x++)
		{
			char state = 'u';
			float score = dg->getBestScore(x,y, &state);
			setWinnersImage(state, score, x, y);
			float prior = grid_row[x];
			sum += grid_row[x] = pow(score, measurement_exponent_) * ((prior == 0.) ? 1. : prior);
		}
	}

	//normalize again
	NormalizeGrid(sum);

}

IplImage* HistogramFilterTL::getGridPrior()
{
	return grid_prior_;
}

IplImage* HistogramFilterTL::getWinnersImage()
{
	return winners_image_;
}

