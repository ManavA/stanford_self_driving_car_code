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

#include "image_data.h"

#include <limits>
#include <time.h>
#include <sys/types.h>

ImageData::ImageData
(bool debug_mode, 
 ShapeTable<uint8_t>* _red, 
 ShapeTable<uint8_t>* _ylw, 
 ShapeTable<uint8_t>* _grn,
 ShapeTable<uint8_t>* _sat,
 const char** _window) :
	stRedHue_(*_red), 
	stYlwHue_(*_ylw), 
	stGrnHue_(*_grn),
	stSaturation_(*_sat),
	m_(225.),
	b_(1.),
	n_(30),
	c_(2),
	debug_mode_(debug_mode), 
	window(_window)
{
	stFrameVal_.SetParams(0,	255,	255,	-.035,	0.);
	stFrameVal_.GenerateExponentialTable();
	stFrameVal_.InvertValues();
}

void ImageData::ConvolveHue
(IplImage *tpl, IplImage *img, IplImage *diffs, int frame_weight, int lens_weight, 
 ShapeTable<uint8_t>* h_table, ShapeTable<uint8_t>* s_table, ShapeTable<uint8_t>* v_table)
{
	uchar h_lut[256];
	uchar s_lut[256];
	uchar v_lut[256];

	h_table->Export(h_lut, 256);
	s_table->Export(s_lut, 256);
	v_table->Export(v_lut, 256);

	int halftpl_u = tpl->width  / 2;
	int halftpl_v = tpl->height / 2;

	CvRect roi;

	if (img->roi == NULL)
	{
		roi.x = 0;
		roi.y = 0;
		roi.height = img->height;
		roi.width  = img->width;
	}
	else 
	{
		roi.x = img->roi->xOffset;
		roi.y = img->roi->yOffset;
		roi.height = img->roi->height;
		roi.width  = img->roi->width;
	}

	int start_u = std::max(roi.x, halftpl_u + 1);
	int start_v = std::max(roi.y, halftpl_v + 1);
	int end_u = std::min(roi.x + roi.width  - 1, img->width  - halftpl_u - 1);
	int end_v = std::min(roi.y + roi.height - 1, img->height - halftpl_v - 1);
	
	int64_t score;
	int tpl_index_u, tpl_index_v; 
	int img_index_u, img_index_v;
	int v_tpl, h_img, s_img, v_img;
	
	for (int v = start_v; v <= end_v; v++)
	{
		for (int u = start_u; u <= end_u; u++)
		{
			score = 0; //reset features for this pixel

			int64_t si=0;
			int64_t vi=0;

			for (int y = -halftpl_v; y <= halftpl_v; y++)
			{
				tpl_index_v = y + halftpl_v;
				img_index_v = v + y;
				uchar* tpl_offset_v = (uchar *)(tpl->imageData + tpl_index_v*tpl->widthStep);
				uchar* img_offset_v = (uchar *)(img->imageData + img_index_v*img->widthStep);
				
				for (int x = -halftpl_u; x <= halftpl_u; x++)
				{
					tpl_index_u = x + halftpl_u;
					img_index_u = u + x;

					int tpl_offset_u = tpl_index_u*tpl->nChannels;

					v_tpl = tpl_offset_v[tpl_offset_u + 2];

					int img_offset_u = img_index_u*img->nChannels;

					h_img = img_offset_v[img_offset_u + 0];
					s_img = img_offset_v[img_offset_u + 1];
					v_img = img_offset_v[img_offset_u + 2];

					if (x == 0 && y == 0)
					{
						si = s_img;
						vi = v_img;
					}

					score += (v_tpl > 0) ?
							(int64_t)(lens_weight  * (int64_t)h_lut[h_img] * (int64_t)s_lut[s_img] / (255 - v_tpl + 1)) : 
							(int64_t)(frame_weight * (int64_t)v_lut[v_img]);
				}
			}//end template match loops

			//set features value
			int64_t sp = si*si*si*si*si / ((int64_t)2<<32);
			int64_t vp = (int64_t)round(85.4*atan(((double)vi+730.)/8. - 100.)+124.5);
			score *= sp * vp;
			((float *)(diffs->imageData + (v-start_v)*diffs->widthStep))[u-start_u] = (float)score;

		}
	}
}

IplImage* ImageData::GetLightTemplate(double dist_to_light, bool downsample)
{
	// TODO: replace this radius equation with a more accurate (and principaled one).
//	int radius = (int)round(12.138 * pow(dist_to_light, -0.28378));
//	int radius = (int)round(-5.0035 * log(0.006 * dist_to_light));
   int radius = (int)floor(10. * exp(dist_to_light * -0.015818));
	if (downsample) radius /=2;

	int len = radius*4;
	if ((len%2) == 0) len++;
	CvPoint center = { len/2, len/2 };

	//record the last-used template dimensions
	last_tpl_width_  = len;
	last_tpl_height_ = len;

	IplImage *tpl = cvCreateImage(cvSize(len,len), IPL_DEPTH_8U, 1);
	cvSetZero(tpl);

	cvCircle(tpl, center, radius, cvScalar(255), -1);
	cvSmooth(tpl, tpl, CV_GAUSSIAN, 9, 9);

	return tpl; 
}

// normalize to [0,1]
void ImageData::NormalizeFeatures(IplImage *red_features, IplImage *grn_features, IplImage *ylw_features)
{
	// Normalize featuress
	float min_val = std::numeric_limits<float>::max();
	float max_val = std::numeric_limits<float>::min();
	
	CvRect roi;

	if (red_features->roi == NULL)
	{
		roi.x = 0;
		roi.y = 0;
		roi.height = red_features->height;
		roi.width  = red_features->width;
	}
	else 
	{
		roi.x = red_features->roi->xOffset;
		roi.y = red_features->roi->yOffset;
		roi.height = red_features->roi->height;
		roi.width  = red_features->roi->width;
	}

	RecalcMinMax(red_features, min_val, max_val);
	RecalcMinMax(ylw_features, min_val, max_val);
	RecalcMinMax(grn_features, min_val, max_val);

	float range = max_val - min_val;

	for (int v = 0; v < roi.height; v++)
	{
		float* red_row = ((float *)(red_features->imageData + (v+roi.y)*red_features->widthStep));
		float* ylw_row = ((float *)(ylw_features->imageData + (v+roi.y)*ylw_features->widthStep));
		float* grn_row = ((float *)(grn_features->imageData + (v+roi.y)*grn_features->widthStep));

		for (int u = 0; u < roi.width; u++)
		{
			red_row[u+roi.x] = (red_row[u+roi.x] - min_val)/range;
			ylw_row[u+roi.x] = (ylw_row[u+roi.x] - min_val)/range;
			grn_row[u+roi.x] = (grn_row[u+roi.x] - min_val)/range;
		}
	}
}

void ImageData::FindMinMax(int a, int b, int c, int d, int *min, int *max)
{
	*min = std::min(a, std::min(b, std::min(c, d)));
	*max = std::max(a, std::max(b, std::max(c, d)));
}

void ImageData::CalculatePixelScore
(int u, int v, IplImage *red_features, IplImage *grn_features, IplImage *ylw_features,
 float *r_score, float *g_score, float *y_score)
{
	*r_score = ((float *)(red_features->imageData + v*red_features->widthStep))[u];
	*g_score = ((float *)(grn_features->imageData + v*grn_features->widthStep))[u];
	*y_score = ((float *)(ylw_features->imageData + v*ylw_features->widthStep))[u];
}

void ImageData::CalculateCellScore
(int iymin, int iymax, IplImage *red_features, IplImage *grn_features, IplImage *ylw_features,
 std::vector<Point> *ps, float *r_score, float *g_score, float *y_score)
{

	int num_scanlines = iymax-iymin + 1;
	std::vector<double> xmins(num_scanlines,10000);
	std::vector<double> xmaxs(num_scanlines,-10000);

	// find bounds of polygons and scanline min and max points
	for ( int lc = 0; lc < 4; ++lc )
	{
		Point from = ps->at( lc );
		Point to = ps->at( (lc+1) % 4 );

		double fx = from.X;
		double fy = from.Y;
		double tx = to.X;
		double ty = to.Y;

		if ( fy == ty ) // Horizontal line.
		{
			double y = floor( fy );
			int iy = (int)y;
			int offs = iy - iymin;

			if ( fx < xmins[offs] ) xmins[offs] = fx;
			if ( fx > xmaxs[offs] ) xmaxs[offs] = fx;
			if ( tx < xmins[offs] ) xmins[offs] = tx;
			if ( tx > xmaxs[offs] ) xmaxs[offs] = tx;
		}
		else   // Not horizontal.  Make from Point lower.
		{
			if ( fy > ty )
			{
				double tmp = fy;
				fy = ty;
				ty = tmp;
				tmp = fx;
				fx = tx;
				tx = tmp;
			}

			double y = floor( fy );
			int iy = (int)y;

			double ym = floor( ty );
			double iym = (int)ym;

			double m = (tx-fx)/(ty-fy);
			double x = fx + (y - fy) * m;

			while ( iy <= iym )
			{
				int offs = iy - iymin;
				double nx = x + m;
				double ny = y + 1;

				if ( y <= fy )
				{
					if (fx < xmins[offs]) xmins[offs] = fx;
					if (fx > xmaxs[offs]) xmaxs[offs] = fx;
				}
				else
				{
					if ( x < xmins[offs] ) xmins[offs] = x;
					if ( x > xmaxs[offs] ) xmaxs[offs] = x;
				}
				if ( ny > ty )
				{
					if ( tx < xmins[offs] ) xmins[offs] = tx;
					if ( tx > xmaxs[offs] ) xmaxs[offs] = tx;
				}
				else
				{
					if ( nx < xmins[offs] ) xmins[offs] = nx;
					if ( nx > xmaxs[offs] ) xmaxs[offs] = nx;
				}

				iy++;
				y = ny;
				x = nx;
			}
		}
	}

	//go through each scanline from min bound to max bound to operate row by row on pixels
	float max_r_features = std::numeric_limits<float>::min();
	float max_y_features = std::numeric_limits<float>::min();
	float max_g_features = std::numeric_limits<float>::min();

	float r_features, g_features, y_features;

	int n = 0;
	int roi_x = conv_roi_u0 + roi_u0;
	int roi_y = conv_roi_v0 + roi_v0;
	
	for (int idx = 0; idx < num_scanlines; idx++)
	{
		int y = idx + iymin;
		int startx = (int)floor( xmins[idx] );
		int endx = (int)floor( xmaxs[idx] );

		while ( startx < endx )
		{
			r_features = ((float *)(red_features->imageData + (y-roi_y)*red_features->widthStep))[startx-roi_x];
			y_features = ((float *)(ylw_features->imageData + (y-roi_y)*ylw_features->widthStep))[startx-roi_x];
			g_features = ((float *)(grn_features->imageData + (y-roi_y)*grn_features->widthStep))[startx-roi_x];

			if (r_features > max_r_features) max_r_features = r_features;
			if (y_features > max_y_features) max_y_features = y_features;
			if (g_features > max_g_features) max_g_features = g_features;

			startx++;
			n ++;
		}
	}

	// return scores
	*r_score = (float)max_r_features;
	*g_score = (float)max_g_features;
	*y_score = (float)max_y_features;
}


void ImageData::RecalcMinMax(IplImage* a, float& min, float& max)
{
	CvRect roi;

	if (a->roi == NULL)
	{
		roi.x = 0;
		roi.y = 0;
		roi.height = a->height;
		roi.width  = a->width;
	}
	else 
	{
		roi.x = a->roi->xOffset;
		roi.y = a->roi->yOffset;
		roi.height = a->roi->height;
		roi.width  = a->roi->width;
	}

	for (int v = 0; v < roi.height; v ++)
	{
		float* a_row = (float*)(a->imageData + (v+roi.y)*a->widthStep);

		for (int u = 0; u < roi.width; u ++)
		{
			if (a_row[u+roi.x] < min) min = a_row[u+roi.x];
			if (a_row[u+roi.x] > max) max = a_row[u+roi.x];
		}
	}
}

bool ImageData::UpdateDataScores
(DetectionGrid *dg, IplImage *curr_image, double dist_to_light, 
 bool downsample, IplImage* videoframe, CvRect subIm, double* p)
{
	//set ROI with respect to camera image
	int min_u_ = dg->getMinU();
	int max_u_ = dg->getMaxU();
	int min_v_ = dg->getMinV();
	int max_v_ = dg->getMaxV();

	if (min_u_ < 0 || min_u_ > max_u_ || max_u_ > curr_image->width ||  
	    min_v_ < 0 || min_v_ > max_v_ || max_v_ > curr_image->height) 
	{
		fprintf(stderr, "Detection grid roi is out of bounds, not processing this frame.\n");
		fprintf(stderr, "roi min(%d, %d) max(%d, %d); image(%d, %d)\n", min_u_, min_v_,
				max_u_, max_v_, curr_image->width, curr_image->height);
		return 0.;
	}

	IplImage *tpl = GetLightTemplate(dist_to_light, downsample);

	roi_u0 = tpl->width/2 + 1;
	roi_v0 = tpl->height/2 + 1;
	roi_u_len = (max_u_ - min_u_) + 1;
	roi_v_len = (max_v_ - min_v_) + 1;

	conv_roi_u0 = min_u_ - tpl->width/2  - 1;
	conv_roi_v0 = min_v_ - tpl->height/2 - 1;
	conv_roi_u1 = max_u_ + tpl->width/2  + 1;
	conv_roi_v1 = max_v_ + tpl->height/2 + 1;

	CvRect roi = { roi_u0, roi_v0, roi_u_len, roi_v_len };
	CvRect conv_roi = { conv_roi_u0, conv_roi_v0, conv_roi_u1-conv_roi_u0+1, conv_roi_v1-conv_roi_v0+1 };

	if (dg == NULL || curr_image == NULL)
	{
		fprintf(stderr, "Null pointer given to image_data.cpp:UpdateDataScores()."
				"Not processing this frame.\n");
		return 0.;
	}

	if ((double)roi.height/(double)roi.width > 9. || (double)roi.width/(double)roi.height > 9.) 
	{
		fprintf(stderr, "roi height to width ratio seems weird. "
				"This tends to happen when our perspective of the light is skew. "
				"I'm returning before doing any image processing.\n");
		return 0.;
	}

	cvSetImageROI(curr_image, conv_roi);

	IplImage *hsv = cvCreateImage(cvGetSize(curr_image), 8, 3);
	cvCvtColor(curr_image, hsv, CV_RGB2HSV);

	cvSetImageROI(hsv, roi);

	IplImage *red_features = cvCreateImage(cvGetSize(hsv), IPL_DEPTH_32F, 1);
	IplImage *ylw_features = cvCreateImage(cvGetSize(hsv), IPL_DEPTH_32F, 1);
	IplImage *grn_features = cvCreateImage(cvGetSize(hsv), IPL_DEPTH_32F, 1);

	ConvolveHue(tpl, hsv, red_features, 1, 6, &stRedHue_, &stSaturation_, &stFrameVal_);
	ConvolveHue(tpl, hsv, ylw_features, 1, 1, &stYlwHue_, &stSaturation_, &stFrameVal_);
	ConvolveHue(tpl, hsv, grn_features, 1, 4, &stGrnHue_, &stSaturation_, &stFrameVal_);

	cvReleaseImage(&tpl);
	cvReleaseImage(&hsv);

	NormalizeFeatures(red_features, grn_features, ylw_features);

	if (window != NULL || videoframe != NULL)
	{
		IplImage* features = cvCreateImage(cvGetSize(red_features), IPL_DEPTH_32F, 3);
		IplImage* features8 = cvCreateImage(cvGetSize(features), 8, 3);
		IplImage* disp = cvCreateImage(cvSize(subIm.width, subIm.height), 8, 3);

		cvMerge(ylw_features, grn_features, red_features, NULL, features);
		cvConvertImage(features, features8);
		cvResize(features8, disp);

		if (window != NULL)
			cvShowImage(window[0], disp);

		if (videoframe != NULL) 
		{
			if (  subIm.x >= 0 && subIm.x + subIm.width < videoframe->width
				&& subIm.y >= 0 && subIm.y + subIm.height < videoframe->height)
			{
				cvSetImageROI(videoframe, subIm);
				cvCopy(disp, videoframe);
				cvResetImageROI(videoframe);
			}
		}

		cvReleaseImage(&features);
		cvReleaseImage(&features8);
		cvReleaseImage(&disp);
	}

	cvResetImageROI(curr_image);

	static int64_t instance = 0;
	instance ++;

	// Update detection grid's cell scores.
	int grid_width = dg->getWidth();
	int grid_height = dg->getHeight();
	for (int y = 0; y < grid_height; y++)
	{
		for (int x = 0; x < grid_width; x++)
		{
			GridCell *curr_cell = dg->getCell(x,y);
			if (!curr_cell->in_frame) //skip cells that are out of the camera frame
				continue;

			int u1 = curr_cell->u1;
			int u2 = curr_cell->u2;
			int u3 = curr_cell->u3;
			int u4 = curr_cell->u4;
			int v1 = curr_cell->v1;
			int v2 = curr_cell->v2;
			int v3 = curr_cell->v3;
			int v4 = curr_cell->v4;

			float r_score, g_score, y_score;
			r_score = g_score = y_score = 0;

			if ((u1 == u2) && (u1 == u3) && (u1 == u4) && (v1 == v2) && (v1 == v3)
			        && (v1 == v4)) //handle trivial case where cell is one pixel or less in size
			{
				CalculatePixelScore(u1, v1, red_features, grn_features, ylw_features, 
					&r_score, &g_score, &y_score);
			}
			else //otherwise, operate on pixels that fall within the cell
			{
				int minV, maxV;
				FindMinMax(v1, v2, v3, v4, &minV, &maxV);

				//make sure before this that points are in some kind of clockwise winding order...
				Point p1 = {u1, v1};
				Point p2 = {u2, v2};
				Point p3 = {u3, v3};
				Point p4 = {u4, v4};
				std::vector<Point> points;
				points.push_back(p1);
				points.push_back(p2);
				points.push_back(p3);
				points.push_back(p4);
				CalculateCellScore(minV, maxV, red_features, grn_features, ylw_features, 
					&points, &r_score, &g_score, &y_score);
			}

			//set new scores
			curr_cell->r_score = r_score;
			curr_cell->g_score = g_score;
			curr_cell->y_score = y_score;

		}//end width loop
	}//end height loop

	double rMax = 0.;
	double yMax = 0.;
	double gMax = 0.;

	// calculate goodness
	for (int y = 0; y < grid_height; y++)
	{
		for (int x = 0; x < grid_width; x++)
		{
			GridCell *curr_cell = dg->getCell(x,y);
			if (!curr_cell->in_frame) //skip cells that are out of the camera frame
				continue;

			double r = curr_cell->r_score;
			double y = curr_cell->y_score;
			double g = curr_cell->g_score;

			if (r > rMax) rMax = r;
			if (g > gMax) gMax = g;
			if (y > yMax) yMax = y;
		}
	}

	double t = rMax + gMax + yMax;
	
	if (t < 0.000000000001)
	{
		p[0] = 0.;
		p[1] = 0.;
		p[2] = 0.;
	}

	p[0] = rMax / t;
	p[1] = yMax / t;
	p[2] = gMax / t;

	//release images
	cvReleaseImage(&red_features);
	cvReleaseImage(&grn_features);
	cvReleaseImage(&ylw_features);

	return true;
}

int ImageData::getLastTplWidth()
{
	return last_tpl_width_;
}
int ImageData::getLastTplHeight()
{
	return last_tpl_height_;
}

