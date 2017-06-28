/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author: Marius Muja

#include <cstdio>
#include <queue>
#include <algorithm>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"


#include <chamfer_matching/chamfer_matching.h>



#define CV_PIXEL(type,img,x,y) (((type*)(img->imageData+y*img->widthStep))+x*img->nChannels)






class SlidingWindowImageIterator : public ImageIterator
{
	int x_;
	int y_;
	float scale_;
	float scale_step_;
	int scale_cnt_;

	bool has_next_;


	int width_;
	int height_;
	int x_step_;
	int y_step_;
	int scales_;
	float min_scale_;
	float max_scale_;


public:

	SlidingWindowImageIterator(int width, int height, int x_step = 3, int y_step = 3, int scales = 5, float min_scale = 0.6, float max_scale = 1.6) :
		width_(width), height_(height), x_step_(x_step),y_step_(y_step), scales_(scales), min_scale_(min_scale), max_scale_(max_scale)
	{
		x_ = 0;
		y_ = 0;
		scale_cnt_ = 0;
		scale_ = min_scale_;
		has_next_ = true;
		scale_step_ = (max_scale_-min_scale_)/scales_;
	}

	bool hasNext() const {
		return has_next_;
	}

	location_scale_t next()
	{
		location_scale_t next_val = make_pair(cvPoint(x_,y_),scale_);

		x_ += x_step_;

		if (x_ >= width_) {
			x_ = 0;
			y_ += y_step_;

			if (y_ >= height_) {
				y_ = 0;
				scale_ += scale_step_;
				scale_cnt_++;

				if (scale_cnt_ == scales_) {
					has_next_ = false;
					scale_cnt_ = 0;
					scale_ = min_scale_;
				}
			}
		}

		return next_val;
	}
};


ImageIterator* SlidingWindowImageRange::iterator() const
{
	return new SlidingWindowImageIterator(width_, height_, x_step_, y_step_, scales_, min_scale_, max_scale_);
}




class LocationImageIterator : public ImageIterator
{
	const vector<CvPoint>& locations_;

	size_t iter_;

	int scales_;
	float min_scale_;
	float max_scale_;

	float scale_;
	float scale_step_;
	int scale_cnt_;

	bool has_next_;

public:
	LocationImageIterator(const vector<CvPoint>& locations, int scales = 5, float min_scale = 0.6, float max_scale = 1.6) :
		locations_(locations), scales_(scales), min_scale_(min_scale), max_scale_(max_scale)
	{
		iter_ = 0;
		scale_cnt_ = 0;
		scale_ = min_scale_;
		has_next_ = (locations_.size()==0 ? false : true);
		scale_step_ = (max_scale_-min_scale_)/scales_;
	}

	bool hasNext() const {
		return has_next_;
	}

	location_scale_t next()
	{
		location_scale_t next_val = make_pair(locations_[iter_],scale_);

		iter_ ++;
		if (iter_==locations_.size()) {
			iter_ = 0;
			scale_ += scale_step_;
			scale_cnt_++;

			if (scale_cnt_ == scales_) {
				has_next_ = false;
				scale_cnt_ = 0;
				scale_ = min_scale_;
			}
		}

		return next_val;
	}
};


ImageIterator* LocationImageRange::iterator() const
{
	return new LocationImageIterator(locations_, scales_, min_scale_, max_scale_);
}





class LocationScaleImageIterator : public ImageIterator
{
	const vector<CvPoint>& locations_;
	const vector<float>& scales_;

	size_t iter_;

	bool has_next_;

public:
	LocationScaleImageIterator(const vector<CvPoint>& locations, const vector<float>& scales) :
		locations_(locations), scales_(scales)
	{
		assert(locations.size()==scales.size());
		reset();
	}

	void reset()
	{
		iter_ = 0;
		has_next_ = (locations_.size()==0 ? false : true);
	}

	bool hasNext() const {
		return has_next_;
	}

	location_scale_t next()
	{
		location_scale_t next_val = make_pair(locations_[iter_],scales_[iter_]);

		iter_ ++;
		if (iter_==locations_.size()) {
			iter_ = 0;

			has_next_ = false;
		}

		return next_val;
	}
};

ImageIterator* LocationScaleImageRange::iterator() const
{
	return new LocationScaleImageIterator(locations_, scales_);
}






/**
* Finds a point in the image from which to start contour following.
* @param templ_img
* @param p
* @return
*/
bool findFirstContourPoint(IplImage* templ_img, coordinate_t& p)
{
	unsigned char* ptr = (unsigned char*) templ_img->imageData;
	for (int y=0;y<templ_img->height;++y) {
		for (int x=0;x<templ_img->width;++x) {
			if (*(ptr+y*templ_img->widthStep+x)!=0) {
				p.first = x;
				p.second = y;
				return true;
			}
		}
	}
	return false;
}


/**
* Method that extracts a single continuous contour from an image given a starting point.
* When it extracts the contour it tries to maintain the same direction (at a T-join for example).
*
* @param templ_
* @param coords
* @param crt
*/
void followContour(IplImage* templ_img, template_coords_t& coords, int direction = -1)
{
	const int dir[][2] = { {-1,-1}, {-1,0}, {-1,1}, {0,1}, {1,1}, {1,0}, {1,-1}, {0,-1} };
	coordinate_t next;
	coordinate_t next_temp;
	unsigned char* ptr;

	assert (direction==-1 || !coords.empty());

	coordinate_t crt = coords.back();
	//		printf("Enter followContour, point: (%d,%d)\n", crt.first, crt.second);

	// mark the current pixel as visited
	CV_PIXEL(unsigned char, templ_img, crt.first, crt.second)[0] = 0;
	if (direction==-1) {
		for (int j = 0; j<7; ++j) {
			next.first = crt.first + dir[j][1];
			next.second = crt.second + dir[j][0];
			ptr = CV_PIXEL(unsigned char, templ_img, next.first, next.second);
			if (*ptr!=0) {
				coords.push_back(next);
				followContour(templ_img, coords,j);
				// try to continue contour in the other direction
				reverse(coords.begin(), coords.end());
				followContour(templ_img, coords, (j+4)%8);
				break;
			}
		}
	}
	else {
		int k = direction;
		int k_cost = 3;
		next.first = crt.first + dir[k][1];
		next.second = crt.second + dir[k][0];
		ptr = CV_PIXEL(unsigned char, templ_img, next.first, next.second);
		if (*ptr!=0) {
			k_cost = abs(dir[k][1])+abs(dir[k][0]);
		}
		int p = k;
		int n = k;

		for (int j = 0 ;j<3; ++j) {
			p = (p + 7) % 8;
			n = (n + 1) % 8;
			next.first = crt.first + dir[p][1];
			next.second = crt.second + dir[p][0];
			ptr = CV_PIXEL(unsigned char, templ_img, next.first, next.second);
			if (*ptr!=0) {
				int p_cost = abs(dir[p][1])+abs(dir[p][0]);
				if (p_cost<k_cost) {
					k_cost = p_cost;
					k = p;
				}
			}
			next.first = crt.first + dir[n][1];
			next.second = crt.second + dir[n][0];
			ptr = CV_PIXEL(unsigned char, templ_img, next.first, next.second);
			if (*ptr!=0) {
				int n_cost = abs(dir[n][1])+abs(dir[n][0]);
				if (n_cost<k_cost) {
					k_cost = n_cost;
					k = n;
				}
			}
		}

		if (k_cost!=3) {
			next.first = crt.first + dir[k][1];
			next.second = crt.second + dir[k][0];
			coords.push_back(next);
			followContour(templ_img, coords, k);
		}
	}
}

/**
* Finds a contour in an edge image. The original image is altered by removing the found contour.
* @param templ_img Edge image
* @param coords Coordinates forming the contour.
* @return True while a contour is still found in the image.
*/
bool findContour(IplImage* templ_img, template_coords_t& coords)
{
	coordinate_t start_point;

	bool found = findFirstContourPoint(templ_img,start_point);
	if (found) {
		coords.push_back(start_point);
		followContour(templ_img, coords);
		return true;
	}

	return false;
}

/**
* Computes the angle of a line segment.
*
* @param a One end of the line segment
* @param b The other end.
* @param dx
* @param dy
* @return Angle in radians.
*/

float getAngle(coordinate_t a, coordinate_t b, int& dx, int& dy)
{
	dx = b.first-a.first;
	dy = -(b.second-a.second);  // in image coordinated Y axis points downward
	float angle = atan2(dy,dx);

	if (angle<0) {
		angle+=M_PI;
	}

	return angle;
}



/**
* Computes contour points orientations using the approach from:
*
* Matas, Shao and Kittler - Estimation of Curvature and Tangent Direction by
* Median Filtered Differencing
*
* @param coords Contour points
* @param orientations Contour points orientations
*/
void findContourOrientations(const template_coords_t& coords, template_orientations_t& orientations)
{
	const int M = 5;
	int coords_size = coords.size();

	vector<float> angles(2*M);
	orientations.insert(orientations.begin(), coords_size, float(-3*M_PI)); // mark as invalid in the beginning

	if (coords_size<2*M+1) {  // if contour not long enough to estimate orientations, abort
		return;
	}

	for (int i=M;i<coords_size-M;++i) {
		coordinate_t crt = coords[i];
		coordinate_t other;
		int k = 0;
		int dx, dy;
		// compute previous M angles
		for (int j=M;j>0;--j) {
			other = coords[i-j];
			angles[k++] = getAngle(other,crt, dx, dy);
		}
		// compute next M angles
		for (int j=1;j<=M;++j) {
			other = coords[i+j];
			angles[k++] = getAngle(crt, other, dx, dy);
		}

		// get the middle two angles
		nth_element(angles.begin(), angles.begin()+M-1,  angles.end());
		nth_element(angles.begin()+M-1, angles.begin()+M,  angles.end());
		//		sort(angles.begin(), angles.end());

		// average them to compute tangent
		orientations[i] = (angles[M-1]+angles[M])/2;
	}
}



//////////////////////// ChamferTemplate /////////////////////////////////////

ChamferTemplate::ChamferTemplate(IplImage* edge_image, float scale_) : addr_width(-1), scale(scale_)
{
	template_coords_t local_coords;
	template_orientations_t local_orientations;

	while (findContour(edge_image, local_coords)) {
		findContourOrientations(local_coords, local_orientations);

		coords.insert(coords.end(), local_coords.begin(), local_coords.end());
		orientations.insert(orientations.end(), local_orientations.begin(), local_orientations.end());
		local_coords.clear();
		local_orientations.clear();
	}

	size = cvGetSize(edge_image);
	CvPoint min, max;
	min.x = size.width;
	min.y = size.height;
	max.x = 0;
	max.y = 0;

	center = cvPoint(0,0);
	for (size_t i=0;i<coords.size();++i) {
		center.x += coords[i].first;
		center.y += coords[i].second;

		if (min.x>coords[i].first) min.x = coords[i].first;
		if (min.y>coords[i].second) min.y = coords[i].second;
		if (max.x<coords[i].first) max.x = coords[i].first;
		if (max.y<coords[i].second) max.y = coords[i].second;
	}

	size.width = max.x - min.x;
	size.height = max.y - min.y;

	center.x /= coords.size();
	center.y /= coords.size();

	for (size_t i=0;i<coords.size();++i) {
		coords[i].first -= center.x;
		coords[i].second -= center.y;
	}
//	printf("Template coords\n");
//	for (size_t i=0;i<coords.size();++i) {
//		printf("(%d,%d), ", coords[i].first, coords[i].second);
//	}
//	printf("\n");
}


vector<int>& ChamferTemplate::getTemplateAddresses(int width)
{
	if (addr_width!=width) {
		addr.resize(coords.size());
		addr_width = width;

		for (size_t i=0; i<coords.size();++i) {
			addr[i] = coords[i].second*width+coords[i].first;
//			printf("Addr: %d, (%d,%d), %d\n", addr[i], coords[i].first, coords[i].second, width);
		}
//		printf("%d,%d\n", center.x, center.y);
	}
	return addr;
}


/**
* Resizes a template
*
* @param scale Scale to be resized to
*/
ChamferTemplate* ChamferTemplate::rescale(float new_scale)
{
	if (fabs(scale-new_scale)<1e-6) return this;

	for (size_t i=0;i<scaled_templates.size();++i) {
		if (fabs(scaled_templates[i]->scale-new_scale)<1e-6) {
			return scaled_templates[i];
		}
	}

	float scale_factor = new_scale/scale;

	ChamferTemplate* tpl = new ChamferTemplate();
	tpl->scale = new_scale;

	tpl->center.x = int(center.x*scale_factor+0.5);
	tpl->center.y = int(center.y*scale_factor+0.5);

	tpl->size.width = int(size.width*scale_factor+0.5);
	tpl->size.height = int(size.height*scale_factor+0.5);

	tpl->coords.resize(coords.size());
	tpl->orientations.resize(orientations.size());
	for (size_t i=0;i<coords.size();++i) {
		tpl->coords[i].first = int(coords[i].first*scale_factor+0.5);
		tpl->coords[i].second = int(coords[i].second*scale_factor+0.5);
		tpl->orientations[i] = orientations[i];
	}
	scaled_templates.push_back(tpl);

	return tpl;

}



void ChamferTemplate::show() const
{
	IplImage* templ_color = cvCreateImage(size, IPL_DEPTH_8U, 3);

	for (size_t i=0;i<coords.size();++i) {

		int x = center.x+coords[i].first;
		int y = center.y+coords[i].second;
		CV_PIXEL(unsigned char, templ_color,x,y)[1] = 255;

		if (i%3==0) {
			if (orientations[i] < -M_PI) {
				continue;
			}
			CvPoint p1;
			p1.x = x;
			p1.y = y;
			CvPoint p2;
			p2.x = x + 10*sin(orientations[i]);
			p2.y = y + 10*cos(orientations[i]);

			cvLine(templ_color, p1,p2, CV_RGB(255,0,0));
		}
	}


	cvCircle(templ_color, center, 1, CV_RGB(0,255,0));

	cvNamedWindow("templ",1);
	cvShowImage("templ",templ_color);

	//	cvWaitKey(0);

	cvReleaseImage(&templ_color);
}


//////////////////////// ChamferMatching /////////////////////////////////////


void ChamferMatching::addTemplateFromImage(IplImage* templ, float scale)
{
	ChamferTemplate* cmt = new ChamferTemplate(templ, scale);
	templates.push_back(cmt);
//	printf("Added a new template\n");
	//	cmt->show();
}


/**
* Alternative version of computeDistanceTransform, will probably be used to compute distance
* transform annotated with edge orientation.
*/
void computeDistanceTransform(IplImage* edges_img, IplImage* dist_img, IplImage* annotate_img, float truncate_dt, float a = 1.0, float b = 1.5)
{
	int d[][2] = { {-1,-1}, { 0,-1}, { 1,-1},
			{-1,0},          { 1,0},
			{-1,1}, { 0,1}, { 1,1} };


	CvSize s = cvGetSize(edges_img);
	int w = s.width;
	int h = s.height;
	// set distance to the edge pixels to 0 and put them in the queue
	queue<pair<int,int> > q;
	for (int y=0;y<h;++y) {
		for (int x=0;x<w;++x) {

			unsigned char edge_val = CV_PIXEL(unsigned char, edges_img, x,y)[0];
//			float orientation_val =  CV_PIXEL(float, orientation_img, x,y)[0];

//			if ( (edge_val!=0) && !(orientation_val<-M_PI) ) {
			if ( (edge_val!=0) ) {
				q.push(make_pair(x,y));
				CV_PIXEL(float, dist_img, x, y)[0] = 0;

				if (annotate_img!=NULL) {
					int *aptr = CV_PIXEL(int,annotate_img,x,y);
					aptr[0] = x;
					aptr[1] = y;
				}
			}
			else {
				CV_PIXEL(float, dist_img, x, y)[0] = -1;
			}
		}
	}

	// breadth first computation of distance transform
	pair<int,int> crt;
	while (!q.empty()) {
		crt = q.front();
		q.pop();

		int x = crt.first;
		int y = crt.second;
		float dist_orig = CV_PIXEL(float, dist_img, x, y)[0];
		float dist;

		for (size_t i=0;i<sizeof(d)/sizeof(d[0]);++i) {
			int nx = x + d[i][0];
			int ny = y + d[i][1];

			if (nx<0 || ny<0 || nx>=w || ny>=h) continue;

			if (abs(d[i][0]+d[i][1])==1) {
				dist = dist_orig+a;
			}
			else {
				dist = dist_orig+b;
			}

			float* dt = CV_PIXEL(float, dist_img, nx, ny);

			if (*dt==-1 || *dt>dist) {
				*dt = dist;
				q.push(make_pair(nx,ny));

				if (annotate_img!=NULL) {
					int *aptr = CV_PIXEL(int,annotate_img,nx,ny);
					int *optr = CV_PIXEL(int,annotate_img,x,y);
					aptr[0] = optr[0];
					aptr[1] = optr[1];
				}
			}
		}
	}
	// truncate dt
	if (truncate_dt>0) {
		cvMinS(dist_img, truncate_dt, dist_img);
	}
}


void computeEdgeOrientations(IplImage* edge_img, IplImage* orientation_img)
{
	IplImage* contour_img = cvCreateImage(cvGetSize(edge_img), IPL_DEPTH_8U, 1);

	template_coords_t coords;
	template_orientations_t orientations;

	while (findContour(edge_img, coords)) {

		findContourOrientations(coords, orientations);

		// set orientation pixel in orientation image
		for (size_t i = 0; i<coords.size();++i) {
			int x = coords[i].first;
			int y = coords[i].second;
			//			if (orientations[i]>-M_PI)
			{
				CV_PIXEL(unsigned char, contour_img, x, y)[0] = 255;
			}
			CV_PIXEL(float, orientation_img, x, y)[0] = orientations[i];
		}
		//		cvNamedWindow("contours"); cvShowImage("contours", contour_img);
		//		cvWaitKey(0);

		coords.clear();
		orientations.clear();
	}

	cvSaveImage("contours.pgm", contour_img);
}


void fillNonContourOrientations(IplImage* annotated_img, IplImage* orientation_img)
{
	int width = annotated_img->width;
	int height = annotated_img->height;

	assert(orientation_img->width==width && orientation_img->height==height);

	for (int y=0;y<height;++y) {
		for (int x=0;x<width;++x) {
			int* ptr = CV_PIXEL(int, annotated_img, x,y);
			int xorig = ptr[0];
			int yorig = ptr[1];

			if (x!=xorig || y!=yorig) {
				float val = CV_PIXEL(float,orientation_img,xorig,yorig)[0];
				CV_PIXEL(float,orientation_img,x,y)[0] = val;
			}
		}
	}
}

static float orientation_diff(float o1, float o2)
{
	return fabs(o1-o2);
}



void ChamferMatching::localChamferDistance(CvPoint offset, IplImage* dist_img, IplImage* orientation_img,
			ChamferTemplate* tpl, ChamferMatch& cm, float alpha)
{
	int x = offset.x;
	int y = offset.y;

	float beta = 1-alpha;

	vector<int>& addr = tpl->getTemplateAddresses(dist_img->width);
	vector<float> costs(addr.size());

	float* ptr = CV_PIXEL(float, dist_img, x, y);
	float sum_distance = 0;
	for (size_t i=0;i<addr.size();++i) {
//		if (x==279 && y==176 && fabs(tpl.template_scale-0.7)<0.01) {
//			printf("%g, ", *(ptr+templ_addr[i]));
//		}

		sum_distance += *(ptr+addr[i]);
		costs[i] = *(ptr+addr[i]);
	}

	float cost = (sum_distance/truncate_)/addr.size();

//	prinddrtf("%d,%d\n", tpl->center.x, tpl->center.y);

	if (orientation_img!=NULL) {
		float* optr = CV_PIXEL(float, orientation_img, x, y);
		float sum_orientation = 0;
		int cnt_orientation = 0;
		for (size_t i=0;i<addr.size();++i) {
			if (tpl->orientations[i]>=-M_PI && *(optr+addr[i])>=-M_PI) {
				sum_orientation += orientation_diff(tpl->orientations[i], *(optr+addr[i]));
				cnt_orientation++;
				//			costs[i] = orientation_diff(tpl.orientations[i], *(optr+addr[i]));
			}
		}
//		printf("\n");

		if (cnt_orientation>0) {
			cost = beta*cost+alpha*(sum_orientation/(2*M_PI))/cnt_orientation;
		}
	}


//	if (x>260 && x<300 && y>160 && y<190 && fabs(tpl->scale-1.0)<0.01) {
//		printf("\nCost: %g\n", cost);
//	}


	cm.addMatch(cost, offset, tpl, addr, costs);

}


void ChamferMatching::matchTemplates(IplImage* dist_img, IplImage* orientation_img, ChamferMatch& cm, const ImageRange& range, float orientation_weight)
{
	// try each template
	for(size_t i = 0; i < templates.size(); i++) {
		ImageIterator* it = range.iterator();
		while (it->hasNext()) {
			location_scale_t crt = it->next();

			CvPoint loc = crt.first;
			float scale = crt.second;
			ChamferTemplate* tpl = templates[i]->rescale(scale);

//			printf("Location: (%d,%d), template: (%d,%d), img: (%d,%d)\n",loc.x,loc.y,
//					templates[i]->size.width,templates[i]->size.height,
//					dist_img->width, dist_img->height);

//			if (loc.x-tpl->center.x<0 || loc.x+tpl->size.width-tpl->center.x>=dist_img->width) continue;
//			if (loc.y-tpl->center.y<0 || loc.y+tpl->size.height-tpl->center.y>=dist_img->height) continue;
			if (loc.x-tpl->center.x<0 || loc.x+tpl->size.width>=dist_img->width) continue;
			if (loc.y-tpl->center.y<0 || loc.y+tpl->size.height>=dist_img->height) continue;
//			printf("%d,%d - %d,%d\n", loc.x, loc.y, templates[i]->center.x, templates[i]->center.y);
			localChamferDistance(loc, dist_img, orientation_img, tpl, cm, orientation_weight);
		}

		delete it;
	}
}



/**
* Run matching using an edge image.
* @param edge_img Edge image
* @return a match object
*/
ChamferMatch ChamferMatching::matchEdgeImage(IplImage* edge_img, const ImageRange& range, float orientation_weight, int max_matches, float min_match_distance)
{
	CV_Assert(edge_img->nChannels==1);

	IplImage* dist_img;
	IplImage* annotated_img;
	IplImage* orientation_img;
	ChamferMatch cm (max_matches, min_match_distance);


	dist_img = cvCreateImage(cvSize(edge_img->width, edge_img->height), IPL_DEPTH_32F, 1);
	annotated_img = cvCreateImage(cvSize(edge_img->width, edge_img->height), IPL_DEPTH_32S, 2);

	// Computing distance transform
	computeDistanceTransform(edge_img,dist_img, annotated_img, truncate_);

	orientation_img = NULL;
	if (use_orientation_) {
		orientation_img = cvCreateImage(cvSize(edge_img->width, edge_img->height), IPL_DEPTH_32F, 1);
		IplImage* edge_clone = cvCloneImage(edge_img);
		computeEdgeOrientations(edge_clone, orientation_img );
		cvReleaseImage(&edge_clone);
		fillNonContourOrientations(annotated_img, orientation_img);
	}

	// Template matching
	matchTemplates(dist_img, orientation_img, cm, range, orientation_weight);

	cvReleaseImage(&dist_img);
	cvReleaseImage(&annotated_img);
	if (use_orientation_) {
		cvReleaseImage(&orientation_img);
	}

	return cm;
}


void ChamferMatch::addMatch(float cost, CvPoint offset, ChamferTemplate* tpl, const vector<int>& addr, const vector<float>& costs)
{
	bool new_match = true;

	for (int i=0; i<count; ++i) {
		if (abs(matches[i].offset.x-offset.x)+abs(matches[i].offset.y-offset.y)<min_match_distance_) {
			// too close, not a new match
			new_match = false;
			// if better cost, replace existing match
			if (cost<matches[i].cost) {
				matches[i].cost = cost;
				matches[i].offset = offset;
				matches[i].tpl = tpl;
//				matches[i].costs = costs;
//				matches[i].img_offs = addr;
			}
			// re-bubble to keep ordered
			int k = i;
			while (k>0) {
				if (matches[k-1].cost>matches[k].cost) {
					swap(matches[k-1],matches[k]);
				}
				k--;
			}

			break;
		}
	}

	if (new_match) {
		// if we don't have enough matches yet, add it to the array
		if (count<max_matches_) {
			matches[count].cost = cost;
			matches[count].offset = offset;
			matches[count].tpl = tpl;
//			matches[count].costs = costs;
//			matches[count].img_offs = addr;
			count++;
		}
		// otherwise find the right position to insert it
		else {
			// if higher cost than the worst current match, just ignore it
			if (matches[count-1].cost<cost) {
				return;
			}

			int j = 0;
			// skip all matches better than current one
			while (matches[j].cost<cost) j++;

			// shift matches one position
			int k = count-2;
			while (k>=j) {
				matches[k+1] = matches[k];
				k--;
			}

			matches[j].cost = cost;
			matches[j].offset = offset;
			matches[j].tpl = tpl;
//			matches[j].costs = costs;
//			matches[j].img_offs = addr;
		}
	}
}

void ChamferMatch::showMatch(IplImage* img, int index)
{
	if (index>=count) {
		printf("Index too big.\n");
	}

	assert(img->nChannels==3);

	ChamferMatchInstance match = matches[index];

	const template_coords_t& templ_coords = match.tpl->coords;
	for (size_t i=0;i<templ_coords.size();++i) {
		//			printf("%g, ", match.costs[i]);
		int x = match.offset.x + templ_coords[i].first;
		int y = match.offset.y + templ_coords[i].second;

		unsigned char *p = CV_PIXEL(unsigned char, img,x,y);
		p[0] = p[2] = 0;
		p[1] = 255;
	}
}
