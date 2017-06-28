/*********************************************************************
*
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
*
* Author: Alex Teichman
*********************************************************************/

#include <cstdio>
#include <descriptors_2d/descriptors_2d.h>

using namespace std;
using namespace cv;
using namespace Eigen;

/****************************************************************************
*************  Generally Useful Functions
*****************************************************************************/


void whiten(std::vector<float>& m) {
  // -- Get sum and mean.
  float sum = 0;
  for(size_t i=0; i<m.size(); i++) { 
    sum += m[i];
  }
  float mean = sum / m.size();

  // -- Translate and get variance.
  float var=0.0;
  for(size_t i=0; i<m.size(); i++) {
    m[i] = m[i] - mean;
    var += pow(m[i], 2);
  }
  var /= m.size();

  // -- Scale.
  if(var != 0.0) { 
    float std = sqrt(var);
    for(size_t i=0; i<m.size(); i++) {
      m[i] = m[i] / std;
    }
  }

  // -- Check.
  mean = 0;
  var = 0;
  for(size_t i=0; i<m.size(); i++) {
    assert(!isnan(m[i]));
    mean += m[i];
    var += pow(m[i], 2);
  }
  var /= m.size();
  mean /= m.size();
  assert(abs(mean) < 1e-3);
  assert(abs(var - 1) < 1e-3);
}

//! Remove -1 (boundary) entries by nearest neighbor.  Designed for 32S images.
void inPaintNN(IplImage* img) {
  bool all_assigned = false;
  while(!all_assigned) {
    all_assigned = true;
    for(int r=0; r<img->height; r++) {
      int* ptr = (int*)(img->imageData + r * img->widthStep);
      for(int c=0; c<img->width; c++) {
	if(*ptr >= 1) {
	  ptr++;
	  continue;
	}	

	if(c<img->width-1 && ptr[1] > 0) {
	  *ptr = ptr[1];
	  ptr++; 
	  continue;
	}
	if(c>0 && ptr[-1] > 0) {
	  *ptr = ptr[-1];
	  ptr++; 
	  continue;
	}
	if(r>0) {
	  int* ptr2 = (int*)(img->imageData + (r-1) * img->widthStep);
	  if(ptr2[c] > 0) {
	    *ptr = ptr2[c];
	    ptr++; 
	    continue;
	  }
	}
	if(r<img->height-1) {
	  int* ptr2 = (int*)(img->imageData + (r+1) * img->widthStep);
	  if(ptr2[c] > 0) {
	    *ptr = ptr2[c];
	    ptr++; 
	    continue;
	  }
	}
	all_assigned = false;
	ptr++;
      }
    }
  }
}

int getDir(string dir, vector<string> &files) {
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL) {
    cout << "Error(" << errno << ") opening " << dir << endl;
    return errno;
  }

  while ((dirp = readdir(dp)) != NULL) {
    files.push_back(string(dirp->d_name));
  }
  closedir(dp);
  return 0;
}

/****************************************************************************
*************  ImageDescriptor
****************************************************************************/

ImageDescriptor::ImageDescriptor() :
  debug_(false),
  name_(string()), 
  result_size_(0), 
  img_(NULL)
{
}

void ImageDescriptor::compute(IplImage* img, const std::vector<cv::KeyPoint>& points, vvf& results) {
  assert(results.empty());
  assert(result_size_ != 0);
  assert(name_.size() != 0);
  results.resize(points.size());
  img_ = img;
  clearImageCache();

  doComputation(img, points, results);
}

string ImageDescriptor::getName() {
  return name_;
}

unsigned int ImageDescriptor::getSize() {
  return result_size_;
}

void ImageDescriptor::commonDebug(KeyPoint kp, IplImage* vis) {
  int row = (int)kp.pt.y;
  int col = (int)kp.pt.x;

  bool cleanup = false;
  if(!vis) {
    cleanup = true;
    vis = cvCloneImage(img_);
    cvResetImageROI(vis);
  }

  cvLine(vis, cvPoint(col-10, row), cvPoint(col+10, row), cvScalar(0,0,255));
  cvLine(vis, cvPoint(col, row-10), cvPoint(col, row+10), cvScalar(0,0,255));
  CVSHOW("Input Image", vis);
  cvWaitKey(0);

  if(cleanup)
    cvReleaseImage(&vis);
}

/****************************************************************************
*************  ImageDescriptor::SurfWrapper
****************************************************************************/

SurfWrapper::SurfWrapper(bool extended, int size) :
  ImageDescriptor(), 
  extended_(extended),
  size_(size)
{
  char buf[100];
  sprintf(buf, "SURF_extended%d_size%d", extended_, size_);
  name_.assign(buf);

  if(extended_)
    result_size_ = 128;
  else
    result_size_ = 64;
}

SurfWrapper::~SurfWrapper() {
}

void SurfWrapper::doComputation(IplImage* img, const std::vector<KeyPoint>& points, vvf& results) {

  // -- Get params.
  int threshold = 500; // For keypoint detection; we aren't using this.
  CvSURFParams params;
  if(extended_)
    params = cvSURFParams(threshold, 1);
  else
    params = cvSURFParams(threshold, 0);

  // -- Construct keypoints.
  CvMemStorage* kp_storage = cvCreateMemStorage(0);
  CvSeq* surf_kp = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSURFPoint), kp_storage);
  for(size_t i=0; i<points.size(); i++) { 
    assert(points[i].size > 0);
    assert(points[i].size == 1); // Make sure I am testing things right.  TODO: remove this.

    int laplacian = 1; //Not used for dense SURF computation.  Only involved in faster matching.
    int direction = 0; //Filled in by extractSURF.  We don't use it anyway. 
    int hessian = threshold+1; //We want all the points to be evaluated, so we make the hessian be larger than the threshold.
    CvSURFPoint point = cvSURFPoint(cvPoint2D32f(points[i].pt.x, points[i].pt.y), 
				    laplacian, size_, direction, hessian);
    assert(point.pt.x == points[i].pt.x);
    assert(point.pt.y == points[i].pt.y);
    cvSeqPush(surf_kp, &point);
  }

  // -- Get a grayscale image.
  IplImage* gray;
  bool deallocGray = false;
  if(img->nChannels == 3) {
    gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
    cvCvtColor(img, gray, CV_BGR2GRAY);
    deallocGray = true;
  }
  else
    gray = img;

  // -- Get SURF features.
  CvMemStorage* feature_storage = cvCreateMemStorage(0);
  CvSeq* features = NULL;
  int useProvidedKeyPoints = 1; //Otherwise it will find its own keypoints.
  cvExtractSURF(gray, NULL, &surf_kp, &features, feature_storage, params, useProvidedKeyPoints);
  assert(features->total == surf_kp->total);

  // -- Put into vvf results.
  CvSeqReader reader;
  CvSeqReader kp_reader;
  cvStartReadSeq(features, &reader);
  cvStartReadSeq(surf_kp, &kp_reader);
  int matches = 0;
  for(size_t i=0; i<points.size(); ++i) {
    // -- Get the feature.
    const float* feature = (const float*)reader.ptr;
    const CvSURFPoint* kp = (const CvSURFPoint*)kp_reader.ptr;

    // -- See if this feature is for the point we are on.
    if(kp->pt.x != points[i].pt.x ||
       kp->pt.y != points[i].pt.y) {
      
      assert(results[i].empty());
      //cout << "Skipped" << endl;
      continue;
    }

    // -- Advance the readers.
    CV_NEXT_SEQ_ELEM(kp_reader.seq->elem_size, kp_reader);
    CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);
    matches++;

    // -- Check length.
    int length = (int)(features->elem_size/sizeof(float));
    assert(length == (int)result_size_);

    // -- Copy into result.
    results[i] = std::vector<float>(result_size_, 0);
    for(size_t j=0; j<result_size_; ++j) {
      results[i][j] = feature[j];
    }
  }

  // -- Display for debugging.
  if(debug_) {
    cout << "Debugging " << name_ << endl;
    cout << "Vector: ";
    float l2 = 0;
    for(size_t i=0; i<results[0].size(); ++i) {
      cout << results[0][i] << " ";
      l2 += results[0][i] * results[0][i];
    }
    cout << endl;

    l2 = sqrt(l2);
    cout << "L2 norm: " << l2 << endl;
    commonDebug(points[0]);
  }

  // -- Cleanup.
  cvClearSeq(surf_kp);
  cvClearMemStorage(kp_storage);
  cvClearSeq(features);
  cvClearMemStorage(feature_storage);
  if(deallocGray)
    cvReleaseImage(&gray);
}



/****************************************************************************
*************  ImageDescriptor::HogWrapper
****************************************************************************/

HogWrapper::HogWrapper() : 
  ImageDescriptor(), 
  hog_()
{
  char buf[400];
  sprintf(buf, "Hog_winSize%dx%d_blockSize%dx%d_blockStride%dx%d_cellSize%dx%d_nBins%d_derivAperture%d_winSigma%g_histNormType%d_L2HysThreshold%g_gammaCorrection%d", 
	  hog_.winSize.width, hog_.winSize.height, hog_.blockSize.width, hog_.blockSize.height, hog_.blockStride.width, hog_.blockStride.height, 
	  hog_.cellSize.width, hog_.cellSize.height, hog_.nbins, hog_.derivAperture, hog_.winSigma, hog_.histogramNormType, 
	  hog_.L2HysThreshold, hog_.gammaCorrection);
  name_.assign(buf);
  result_size_ = hog_.getDescriptorSize();
}

//! All units are pixels.
//! @param winSize The size of the HOG window.  Must be a multiple of cellSize.
//! @param blockSize The size of block (the unit of normalization in HOG).  Must be a multiple of cellSize.
//! @param blockStride How much the block should move as it slides around the window. Must be a multiple of cellSize.
//! @param cellSize The size of the cell (the patch that histograms are computed for).
//! @param num_bins Number of bins to use in the cell histogram.
HogWrapper::HogWrapper(Size winSize, Size blockSize, Size blockStride, Size cellSize,
			int num_bins, int derivAperture, double winSigma,
			int histogramNormType, double L2HysThreshold, bool gammaCorrection) : 
  ImageDescriptor(), 
  hog_(winSize, blockSize, blockStride, cellSize, num_bins, derivAperture, winSigma, histogramNormType, L2HysThreshold, gammaCorrection)
{
  char buf[400];
  sprintf(buf, "Hog_winSize%dx%d_blockSize%dx%d_blockStride%dx%d_cellSize%dx%d_nBins%d_derivAperture%d_winSigma%g_histNormType%d_L2HysThreshold%g_gammaCorrection%d", 
	  winSize.width, winSize.height, blockSize.width, blockSize.height, blockStride.width, blockStride.height, cellSize.width, cellSize.height,
	  num_bins, derivAperture, winSigma, histogramNormType, L2HysThreshold, gammaCorrection);
  name_.assign(buf);
  result_size_ = hog_.getDescriptorSize();
}


void HogWrapper::doComputation(IplImage* img, const std::vector<KeyPoint>& points, vvf& results) {

  // -- Translate from KeyPoint to Point.
  std::vector<Point> locations;
  locations.reserve(points.size());
  for(size_t i=0; i<points.size(); i++) {
    //Subtracting off half winSize since the feature is computed in a window where location[i] is 
    //the upper left corner.  points[i] is the center of the window.
    Point pt(round(points[i].pt.x) - hog_.winSize.width/2, round(points[i].pt.y) - hog_.winSize.height/2); 
    locations.push_back(pt);
  }
  
  // -- Call opencv.
  std::vector<float> result;

  double t = (double)cvGetTickCount();
  hog_.compute(img, result, Size(), Size(), locations); //winStride and padding are set to default
  t = (double)cvGetTickCount() - t;

  // -- Construct vvf from the long concatenation that hog_ produces.
  //    Assume that an all 0 vector was the result of an edge case.
  size_t sz = hog_.getDescriptorSize();
  int nValid = 0;
  for(size_t i=0; i<points.size(); i++) {
    bool valid = false;
    for(size_t j=i*sz; j<(i+1)*sz; j++) {
      if(result[j] != 0) {
	valid = true;
	break;
      }
    }
    if(valid) {
      results[i] = std::vector<float>(&result[(i*sz)], &result[(i*sz+sz)]);
      nValid++; 
    } 
  }

  if(debug_) {
    cout << "debugging " << result_size_ << " dim feature " << name_ << endl;
    printf("exec time for %s = %gms\n", name_.c_str(), t/(cvGetTickFrequency()*1000.));
    // -- Find first returned descriptor.
    size_t i;
    for(i=0; i<results.size(); i++) {
      if(!results[i].empty())
	break;
    }
    
    if(i == results.size())
      cout << "No valid " << name_ << " descriptor." << endl;
    else {
      std::vector<float>& r = results[i];
      cout << name_ << " descriptor: " << endl;
      for(size_t j=0; j<r.size(); j++) {
	cout << " " << r[j];
      }
      cout << endl;

      // -- Create the visualization image.
      IplImage* vis = cvCloneImage(img);
	
      // Make it color if necessary.
      if(vis->nChannels == 1) {
	IplImage* foo = vis;
	vis = cvCreateImage(cvGetSize(foo), foo->depth, 3);
	cvCvtColor(foo, vis, CV_GRAY2RGB);
      }
      
      // -- Draw window.
      cvRectangle(vis, cvPoint(points[i].pt.x + hog_.winSize.width/2, points[i].pt.y + hog_.winSize.height/2), 
		  cvPoint(points[i].pt.x - hog_.winSize.width/2, points[i].pt.y - hog_.winSize.height/2), cvScalar(255,0,0));
      // -- Draw block.
      int ul_x = points[i].pt.x - hog_.winSize.width/2;
      int ul_y = points[i].pt.y - hog_.winSize.height/2;
      cvRectangle(vis, cvPoint(ul_x, ul_y), cvPoint(ul_x + hog_.blockSize.width, ul_y + hog_.blockSize.height), cvScalar(0,255,0));
      
      // -- Draw cell.
      cvRectangle(vis, cvPoint(ul_x, ul_y), cvPoint(ul_x + hog_.cellSize.width, ul_y + hog_.cellSize.height), cvScalar(0,0,255));

      // -- Make it larger if necessary.
      if(vis->width < 100) { 
	float scale = 10;
	IplImage* foo = vis;
	vis = cvCreateImage(cvSize(foo->width * scale, foo->height * scale), foo->depth, foo->nChannels);
	cvResize(foo, vis, CV_INTER_NN);
      }
      
      cout << "Showing window (blue), block (green), and cell (red)." << endl;
      CVSHOW("Visualization", vis);
      cvWaitKey();
      cvDestroyWindow("Visualization");
      cvReleaseImage(&vis);
    }
  }
} 


/***************************************************************************
***********  ContourFragmentManager
****************************************************************************/

ContourFragmentManager::ContourFragmentManager(int num_templates_per_label, bool debug, int min_area, 
						   float min_density, int min_side, int min_edge_pix, int min_edge_pix_besides_line) : 
  num_templates_per_label_(num_templates_per_label),
  debug_(debug),
  min_area_(min_area), 
  min_density_(min_density),
  min_side_(min_side), 
  min_edge_pix_(min_edge_pix),
  min_edge_pix_besides_line_(min_edge_pix_besides_line)
{
}

ContourFragmentManager::~ContourFragmentManager() {
  // -- Deallocate the contours.
  for(size_t i=0; i<contours_.size(); i++) {
    cvReleaseImage(&contours_[i]);
  }
}

void ContourFragmentManager::learnContours(vector<IplImage*> imgs, vector<IplImage*> masks) {
  for(size_t i=0; i<masks.size(); i++) {

    IplImage* mask_gray = cvCreateImage(cvGetSize(masks[i]), IPL_DEPTH_8U, 1);
    cvCvtColor( masks[i], mask_gray, CV_BGR2GRAY );

    IplImage* mask_edge = cvCloneImage(mask_gray);
    cvCanny(mask_gray, mask_edge, 80, 160);

    IplImage* img_gray = cvCreateImage(cvGetSize(imgs[i]), IPL_DEPTH_8U, 1);
    cvCvtColor( imgs[i], img_gray, CV_BGR2GRAY );
    IplImage* img_edge = cvCloneImage(img_gray);
    cvCanny(img_gray, img_edge, 80, 160);

    // -- Get bounding box of label.
    CvPoint tl = cvPoint(mask_edge->width, mask_edge->height);
    CvPoint br = cvPoint(0,0);
    for(int r=0; r<mask_edge->height; r++) {
      uchar* ptr = (uchar*)(mask_edge->imageData + r * mask_edge->widthStep);
      for(int c=0; c<mask_edge->width; c++) {
	if(*ptr == 255) {
	  if(r>br.y)
	    br.y = r;
	  if(r<tl.y)
	    tl.y = r;
	  if(c>br.x)
	    br.x = c;
	  if(c<tl.x)
	    tl.x = c;
	}

	ptr++;
      }
    }

    // -- Display things.
    if(debug_) {
      CVSHOW("mask", mask_gray);
      CVSHOW("mask_edge", mask_edge);
      CVSHOW("img", imgs[i]);
    }
    
    // -- Choose random rectangles in the bounding box.
    int num = 0;
    while(num < num_templates_per_label_) {
      int x1 = rand() % (br.x - tl.x) + tl.x;
      int x2 = rand() % (br.x - tl.x) + tl.x;
      int y1 = rand() % (br.y - tl.y) + tl.y;
      int y2 = rand() % (br.y - tl.y) + tl.y;
      CvRect rect = cvRect(min(x1, x2), min(y1, y2), max(x1, x2) - min(x1, x2) + 1, max(y1, y2) - min(y1, y2) + 1);
      cvRectangle(img_edge, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height), cvScalar(255, 255, 255));
      cvSetImageROI(mask_edge, rect);
     
      // -- Show the rects we are checking.
      if(debug_) {
	CVSHOW("img_edge", img_edge);
	cvWaitKey(50);
      }

      // -- Create random perturbations of the edge.

      // -- Check that it's large enough and well populated enough.  
      if(!contourTest(mask_edge))
	continue;
      
      num++;
      IplImage* mask_edge_cropped = cvCreateImage(cvSize(rect.width, rect.height), 8, 1);
      cvCopy(mask_edge, mask_edge_cropped);
      contours_.push_back(mask_edge_cropped);
      assert(contours_.back()->height != mask_edge->height);

      if(debug_) {
	int scale = 5;
	CvSize sz = cvSize(rect.width*scale, rect.height*scale);
	IplImage* big = cvCreateImage(sz, IPL_DEPTH_8U, 1);
	cvResize(mask_edge, big, CV_INTER_NN);

	CVSHOW("mask_template_big", big);
	CVSHOW("mask_template", mask_edge);
	cvWaitKey(0);    
	cvReleaseImage(&big);
      }
    }
  
    cvResetImageROI(mask_edge);

    cvReleaseImage(&mask_gray);
    cvReleaseImage(&mask_edge);
    cvReleaseImage(&img_gray);
    cvReleaseImage(&img_edge);
  }
}

bool ContourFragmentManager::contourTest(IplImage* img) {

  // -- Enforce minimum size.
  int area = img->roi->height * img->roi->width;
  if(area < min_area_)
    return false;
  if(img->roi->height < min_side_ || img->roi->width < min_side_)
    return false;

  // -- Get statistics.
  int nEdgePix = 0;
  for(int r=img->roi->yOffset; r<img->roi->yOffset + img->roi->height; r++) {
    uchar* ptr = (uchar*)(img->imageData + r * img->widthStep + img->roi->xOffset);
    for(int c=img->roi->xOffset; c<img->roi->width + img->roi->xOffset; c++) {
      assert(*ptr == 0 || *ptr == 255);
      if(*ptr == 255)
	nEdgePix++;

      ptr++;
    }
  }
  
  // -- Enforce minimum density.
  if((float)nEdgePix / (float)area < min_density_) {
    return false;
  }

  // -- Enforce minimum num edge points.
  if(nEdgePix < min_edge_pix_) {
    return false;
  }

  // -- Get lines.
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* lines = cvHoughLines2(img,
			 storage,
			 CV_HOUGH_STANDARD,
			 1,
			 CV_PI/180,
			 3,
			 0,
			 0 );

  if(lines->total > 0) {
    // -- Black out the line.
    IplImage* color_dst = cvCreateImage( cvGetSize(img), 8, 3 );
    cvCvtColor( img, color_dst, CV_GRAY2BGR );

    assert(color_dst->width > 0);
    assert(color_dst->height > 0);
    float* line = (float*)cvGetSeqElem(lines,1);
    float rho = line[0];
    float theta = line[1];
    CvPoint pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));
    int thickness = 8;
    cvLine( color_dst, pt1, pt2, CV_RGB(0,0,0), thickness, 8 );
    if(debug_)
      CVSHOW("line", color_dst);


    // -- Get the number of pixels that are not the primary line.
    nEdgePix = 0;
    float mean = 0;
    for(int r=0; r<color_dst->height; r++) {
      uchar* ptr = (uchar*)(color_dst->imageData + r * color_dst->widthStep);
      for(int c=0; c<color_dst->width; c++) {
	if(*ptr != 0) {
	  nEdgePix++;
	  mean += *ptr;
	}
	ptr+=3; //3chan
      }
    }
    assert(mean / area == mean / (float)area);
    mean /= area;
    //cout << "nEdgePix " << nEdgePix << " mean " << mean << endl;
    //-- Make sure that the contour is more than just a single line.
    if(nEdgePix < min_edge_pix_besides_line_) {
      return false;
    }
  }
  else
    cout << "no line found" << endl;

  return true;
}

void ContourFragmentManager::saveContours(string dir) {
  // -- See if the dir exists. 
  DIR *dp;
  if((dp  = opendir(dir.c_str())) != NULL) {
    cout << "Dir " << dir << " already exists." << endl;
    return;
  }

  mkdir(dir.c_str(), S_IRWXG | S_IRWXU | S_IRWXO);
  for(size_t i=0; i<contours_.size(); i++) {
    ostringstream oss (ostringstream::out);
    oss << dir <<  "/contour" << i << ".png";
    cvSaveImage(oss.str().c_str(), contours_[i]);
  } 
}

void ContourFragmentManager::loadContours(string dir) {
  vector<string> files;
  getDir(dir, files);
  for(size_t i=0; i<files.size(); i++) {
    if(files[i].find(string(".png")) != string::npos) {
      string name = dir + "/" + files[i];
      contours_.push_back(cvLoadImage(name.c_str(), CV_LOAD_IMAGE_GRAYSCALE));
    }
  }
}


/***************************************************************************
***********  ImageDescriptor::ContourFragmentDescriptor
****************************************************************************/


ContourFragmentDescriptor::ContourFragmentDescriptor(int cf_id, string dir) :
  ImageDescriptor(),
  chamfer_provider_(NULL),
  cf_id_(cf_id),
  cfc_(),
  chamfer_(new ChamferMatching(true)),
  matches_(new ChamferMatch())
{

  // -- Load the contours and put into chamfer matcher.
  cfc_.loadContours(dir);
  for(size_t i=0; i<cfc_.contours_.size(); i++) {
//     CVSHOW("test", cfc_.contours_[i]);
//     cvWaitKey(0);
    chamfer_->addTemplateFromImage(cfc_.contours_[i]);
  }

  result_size_ = 2;
  name_ = "temp";
}

ContourFragmentDescriptor::ContourFragmentDescriptor(int cf_id, ContourFragmentDescriptor* chamfer_provider) :
  ImageDescriptor(),
  chamfer_provider_(chamfer_provider),
  cf_id_(cf_id),
  cfc_(),
  chamfer_(NULL),
  matches_(NULL)
{

}


void ContourFragmentDescriptor::doComputation(IplImage* img, const std::vector<KeyPoint>& points, vvf& results) {
  IplImage *edge_img = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
  cvCvtColor(img, edge_img, CV_BGR2GRAY);
  cvCanny(edge_img, edge_img, 80, 160);
  *matches_ = chamfer_->matchEdgeImage(edge_img, SlidingWindowImageRange(edge_img->width, edge_img->height));

  if(debug_) {
    //IplImage* vis = cvCloneImage(edge_img);
    IplImage* vis = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
    cvCvtColor(edge_img, vis, CV_GRAY2RGB);

    ChamferMatch::ChamferMatches match_instances = matches_->getMatches();
    for (size_t i = 0; i<match_instances.size();++i) {
    	printf("Match with cost: %g at lcation: (%d,%d)\n", match_instances[i].cost, match_instances[i].offset.x,match_instances[i].offset.y);
    	matches_->showMatch(vis, i);
    }

    CVSHOW("vis", vis);
//    CVSHOW("edge", edge_img);
    cvWaitKey(0);
    cvReleaseImage(&vis);
  }
  
  cvReleaseImage(&edge_img);
}


/***************************************************************************
***********  ImageDescriptor::SuperpixelStatistic
****************************************************************************/


SuperpixelStatistic::SuperpixelStatistic(int seed_spacing, float scale, SuperpixelStatistic* seg_provider) :
  ImageDescriptor(), 
  index_(NULL), 
  seg_(NULL),
  seed_spacing_(seed_spacing), 
  scale_(scale), 
  seg_provider_(seg_provider)

{
  char buf[1000];
  sprintf(buf, "SuperpixelStatistic_seedSpacing%d_scale%g", seed_spacing_, scale_);
  name_ = string(buf);
}

IplImage* SuperpixelStatistic::createSegmentMask(int label, CvRect* rect) {
  if(!img_ || !seg_) {
    cerr << "Trying to create mask when segmentation does not exist." << endl;
    return NULL;
  }
   
  assert(sizeof(uchar) == 1);

  IplImage* mask = cvCreateImage(cvGetSize(img_), IPL_DEPTH_8U, 1);
  cvZero(mask);

  int max_x = 0;
  int max_y = 0;
  int min_x = img_->width;
  int min_y = img_->height;
  const vector<CvPoint>& l = (*index_)[label];
  vector<CvPoint>::const_iterator lit;
  for(lit = l.begin(); lit != l.end(); lit++) {
    CV_IMAGE_ELEM(mask, uchar, lit->y, lit->x) = 255;
    if(lit->x > max_x)
      max_x = lit->x;
    if(lit->x < min_x)
      min_x = lit->x;
    if(lit->y > max_y)
      max_y = lit->y;
    if(lit->y < min_y)
      min_y = lit->y;
  }

  *rect = cvRect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
  
  return mask;
}

void SuperpixelStatistic::segment(IplImage* img) {
  img_ = img;
  clearImageCache();
  segment();
}

void SuperpixelStatistic::segment() {
  assert(!seg_); //Should be null if this is called.
  assert(!index_);
  assert(img_);
  index_ = new vector< vector<CvPoint> >;  

  // -- Downsample image.
  IplImage* img_small = cvCreateImage(cvSize(((float)img_->width)*scale_, ((float)img_->height)*scale_), img_->depth, img_->nChannels);
  cvResize(img_, img_small, CV_INTER_AREA);

  // -- Create a grid of seed points.
  assert(sizeof(int) == 4);
  IplImage* seg_small = cvCreateImage( cvGetSize(img_small), IPL_DEPTH_32S, 1 );
  cvZero(seg_small);
  int label = 1;
  for(int r=0; r<seg_small->height; r++) {
    int* ptr = (int*)(seg_small->imageData + r * seg_small->widthStep);
    for(int c=0; c<seg_small->width; c++) {
      if(c%seed_spacing_==0 && r%seed_spacing_==0) {
	*ptr = label;
	label++;
      }
      ptr++;
    }
  }

  // -- Compute segmentation.
  double t = (double)cvGetTickCount();
  cvWatershed( img_small, seg_small );
  t = (double)cvGetTickCount() - t;
  inPaintNN(seg_small);

  // -- Check.
  for(int r=0; r<seg_small->height; r++) {
    int* ptr = (int*)(seg_small->imageData + r * seg_small->widthStep);
    for(int c=0; c<seg_small->width; c++) {
      assert(*ptr >= 1);
      assert(*ptr < label);
    }
  }

  // -- Enlarge segmentation back to size of image.
  seg_ = cvCreateImage( cvGetSize(img_), IPL_DEPTH_32S, 1 );
  cvResize(seg_small, seg_, CV_INTER_NN);


  // -- Check. 
//   IplImage* negs = cvCreateImage( cvGetSize(seg_), 8, 1 );
//   for(int r=0; r<seg_->height; r++) {
//     int* ptr = (int*)(seg_->imageData + r * seg_->widthStep);
//     uchar* negs_ptr = (uchar*)(negs->imageData + r * negs->widthStep);
//     for(int c=0; c<seg_->width; c++) {
//       if(*ptr == -1)
// 	*negs_ptr = 255;
//       else
// 	*negs_ptr = 0;
//     }
//   }

//   cvNamedWindow("negs");
//   cvShowImage("negs", negs);
//   cvWaitKey(0);

  for(int r=0; r<seg_->height; r++) {
    int* ptr = (int*)(seg_->imageData + r * seg_->widthStep);
    for(int c=0; c<seg_->width; c++) {
      assert(*ptr >= 1);
      assert(*ptr < label);
    }
  }

  // -- Reserve space for the index.
  int nPixels = seg_->height * seg_->width;
  index_->resize(label+1);
  for(size_t i=0; i<index_->size(); i++) {
    (*index_)[i].reserve((nPixels / label) * 2);
  }

  // -- Compute the index.
  for(int r=0; r<seg_->height; r++) {
    int* ptr = (int*)(seg_->imageData + r * seg_->widthStep);
    for(int c=0; c<seg_->width; c++) {
      assert(*ptr != -1);
      (*index_)[*ptr].push_back(cvPoint(c,r));
      ptr++;
    }
  }

  if(debug_) {
    cout << "Debugging " << name_.c_str() << endl;
    printf( "exec time = %gms\n", t/(cvGetTickFrequency()*1000.) );
  
    // -- Display the results.
    CvMat* color_tab;
    color_tab = cvCreateMat( 1, label, CV_8UC3 );
    CvRNG rng = cvRNG(-1);
    for(int i = 0; i < label; i++ )
      {
	uchar* ptr = color_tab->data.ptr + i*3;
	ptr[0] = (uchar)(cvRandInt(&rng)%180 + 50);
	ptr[1] = (uchar)(cvRandInt(&rng)%180 + 50);
	ptr[2] = (uchar)(cvRandInt(&rng)%180 + 50);
      }
  
    // paint the watershed image
    IplImage* wshed = cvCloneImage( img_ );
    IplImage* img_gray = cvCloneImage( img_ );
    cvZero( wshed );
    IplImage* marker_mask = cvCreateImage( cvGetSize(img_), 8, 1 );
    cvCvtColor( img_, marker_mask, CV_BGR2GRAY );
    cvCvtColor( marker_mask, img_gray, CV_GRAY2BGR );

    for(int i = 0; i < seg_->height; i++ )
      for(int j = 0; j < seg_->width; j++ )
	{
	  int idx = CV_IMAGE_ELEM( seg_, int, i, j );
	  uchar* dst = &CV_IMAGE_ELEM( wshed, uchar, i, j*3 );
	  if( idx == -1 )
	    dst[0] = dst[1] = dst[2] = (uchar)255;
	  else if( idx <= 0 || idx > label )
	    dst[0] = dst[1] = dst[2] = (uchar)0; // should not get here
	  else
	    {
	      uchar* ptr = color_tab->data.ptr + (idx-1)*3;
	      dst[0] = ptr[0]; dst[1] = ptr[1]; dst[2] = ptr[2];
	    }
	}

    cvAddWeighted( wshed, 0.5, img_gray, 0.5, 0, wshed );
    cvNamedWindow(name_.c_str(), CV_WINDOW_AUTOSIZE);
    cvShowImage( name_.c_str(), wshed );

    cvWaitKey(0);
    cvReleaseMat(&color_tab);
    cvReleaseImage(&wshed);
    cvReleaseImage(&marker_mask);
    cvDestroyWindow(name_.c_str());
  }

  cvReleaseImage(&img_small);
  cvReleaseImage(&seg_small);
}

int SuperpixelStatistic::getSeedSpacing() {
  return seed_spacing_;
}

float SuperpixelStatistic::getScale() {
  return scale_;
}

SuperpixelStatistic* SuperpixelStatistic::getSegProvider() {
  return seg_provider_;
}

void SuperpixelStatistic::clearImageCache() {
  if(seg_provider_ == NULL) {
    if(seg_)
      cvReleaseImage(&seg_);
    if(index_) {
      delete index_;
      index_ = NULL;
    }
  }
  else {
    seg_ = NULL;
    index_ = NULL;
  }
}
 
SuperpixelStatistic::~SuperpixelStatistic() {
  clearImageCache();
}

IplImage* SuperpixelStatistic::getSegmentation() {
  return seg_;
}

std::vector< std::vector<CvPoint> >* SuperpixelStatistic::getIndex() {
  return index_;
}

 
/***************************************************************************
***********  ImageDescriptor::SuperpixelStatistic::SuperpixelColorHistogram
****************************************************************************/
 
SuperpixelColorHistogram::SuperpixelColorHistogram(int seed_spacing, float scale, int num_bins, SuperpixelStatistic* seg_provider, 
						   SuperpixelColorHistogram* hsv_provider) : 
  SuperpixelStatistic(seed_spacing, scale, seg_provider), 
  hsv_(NULL), 
  hue_(NULL), 
  sat_(NULL), 
  val_(NULL), 
  num_bins_(num_bins), 
  hsv_provider_(hsv_provider), 
  hists_reserved_(false)
{
  char buf[100];
  sprintf(buf, "_colorHistogram_nBins%d", num_bins);
  name_.append(buf);

  result_size_ = num_bins_*num_bins_;
}

SuperpixelColorHistogram::~SuperpixelColorHistogram() {
  clearImageCache();
}


void SuperpixelColorHistogram::doComputation(IplImage* img, const std::vector<KeyPoint>& points, vvf& results) {
  assert(img->nChannels == 3);

  // -- Make sure we have access to a segmentation.
  if(seg_provider_ == NULL && seg_ == NULL) {    
    segment();
  }
  else if(seg_provider_ != NULL && seg_ == NULL) {
    assert(seg_provider_->getSeedSpacing() == seed_spacing_);
    assert(seg_provider_->getScale() == scale_);
    assert(seg_provider_->getSegProvider() == NULL);
   
    seg_ = seg_provider_->getSegmentation();
    index_ = seg_provider_->getIndex();
  }

  // -- Make sure we have access to HSV image.
  if(hsv_provider_ != NULL && hsv_ == NULL) {
    hsv_ = hsv_provider_->hsv_;
    hue_ = hsv_provider_->hue_;
    sat_ = hsv_provider_->sat_;
    val_ = hsv_provider_->val_;
  }
  else if(hsv_provider_ == NULL && hsv_ == NULL) {
    hsv_ = cvCreateImage( cvGetSize(img_), 8, 3 );
    hue_ = cvCreateImage( cvGetSize(img_), 8, 1 );
    sat_ = cvCreateImage( cvGetSize(img_), 8, 1 );
    val_ = cvCreateImage( cvGetSize(img_), 8, 1 );
    cvCvtColor(img_, hsv_, CV_BGR2HSV);
    cvSplit(hsv_, hue_, sat_, val_, 0);
  }

  assert(hsv_ != NULL);
  assert(hue_ != NULL);
  assert(sat_ != NULL);
  assert(val_ != NULL);
  assert(seg_ != NULL);

  // -- Make sure we have set up the histograms.
  if(!hists_reserved_) {
    histograms_cv_ = vector<CvHistogram*>(index_->size()+1, NULL);
    hists_reserved_ = true;
  }

  
  // -- Compute at each point.
  for(size_t i=0; i<points.size(); i++) {
    doComputation(img, points[i], results[i]);
  }

  if(debug_) {
    cout << name_ << endl;;
    cout << "cv hist: " << endl;
    for(size_t i=0; i<results[0].size(); ++i)
      cout << results[0][i] << " " << endl;

    cout << endl;
    commonDebug(points[0]);
  }

}


void SuperpixelColorHistogram::doComputation(IplImage* img, const KeyPoint& point, std::vector<float>& result) {

  // -- Get the label at this point.
  int label = CV_IMAGE_ELEM(seg_, int, (size_t)point.pt.y, (size_t)point.pt.x);
  assert(label != -1);

  // -- Compute the histogram.
  computeHistogramCV(label);
  assert(sizeof(float) == sizeof(int));

  // -- Copy into result.
  result.resize(result_size_);
  int ctr = 0;
  for(int i=0; i<num_bins_; i++) {
    for(int j=0; j<num_bins_; j++) {
      result[ctr] = cvQueryHistValue_2D(histograms_cv_[label], i, j);
      ctr++;
    }
  }
}

void SuperpixelColorHistogram::computeHistogramCV(int label) {
  if(!hue_) 
    ROS_FATAL("Trying to compute cv hist when hue_ is null");

  if(histograms_cv_[label] == NULL) {
    float huerange[] = {0, 181}; //hsv
    float satrange[] = {0, 256}; //hsv
    float* ranges[] = {huerange, satrange}; //hue and sat.
    int sizes[] = {num_bins_, num_bins_};
    CvHistogram* hist = cvCreateHist(2, sizes, CV_HIST_ARRAY, ranges, 1);
    CvRect rect;
    IplImage* mask = createSegmentMask(label, &rect);

    //Debugging
//     CVSHOW("mask", mask);
//     cvWaitKey(0);
//     cout << "x " << rect.x << ", y " << rect.y << ", w " << rect.width << ", h " << rect.height << endl;


    //For efficiency, set the ROI's and compute the histogram.
    cvSetImageROI(hue_, rect);
    cvSetImageROI(sat_, rect);
    cvSetImageROI(mask, rect);
    IplImage* imgs[] = {hue_, sat_};
    cvCalcHist(imgs, hist, 0, mask);
    cvResetImageROI(hue_);
    cvResetImageROI(sat_);
    cvResetImageROI(mask);

//     // -- Make sure it works without the ROI. 
//     CvHistogram* hist2 = cvCreateHist(2, sizes, CV_HIST_ARRAY, ranges, 1);
//     cvCalcHist(imgs, hist2, 0, mask);
//     for(int i=0; i<num_bins_; i++) {
//       for(int j=0; j<num_bins_; j++) {
// 	assert(cvQueryHistValue_2D(hist2, i, j) == cvQueryHistValue_2D(hist, i, j));
//       }
//     }


    cvReleaseImage(&mask);

    //Debugging
//     cout << "cv hist (before norm): " << endl;
//     for(int i=0; i<num_bins_; i++) {
//       for(int j=0; j<num_bins_; j++) {
//       cout << cvQueryHistValue_2D(hist, i, j) << " ";
//       }
//     }
//     cout << endl;

    cvNormalizeHist(hist, 1);
    histograms_cv_[label] = hist;
  

  }
}

void SuperpixelColorHistogram::clearImageCache() {
  // -- Clean up images.
  if(hsv_provider_ == NULL) {
    cvReleaseImage(&hsv_);
    cvReleaseImage(&hue_);
    cvReleaseImage(&sat_);
    cvReleaseImage(&val_);
  }
  else {
    hsv_ = NULL;
    hue_ = NULL;
    sat_ = NULL;
    val_ = NULL;
  }

  // -- Clean up index and seg_.
  SuperpixelStatistic::clearImageCache();

  // -- Clean up cv histograms.
  for(size_t i=0; i<histograms_cv_.size(); i++) {
    if(histograms_cv_[i])
      cvReleaseHist(&histograms_cv_[i]);
  }
  histograms_cv_.clear();
  hists_reserved_ = false;

}


 
/***************************************************************************
***********  ImageDescriptor::IntegralImageDescriptor
****************************************************************************/
 

IntegralImageDescriptor::IntegralImageDescriptor(IntegralImageDescriptor* ii_provider) : 
  ImageDescriptor(), 
  ii_(NULL), 
  ii_tilt_(NULL), 
  gray_(NULL), 
  ii_provider_(ii_provider),
  dealloc_gray_(false)
{
  char buf[100];
  sprintf(buf, "IntegralImageDescriptor");
  name_ = string(buf);
}

IntegralImageDescriptor::~IntegralImageDescriptor() {
  if(!ii_provider_) {
    if(ii_)
      cvReleaseImage(&ii_);
    if(ii_tilt_)
      cvReleaseImage(&ii_tilt_);
    if(gray_ && dealloc_gray_)
      cvReleaseImage(&gray_);
  }

  dealloc_gray_ = false;
}

void IntegralImageDescriptor::integrate() {
  
  if(!gray_) {
    if(img_->nChannels == 3) {
      gray_ = cvCreateImage(cvGetSize(img_), IPL_DEPTH_8U, 1);
      cvCvtColor(img_, gray_, CV_BGR2GRAY);
      dealloc_gray_ = true;
    }
    else {
      gray_ = img_;
      dealloc_gray_ = false;
    }
  }

  // TODO: Don't reallocate for every image.
  CvSize sz = cvGetSize(img_);
  sz.height++;
  sz.width++;
  ii_ = cvCreateImage(sz, IPL_DEPTH_32S, 1);
  ii_tilt_ = cvCreateImage(sz, IPL_DEPTH_32S, 1);
  cvIntegral(gray_, ii_, NULL, ii_tilt_);
}

void IntegralImageDescriptor::clearImageCache() {
  if(ii_) {
    if(!ii_provider_) {
      cvReleaseImage(&ii_);
      cvReleaseImage(&ii_tilt_);
      cvReleaseImage(&gray_);
    }
    else {
      ii_ = NULL;
      ii_tilt_ = NULL;
      gray_ = NULL;
    }
  }
} 

bool IntegralImageDescriptor::integrateRect(float* result, int row_offset, int col_offset, int half_height, int half_width, const KeyPoint& kp, float* area) {
  // -- Check that we have an integral image.
  assert(ii_);

  // -- Check bounds.
  int col = kp.pt.x;
  int row = kp.pt.y;
    
  int ul_x = col + col_offset - half_width; 
  int ul_y = row + row_offset - half_height;
  int ll_x = ul_x;
  int ll_y = row + row_offset + half_height;
  int ur_x = col + col_offset + half_width;
  int ur_y = ul_y;
  int lr_x = ur_x;
  int lr_y = ll_y;
  
  if(ul_x < 0 || ul_y < 0 || lr_y >= img_->height || lr_x >= img_->width)  {
    return false;
  }

  // -- Compute the rectangle sum.  +1 is because of ii_ dfn
  *result =   CV_IMAGE_ELEM(ii_, int, ul_y    , ul_x    ) \
            + CV_IMAGE_ELEM(ii_, int, lr_y + 1, lr_x + 1) \
            - CV_IMAGE_ELEM(ii_, int, ur_y    , ur_x + 1) \
            - CV_IMAGE_ELEM(ii_, int, ll_y + 1, ll_x    );

  if(area)
    *area = (2*half_width+1) * (2*half_height+1);

  // -- Check that it's right.  
//   int check = 0;
//   float area2 = 0;
//   for(int r=ul_y; r<=ll_y; r++) {
//     for(int c=ul_x; c<=ur_x; c++) {
//       check += CV_IMAGE_ELEM(gray_, uchar, r, c);
//       area2++;
//     }
//   }
//   if(abs(check - *result) > 0) {
//     cout << check - *result << " difference for computing at row " << row_ << " and col " << col_ << endl;
//     assert(0);
//   }
//   if(area) {
//     //  cout << "theory: " << *area;
//     //  cout << "  integrate: " << area2 << endl;
//     assert(abs(*area - area2) < 1e-4);
//   }

  return true;
}


bool IntegralImageDescriptor::integrateRect(float* result, const KeyPoint& kp, const CvRect& rect) {
  // -- Check that we have an integral image.
  assert(ii_);

  // -- Make sure that keypoint sizes are being used, and round to nearest int.
  int multiplier = (int)kp.size;
  assert(multiplier > 0);
  

  // -- Check bounds.
  int ul_x = kp.pt.x + (rect.x * multiplier);
  int ul_y = kp.pt.y + (rect.y * multiplier);
  int ll_x = ul_x;
  int ll_y = kp.pt.y + (rect.y * multiplier) + (rect.height * multiplier);
  int ur_x = kp.pt.x + (rect.x * multiplier) + (rect.width * multiplier);
  int ur_y = ul_y;
  int lr_x = ur_x;
  int lr_y = ll_y;
  
  if(ul_x < 0 || ul_y < 0 || lr_y >= img_->height || lr_x >= img_->width)  {
    return false;
  }

  // -- Compute the rectangle sum.  +1 is because of ii_ dfn
  *result =   CV_IMAGE_ELEM(ii_, int, ul_y    , ul_x    ) \
            + CV_IMAGE_ELEM(ii_, int, lr_y + 1, lr_x + 1) \
            - CV_IMAGE_ELEM(ii_, int, ur_y    , ur_x + 1) \
            - CV_IMAGE_ELEM(ii_, int, ll_y + 1, ll_x    );

  // -- Check that it's right.  
//   int check = 0;
//   float area2 = 0;
//   for(int r=ul_y; r<=ll_y; r++) {
//     for(int c=ul_x; c<=ur_x; c++) {
//       check += CV_IMAGE_ELEM(gray_, uchar, r, c);
//       area2++;
//     }
//   }
//   if(abs(check - *result) > 0) {
//     cout << check - *result << " difference for computing at row " << row_ << " and col " << col_ << endl;
//     assert(0);
//   }
//   if(area) {
//     //  cout << "theory: " << *area;
//     //  cout << "  integrate: " << area2 << endl;
//     assert(abs(*area - area2) < 1e-4);
//   }

  return true;
}


/***************************************************************************
***********  ImageDescriptor::IntegralImageDescriptor::HaarDescriptor
****************************************************************************/



HaarDescriptor::HaarDescriptor(std::vector<CvRect> rects, std::vector<int> weights, IntegralImageDescriptor* ii_provider) :
  IntegralImageDescriptor(ii_provider),
  rects_(rects),
  weights_(weights)
{
  assert(rects_.size() == weights_.size());
  
  char buf[100];
  sprintf(buf, "_HaarDescriptor");
  name_.append(buf);

  for(size_t i=0; i<rects_.size(); ++i) {
    CvRect& r = rects[i];
    sprintf(buf, "_rect%zd_weight%d_x%d_y%d_w%d_h%d", i, weights_[i], r.x, r.y, r.width, r.height);
    name_.append(buf);
  }

  result_size_ = 1;
}


void HaarDescriptor::doComputation(IplImage* img, const std::vector<KeyPoint>& points, vvf& results) {
  // -- Get the integral image.
  if(!ii_ && !ii_provider_)
    integrate();
  if(!ii_ && ii_provider_)  {
    ii_ = ii_provider_->ii_;
    gray_ = ii_provider_->gray_;
  }
  
  // -- Setup results.
  float val = 0;

  // -- Compute feature at all keypoints.
  for(size_t i=0; i<points.size(); ++i) {
    bool success = true;
    results[i].reserve(result_size_);
    results[i].push_back(0);
    assert(!results[i].empty());
    for(size_t j=0; j<rects_.size(); ++j) {
      success &= integrateRect(&val, points[i], rects_[j]); 
      if(!success) {
	results[i].clear();
	break;
      }
      results[i][0] += val * weights_[j];
    }
  }

  // -- Draw the rectangles for debugging.
  if(debug_) {
    cout << name_ << " descriptor: " << results[0][0] << endl;
    IplImage* vis = cvCloneImage(img);
    for(size_t i=0; i<rects_.size(); ++i) {
      CvRect& r = rects_[i];
      const Point2f& pt = points[0].pt;

      CvScalar color;
      if(weights_[i] > 0) 
	color = cvScalar(255, 255, 255);
      else 
	color = cvScalar(0, 0, 0);

      cvRectangle(vis, 
		  cvPoint((int)pt.x + r.x, (int)pt.y + r.y), 
		  cvPoint((int)pt.x + r.x + r.width, (int)pt.y + r.y + r.height), 
		  color);
    }
    CVSHOW("Input Image", vis);
    commonDebug(points[0], vis);
    cvReleaseImage(&vis);
  }
}

vector<ImageDescriptor*> setupDefaultHaarDescriptors() {
  vector<ImageDescriptor*> haar;

  
  // -- Many translations of all wavelets.
  for(int tx=-50; tx <= 50; tx+=25) {
    for(int ty=-50; ty <= 50; ty+=25) {
      // -- Many scales of all wavelets.
      for(int scale = 1; scale < 7; scale+=2) {


	// -- Vertical edges.
	{ // These { }'s are here for the std::vector header-only copy.
	  std::vector<CvRect> rects;
	  std::vector<int> weights;

	  rects.push_back(cvRect(-4*scale + tx, -4*scale + ty, 4*scale, 8*scale));
	  weights.push_back(1);
	  rects.push_back(cvRect(0*scale + tx, -4*scale + ty, 4*scale, 8*scale));
	  weights.push_back(-1);
	  if(haar.size() == 0)
	    haar.push_back(new HaarDescriptor(rects, weights));
	  else
	    haar.push_back(new HaarDescriptor(rects, weights, (IntegralImageDescriptor*)haar[0]));
	}
      

	// -- Vertical lines.
	{
	  std::vector<CvRect> rects;
	  std::vector<int> weights;
	  
	  rects.push_back(cvRect(-2*scale + tx, -8*scale + ty, 2*scale, 16*scale));
	  weights.push_back(-1);
	  rects.push_back(cvRect(0*scale + tx, -8*scale + ty, 2*scale, 16*scale));
	  weights.push_back(1);
	  rects.push_back(cvRect(2*scale + tx, -8*scale + ty, 2*scale, 16*scale));
	  weights.push_back(-1);
	  haar.push_back(new HaarDescriptor(rects, weights, (IntegralImageDescriptor*)haar[0]));
	}

	
	// -- Horizontal edges.
	{
	  std::vector<CvRect> rects;
	  std::vector<int> weights;
	  
	  rects.push_back(cvRect(-4*scale + tx, -4*scale + ty, 8*scale, 4*scale));
	  weights.push_back(1);
	  rects.push_back(cvRect(-4*scale + tx, 0*scale + ty, 8*scale, 4*scale));
	  weights.push_back(-1);
	  haar.push_back(new HaarDescriptor(rects, weights, (IntegralImageDescriptor*)haar[0]));
	}


	// -- Horizontal lines.
	{
	  std::vector<CvRect> rects;
	  std::vector<int> weights;
	  
	  rects.push_back(cvRect(-8*scale + tx, -2*scale + ty, 16*scale, 2*scale));
	  weights.push_back(-1);
	  rects.push_back(cvRect(-8*scale + tx, 0*scale + ty, 16*scale, 2*scale));
	  weights.push_back(1);
	  rects.push_back(cvRect(-8*scale + tx, 2*scale + ty, 16*scale, 2*scale));
	  weights.push_back(-1);
	  haar.push_back(new HaarDescriptor(rects, weights, (IntegralImageDescriptor*)haar[0]));
	}

      }
    }
  }

  return haar;
}


/***************************************************************************
***********  ImageDescriptor::IntegralImageDescriptor::IntegralImageTexture
****************************************************************************/

IntegralImageTexture::IntegralImageTexture(int scale, IntegralImageDescriptor* ii_provider) : 
  IntegralImageDescriptor(ii_provider), 
  scale_(scale) 
{
  char buf[100];
  sprintf(buf, "_IntegralImageTexture_scale%d", scale_);
  name_.append(buf);

  result_size_ = 21;
}

void IntegralImageTexture::doComputation(IplImage* img, const std::vector<KeyPoint>& points, vvf& results) {
  for(size_t i=0; i<points.size(); i++) {
    doComputation(img, points[i], results[i]);
  }
}

void IntegralImageTexture::doComputation(IplImage* img, const KeyPoint& point, std::vector<float>& result) {
  
  //bool IntegralImageTexture::doComputation(Eigen::MatrixXf** result) {
  if(!ii_ && !ii_provider_)
    integrate();
  if(!ii_ && ii_provider_)  {
    ii_ = ii_provider_->ii_;
    gray_ = ii_provider_->gray_;
  }
  
  int szs = 8;
  vector<float> val(8,0);
  vector<float> area(8,0);
  
  bool success=true;
  result.resize(result_size_);
  int ctr = 0;

  // -- 8 Center-surround
  success &= integrateRect(&val[0], 0, 0, 1*scale_, 1*scale_, point, &area[0]);
  for(int i=1; i<=szs; i++) {
    success &= integrateRect(&val[i], 0, 0, (i+1)*scale_, (i+1)*scale_, point, &area[i]); 
    result[ctr] = (val[i-1] - val[i]) / (area[i]-area[i-1]) + val[i-1] / area[i-1];
    ctr++;
  }
  
  //5 finer grained center-surround.
  for(size_t i=3; i<val.size(); i++) {
    result[ctr] = (2./3.) * val[i-3] / area[i-3] + (1./3.) * (val[i-2] - val[i-3]) / (area[i-2] - area[i-3]) \
      - (1./3.) * (val[i-1] - val[i-2]) / (area[i-1] - area[i-2]) - (2./3.) * (val[i] - val[i-1]) / (area[i] - area[i-1]);
    ctr++;
  }

  // -- Gabor Vert
  assert(5.*scale_ == 5*scale_);

  success &= integrateRect(&val[0], 0, 1*scale_, 2*scale_, 0, point);
  success &= integrateRect(&val[1], 0, -1*scale_, 2*scale_, 0, point, &area[0]); 
  result[ctr] = (val[0] - val[1]) / area[0];
  ctr++;

  success &= integrateRect(&val[0], 0, 1*scale_, 5*scale_, 0, point); 
  success &= integrateRect(&val[1], 0, -1*scale_, 5*scale_, 0, point, &area[0]); 
  result[ctr] = (val[0] - val[1]) / area[0]; 
  ctr++;

  success &= integrateRect(&val[0], 0, 2*scale_, 2*scale_, 1*scale_, point); 
  success &= integrateRect(&val[1], 0, -2*scale_, 2*scale_, 1*scale_, point, &area[0]); 
  result[ctr] = (val[0] - val[1]) / area[0]; 
  ctr++;

  success &= integrateRect(&val[0], 0, 2*scale_, 5*scale_, 1*scale_, point); 
  success &= integrateRect(&val[1], 0, -2*scale_, 5*scale_, 1*scale_, point, &area[0]); 
  result[ctr] = (val[0] - val[1]) / area[0]; 
  ctr++;

  // -- Horiz
  success &= integrateRect(&val[0], 1, 0, 0, 2*scale_, point); 
  success &= integrateRect(&val[1], -1, 0, 0, 2*scale_, point, &area[0]); 
  result[ctr] = (val[0] - val[1]) / area[0];
  ctr++;
  
  success &= integrateRect(&val[0], 1, 0, 0, 5*scale_, point); 
  success &= integrateRect(&val[1], -1, 0, 0, 5*scale_, point, &area[0]); 
  result[ctr] = (val[0] - val[1]) / area[0];
  ctr++;
  
  success &= integrateRect(&val[0], 2, 0, 1*scale_, 2*scale_, point); 
  success &= integrateRect(&val[1], -2, 0, 1*scale_, 2*scale_, point, &area[0]); 
  result[ctr] = (val[0] - val[1]) / area[0];
  ctr++;

  success &= integrateRect(&val[0], 2, 0, 1*scale_, 5*scale_, point); 
  success &= integrateRect(&val[1], -2, 0, 1*scale_, 5*scale_, point, &area[0]);
  result[ctr] = (val[0] - val[1]) / area[0];
  ctr++;


  if(!success) {
    result.clear();
  }


  // -- Check that the scaling is between -255 and 255.
//   static vector<float> max(result_size_, -1e20);
//   static vector<float> min(result_size_, 1e20);
//   for(size_t i=0; i<result_size_; i++) {
//     //cout << i << ": " << result[i] << endl;
//     assert(result[i] <= 255 && result[i] >= -255);
//     if(result[i] > max[i])
//       max[i] = result[i];
//     if(result[i] < min[i])
//       min[i] = result[i];
//   }
//   static int count = 0;
//   count++;
//   if(count % 1000 == 0) {
//     cout << "Max: ";
//     for(size_t i=0; i<result_size_; i++) {
//       cout << max[i] << " ";
//     }
//     cout << endl;
//     cout << "Min: ";
//     for(size_t i=0; i<result_size_; i++) {
//       cout << min[i] << " ";
//     }
//     cout << endl;
//   }


  if(debug_) {
    cout << name_ << " descriptor: ";
    for(size_t i=0; i<result.size(); i++) {
      cout << result[i] << " ";
    }
    cout << endl;
    commonDebug(point);
  }
}
  


 
/***************************************************************************
***********  Histogram
****************************************************************************/
 

Histogram::Histogram(int num_bins, float min, float max) : 
  num_insertions_(0), 
  num_bins_(num_bins), 
  min_(min), 
  max_(max)
{
  bin_size_ = (max_ - min_) / (float)num_bins_;
  boundaries_.reserve(num_bins_+1);

  float b = min_;
  while(b <= max_) {
    boundaries_.push_back(b);
    b += bin_size_;
  } 
  boundaries_.push_back(max);

  bins_.reserve(num_bins_);
  for(int i=0; i<num_bins; i++) {
    bins_.push_back(0);
  }

//   cout << "Boundaries: ";
//   for(size_t i=0; i<boundaries_.size(); i++) {
//     cout << boundaries_[i] << " ";
//   }
//   cout << endl;
}

bool Histogram::insert(float val) {
  num_insertions_++;
  for(size_t i=0; i<boundaries_.size() - 1; i++) {
    if (boundaries_[i] <= val && boundaries_[i+1] + .001 >= val) {
      bins_[i]++;
      return true;
    }
  }
  return false;
}

void Histogram::normalize() {
  float total = 0;
  for(size_t i=0; i<bins_.size(); i++) {
    total += bins_[i];
  }
  if(total == 0) {
    return;
  }
  for(size_t i=0; i<bins_.size(); i++) {
    bins_[i] /= total;
  }
}

void Histogram::print() {
  cout << "Histogram (" << num_insertions_ << " insertions): " << endl;
  for(size_t i=0; i<bins_.size(); i++) {
    cout << bins_[i] << " ";
  }
  cout << endl;
}

void Histogram::printGraph() {
  Histogram h2 = *this;
  h2.normalize();

  cout << "Histogram (" << num_insertions_ << " insertions) graph: " << endl;
  for(float row=1; row >= 0; row=row-.1) {
    for(size_t i=0; i<h2.bins_.size(); i++) {
      if(h2.bins_[i] >= row)
	cout << "*";
      else
	cout << " ";
    }
    cout << endl;
  }
  for(size_t i=0; i<h2.bins_.size(); i++) {
    cout << "-";
  }
  cout << endl;
}


void Histogram::printBoundaries() {
  cout << "Histogram Boundaries: " << endl;
  for(size_t i=0; i<boundaries_.size(); i++) {
    cout << boundaries_[i] << " ";
  }
  cout << endl;
}

void Histogram::clear() {
  for(size_t i=0; i<bins_.size(); i++) {
    bins_[i] = 0;
  }
  num_insertions_ = 0;
}
