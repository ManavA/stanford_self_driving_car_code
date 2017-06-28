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

#include "descriptors_2d/descriptors_2d.h"

using namespace std;
using namespace cv;

/* 
   Notes:
   vector == std::vector
   Vector == cv::Vector.  
   vvf == std::vector< std::vector<float> >
   
   You should not be able to screw yourself with precomputation sharing, 
   e.g. d.push_back(new SuperpixelColorHistogram(5, 0.25, 10, sch1, sch1));
   should die saying that the segmentation params do not match. 
*/

#define NSAMPLES 2

vector<ImageDescriptor*> setupImageDescriptors() {
  vector<ImageDescriptor*> descriptors;

  //HogWrapper(Size winSize, Size blockSize, Size blockStride, Size cellSize,
  //           int nbins, int derivAperture=1, double winSigma=-1,
  //           int histogramNormType=L2Hys, double L2HysThreshold=0.2, bool gammaCorrection=false)
  descriptors.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
  descriptors.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
  descriptors.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 7, 1, -1, 0, 0.2, true));
  descriptors.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(32,32), 7, 1, -1, 0, 0.2, true));


  //SuperpixelColorHistogram(int seed_spacing, float scale, int nBins, std::string type, SuperpixelStatistic* seg_provider=NULL,
  //                         SuperpixelColorHistogram* hsv_provider_=NULL);
  SuperpixelColorHistogram* sch1 = new SuperpixelColorHistogram(20, 0.5, 10);
  descriptors.push_back(sch1);
  descriptors.push_back(new SuperpixelColorHistogram(5, 0.5, 10, NULL, sch1));
  descriptors.push_back(new SuperpixelColorHistogram(5, 1, 10, NULL, sch1));
  descriptors.push_back(new SuperpixelColorHistogram(5, 0.25, 10, NULL, sch1));
 
  //SurfWrapper(bool extended = true, int size = 100)
  descriptors.push_back(new SurfWrapper(true, 150));
  descriptors.push_back(new SurfWrapper(true, 100));
  descriptors.push_back(new SurfWrapper(true, 50));
  
  return descriptors;
}

void releaseImageDescriptors(vector<ImageDescriptor*>* descriptors) {
  for(size_t i=0; i<descriptors->size(); i++) {
    delete (*descriptors)[i];
  }
  descriptors->clear();
}

void computeSegmentation(IplImage* img) {
  SuperpixelStatistic ss(5, 0.5, NULL);
  ss.debug_ = true;
  ss.segment(img);

  //Making a copy of the segmentation.  You can also just use the pointers that are returned, but 
  //be careful that clearImageCache() is never called as it will delete the data out from under you otherwise.
  IplImage* seg = cvCloneImage(ss.getSegmentation()); 
  //index[i] returns the vector of CvPoints for segment i of the image.
  std::vector< std::vector<CvPoint> > index = *ss.getIndex();
  
  // -- Do something with seg and index.
  size_t y = 100; size_t x = 100;
  int segment = CV_IMAGE_ELEM(seg, int, y, x);
  cout << endl << "pixels in segment " << segment << ":" << endl;
  for(size_t i=0; i<index[segment].size(); ++i) {
    cout << "x: " << index[segment][i].x << ", y: " << index[segment][i].y << endl;
  } 
}

// Load an image and compute features at NSAMPLES random points.
int main(int argc, char** argv)  {

  // -- Get many descriptors.
  vector<ImageDescriptor*> descriptors = setupImageDescriptors();

  // -- Load an image.
  if(argc < 2) {
    cout << "usage: " << argv[0] << " [image]" << endl;
    return 1;
  }
  IplImage* img = cvLoadImage(argv[1]);
  if(!img) {
    cout << "Could not load image " << argv[1] << endl;
    return 1;
  }

  // -- Choose random locations and make keypoints.
  std::vector<KeyPoint> kp;
  kp.reserve(NSAMPLES);
  for(int i=0; i<NSAMPLES; i++)  {
    int r = rand() % img->height;
    int c = rand() % img->width;
    int size = 1;
    kp.push_back(KeyPoint(c, r, size));      
  }
  
  // -- Call all descriptors, get vectorized results.
  //    results[i][j] is the jth feature vector for the ith descriptor.
  vector<vvf> results(descriptors.size());
  for(size_t i=0; i<descriptors.size(); i++) {
    descriptors[i]->compute(img, kp, results[i]);
  }

  // -- Print out the results.
  for(size_t j=0; j<NSAMPLES; j++) {
    for(size_t i=0; i<descriptors.size(); i++) {
      cout << endl << endl << descriptors[i]->getName() << " descriptor, sample number " << j << ": " << endl;
      for(size_t k=0; k<results[i][j].size(); k++) {
	cout << results[i][j][k] << " ";
      }
    }
  }
  cout << endl << endl;

  // -- Demonstrate standalone segmentation usage.
  computeSegmentation(img);

  // -- Clean up.
  releaseImageDescriptors(&descriptors);
  cvReleaseImage(&img);

  return 0;
}



  
