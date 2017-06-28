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


#include <iostream>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  // -- Get img.
  if(argc == 1) { 
    cout << "Usage: " << argv[0] << " imgname" << endl;
    return 1;
  }
  IplImage* img = cvLoadImage(argv[1]);
  if(!img) {
    cout << "Could not load image." << endl; 
    return 1;
  }

  Vector<KeyPoint> points;
  points.push_back(KeyPoint(200,200,1));


  // -- Get params.
  int threshold = 500; // For keypoint detection; we aren't using this.
  CvSURFParams params;
  params = cvSURFParams(threshold, 1);

  // -- Construct keypoints.
  CvMemStorage* kp_storage = cvCreateMemStorage(0);
  CvSeq* surf_kp = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSURFPoint), kp_storage);
  for(size_t i=0; i<points.size(); i++) { 
    int size = 100;
    int laplacian = 1; //Not used for dense SURF computation.  Only involved in faster matching.
    int direction = 0; //Filled in by extractSURF.  We don't use it anyway. 
    int hessian = threshold+1; //We want all the points to be evaluated, so we make the hessian be larger than the threshold.
    CvSURFPoint point = cvSURFPoint(cvPoint2D32f(points[i].pt.x, points[i].pt.y), 
				    laplacian, size, direction, hessian);
    assert(point.pt.x == points[i].pt.x);
    assert(point.pt.y == points[i].pt.y);
    cvSeqPush(surf_kp, &point);
  }

  // -- Get a grayscale image.
  IplImage* gray;
  gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
  cvCvtColor(img, gray, CV_BGR2GRAY);
  
  // -- Get SURF features.
  CvMemStorage* feature_storage = cvCreateMemStorage(0);
  CvSeq* features = NULL;
  int useProvidedKeyPoints = 1; //Otherwise it will find its own keypoints.
  cvExtractSURF(gray, NULL, &surf_kp, &features, feature_storage, params, useProvidedKeyPoints);

  
  // -- Output.
  CvSeqReader reader;
  CvSeqReader kp_reader;
  cvStartReadSeq(features, &reader);
  cvStartReadSeq(surf_kp, &kp_reader);
  for(size_t i=0; i<points.size(); ++i) {
    // -- Get the feature.
    const float* feature = (const float*)reader.ptr;
    const CvSURFPoint* kp = (const CvSURFPoint*)kp_reader.ptr;

    // -- See if this feature is for the point we are on.
    if(kp->pt.x != points[i].pt.x ||
       kp->pt.y != points[i].pt.y) {
      
      cout << "Skipped" << endl;
      continue;
    }

    // -- Advance the readers.
    CV_NEXT_SEQ_ELEM(kp_reader.seq->elem_size, kp_reader);
    CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);

    // -- Copy into result.
    cout << "feature " << i << endl;
    for(size_t j=0; j<128; ++j) {
      cout << feature[j] << " ";
    }
    cout << endl;
  }

  return 0;
}

