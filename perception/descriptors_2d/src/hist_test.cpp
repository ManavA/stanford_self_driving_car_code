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

using namespace std;


void rgbTest(IplImage* img) {
  // -- Get RGB
  IplImage* r = cvCreateImage( cvGetSize(img), 8, 1 );
  IplImage* g = cvCreateImage( cvGetSize(img), 8, 1 );
  IplImage* b = cvCreateImage( cvGetSize(img), 8, 1 );
  cvSplit(img, b, g, r, 0);

  // -- Setup hist.
  int num_bins = 8;
  float range[] = {0, 256}; 
  float* ranges[] = {range, range};
  int sizes[] = {num_bins, num_bins};
  CvHistogram* hist = cvCreateHist(2, sizes, CV_HIST_ARRAY, ranges, 1);
  IplImage* imgs[] = {r, g};

  // -- Compute.
  cvCalcHist(imgs, hist, 0, NULL);

  // -- Output.
  cout << "RGB" << endl;
  for(int i=0; i<num_bins; i++) {
    for(int j=0; j<num_bins; j++) {
      cout << cvQueryHistValue_2D(hist, i, j) << " ";
    }
  }
  cout << endl;
}
  


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

  // -- Get HSV.
  IplImage* hsv = cvCreateImage( cvGetSize(img), 8, 3 );
  IplImage* hue = cvCreateImage( cvGetSize(img), 8, 1 );
  IplImage* sat = cvCreateImage( cvGetSize(img), 8, 1 );
  IplImage* val = cvCreateImage( cvGetSize(img), 8, 1 );
  cvCvtColor(img, hsv, CV_BGR2HSV);
  cvSplit(hsv, hue, sat, val, 0);

  // -- Setup hist.
  int num_bins = 8;
  float huerange[] = {0, 181}; 
  float satrange[] = {0, 256}; 
  float* ranges[] = {huerange, satrange};
  int sizes[] = {num_bins, num_bins};
  CvHistogram* hist = cvCreateHist(2, sizes, CV_HIST_ARRAY, ranges, 1);
  IplImage* imgs[] = {hue, sat};

  // -- Compute.
  cvCalcHist(imgs, hist, 0, NULL);

  // -- Output.
  cout << "HSV" << endl;
  for(int i=0; i<num_bins; i++) {
    for(int j=0; j<num_bins; j++) {
      cout << cvQueryHistValue_2D(hist, i, j) << " ";
    }
  }
  cout << endl;

  rgbTest(img);

  return 0;
}

