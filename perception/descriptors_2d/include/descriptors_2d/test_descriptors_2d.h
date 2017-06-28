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


#ifndef TEST_DESCRIPTORS_2D_H
#define TEST_DESCRIPTORS_2D_H

#include <descriptors_2d/descriptors_2d.h>

#define MAX_INF_DIST 1e-3
#define MAX_L2_DIST 1e-3

using namespace cv;
using namespace std;

void writeResultsToFile(vvf results, string name) {
  ofstream f(name.c_str(), ios::out);
  for(size_t i=0; i<results.size(); ++i) {
    for(size_t j=0; j<results[i].size(); ++j) {
      f << results[i][j] << " ";
    }
    f << endl;
  }
  f.close();
}

void writeResultsToFileBinary(const vvf& results, int result_size, string name) {
  FILE * pFile;
  pFile = fopen(name.c_str(), "wb");

  int num_results = results.size();
  fwrite(&num_results, sizeof(int), 1, pFile);
  fwrite(&result_size, sizeof(int), 1, pFile);
  bool valid = true;
  bool invalid = false;
  for(size_t i=0; i<results.size(); ++i) {
    if(results[i].empty()) 
      fwrite(&invalid, sizeof(bool), 1, pFile);
    else {
      fwrite(&valid, sizeof(bool), 1, pFile);
      fwrite(&results[i][0], sizeof(float), results[i].size(), pFile);
    }
  }
  fclose(pFile);
}

void readResultsFromFileBinary(string name, vvf& results) {
  assert(results.empty());

  FILE* pFile;
  pFile = fopen(name.c_str(), "rb");
  if(!pFile)
    return;

  int result_size = 0;
  int num_results = 0;
  bool valid = false;
  fread(&num_results, sizeof(int), 1, pFile);
  fread(&result_size, sizeof(int), 1, pFile);
  for(int i=0; i<num_results; ++i) {
    fread(&valid, sizeof(bool), 1, pFile);
    if(valid) {
      float* results_buf = (float*) malloc(sizeof(float)*result_size);
      fread(results_buf, sizeof(float), result_size, pFile);
      std::vector<float> tmp = std::vector<float>(results_buf, results_buf+result_size);
      assert((int)tmp.size() == result_size);
      results.push_back(std::vector<float>(results_buf, results_buf+result_size));
    }
    else {
      results.push_back(std::vector<float>());
      assert(results.back().empty());
    }
  }
  fclose(pFile);
}


std::vector<KeyPoint> getPoints() {
  srand(0);
  std::vector<KeyPoint> points;
  points.push_back(KeyPoint(342, 342, 1));
  points.push_back(KeyPoint(442, 442, 1));
  points.push_back(KeyPoint(242, 242, 1));
  points.push_back(KeyPoint(639, 479, 1));
  return points;
}

bool compareResults(const Vector<float>& v1, const Vector<float>& v2) {
  assert(v1.size() == v2.size());
  double l2 = 0;
  double linf = 0;

  double sum_squares = 0;
  double mag1 = 0, mag2 = 0;
  for(size_t i=0; i<v1.size(); ++i) {
    if(fabs(v1[i] - v2[i]) > linf) 
      linf = fabs(v1[i] - v2[i]);
    sum_squares += pow(v1[i] - v2[i], 2);
    mag1 += pow(v1[i], 2);
    mag2 += pow(v2[i], 2);
  }
  l2 = sqrt(sum_squares);
  mag1 = sqrt(mag1);
  mag2 = sqrt(mag2);

  cout << "linf: " << linf << ", l2 distance: " << l2 << ".  Magnitudes of features: " << mag1 << ", " << mag2 << ".  MAX_L2_DIST = " << MAX_L2_DIST << " and MAX_INF_DIST = " << MAX_INF_DIST << endl;
  if(linf < MAX_INF_DIST && l2 < MAX_L2_DIST)
    return true;
  else {
    return false;
  }
}


bool descriptorTest(ImageDescriptor* desc, string name) {
  IplImage* img = cvLoadImage("test/frame0000.jpg");
  vvf results;
  std::vector<KeyPoint> points = getPoints();
  desc->compute(img, points, results);
  
  mkdir("test/output", S_IRWXO | S_IRWXU);
  writeResultsToFileBinary(results, desc->getSize(), "test/output/" + name);
  vvf results2;
  readResultsFromFileBinary("test/correct-output/" + name, results2);
  bool success = true;
  for(size_t i=0; i<results.size(); ++i) {
    success &= compareResults(results[i], results2[i]);
  }
  return success;
}

#endif
