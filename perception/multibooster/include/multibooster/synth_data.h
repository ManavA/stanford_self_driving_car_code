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


#include <multibooster/multibooster.h>

using namespace Eigen;
using namespace std;


void swapNames(vector<string>* pvec, size_t i, size_t j) {
  vector<string>& vec = *pvec;
  string tmp = vec[i];
  vec[i] = vec[j];
  vec[j] = tmp;
}

void permuteNames(vector<string>* names) {
  for(size_t i=0; i<100; ++i) {
    size_t idx1 = rand() % names->size();
    size_t idx2 = rand() % names->size();
    swapNames(names, idx1, idx2);
  }
}

VectorXf* sampleFromMultidimGaussian(VectorXf mean, double stdev) {
  VectorXf* sample = new VectorXf;
  *sample = VectorXf::Zero(mean.rows());
  for(size_t i=0; i<(size_t)mean.rows(); ++i) {
    (*sample)(i) = sampleFromGaussian(stdev) + mean(i);
  }
  return sample;
}

class SynthDatasetGenerator { 
 public:
  size_t num_objs_;
  size_t num_feature_spaces_;
  size_t num_classes_;
  size_t num_dims_;
  size_t num_bg_;
  double variance_;
  double feature_probability_;
  bool labeled_;
  
  MultiBoosterDataset* generateDataset();
  SynthDatasetGenerator();
};

SynthDatasetGenerator::SynthDatasetGenerator() :
  num_objs_(100),
  num_feature_spaces_(4),
  num_classes_(3),
  num_dims_(2),
  num_bg_(500),
  variance_(1),
  feature_probability_(.75),
  labeled_(true)  
{
}

MultiBoosterDataset* SynthDatasetGenerator::generateDataset() {
  // -- Make names for classes and feature spaces.
  vector<string> feature_spaces;
  vector<string> classes;
  for(size_t i=0; i<num_feature_spaces_; ++i) {
    ostringstream oss;
    oss << "Feature_space_" << i;
    feature_spaces.push_back(oss.str());
  }
  for(size_t i=0; i<num_classes_; ++i) {
    ostringstream oss;
    oss << "Class_" << i;
    classes.push_back(oss.str());
  }
  
  // -- Get a random permutation of those.
  permuteNames(&feature_spaces);
  permuteNames(&classes);

  NameMapping cm(classes);
  NameMapping fsm(feature_spaces);

  // -- Randomly sample the means of each class in each feature space.
  vector< vector<VectorXf> > means(num_classes_); //means[i][j] is the mean for the ith class, jth feature space.
  for(size_t i=0; i<num_classes_; ++i) {
    means[i] = vector<VectorXf>(num_feature_spaces_);
    for(size_t j=0; j<num_feature_spaces_; ++j) {
      means[i][j] = VectorXf::Random(num_dims_) * 100;
    }
  }
  
  // -- Generate the objects.
  vector<Object*> objs(0);
  objs.reserve(num_objs_ + num_bg_);
  size_t ctr = 0;
  while(ctr < num_objs_) {
    Object* obj = new Object();
    obj->descriptors_ = vector<descriptor>(num_feature_spaces_);
    int lbl = rand() % num_classes_;
    if(labeled_)
      obj->label_ = lbl;
    else
      obj->label_ = -2;
    for(size_t j=0; j<num_feature_spaces_; ++j) {
      if((double)rand() / (double)RAND_MAX <= feature_probability_) {
	obj->descriptors_[j].vector = sampleFromMultidimGaussian(means[lbl][j], variance_);
	obj->descriptors_[j].length_squared = obj->descriptors_[j].vector->dot(*obj->descriptors_[j].vector);
      }
      else {
	obj->descriptors_[j].vector = NULL;
	obj->descriptors_[j].length_squared = 0;
      }
    }
    objs.push_back(obj);
    ctr++;
  }
   
  // -- Generate random background data.
  while(ctr < num_objs_ + num_bg_) { 
    Object* obj = new Object();
    obj->descriptors_ = vector<descriptor>(num_feature_spaces_);
    if(labeled_) 
      obj->label_ = -1;
    else
      obj->label_ = -2;
    for(size_t j=0; j<num_feature_spaces_; ++j) {
      if((double)rand() / (double)RAND_MAX <= feature_probability_) {
	obj->descriptors_[j].vector = new VectorXf;
	*obj->descriptors_[j].vector = VectorXf::Random(num_dims_)*100;
	obj->descriptors_[j].length_squared = obj->descriptors_[j].vector->dot(*obj->descriptors_[j].vector);
      }
      else {
	obj->descriptors_[j].vector = NULL;
	obj->descriptors_[j].length_squared = 0;
      }
    }
    objs.push_back(obj);
    ctr++;
  }

  // -- Make a new dataset from this and return it.
  MultiBoosterDataset* mbd = new MultiBoosterDataset(classes, feature_spaces);
  mbd->setObjs(objs);
  return mbd;
}
