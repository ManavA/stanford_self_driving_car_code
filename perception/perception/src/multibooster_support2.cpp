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


#include <multibooster_support2.h>

using namespace std;
using namespace pipeline;
using namespace dgc;
using namespace vlr;
using boost::shared_ptr;

using namespace Eigen;

namespace perception {

//0 = unknown, 1 = car, 2 = ped.
vector<string> getClassNames() {
  vector<string> classes;
  classes.push_back("car");
  classes.push_back("pedestrian");
  classes.push_back("bicyclist");

  return classes;
}



std::vector<std::string> getDescriptorNames() { 
  boost::shared_ptr<MultiBoosterObjectConstructor> output_constructor;
  vector< boost::shared_ptr<DescriptorNode> > output_descriptors;
  boost::shared_ptr<PointCloudInterface> input_orienter;
  vector< boost::shared_ptr<ComputeNode> > all_nodes;
  generateDescriptorPipeline(&all_nodes, &input_orienter, &output_constructor, &output_descriptors);

  vector<string> names(output_descriptors.size());
  for(size_t i = 0; i < output_descriptors.size(); ++i) {
    names[i] = output_descriptors[i]->getFullName();
  }

  return names;
}



vector<Object*> buildObjects(const vector< vector< boost::shared_ptr<DescriptorNode> > >& dns) {
  vector<Object*> objects(dns.size());
  for(size_t i = 0; i < dns.size(); ++i) {
    objects[i] = buildObject(dns[i]);
  }
  return objects;
}
  

Object* buildObject(const vector< boost::shared_ptr<DescriptorNode> >& dns, int label) {
  Object* obj = new Object;
  obj->label_ = label;
  obj->descriptors_ = vector<descriptor>(dns.size());
  
  for(size_t i = 0; i < dns.size(); ++i) {
    descriptor& desc = obj->descriptors_[i];
    if(dns[i]->getDescriptor()) {
      desc.vector = new VectorXf(); 
      *desc.vector = *dns[i]->getDescriptor(); //TODO: Make multibooster take a boost::shared_ptr.
      desc.length_squared = desc.vector->dot(*desc.vector);
    }
    else { 
      desc.vector = NULL;
      desc.length_squared = -1;
    }
  }
  
  return obj;
}

void dgcToEigen(const vector<point3d_t>& points,
		boost::shared_ptr<MatrixXf>* cloud,
		boost::shared_ptr<VectorXf>* intensities) {

  *cloud = boost::shared_ptr<MatrixXf>(new MatrixXf(points.size(), 3));
  *intensities = boost::shared_ptr<VectorXf>(new VectorXf(points.size()));
  for(size_t i = 0; i < points.size(); ++i) {
    (*cloud)->coeffRef(i, 0) = points[i].x;
    (*cloud)->coeffRef(i, 1) = points[i].y;
    (*cloud)->coeffRef(i, 2) = points[i].z;
    (*intensities)->coeffRef(i) = points[i].intensity;
  }
}

void rosToEigen(const sensor_msgs::PointCloud &cloud, boost::shared_ptr<MatrixXf>* points, boost::shared_ptr<VectorXf>* intensities) {
  assert(points);
  assert(intensities);
  *points = boost::shared_ptr<MatrixXf>(new MatrixXf(cloud.points.size(), 3));
  *intensities = boost::shared_ptr<VectorXf>(new VectorXf(cloud.points.size()));
  
  for(size_t i = 0; i < cloud.points.size(); ++i) {
    (*points)->coeffRef(i, 0) = cloud.points[i].x;
    (*points)->coeffRef(i, 1) = cloud.points[i].y;
    (*points)->coeffRef(i, 2) = cloud.points[i].z;
    (*intensities)->coeffRef(i) = cloud.channels[0].values[i];
  }
}


//! Connects classifier nodes to descriptor pipeline.
void
generateClassifierPipeline(MultiBooster* mb,
			   std::vector< boost::shared_ptr<pipeline::ComputeNode> >* all_nodes,
			   boost::shared_ptr<PointCloudInterface>* input_orienter,
			   boost::shared_ptr<MultiBoosterNode>* output_classifier)
{
  boost::shared_ptr<MultiBoosterObjectConstructor> output_constructor;
  vector< boost::shared_ptr<DescriptorNode> > output_descriptors;
  generateDescriptorPipeline(all_nodes, input_orienter, &output_constructor, &output_descriptors);
  
  boost::shared_ptr<MultiBoosterNode> mbn(new MultiBoosterNode(mb, output_constructor));
  all_nodes->push_back(mbn);
  *output_classifier = mbn;
}

//! Core function that determines what descriptors we use.
void
generateDescriptorPipeline(std::vector< boost::shared_ptr<pipeline::ComputeNode> >* all_nodes,
			   boost::shared_ptr<PointCloudInterface>* input_orienter,
			   boost::shared_ptr<MultiBoosterObjectConstructor>* output_constructor,
			   std::vector< boost::shared_ptr<pipeline::DescriptorNode> >* output_descriptors)
{

  if(getenv("EXPERIMENTAL_DESCRIPTORS"))
    generateExperimentalDescriptorPipeline(all_nodes, input_orienter, output_constructor, output_descriptors);
  else
    generateDefaultDescriptorPipeline(all_nodes, input_orienter, output_constructor, output_descriptors);
}

void
generateDefaultDescriptorPipeline(std::vector< boost::shared_ptr<pipeline::ComputeNode> >* all_nodes,
				  boost::shared_ptr<PointCloudInterface>* input_orienter,
				  boost::shared_ptr<MultiBoosterObjectConstructor>* output_constructor,
				  std::vector< boost::shared_ptr<pipeline::DescriptorNode> >* all_output_descriptors)
{
  all_nodes->reserve(300);
  all_output_descriptors->reserve(300);
  vector< boost::shared_ptr<DescriptorNode> > output_descriptors;

  //boost::shared_ptr<PlaneFittingCloudOrienter> co = boost::shared_ptr<PlaneFittingCloudOrienter>(new PlaneFittingCloudOrienter(100, 0.05));
  //  boost::shared_ptr<HoughCloudOrienter> co = boost::shared_ptr<HoughCloudOrienter>(new HoughCloudOrienter());
  boost::shared_ptr<CloudOrienter> co = boost::shared_ptr<CloudOrienter>(new CloudOrienter());
  all_nodes->push_back(co);
  *input_orienter = co;

  boost::shared_ptr<OrientedBoundingBoxSize> obbs(new OrientedBoundingBoxSize(co));
  all_nodes->push_back(obbs);
  output_descriptors.push_back(obbs);
  
  boost::shared_ptr<CloudSpinner> cs(new CloudSpinner(co));
  all_nodes->push_back(cs);
  vector< boost::shared_ptr<SpinImage> > sis;
  sis.push_back(boost::shared_ptr<SpinImage>(new SpinImage(cs, 3, 10, 10)));
  sis.push_back(boost::shared_ptr<SpinImage>(new SpinImage(cs, 7, 14, 7)));
  sis.push_back(boost::shared_ptr<SpinImage>(new SpinImage(cs, 10, 20, 10)));
  sis.push_back(boost::shared_ptr<SpinImage>(new SpinImage(cs, 5, 10, 5)));
  for(size_t i = 0; i < sis.size(); ++i) {
    all_nodes->push_back(sis[i]);
    boost::shared_ptr<Whitener> wh(new Whitener(sis[i]));
    all_nodes->push_back(wh);
    output_descriptors.push_back(wh);
  }

  int ppm = 15;
  boost::shared_ptr<CloudProjector> cp0(new CloudProjector(0, ppm, co, 3, 2*ppm, 2*ppm)); //Show at least 2m in each direction.
  boost::shared_ptr<CloudProjector> cp1(new CloudProjector(1, ppm, co, 3, 2*ppm, 2*ppm));
  boost::shared_ptr<CloudProjector> cp2(new CloudProjector(2, ppm, co, 3, 2*ppm, 2*ppm));
  all_nodes->push_back(cp0);
  all_nodes->push_back(cp1);
  all_nodes->push_back(cp2);

  
  // -- For each HogArray, add the HogWindows and RandomProjectors.
  vector<float> u_offset_pcts, v_offset_pcts;
  u_offset_pcts.push_back(0); v_offset_pcts.push_back(1);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0.5);
  u_offset_pcts.push_back(1); v_offset_pcts.push_back(1);

  vector< boost::shared_ptr<HogArray> > hog_arrays;
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(ppm, ppm), cv::Size(ppm, ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  for(size_t i = 0; i < hog_arrays.size(); ++i) { 
    all_nodes->push_back(hog_arrays[i]);

    for(size_t j = 0; j < u_offset_pcts.size(); ++j) {
      boost::shared_ptr<HogWindow> hw(new HogWindow(j, hog_arrays[i]));
      boost::shared_ptr<RandomProjector> rp = boost::shared_ptr<RandomProjector>(new RandomProjector(20, (j+1) * 42000, hw));
	
      all_nodes->push_back(hw);
      all_nodes->push_back(rp);
      output_descriptors.push_back(rp);
    }
  }

  boost::shared_ptr<MultiBoosterObjectConstructor> constructor(new MultiBoosterObjectConstructor(output_descriptors));
  all_nodes->push_back(constructor);
  *output_constructor = constructor;

  for(size_t i = 0; i < output_descriptors.size(); ++i)
    all_output_descriptors->push_back(output_descriptors[i]);
}



void
generateExperimentalDescriptorPipeline(std::vector< boost::shared_ptr<pipeline::ComputeNode> >* all_nodes,
				       boost::shared_ptr<PointCloudInterface>* input_orienter,
				       boost::shared_ptr<MultiBoosterObjectConstructor>* output_constructor,
				       std::vector< boost::shared_ptr<pipeline::DescriptorNode> >* all_output_descriptors)
{
  all_nodes->reserve(300);
  all_output_descriptors->reserve(300);
  vector< boost::shared_ptr<DescriptorNode> > output_descriptors;

  boost::shared_ptr<PlaneFittingCloudOrienter> co = boost::shared_ptr<PlaneFittingCloudOrienter>(new PlaneFittingCloudOrienter(100, 0.05));
  //boost::shared_ptr<CloudOrienter> co = boost::shared_ptr<CloudOrienter>(new CloudOrienter());
  all_nodes->push_back(co);
  *input_orienter = co;

  boost::shared_ptr<OrientedBoundingBoxSize> obbs(new OrientedBoundingBoxSize(co));
  all_nodes->push_back(obbs);
  output_descriptors.push_back(obbs);
  
  boost::shared_ptr<CloudSpinner> cs(new CloudSpinner(co));
  all_nodes->push_back(cs);
  vector< boost::shared_ptr<SpinImage> > sis;
  sis.push_back(boost::shared_ptr<SpinImage>(new SpinImage(cs, 3, 10, 10)));
  sis.push_back(boost::shared_ptr<SpinImage>(new SpinImage(cs, 7, 14, 7)));
  sis.push_back(boost::shared_ptr<SpinImage>(new SpinImage(cs, 10, 20, 10)));
  sis.push_back(boost::shared_ptr<SpinImage>(new SpinImage(cs, 5, 10, 5)));
  for(size_t i = 0; i < sis.size(); ++i) {
    all_nodes->push_back(sis[i]);
    boost::shared_ptr<Whitener> wh(new Whitener(sis[i]));
    all_nodes->push_back(wh);
    output_descriptors.push_back(wh);
  }

  int ppm = 15;
  boost::shared_ptr<CloudProjector> cp0(new CloudProjector(0, ppm, co, 3, 2*ppm, 2*ppm)); //Show at least 2m in each direction.
  boost::shared_ptr<CloudProjector> cp1(new CloudProjector(1, ppm, co, 3, 2*ppm, 2*ppm));
  boost::shared_ptr<CloudProjector> cp2(new CloudProjector(2, ppm, co, 3, 2*ppm, 2*ppm));
  all_nodes->push_back(cp0);
  all_nodes->push_back(cp1);
  all_nodes->push_back(cp2);

  
  // -- For each HogArray, add the HogWindows and RandomProjectors.
  vector<float> u_offset_pcts, v_offset_pcts;
  u_offset_pcts.push_back(0); v_offset_pcts.push_back(1);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0.5);
  u_offset_pcts.push_back(1); v_offset_pcts.push_back(1);

  vector< boost::shared_ptr<HogArray> > hog_arrays;
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(ppm, ppm), cv::Size(ppm, ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(boost::shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  for(size_t i = 0; i < hog_arrays.size(); ++i) { 
    all_nodes->push_back(hog_arrays[i]);
        
    for(size_t j = 0; j < u_offset_pcts.size(); ++j) {
      boost::shared_ptr<HogWindow> hw(new HogWindow(j, hog_arrays[i]));
      //boost::shared_ptr<RandomProjector> rp = boost::shared_ptr<RandomProjector>(new RandomProjector(20, (j+1) * 42000, hw));
	
      all_nodes->push_back(hw);
      output_descriptors.push_back(hw);
      //all_nodes->push_back(rp);
      //output_descriptors.push_back(rp);
    }
  }

  boost::shared_ptr<MultiBoosterObjectConstructor> constructor(new MultiBoosterObjectConstructor(output_descriptors));
  all_nodes->push_back(constructor);
  *output_constructor = constructor;

  for(size_t i = 0; i < output_descriptors.size(); ++i)
    all_output_descriptors->push_back(output_descriptors[i]);
}



/****************************************
 * ClassifierPipeline
 ****************************************/


ClassifierPipeline::ClassifierPipeline(MultiBooster* mb, int num_threads) :
  input_orienters_(vector< boost::shared_ptr<PointCloudInterface> >(num_threads)),
  output_classifiers_(vector< boost::shared_ptr<MultiBoosterNode> >(num_threads))
{
  vector< boost::shared_ptr<ComputeNode> > all_nodes;
  
  for(int i = 0; i < num_threads; ++i) { 
    generateClassifierPipeline(mb, &all_nodes, &input_orienters_[i], &output_classifiers_[i]);
  }

  pipeline_ = Pipeline(all_nodes, num_threads);
}

std::vector<Eigen::VectorXf>
ClassifierPipeline::classify(std::vector< boost::shared_ptr<Eigen::MatrixXf> > clouds,
			     std::vector< boost::shared_ptr<Eigen::VectorXf> > intensities) {
  assert(clouds.size() == intensities.size());
  assert(!clouds.empty());
  
  vector<VectorXf> responses;
  responses.reserve(clouds.size());
  size_t idx = 0;

  while(idx < clouds.size()) { 
    // -- Set num_threads_ inputs.
    int enabled = 0;
    for(size_t i = 0; i < input_orienters_.size(); ++i, ++idx) {
      if(idx < clouds.size()) {
	input_orienters_[i]->disabled_ = false;
	input_orienters_[i]->setInputCloud(clouds[idx]);
	input_orienters_[i]->setInputIntensity(intensities[idx]);
	enabled++;
      }
      else {
	input_orienters_[i]->disabled_ = true;
      }
    }

    //cout << "Using " << enabled << " segments." << endl;
    // -- Get the classifications.
    pipeline_.compute();
    for(size_t i = 0; i < output_classifiers_.size(); ++i) {
      if(!input_orienters_[i]->disabled_)
	responses.push_back(output_classifiers_[i]->response_);
    }

    // -- Clean out cached computation.
    pipeline_.flush();
  }

  return responses;
}


/****************************************
 * DescriptorPipeline
 ****************************************/

//   pipeline::Pipeline pipeline_;
//   std::vector< boost::shared_ptr<PointCloudInterface> > input_orienters_;
//   std::vector< boost::shared_ptr<MultiBoosterObjectConstructor> > output_constructors_;
  
DescriptorPipeline::DescriptorPipeline(int num_threads) :
  input_orienters_(vector< boost::shared_ptr<PointCloudInterface> >(num_threads)),
  output_constructors_(vector< boost::shared_ptr<MultiBoosterObjectConstructor> >(num_threads)),
  output_descriptors_(vector< vector< boost::shared_ptr<DescriptorNode> > >(num_threads))
{
  vector< boost::shared_ptr<ComputeNode> > all_nodes;
  for(int i = 0; i < num_threads; ++i) { 
    generateDescriptorPipeline(&all_nodes, &input_orienters_[i], &output_constructors_[i], &output_descriptors_[i]);
  }

  pipeline_ = Pipeline(all_nodes, num_threads);
}

vector<Object*>
DescriptorPipeline::computeDescriptors(vector< boost::shared_ptr<MatrixXf> > clouds,
				       vector< boost::shared_ptr<VectorXf> > intensities)
{
  assert(clouds.size() == intensities.size());
  assert(!clouds.empty());
  
  vector<Object*> objects;
  objects.reserve(clouds.size());
  size_t idx = 0;

  while(idx < clouds.size()) { 
    // -- Set num_threads_ inputs.
    int enabled = 0;
    for(size_t i = 0; i < input_orienters_.size(); ++i, ++idx) {
      if(idx < clouds.size()) {
	input_orienters_[i]->disabled_ = false;
	input_orienters_[i]->setInputCloud(clouds[idx]);
	input_orienters_[i]->setInputIntensity(intensities[idx]);
	enabled++;
      }
      else {
	input_orienters_[i]->disabled_ = true;
      }
    }

    // -- Get the objects.
    pipeline_.compute();
    for(size_t i = 0; i < output_constructors_.size(); ++i) {
      if(!input_orienters_[i]->disabled_) {
	objects.push_back(new Object(*output_constructors_[i]->object_)); //Deep copy.
      }
    }

    // -- Clean out cached computation.
    pipeline_.flush();
  }

  return objects;
}

Object*
DescriptorPipeline::computeDescriptors(boost::shared_ptr<Eigen::MatrixXf> cloud,
				       boost::shared_ptr<Eigen::VectorXf> intensity)
{
  vector< boost::shared_ptr<MatrixXf> > clouds;
  clouds.push_back(cloud);
  vector< boost::shared_ptr<VectorXf> > intensities;
  intensities.push_back(intensity);

  vector<Object*> objects;
  objects = computeDescriptors(clouds, intensities);
  return objects[0];
}

int
DescriptorPipeline::getNumBytes()
{
  int num_bytes = 0;
  for(size_t i = 0; i < output_descriptors_[0].size(); ++i) {
    num_bytes += output_descriptors_[0][i]->getDescriptorLength() * 4;
  }
  return num_bytes;
}

} // namespace perception
