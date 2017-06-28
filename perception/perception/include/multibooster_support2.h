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


#ifndef MULTIBOOSTER_SUPPORT2_H
#define MULTIBOOSTER_SUPPORT2_H

#include <multibooster/multibooster.h>
#include <cluster_descriptors/cluster_descriptors.h>
#include <sensor_msgs/PointCloud.h>
#include <perception_types.h>

namespace perception {

std::vector<std::string> getClassNames();
std::vector<std::string> getDescriptorNames();

void rosToEigen(const sensor_msgs::PointCloud &cloud,
		boost::shared_ptr<Eigen::MatrixXf>* points,
		boost::shared_ptr<Eigen::VectorXf>* intensities);

void dgcToEigen(const std::vector<point3d_t>& points,
		boost::shared_ptr<Eigen::MatrixXf>* cloud,
		boost::shared_ptr<Eigen::VectorXf>* intensities);

//! default label is "unlabeled"
Object* buildObject(const std::vector< boost::shared_ptr<pipeline::DescriptorNode> >& dns,
		    int label = -2);

std::vector<Object*> buildObjects(const std::vector< boost::shared_ptr<pipeline::DescriptorNode> >& dns);

void
generateDefaultDescriptorPipeline(std::vector< boost::shared_ptr<pipeline::ComputeNode> >* all_nodes,
				  boost::shared_ptr<PointCloudInterface>* input_orienter,
				  std::vector< boost::shared_ptr<pipeline::DescriptorNode> >* output_descriptors);


//! Core function that determines what descriptors we use.
void
generateDescriptorPipeline(std::vector< boost::shared_ptr<pipeline::ComputeNode> >* all_nodes,
			   boost::shared_ptr<PointCloudInterface>* input_orienter,
			   boost::shared_ptr<MultiBoosterObjectConstructor>* output_constructor,
			   std::vector< boost::shared_ptr<pipeline::DescriptorNode> >* output_descriptors);

void
generateDefaultDescriptorPipeline(std::vector< boost::shared_ptr<pipeline::ComputeNode> >* all_nodes,
				  boost::shared_ptr<PointCloudInterface>* input_orienter,
				  boost::shared_ptr<MultiBoosterObjectConstructor>* output_constructor,
				  std::vector< boost::shared_ptr<pipeline::DescriptorNode> >* output_descriptors);
void
generateExperimentalDescriptorPipeline(std::vector< boost::shared_ptr<pipeline::ComputeNode> >* all_nodes,
				       boost::shared_ptr<PointCloudInterface>* input_orienter,
				       boost::shared_ptr<MultiBoosterObjectConstructor>* output_constructor,
				       std::vector< boost::shared_ptr<pipeline::DescriptorNode> >* output_descriptors);


			   
				

class ClassifierPipeline
{
 public:
  pipeline::Pipeline pipeline_;
  std::vector< boost::shared_ptr<PointCloudInterface> > input_orienters_;
  std::vector< boost::shared_ptr<MultiBoosterNode> > output_classifiers_;
  
  ClassifierPipeline(MultiBooster* mb, int num_threads);
  std::vector<Eigen::VectorXf>
    classify(std::vector< boost::shared_ptr<Eigen::MatrixXf> > clouds,
	     std::vector< boost::shared_ptr<Eigen::VectorXf> > intensities);
};

class DescriptorPipeline
{
 public:
  pipeline::Pipeline pipeline_;
  std::vector< boost::shared_ptr<PointCloudInterface> > input_orienters_;
  std::vector< boost::shared_ptr<MultiBoosterObjectConstructor> > output_constructors_;
  //! output_descriptors_[i] is the vector of descriptor nodes corresponding to input_orienters_[i].
  std::vector< std::vector< boost::shared_ptr<pipeline::DescriptorNode> > > output_descriptors_;
  
  DescriptorPipeline(int num_threads);
  //! Returns the number of bytes in the descriptors for a single object.
  int getNumBytes();
  
  std::vector<Object*>
    computeDescriptors(std::vector< boost::shared_ptr<Eigen::MatrixXf> > clouds,
		       std::vector< boost::shared_ptr<Eigen::VectorXf> > intensities);
  Object* computeDescriptors(boost::shared_ptr<Eigen::MatrixXf> cloud,
			     boost::shared_ptr<Eigen::VectorXf> intensity);
};

} // namespace perception

#endif //MULTIBOOSTER_SUPPORT2_H
