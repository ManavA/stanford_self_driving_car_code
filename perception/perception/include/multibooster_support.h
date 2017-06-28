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


#ifndef MULTIBOOSTER_SUPPORT_H
#define MULTIBOOSTER_SUPPORT_H

#include "extra_features.h"
#include "perception.h"
#include "segment.h"
#include <boost/math/distributions/normal.hpp>

namespace perception {

std::vector<std::string> getClassNames();
std::vector<std::string> getFeatureNames(std::vector<Descriptor3D*> features);
std::vector<Descriptor3D*> getSpinFeatures();
std::vector<Descriptor3D*> getSpinRPs(std::vector<Descriptor3D*>* delete_me);
std::vector<Descriptor3D*> getLimitedClusterFeatures(double z, std::vector<SpectralAnalysis*>* delete_me);
std::vector<Descriptor3D*> getAllClusterFeatures(double z);
std::vector<Descriptor3D*> getClusterFeatures(double z, std::vector<SpectralAnalysis*>* delete_me);
void convertSpinToRos(const dgc::dgc_velodyne_spin& spin, sensor_msgs::PointCloud* cloud, std::vector<size_t>* spin_idx);
void convertClustersToRos(const std::vector< std::vector<point3d_t> >& clusters,
			  std::vector< const std::vector<int>* >* interest_regions_indices,
			  sensor_msgs::PointCloud* cloud);
void collectDescriptorsIntoObjects(const std::vector<vvf>& all_results, std::vector<Object*>* objs);
void collectDatasetForSpin(const std::vector< std::vector<point3d_t> >& clusters, double z, std::vector<Object*>* objs);
std::vector<Descriptor3D*> getFeatures(std::vector<SpectralAnalysis*>* delete_me, bool debug = false);

} // namespace perception
#endif //MULTIBOOSTER_SUPPORT_H
