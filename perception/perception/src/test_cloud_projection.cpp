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
#include "track_manager.h"
#include "multibooster_support.h"

using namespace std;
using namespace track_manager;
using namespace sensor_msgs;

// Horrible copy from track_visualizer.cpp.
Object* getDescriptorsForCloud(const PointCloud& cloud, const vector<Descriptor3D*>& features) {
  // -- Set up for using descriptors_3d
  vector<int>* ptr = new vector<int>(cloud.get_points_size());
  for(size_t i=0; i<ptr->size(); ++i) {
    ptr->at(i) = i;
  }
  vector< const vector<int>* > interest_regions_indices(1); //Only one cluster in this cloud.
  interest_regions_indices[0] = ptr;

  // -- Compute the descriptors.
  vector<vvf> cluster_descriptors(features.size());
  cloud_kdtree::KdTree* kdt = new cloud_kdtree::KdTreeANN(cloud);
  for(size_t i=0; i<features.size(); ++i) {
    features[i]->compute(cloud, *kdt, interest_regions_indices, cluster_descriptors[i]);
  }
  for(size_t i=0; i<features.size(); ++i)
    features[i]->clearShared(); //Should be unnecessary, but good practice.

  // -- Put into object form.
  vector<Object*> objs;
  collectDescriptorsIntoObjects(cluster_descriptors, &objs);

  // -- Clean up
  delete kdt;
  for(size_t i=0; i<interest_regions_indices.size(); ++i) {
    delete interest_regions_indices[i];
  }

  return objs[0];
}


void collectDatasetForTrack(const Track& track, vector<Object*>* objects) {
  objects->reserve(track.clouds_.size()); //Doesn't shrink the capacity ever.

  // -- Get features.
  vector<SpectralAnalysis*> delete_me;
  vector<Descriptor3D*> desc = getFeatures(&delete_me);

  // -- Compute all descriptors.
  NameMapping class_map(getClassNames());
  for(size_t i=0; i<track.clouds_.size(); ++i) {
    Object* obj = getDescriptorsForCloud(*track.clouds_[i], desc);
    for(size_t j=0; j<delete_me.size(); ++j) {
      delete_me[j]->clearSpectral();
    }
    if(track.label_.compare("unlabeled") == 0)
      obj->label_ = -2;
    else if(track.label_.compare("background") == 0)
      obj->label_ = -1;
    else
      obj->label_ = class_map.toId(track.label_);

    objects->push_back(obj);
  }

  // -- Clean up.
  for(size_t i=0; i<delete_me.size(); ++i)
    delete delete_me[i];
  for(size_t i=0; i<desc.size(); ++i)
    delete desc[i]; 
}


int main(int argc, char** argv) {
  TrackManager tm("/home/alex/rss_tracks/tracks/car/dish_area01-11-17-2009_20-08-30.tm");
  vector<Object*> objects;
  collectDatasetForTrack(*tm.tracks_[0], &objects);

  vector<descriptor>& desc = objects[0]->descriptors_;
  for(size_t i=0; i<desc.size(); ++i)
    cout << i << ": " << objects[0]->descriptors_[i].vector->transpose() << endl << endl;
  
  return 0;
}
