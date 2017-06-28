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


#include <track_manager.h>
#include "multibooster_support2.h"

using namespace pipeline;
using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace sensor_msgs;
using namespace Eigen;

MultiBoosterDataset* extractDatasetFromTrackManager(const TrackManager& tm) {
  NameMapping class_map(getClassNames());
  
  // -- Set up multibooster dataset with the class and descriptor names.
  int num_threads = 1;
  if(getenv("NUM_THREADS"))
    num_threads = atoi(getenv("NUM_THREADS"));
  DescriptorPipeline dp(num_threads);
  cout << dp.getNumBytes() << " bytes per object." << endl;
  MultiBoosterDataset* mbd = new MultiBoosterDataset(getClassNames(), getDescriptorNames());

  // -- Get a vector of Object*.
  vector<Object*> objects;
  objects.reserve(tm.tracks_[0]->clouds_.size() * tm.tracks_.size()); //Should be ordered with longest first.
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
  //for(size_t i = 0; i < 2; ++i) {

    cout << "Working on track " << i << " of " << tm.tracks_.size() << "... "; cout.flush();
    Track& track = *tm.tracks_[i];
    
    // -- Set up the pipeline to compute all the clouds for this track in parallel.
    vector< shared_ptr<MatrixXf> > clouds(track.clouds_.size());
    vector< shared_ptr<VectorXf> > intensities(track.clouds_.size());
    for(size_t j = 0; j < track.clouds_.size(); ++j) { 
      rosToEigen(*track.clouds_[j], &clouds[j], &intensities[j]);
      assert(clouds[j]);
      assert(intensities[j]);
    }

    
    // -- Compute the descriptors and put into object form.
    timeval start, end;
    gettimeofday(&start, NULL);
    vector<Object*> track_objs = dp.computeDescriptors(clouds, intensities);
    gettimeofday(&end, NULL);
    cout << " done in " << (end.tv_sec - start.tv_sec)*1000. + (end.tv_usec - start.tv_usec)/1000. << " ms" << endl;
    
    // -- Set the labels.
    assert(track_objs.size() == track.clouds_.size());
    int label = -2;
    if(track.label_.compare("unlabeled") == 0)
      continue;
    else if(track.label_.compare("background") == 0)
      label = -1;
    else
      label = class_map.toId(track.label_);

    for(size_t j = 0; j < track_objs.size(); ++j)
      track_objs[j]->label_ = label;

    // -- Append to objects.
    for(size_t j = 0; j < track_objs.size(); ++j)
      objects.push_back(track_objs[j]);
    
  }
        
  // -- Add to MultiBoosterDataset and return.
  mbd->setObjs(objects);
  return mbd;
}


int main(int argc, char** argv) {
  if(argc < 3) {
    cout << "Usage: " << argv[0] << " TRACK_MANAGER OUTPUT_DATASET_FILENAME " << endl;
    return 1;
  }

  string dataset_filename(argv[2]);
  cout << "Dataset will be saved to " << dataset_filename << endl;
  
  string track_manager_filename(argv[1]);
  cout << "Loading track manager " << track_manager_filename << endl;
  TrackManager tm(track_manager_filename);
  cout << "Loaded " << tm.tracks_.size() << " tracks." << endl;
  if(tm.tracks_.size() == 0) {
    cerr << "0 tracks.  Aborting." << endl;
    return 2;
  }

  MultiBoosterDataset* mbd = extractDatasetFromTrackManager(tm);
  cout << "Saving dataset to " << dataset_filename << endl;
  mbd->save(dataset_filename);
  delete mbd;
  
  return 0;
}  
