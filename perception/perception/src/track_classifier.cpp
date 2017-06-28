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
#include <boost/filesystem.hpp>
#include "multibooster_support2.h"

using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace pipeline;
using namespace Eigen;



// Object* buildObject(const vector< shared_ptr<DescriptorNode> >& dns, int label) {
//   Object* obj = new Object;
//   obj->label_ = label;
//   obj->descriptors_ = vector<descriptor>(dns.size());
  
//   for(size_t i = 0; i < dns.size(); ++i) {
//     descriptor& desc = obj->descriptors_[i];
//     if(dns[i]->getDescriptor()) {
//       desc.vector = new VectorXf(); 
//       *desc.vector = *dns[i]->getDescriptor(); //TODO: Make multibooster take a shared_ptr.
//       desc.length_squared = desc.vector->dot(*desc.vector);
//     }
//     else { 
//       desc.vector = NULL;
//       desc.length_squared = -1;
//     }
//   }
  
//   return obj;
//}



// Object* getDescriptorsForCloud2(DescriptorPipeline& dp, const sensor_msgs::PointCloud& roscloud, bool debug = false, bool timing = false) {
//   shared_ptr<MatrixXf> cloud((MatrixXf*)NULL);
//   shared_ptr<VectorXf> intensity((VectorXf*)NULL);
//   rosToEigen(roscloud, &cloud, &intensity);
//   dp.setInput(cloud, intensity);

//   timeval start, end;
//   gettimeofday(&start, NULL);
  
//   dp.compute();
//   vector< shared_ptr<DescriptorNode> > dns = dp.getOutputDescriptorNodes(0);

//   Object* obj = new Object;
//   obj->descriptors_ = vector<descriptor>(dns.size());
//   for(size_t i = 0; i < dns.size(); ++i) {
//     descriptor& desc = obj->descriptors_[i];
//     if(dns[i]->getDescriptor()) {
//       desc.vector = new VectorXf(); 
//       *desc.vector = *dns[i]->getDescriptor(); //TODO: Make multibooster take a shared_ptr.
//       desc.length_squared = desc.vector->dot(*desc.vector);
//     }
//     else { 
//       desc.vector = NULL;
//       desc.length_squared = -1;
//     }
//   }
//   dp.flush();
  
//   gettimeofday(&end, NULL);
//   if(timing)
//     cout << (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000. << " ms to compute descriptors and copy into object." << endl;
//   return obj;
// }


void accumulateStats(MultiBooster* mb, const TrackManager& tm, PerfStats* stats, vector< shared_ptr<Track> >* misclassified) {
  size_t max_num_clouds = 0;
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    if(tm.tracks_[i]->clouds_.size() > max_num_clouds)
      max_num_clouds = tm.tracks_[i]->clouds_.size();
  }
  cout << "Building pipeline..."; cout.flush();
  int num_threads = 1;
  ClassifierPipeline cp(mb, num_threads);
  cout << " done." << endl;

  timeval start, end;
  gettimeofday(&start, NULL);
  int num_clouds = 0;
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    if(i % 100 == 0) {
      cout << "."; cout.flush();
    }
    //cout << "Working on track " << i << " / " << tm.tracks_.size() << endl;
    Track& tr = *tm.tracks_[i];

    int label;
    if(tr.label_.compare("unlabeled") == 0)
      continue;
    if(tr.label_.compare("background") == 0)
      label = -1;
    else
      label = mb->class_map_.toId(tr.label_);

    
    vector< shared_ptr<MatrixXf> > clouds(tr.clouds_.size());
    vector< shared_ptr<VectorXf> > intensities(tr.clouds_.size());
    for(size_t j = 0; j < tr.clouds_.size(); ++j) {
      ++num_clouds;
      shared_ptr<MatrixXf> cloud((MatrixXf*)NULL);
      shared_ptr<VectorXf> intensity((VectorXf*)NULL);
      rosToEigen(*tr.clouds_[j], &cloud, &intensity);
      clouds[j] = cloud;
      intensities[j] = intensity;
    }

    timeval start2, end2;
    gettimeofday(&start2, NULL);
    vector<VectorXf> responses = cp.classify(clouds, intensities);
    gettimeofday(&end2, NULL);
    cout << ((end2.tv_sec - start2.tv_sec) * 1000. + (end2.tv_usec - start2.tv_usec) / 1000.) / (float)clouds.size() << " ms per object." << endl;

    // -- Get the prediction and increment statistics.
    VectorXf track_prediction = mb->prior_;
    assert(responses.size() == tr.clouds_.size());
    for(size_t j = 0; j < responses.size(); ++j) {
      track_prediction += 2*responses[j] - mb->prior_;
    }
    cout << track_prediction.transpose() << endl;
    stats->incrementStats(label, track_prediction);

    // -- If we were wrong, set the track aside to be saved.
    int prediction = 0;
    float val = track_prediction.maxCoeff(&prediction);
    if(val < 0)
      prediction = -1;
    if(label != prediction)
      misclassified->push_back(tm.tracks_[i]);
  }
  gettimeofday(&end, NULL);
  cout << "Descriptor computation and classification of " << num_clouds << " clouds took "
       << end.tv_sec - start.tv_sec << " seconds, mean time per cloud is " << (double)(end.tv_sec - start.tv_sec) / (double)num_clouds << endl;
}

int main(int argc, char** argv) {

  if(argc < 3) {
    cout << "Usage: " << argv[0] << " CLASSIFIER TRACK_MANAGER [TRACK_MANAGER ...]" << endl;
    return 1;
  }
  
  cout << "Loading multibooster " << argv[1] << endl;
  MultiBooster* mb = new MultiBooster(argv[1]);
  if(getenv("WC_LIMIT"))
    mb->wc_limiter_ = atoi(getenv("WC_LIMIT"));

  mb->applyNewMappings(mb->class_map_, getDescriptorNames());
  cout << mb->status(false) << endl;

  PerfStats stats(mb->class_map_);
  for(int i = 2; i < argc; ++i) { 
    cout << "Loading track manager " << argv[i] << endl;
    cerr << "Loading track manager " << argv[i] << endl;
    TrackManager tm(argv[i]);
    cout << "Loaded " << tm.tracks_.size() << " tracks." << endl;
    if(tm.tracks_.size() == 0) {
      cerr << "0 tracks.  Skipping." << endl;
      continue;
    }
    vector< shared_ptr<Track> > misclassified;
    accumulateStats(mb, tm, &stats, &misclassified);


    boost::filesystem::path path(argv[i]);
    string stem = path.filename().substr(0, path.filename().size() - 3);
    string savename = "misclassified/" + stem + ".tm";
    cout << misclassified.size() << " misclassified tracks.  Saving to " << savename << endl;
    TrackManager mc(misclassified);
    int foo = system("mkdir -p misclassified");     --foo; // This is to avoid a warning.
    mc.save(savename);
  }

  cout << stats.statString() << endl;
  delete mb;
  return 0;
}
