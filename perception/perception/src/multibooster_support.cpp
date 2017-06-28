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


#include "multibooster_support.h"

using namespace std;
using namespace Eigen;
using namespace dgc;
using namespace std::tr1;


double sampleFromGaussian(double stdev) {
  double sum = 0;
  for(size_t i=0; i<12; ++i) {
    sum += 2 * stdev * (double)rand() / (double)RAND_MAX - stdev;
  }
  return sum / 2;
}

//0 = unknown, 1 = car, 2 = ped.
vector<string> getClassNames() {
  vector<string> classes;
  //classes.push_back("unknown_obstacle"); 
  classes.push_back("car");
  classes.push_back("pedestrian");
  classes.push_back("bicyclist");

  return classes;
}

vector<string> getFeatureNames(vector<Descriptor3D*> features) {
  vector<string> names;
  for(size_t i=0; i<features.size(); ++i) {
    assert(features[i]->getName().compare("invalid") != 0); 
    names.push_back(features[i]->getName());
  }
  return names;
}

vector<Descriptor3D*> getSpinFeatures() {
  vector<Descriptor3D*> features;

  features.push_back(new SpinImageCustom(0, 0, 1, .025, .025, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .05, .05, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .1, .1, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .2, .2, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .15, .15, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .3, .3, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .5, .5, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .025, .025, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .05, .05, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .1, .1, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .2, .2, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .15, .15, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .3, .3, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .5, .5, 10, 5, true));
  return features;
}  

vector<Descriptor3D*> getLimitedClusterFeatures(double z, vector<SpectralAnalysis*>* delete_me) {
  assert(delete_me->empty());
  vector<Descriptor3D*> features;
  
  SpectralAnalysis* sa0 = new SpectralAnalysis(0); //Radius == 0 => use the entire cluster.
  SpectralAnalysis* sa1 = new SpectralAnalysis(1); //Radius == 1 => get the points in a 1m radius sphere around the cluster centroid, then do the spectral analysis.
  delete_me->push_back(sa0);
  delete_me->push_back(sa1);
  features.push_back(new ShapeSpectral(*sa1));
  features.push_back(new BoundingBoxSpectral(0, *sa0));
  features.push_back(new BoundingBoxRaw(0));
  features.push_back(new Position(z)); // z height relative to the car.
  features.push_back(new SpinImageCustom(0, 0, 1, .2, .2, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .2, .2, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .5, .5, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .1, .1, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .15, .15, 10, 5, true));
  return features;
}

// vector<Descriptor3D*> getSpinRPs(vector<Descriptor3D*>* delete_me) {
//   vector<Descriptor3D*> features;
  
//   delete_me->push_back(new SpinImageCustom(0, 0, 1, .2, .2, 10, 5, true));
//   delete_me->push_back(new SpinImageCustom(0, 0, 1, .2, .2, 20, 10, true));
//   delete_me->push_back(new SpinImageCustom(0, 0, 1, .5, .5, 20, 10, true));
//   delete_me->push_back(new SpinImageCustom(0, 0, 1, .1, .1, 10, 5, true));
//   delete_me->push_back(new SpinImageCustom(0, 0, 1, .15, .15, 10, 5, true));

//   features.push_back(new RandomProjector((*delete_me)[0], 5, 100));
//   features.push_back(new RandomProjector((*delete_me)[1], 5, 101));
//   features.push_back(new RandomProjector((*delete_me)[2], 5, 102));
//   features.push_back(new RandomProjector((*delete_me)[3], 5, 103));
//   features.push_back(new RandomProjector((*delete_me)[4], 5, 104));

//   // -- Extended.
//   features.push_back(new RandomProjector((*delete_me)[0], 5, 1000));
//   features.push_back(new RandomProjector((*delete_me)[1], 5, 1010));
//   features.push_back(new RandomProjector((*delete_me)[2], 5, 1020));
//   features.push_back(new RandomProjector((*delete_me)[3], 5, 1030));
//   features.push_back(new RandomProjector((*delete_me)[4], 5, 1040));

//   features.push_back(new RandomProjector((*delete_me)[0], 5, 1100));
//   features.push_back(new RandomProjector((*delete_me)[1], 5, 1101));
//   features.push_back(new RandomProjector((*delete_me)[2], 5, 1102));
//   features.push_back(new RandomProjector((*delete_me)[3], 5, 1103));
//   features.push_back(new RandomProjector((*delete_me)[4], 5, 1104));

//   features.push_back(new RandomProjector((*delete_me)[0], 5, 2100));
//   features.push_back(new RandomProjector((*delete_me)[1], 5, 2101));
//   features.push_back(new RandomProjector((*delete_me)[2], 5, 2102));
//   features.push_back(new RandomProjector((*delete_me)[3], 5, 2103));
//   features.push_back(new RandomProjector((*delete_me)[4], 5, 2104));

//   features.push_back(new RandomProjector((*delete_me)[0], 5, 3100));
//   features.push_back(new RandomProjector((*delete_me)[1], 5, 3101));
//   features.push_back(new RandomProjector((*delete_me)[2], 5, 3102));
//   features.push_back(new RandomProjector((*delete_me)[3], 5, 3103));
//   features.push_back(new RandomProjector((*delete_me)[4], 5, 3104));

//   return features;  
// }

vector<Descriptor3D*> getSpin() {
  vector<Descriptor3D*> features;
  
  features.push_back(new SpinImageCustom(0, 0, 1, .2, .2, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .2, .2, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .5, .5, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .1, .1, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .15, .15, 10, 5, true));
  
  return features;
}

vector<Descriptor3D*> getAllClusterFeatures(double z) {
  vector<Descriptor3D*> features;
  //  features->push_back(SpinImageCustom(0, 0, 1, .05, .05, 20, 10, true));
  SpectralAnalysis* sa0 = new SpectralAnalysis(0); //Radius == 0 => use the entire cluster.
  SpectralAnalysis* sa1 = new SpectralAnalysis(1); //Radius == 1 => get the points in a 1m radius sphere around the cluster centroid, then do the spectral analysis.
  features.push_back(new ShapeSpectral(*sa0));
  features.push_back(new ShapeSpectral(*sa1));
  features.push_back(new BoundingBoxSpectral(0, *sa0));
  features.push_back(new BoundingBoxRaw(0));
  features.push_back(new Position(z)); // z height relative to the car.
  features.push_back(new SpinImageCustom(0, 0, 1, .025, .025, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .05, .05, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .1, .1, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .2, .2, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .15, .15, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .3, .3, 20, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .5, .5, 20, 10, true));

  features.push_back(new SpinImageCustom(0, 0, 1, .025, .025, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .05, .05, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .1, .1, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .2, .2, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .15, .15, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .3, .3, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .5, .5, 10, 5, true));
  return features;
}

//! @param z The z-height of the car at the time the features are collected.
vector<Descriptor3D*> getClusterFeatures(double z, vector<SpectralAnalysis*>* delete_me) {
  //  return getLimitedClusterFeatures(z, delete_me);
  return getFeatures(delete_me);
}

vector<Descriptor3D*> getFeatures(vector<SpectralAnalysis*>* delete_me, bool debug) {
  assert(delete_me->empty());
  vector<Descriptor3D*> features;
  
  SpectralAnalysis* sa0 = new SpectralAnalysis(0); //Radius == 0 => use the entire cluster.
  SpectralAnalysis* sa1 = new SpectralAnalysis(1); //Radius == 1 => get the points in a 1m radius sphere around the cluster centroid, then do the spectral analysis.
  delete_me->push_back(sa0);
  delete_me->push_back(sa1);

  features.push_back(new ShapeSpectral(*sa0));
  features.push_back(new BoundingBoxSpectral(0, *sa0));
  features.push_back(new BoundingBoxRaw(0));
  
  
  features.push_back(new CloudProjection(true, 0, 16, 8, 8));
  features.push_back(new CloudProjection(true, 1, 16, 8, 8));
  features.push_back(new CloudProjection(true, 0, 10, 10, 5));
  features.push_back(new CloudProjection(true, 1, 10, 10, 5));

  features.push_back(new CloudProjection(true, 2, 8, 13, 2.5));
  features.push_back(new CloudProjection(true, 1, 10, 20, 5));
  features.push_back(new CloudProjection(true, 1, 8, 16, 2));  

  features.push_back(new SpinImageCustom(0, 0, 1, .3, .3, 10, 10, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .15, .15, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .1, .1, 10, 5, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .2, .2, 10, 15, true));
  features.push_back(new SpinImageCustom(0, 0, 1, .2, .2, 10, 5, true));
  
  // Car bottom left of fat view.
  CloudProjection* cp0 = new CloudProjection(true, 1, 30, 30, 15, 0, 1);
  HogWrapper* hog0 = new HogWrapper(cvSize(30, 30), cvSize(30, 30), cvSize(30, 30), cvSize(10, 10), 6);
  hog0->debug_ = debug;
  features.push_back(new CloudProjectionHog(shared_ptr<CloudProjection>(cp0), shared_ptr<HogWrapper>(hog0), 5, 5)); 

  // Car bottom right of fat view.
  CloudProjection* cp1 = new CloudProjection(true, 1, 30, 30, 15, 1, 1);
  HogWrapper* hog1 = new HogWrapper(cvSize(30, 30), cvSize(30, 30), cvSize(30, 30), cvSize(10, 10), 6);
  hog1->debug_ = debug;
  features.push_back(new CloudProjectionHog(shared_ptr<CloudProjection>(cp1), shared_ptr<HogWrapper>(hog1), 5, 5)); 

  // Coarse large object overviews.
  CloudProjection* cp2 = new CloudProjection(true, 0, 20, 20, 10, 0.5, 0.5);
  HogWrapper* hog2 = new HogWrapper(cvSize(20, 20), cvSize(20, 20), cvSize(20, 20), cvSize(5, 5), 6);
  hog2->debug_ = debug;
  features.push_back(new CloudProjectionHog(shared_ptr<CloudProjection>(cp2), shared_ptr<HogWrapper>(hog2), 3, 3)); 

  CloudProjection* cp3 = new CloudProjection(true, 1, 30, 30, 10, 0.5, 0.5);
  HogWrapper* hog3 = new HogWrapper(cvSize(30, 30), cvSize(30, 30), cvSize(30, 30), cvSize(10, 10), 6);
  hog3->debug_ = debug;
  features.push_back(new CloudProjectionHog(shared_ptr<CloudProjection>(cp3), shared_ptr<HogWrapper>(hog3), 5, 5)); 
  
  CloudProjection* cp4 = new CloudProjection(true, 2, 30, 60, 10, 0.5, 0.5);
  HogWrapper* hog4 = new HogWrapper(cvSize(60, 30), cvSize(60, 30), cvSize(60, 30), cvSize(10, 10), 6);
  hog4->debug_ = debug;
  features.push_back(new CloudProjectionHog(shared_ptr<CloudProjection>(cp4), shared_ptr<HogWrapper>(hog4), 5, 5)); 

  // Detailed views.
  CloudProjection* cp5 = new CloudProjection(true, 1, 20, 20, 20, 1, 1); // Bottom left.
  HogWrapper* hog5 = new HogWrapper(cvSize(20, 20), cvSize(20, 20), cvSize(20, 20), cvSize(5, 5), 6);
  hog5->debug_ = debug;
  features.push_back(new CloudProjectionHog(shared_ptr<CloudProjection>(cp5), shared_ptr<HogWrapper>(hog5), 3, 3)); 

  CloudProjection* cp6 = new CloudProjection(true, 1, 20, 20, 20, 0, 1); // Bottom right.
  HogWrapper* hog6 = new HogWrapper(cvSize(20, 20), cvSize(20, 20), cvSize(20, 20), cvSize(5, 5), 6);
  hog6->debug_ = debug;
  features.push_back(new CloudProjectionHog(shared_ptr<CloudProjection>(cp6), shared_ptr<HogWrapper>(hog6), 3, 3)); 

  CloudProjection* cp7 = new CloudProjection(true, 1, 20, 20, 20, 0.5, 0); // Top middle.
  HogWrapper* hog7 = new HogWrapper(cvSize(20, 20), cvSize(20, 20), cvSize(20, 20), cvSize(5, 5), 6);
  hog7->debug_ = debug;
  features.push_back(new CloudProjectionHog(shared_ptr<CloudProjection>(cp7), shared_ptr<HogWrapper>(hog7), 3, 3)); 

  // Super wide view for buses.
  CloudProjection* cp8 = new CloudProjection(true, 1, 30, 60, 5, 0.5, 0.5);
  HogWrapper* hog8 = new HogWrapper(cvSize(60, 30), cvSize(60, 30), cvSize(60, 30), cvSize(10, 10), 6);
  hog8->debug_ = debug;
  features.push_back(new CloudProjectionHog(shared_ptr<CloudProjection>(cp8), shared_ptr<HogWrapper>(hog8), 3, 3)); 

  for(size_t i=0; i<features.size(); ++i) {
    features[i]->debug_ = debug;
  }

  // -- Convert all features but the first simple 3 (low dim) to random projections.
//   if(feature_set.compare("many2rp") == 0) {
//     vector<Descriptor3D*> foo = features;
//     features.clear();
//     features.resize(foo.size());
//     for(size_t i=0; i<foo.size(); ++i) {
//       if(i < 3)
// 	features[i] = foo[i];
//       else
// 	features[i] = new RandomProjector(shared_ptr<Descriptor3D>(foo[i]), 20, 4200*i);
//     }
//   }

  return features;  
}

void convertSpinToRos(const dgc_velodyne_spin& spin, sensor_msgs::PointCloud* cloud, vector<size_t>* spin_idx) {
  size_t num_pts = 0;
  for(int i = 0; i < spin.num_scans; i++) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      if(spin.scans[i].p[j].range < 0.01)
	continue;
      num_pts++;
    }
  }
  spin_idx->resize(num_pts, 0);
  cloud->header.stamp = ros::Time::now();
  cloud->header.frame_id = "sensor_frame";
  cloud->set_points_size(num_pts);
  cloud->set_channels_size(0);
  
  size_t idx = 0;
  for(int i = 0; i < spin.num_scans; i++) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      if(spin.scans[i].p[j].range < 0.01)
	continue;
      cloud->points[idx].x = spin.scans[i].p[j].x * 0.01; //TODO: Is this the right conversion? 
      cloud->points[idx].y = spin.scans[i].p[j].y * 0.01;
      cloud->points[idx].z = spin.scans[i].p[j].z * 0.01;
      spin_idx->at(idx) = i*VELO_BEAMS_IN_SCAN + j;
      idx++;
    }
  }
  assert(idx == num_pts);
}


void convertClustersToRos(const vector< vector<point3d_t> >& clusters,
			  vector< const vector<int>* >* interest_regions_indices,
			  sensor_msgs::PointCloud* cloud) {

  // -- Get num_pts in the clusters and initialize output.
  size_t num_pts = 0;
  for(size_t i=0; i<clusters.size(); ++i) {
    vector<point3d_t> cl = clusters[i];
    num_pts += cl.size();
  }
  cloud->set_points_size(num_pts);
  cloud->set_channels_size(1);
  cloud->channels[0].set_values_size(num_pts);
  interest_regions_indices->clear();
  interest_regions_indices->reserve(clusters.size()); // Some clusters may be empty.

  // -- Fill the output.
  size_t idx = 0;
  for(size_t i=0; i<clusters.size(); ++i) {
    vector<point3d_t> cl = clusters[i];
    if(cl.empty())
      continue;
    vector<int>* ptr = new vector<int>(cl.size());
    for(size_t j=0; j<cl.size(); ++j) {
      cloud->points[idx].x = cl[j].x;
      cloud->points[idx].y = cl[j].y;
      cloud->points[idx].z = cl[j].z;
      cloud->channels[0].values[idx] = cl[j].intensity;
      ptr->at(j) = idx;
      idx++;
    }
    interest_regions_indices->push_back(ptr);
  }
  assert(idx == num_pts);
}
  
//! Collects unlabeled objects.
void collectDescriptorsIntoObjects(const vector<vvf>& all_results, vector<Object*>* objs) {
  assert(objs->empty());
  for(size_t i=1; i<all_results.size(); ++i) {
    assert(all_results[i].size() == all_results[i-1].size());
  }
  size_t num_objs = all_results[0].size();
  
  objs->reserve(num_objs);

  for(size_t i=0; i<num_objs; ++i) { 
    Object* obj = new Object;
    vector<descriptor> desc(all_results.size());
    for(size_t j=0; j<all_results.size(); ++j) {
      if(all_results[j][i].empty())
	desc[j].vector = NULL;
      else {
	desc[j].vector = new VectorXf(all_results[j][i].size());
	floatToEigen(all_results[j][i], desc[j].vector);
	desc[j].length_squared = desc[j].vector->dot(*desc[j].vector);
      }
    }
    obj->descriptors_ = desc;
    obj->label_ = -2; //To possibly be overwritten by the calling function.
    objs->push_back(obj);
  }
}

//! Gets unlabeled Objects from a spin.
void collectDatasetForSpin(const vector< vector<point3d_t> >& clusters, double z, vector<Object*>* objs) {
  assert(objs->empty());
  
  // -- Put data in form descriptor3d can handle.
  sensor_msgs::PointCloud cloud;
  vector< const vector<int>* > interest_regions_indices;
  convertClustersToRos(clusters, &interest_regions_indices, &cloud);
  cloud_kdtree::KdTree* kdt = new cloud_kdtree::KdTreeANN(cloud);
//  cout << "cloud pts: " << cloud.get_points_size() << endl;
  
  // -- Compute cluster features.
  vector<SpectralAnalysis*> delete_me;
  vector<Descriptor3D*> features = getClusterFeatures(z, &delete_me);
  vector<vvf> cluster_descriptors(features.size());
  for(size_t i=0; i<features.size(); ++i)
    features[i]->compute(cloud, *kdt, interest_regions_indices, cluster_descriptors[i]);
  for(size_t i=0; i<features.size(); ++i)
    features[i]->clearShared(); //Should be unnecessary, but good practice.


  // -- Compute random projection features.
//   vector<Descriptor3D*> delete_me;
//   vector<Descriptor3D*> features = getSpinRPs(&delete_me);
//   vector<vvf> cluster_descriptors(features.size());
//   for(size_t i=0; i<features.size(); ++i) {
//     features[i]->compute(cloud, *kdt, interest_regions_indices, cluster_descriptors[i]);
//   }
//  for(size_t i=0; i<features.size(); ++i)
//    features[i]->clearShared(); //Should be unnecessary, but good practice.
  
  // -- Put vectorized results into Objects.
  collectDescriptorsIntoObjects(cluster_descriptors, objs);
  
  // -- Clean up.
  delete kdt;
  for(size_t i=0; i<interest_regions_indices.size(); ++i) {
    delete interest_regions_indices[i];
  }
  for(size_t i=0; i<features.size(); ++i) {
    delete features[i];
  }
  for(size_t i=0; i<delete_me.size(); ++i) {
    delete delete_me[i];
  }
}  
