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


#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <velo_support.h>
#include <imagery.h>
#include <gui3D.h>
#include <lltransform.h>
#include <transform.h>
#include <passat_constants.h>
#include <param_interface.h>
#include <passatmodel.h>
#include <vector>
#include <iostream>

#include "perception.h"
#include "segment.h"
#include <track_manager.h>
#include <videoout.h>
#include <sys/stat.h> 
#include "multibooster_support.h"

using namespace dgc;
using namespace std;
using namespace std::tr1;
using namespace track_manager;
using namespace sensor_msgs;
using namespace vlr;

using namespace Eigen;

#define RSS_CAR_PRIOR 0.0406
#define RSS_PED_PRIOR 0.0388
#define RSS_BIKE_PRIOR 0.0182

#define RSS_MIN_HEIGHT_CAR 0.8
#define RSS_MIN_XY_CAR 2
#define RSS_MIN_HEIGHT_PEDESTRIAN 0.6
#define RSS_MIN_XY_PEDESTRIAN 0.5
#define RSS_MIN_HEIGHT_BICYCLIST 1.0
#define RSS_MIN_XY_BICYCLIST 1.1

TrackManager g_tm;
shared_ptr<PointCloud> g_cloud;
int g_track_num = 0;
int g_cloud_num = 0;
bool g_pause = false;
bool g_record = false;
dgc_videoout_p g_vo = NULL;
string g_tm_filename;
bool g_follow = true;
bool g_fast_forward = false;
bool g_wrap = true; //When at the end of a track, if this is true it will wrap back to cloud 0 of the current track; otherwise it will move on to the next track.
bool g_classified_all_tracks = false;
vector<MatrixXf> g_cloud_classifications; // g_cloud_classifications[i].col(j) is the VectorXf of the classification result for cloud j of track i.
MatrixXf g_track_classifications; // g_cloud_classifications.col(i) is the VectorXf of the trackwise classification result for track i.
NameMapping g_class_map = NameMapping(getClassNames());
shared_ptr<MultiBooster> g_multibooster;
shared_ptr<MultiBooster> g_meta_multibooster;
string g_mb_filename;
bool g_display_intensity = false;


Object* createObjectFromTrack(const Track& tr, const MatrixXf& cloud_classifications) {
  // -- Collect features.
  descriptor pct;
  descriptor max_pts_score;
  descriptor total;
  descriptor max_score;
  descriptor min_score;

  // -- Percent of frames classified as each class.
  pct.vector = new VectorXf(cloud_classifications.rows());
  *pct.vector = (cloud_classifications.cwise() > 0).cast<float>().rowwise().sum();
  *pct.vector = *pct.vector / (double) cloud_classifications.cols();

  // -- Responses for the frame with the max number of pts.
  size_t max_pts = 0;
  size_t idx = 0;
  for(size_t i=0; i<tr.clouds_.size(); ++i) {
    if(tr.clouds_[i]->get_points_size() > max_pts) {
      max_pts = tr.clouds_[i]->get_points_size();
      idx = i;
    }
  }
  max_pts_score.vector = new VectorXf(cloud_classifications.rows());
  *max_pts_score.vector = cloud_classifications.col(idx);

  // -- Total response.
  total.vector = new VectorXf(cloud_classifications.rows());
  *total.vector = cloud_classifications.rowwise().sum();

  // -- Maximum response for each class.
  max_score.vector = new VectorXf(cloud_classifications.rows());
  *max_score.vector = cloud_classifications.rowwise().maxCoeff();

  // -- Minimum response for each class.
  min_score.vector = new VectorXf(cloud_classifications.rows());
  *min_score.vector = cloud_classifications.rowwise().minCoeff();


  // -- Put descriptors into Object.
  Object* obj = new Object();
  if(tr.label_.compare("background") == 0)
    obj->label_ = -1;
  else if(tr.label_.compare("unlabeled") == 0)
    obj->label_ = -2;
  else
    obj->label_ = g_class_map.toId(tr.label_);

  obj->descriptors_.push_back(pct);
  obj->descriptors_.push_back(max_pts_score);
  obj->descriptors_.push_back(total);
  obj->descriptors_.push_back(max_score);
  obj->descriptors_.push_back(min_score);
  for(size_t i=0; i<obj->descriptors_.size(); ++i)
    obj->descriptors_[i].length_squared = obj->descriptors_[i].vector->dot(*obj->descriptors_[i].vector);

  return obj;
}

void incrementStats(PerfStats* track_stats, PerfStats* cloud_stats, PerfStats* progressive_stats, PerfStats* meta_stats) { 
  assert(g_cloud_classifications.size() == g_tm.tracks_.size());

  for(size_t i=0; i<g_cloud_classifications.size(); ++i) {
    string str = g_tm.tracks_[i]->label_;
    int label = -2;
    if(str.compare("unlabeled") == 0)
      continue;
    else if(str.compare("background") == 0)
      label = -1;
    else
      label = g_class_map.toId(str);
    
    VectorXf sum = g_cloud_classifications[i].rowwise().sum();

    // // Consider only positive responses.
//     VectorXf sum = VectorXf::Zero(g_cloud_classifications[i].rows());
//     for(int r=0; r<g_cloud_classifications[i].rows(); ++r)
//       for(int c=0; c<g_cloud_classifications[i].cols(); ++c)
// 	if(g_cloud_classifications[i](r,c) > 0)
// 	  sum(r) += g_cloud_classifications[i](r,c);
    
    track_stats->incrementStats(label, sum);

    // -- Classify the track using the meta classifier.
    // Get the priors.
    VectorXf priors(g_class_map.size());
    priors(g_class_map.toId("car")) = RSS_CAR_PRIOR;
    priors(g_class_map.toId("pedestrian")) = RSS_PED_PRIOR;
    priors(g_class_map.toId("bicyclist")) = RSS_BIKE_PRIOR;
    VectorXf inv_priors = VectorXf::Ones(g_class_map.size()) - priors;

    // Get p(y^c | x_t), ie boosting response in probability form.
//     MatrixXf cloud_prob = g_cloud_classifications[i];
//     for(int c=0; c<cloud_prob.rows(); ++c) {
//       for(int m=0; m<cloud_prob.cols(); ++m) {
// 	cloud_prob(c,m) = 1.0 / (1.0 + exp(-2.0 * g_cloud_classifications[i](c,m)));
//       }
//     }
//     MatrixXf inv_cloud_prob = MatrixXf::Ones(cloud_prob.rows(), cloud_prob.cols()) - cloud_prob;

    // Add up the log probabilities.
    assert((int)g_tm.tracks_[i]->clouds_.size() == g_cloud_classifications[i].cols());
    double num_clouds = g_tm.tracks_[i]->clouds_.size();    
    // VectorXf log_odds = (num_clouds - 1) * (inv_priors.cwise().log() - priors.cwise().log()) + (cloud_prob.cwise().log()).rowwise().sum() - (inv_cloud_prob.cwise().log()).rowwise().sum();
//     cout << "log_odds: " << log_odds.transpose() << endl;
    VectorXf log_odds = (num_clouds - 1) * (inv_priors.cwise().log() - priors.cwise().log()) + 2 * g_cloud_classifications[i].rowwise().sum();
    meta_stats->incrementStats(label, log_odds);
			       
    // -- Collect cloudwise and progressive stats.
    for(int j=0; j<g_cloud_classifications[i].cols(); ++j) { 
      cloud_stats->incrementStats(label, g_cloud_classifications[i].col(j));
      progressive_stats->incrementStats(label, g_cloud_classifications[i].block(0, 0, g_cloud_classifications[i].rows(), j+1).rowwise().sum());
    }
  }
}

void drawGrid(double center_x, double center_y) {
  glBegin(GL_LINES);
  glLineWidth(3);
  glColor3f(0.5, 0.5, 0.5);


  int increment = 5;
  double z = -1;
  int extent = 50;
  for(int grid_x = -extent; grid_x < extent; grid_x+=increment) {
    glVertex3f(grid_x - center_x, -extent - center_y, z);
    glVertex3f(grid_x - center_x,  extent - center_y, z);
  }
  for(int grid_y = -extent; grid_y < extent; grid_y+=increment) {
    glVertex3f(-extent - center_x, grid_y - center_y, z );
    glVertex3f( extent - center_x, grid_y - center_y, z );
  }
  glEnd();
}

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

// Use track_classifier to predict class given the data derived from frame classifier g_multibooster.
// void classifyAll(const Multibooster& track_classifier) {
//   assert(g_multibooster);
  
VectorXf classifyTrack(MultiBooster& track_classifier, const Track& tr,
		       const MatrixXf& cloud_classifications) {
  Object* obj = createObjectFromTrack(tr, cloud_classifications);
  VectorXf response = track_classifier.treeClassify(*obj);
  delete obj;
  return response;
}

// Assumes that g_cloud_classifications is filled already.
MultiBoosterDataset* collectMetaDataset() {
  assert(!g_cloud_classifications.empty());
  vector<Object*> objs;
  for(size_t i=0; i<g_tm.tracks_.size(); ++i) {
    if(g_tm.tracks_[i]->label_.compare("unlabeled") == 0)
      continue;
    objs.push_back(createObjectFromTrack(*g_tm.tracks_[i], g_cloud_classifications[i]));
  }

  vector<string> meta_feature_names;
  meta_feature_names.push_back("percent"); // The order here must match that in the createObjectFromTrack function.
  meta_feature_names.push_back("max_numpts_score");
  meta_feature_names.push_back("total");
  meta_feature_names.push_back("max_scores");
  meta_feature_names.push_back("min_scores");
  MultiBoosterDataset* mbd = new MultiBoosterDataset(getClassNames(), meta_feature_names);
  mbd->setObjs(objs);

//   for(size_t i=0; i<objs.size(); ++i) {
//     cout << "Object " << i << endl;
//     cout << objs[i]->status(mbd->class_map_, mbd->feature_map_);
//     cout << "****************************************" << endl;
//   }

  return mbd;
}

void classifyAll(MultiBooster* track_classifier = NULL) {
  assert(g_multibooster);

  g_cloud_classifications.clear();
  g_cloud_classifications.resize(g_tm.tracks_.size());
  vector<double> track_errors(g_tm.tracks_.size(), 0);
  double total_time = 0;
  for(size_t i=0; i<g_tm.tracks_.size(); ++i) {
    if(g_tm.tracks_.size() > 10 && (int)i%((int)floor(g_tm.tracks_.size() / 10.0)) == 0) {
      cout << ".";   cout.flush();
    }
    
    vector<Object*> objs;
    collectDatasetForTrack(*g_tm.tracks_[i], &objs);
    
    assert(objs.size() == g_tm.tracks_[i]->clouds_.size());
    g_cloud_classifications[i] = MatrixXf((int)getClassNames().size(), (int)g_tm.tracks_[i]->clouds_.size());

    for(size_t j=0; j<objs.size(); ++j) {
      clock_t start = clock();
      if(getenv("USE_REG_EVAL"))
	g_cloud_classifications[i].col(j) = g_multibooster->classify(*objs[j]);
      else 
	g_cloud_classifications[i].col(j) = g_multibooster->treeClassify(*objs[j]);

      total_time += clock() - start;
      delete objs[j];
    }

    // -- Get the track error.
    VectorXf total = g_cloud_classifications[i].rowwise().sum();
    assert((size_t)total.rows() == g_class_map.size());
    string label_str = g_tm.tracks_[i]->label_;
    if(label_str.compare("unlabeled") == 0) {
      track_errors[i] = -1; //Make these show up last in the sort.
      continue;
    }
    else {
      int label = -1;
      if(label_str.compare("background") != 0)
	label = g_class_map.toId(label_str);
      
      for(int c=0; c<(int)g_class_map.size(); ++c) {
	if(label != c && total(c) > 0)
	  track_errors[i] += total(c);
	else if(label == c && total(c) < 0)
	  track_errors[i] -= total(c);
      }
    }
  }
  cout << endl;
  cout << setprecision(10) << "Total time to classify is " << total_time * 1000.0 / (double)CLOCKS_PER_SEC << " milliseconds." << endl;

  // -- Use the track classifier if we're given one.
  g_track_classifications = MatrixXf::Zero(g_class_map.size(), g_tm.tracks_.size());
  if(track_classifier) { 
    for(size_t i=0; i<g_tm.tracks_.size(); ++i)
      g_track_classifications.col(i) = classifyTrack(*track_classifier, *g_tm.tracks_[i], g_cloud_classifications[i]);
  }
  
  // -- Sort tracks and classifications based on error.
  //g_tm.sortTracks(track_errors); Can't do this - tracks with no error will be randomly sorted, so we need to do the sort in one shot.
  assert(track_errors.size() == g_cloud_classifications.size());
  vector< pair<double, size_t > > error_idx(g_cloud_classifications.size());
  for(size_t i=0; i<g_cloud_classifications.size(); ++i) {
    error_idx[i].first = track_errors[i];
    error_idx[i].second = i;
  }
  greater< pair<double, size_t > > emacs = greater< pair<double, size_t > >();
  sort(error_idx.begin(), error_idx.end(), emacs); //Descending.

  vector<MatrixXf> new_track_classifications(g_cloud_classifications.size());
  vector< shared_ptr<Track> > new_tracks(g_tm.tracks_.size());
  vector< shared_ptr<Track> > inc;
  inc.reserve(g_tm.tracks_.size());
  for(size_t i=0; i<g_cloud_classifications.size(); ++i) {
    new_track_classifications[i] = g_cloud_classifications[error_idx[i].second];
    new_tracks[i] = g_tm.tracks_[error_idx[i].second];
    if(error_idx[i].first > 0)
      inc.push_back(g_tm.tracks_[error_idx[i].second]);
  }
  g_cloud_classifications = new_track_classifications;
  g_tm.tracks_ = new_tracks;

  // -- Save all the misclassified tracks.
  if(g_tm_filename.find("misclassified") == string::npos) { 
    TrackManager tm_inc(inc);
    string basename = g_tm_filename.substr(0, g_tm_filename.size() - 3);
    string stem = basename.substr(basename.find_last_of("/")+1, basename.size());
    string savename = stem + "_misclassified";
    
    if(getenv("FILTER_OUT_UNREASONABLE")) 
      savename = savename + "_only_unocc.tm";
    else if(getenv("FILTER_OUT_REASONABLE"))
      savename = savename + "_only_occ.tm";
    else if(!getenv("FILTER_OUT_REASONABLE") && !getenv("FILTER_OUT_UNREASONABLE"))
      savename = savename + "_all.tm";
    else { 
      cerr << "Can't be filtering both occ and unocc tracks." << endl;
      exit(1);
    }
    cout << "Saving misclassified to " << savename << endl;
    tm_inc.save(savename);  
  }
}

void collectDatasetForTracks() {
  // -- Reserve space for the descriptors.
  int num_clouds = 0;
  for(size_t i=0; i<g_tm.tracks_.size(); ++i) {
    num_clouds += g_tm.tracks_[i]->clouds_.size();
  }
  vector<Object*> objs;
  objs.reserve(num_clouds);

  // -- Get features.
  vector<SpectralAnalysis*> delete_me;
  vector<Descriptor3D*> desc = getFeatures(&delete_me);

  // -- Compute all descriptors.
  NameMapping class_map(getClassNames());
  cout << "Collecting dataset for tracks";   cout.flush();
  for(size_t i=0; i<g_tm.tracks_.size(); ++i) {
    if(g_tm.tracks_.size() > 10 && (int)i%((int)floor(g_tm.tracks_.size() / 10.0)) == 0)
      cout << ".";   cout.flush();
    if(g_tm.tracks_[i]->label_.compare("unlabeled") == 0) //Don't want to save unlabeled data.
      continue;
    collectDatasetForTrack(*g_tm.tracks_[i], &objs);
  }
  
  // -- Put into MultiBoosterDataset and save.
  MultiBoosterDataset mbd(class_map.getIdToNameMapping(), getFeatureNames(desc));
  mbd.setObjs(objs);
  string filename("multibooster_dataset_trackvis.mbd");
  if(getenv("MBD_SAVENAME"))
    filename = string(getenv("MBD_SAVENAME"));
  mbd.save(filename);
  cout << endl << "Saved new dataset as " << filename << endl;
  
  // -- Clean up.
  for(size_t i=0; i<delete_me.size(); ++i)
    delete delete_me[i];
  for(size_t i=0; i<desc.size(); ++i)
    delete desc[i];
}

void incrementTrack(int i) {
  g_cloud_num = 0;
  g_track_num += i;
  if(g_track_num >= (int)g_tm.tracks_.size())
    g_track_num = 0;
  if(g_track_num < 0)
    g_track_num = g_tm.tracks_.size() - 1;
}

void incrementCloud(int i) {
  g_cloud_num += i;
  if(g_wrap) { 
    if(g_cloud_num >= (int)g_tm.tracks_[g_track_num]->clouds_.size())
      g_cloud_num = 0;
    if(g_cloud_num < 0)
      g_cloud_num = g_tm.tracks_[g_track_num]->clouds_.size() - 1;
  }
  else {
    if(g_cloud_num >= (int)g_tm.tracks_[g_track_num]->clouds_.size()) { 
      g_cloud_num = 0;
      incrementTrack(1);
    }
    if(g_cloud_num < 0) { 
      g_cloud_num = 0;
      incrementTrack(-1);
    }
  }

}

string getNextSavename() {
  int video_num = 0;
  char filename[100];
  while(true) {
    sprintf(filename, "track%04d.avi", video_num);
    struct stat file_info;
    int status = stat(filename, &file_info);
    if(status == 0) //File exists.
      video_num++;
    else
      break;
  }
  string str(filename);
  return str;
}

void getDescriptorsForCurrentCloud() {
  vector<SpectralAnalysis*> delete_me;
  vector<Descriptor3D*> desc = getFeatures(&delete_me, true);
  Object* obj = getDescriptorsForCloud(*g_cloud, desc);
  for(size_t i=0; i<delete_me.size(); ++i)
    delete delete_me[i];
  for(size_t i=0; i<desc.size(); ++i)
    delete desc[i]; 
  delete obj;
}

void keyboard(unsigned char key, int x, int y)
{
  switch(key) {
  case 'i':
    g_display_intensity = !g_display_intensity;
    break;
  case 'd':
    getDescriptorsForCurrentCloud();
    break;
  case 'C':
    if(g_multibooster) { 
      if(g_cloud_classifications.empty())
	classifyAll();
      else
	cout << "Already classified everything." << endl;
    }
    else
      cout << "No classifier!" << endl;
    break;
  case 'f':
    g_fast_forward = !g_fast_forward;
    break;
  case 'k':
    if(g_pause)
      incrementCloud(1);
    else
      incrementTrack(1);
    break;
  case 'w':
    g_wrap = !g_wrap;
    break;
  case 'j':
    if(g_pause)
      incrementCloud(-1);
    else
      incrementTrack(-1);
    break;
  case 'q':
    if(g_vo && getenv("VIDEO"))
      dgc_videoout_release(&g_vo);
    exit(0);
    break;
  case 'v':
    if(getenv("VIDEO")) { 
      g_record = !g_record;
      if(g_record) {
	cout << "Recording." << endl;
      }
      else {
	cout << "Done recording." << endl;
      }
    }
    else
      cout << "Use VIDEO= to record." << endl;
    break;  
  case ' ':
    g_pause = !g_pause;
    break;

  // -- Labeling commands.
  case 'c':
    g_tm.tracks_[g_track_num]->label_ = "car";
    break;
  case 'b':
    g_tm.tracks_[g_track_num]->label_ = "bicyclist";
    break;
  case 'p':
    g_tm.tracks_[g_track_num]->label_ = "pedestrian";
    break;
  case 'u':
    g_tm.tracks_[g_track_num]->label_ = "unlabeled";
    break;
  case 'g':
    g_tm.tracks_[g_track_num]->label_ = "background";
    break;
    
  case 'D':
    collectDatasetForTracks();
    break;
  case 's':
    g_tm.save(g_tm_filename);
    cout << "Saved track manager as " << g_tm_filename << endl;
    break;
  default:
    break;
  }

  g_cloud = g_tm.tracks_[g_track_num]->clouds_[g_cloud_num];
  gui3D_forceRedraw();
}


void mouse(__attribute__ ((unused)) int button, 
    int state, int x, int y)
{
}

void motion(int x, int y)
{
}

double computeXYDist(const sensor_msgs::PointCloud& cloud) {
  float max_dist2 = -FLT_MAX;
  for(size_t i=0; i<cloud.get_points_size(); ++i) {
    for(size_t j=0; j<cloud.get_points_size(); ++j) {
      float dist2 = pow(cloud.points[i].x - cloud.points[j].x, 2) + pow(cloud.points[i].y - cloud.points[j].y, 2);
      if(dist2 > max_dist2)
	max_dist2 = dist2;
    }
  }
  return sqrt(max_dist2);
}

double computeRadius(const sensor_msgs::PointCloud& cloud) {
  double x = 0;
  double y = 0;
  double z = 0;
  for(size_t i=0; i<cloud.get_points_size(); ++i) {
    x += cloud.points[i].x;
    y += cloud.points[i].y;
    z += cloud.points[i].z;
  }
  x /= (double)cloud.get_points_size();
  y /= (double)cloud.get_points_size();
  z /= (double)cloud.get_points_size();

  double max_dist = 0;
  for(size_t i=0; i<cloud.get_points_size(); ++i) {
    double dist = sqrt(pow(cloud.points[i].x - x, 2) + pow(cloud.points[i].y - y, 2) + pow(cloud.points[i].z - z, 2));
    if(dist > max_dist)
      max_dist = dist;
  }
  return max_dist;
}


float computeHeight(const sensor_msgs::PointCloud& cloud) {
  // -- Sort based on height.
  vector< pair<float, size_t> > height_idx(cloud.get_points_size());
  for(size_t i=0; i<height_idx.size(); ++i) {
    height_idx[i].first = cloud.points[i].z;
    height_idx[i].second = i;
  }
  sort(height_idx.begin(), height_idx.end()); //Ascending order.

  float height = height_idx[(size_t)(0.95 * (float)height_idx.size())].first - height_idx.front().first;
  return height;

  // -- If there's a jump in z near the top, ignore points above the jump.
  //  float Z_SEG_THRESH = 0.03;
  

  //   for(size_t i = 0.8 * height_idx.size(); i<height_idx.size(); ++i) {
//     assert(height_idx[i].first - height_idx[i-1].first >= 0);
//     if(height_idx[i].first - height_idx[i-1].first > Z_SEG_THRESH)
//       return height_idx[i].first - height_idx.front().first;
//   }
//   return height_idx.back().first - height_idx.front().first;
}

void draw_info_box(void)
{
  char str[200];
  set_display_mode_2D(gui3D.window_width, gui3D.window_height);
  glColor3f(1, 1, 1);
  
  // -- If we're on the last frame, let the user know.
  if(g_cloud_num == (int)g_tm.tracks_[g_track_num]->clouds_.size() - 1) {
    sprintf(str, "LAST FRAME");
    renderBitmapString((gui3D.window_width - bitmapStringWidth(GLUT_BITMAP_HELVETICA_18, str)) / 2.0,
		       gui3D.window_height / 7.0, GLUT_BITMAP_TIMES_ROMAN_24, str);
  }

  
  sprintf(str, "%s", g_tm.tracks_[g_track_num]->label_.c_str());
  renderBitmapString(gui3D.window_width - bitmapStringWidth(GLUT_BITMAP_HELVETICA_18, str) - 10, 20, GLUT_BITMAP_HELVETICA_18, str);


  // -- Z height debugging.
//   float max_z = -FLT_MAX;
//   float min_z = FLT_MAX;
//   for(size_t i=0; i<g_cloud->get_points_size(); ++i) {
//     if(g_cloud->points[i].z > max_z)
//       max_z = g_cloud->points[i].z;
//     if(g_cloud->points[i].z < min_z)
//       min_z = g_cloud->points[i].z;
//   }

//   sprintf(str, "max xy dist: %f", computeXYDist(*g_cloud));
//   renderBitmapString(20, 140, GLUT_BITMAP_HELVETICA_18, str);
  
  sprintf(str, "z height (denoised): %f", computeHeight(*g_cloud));
  renderBitmapString(20, 120, GLUT_BITMAP_HELVETICA_18, str);

  sprintf(str, "Radius: %f", computeRadius(*g_cloud));
  renderBitmapString(20, 100, GLUT_BITMAP_HELVETICA_18, str);
  
  if(g_pause) { 
    sprintf(str, "Paused");
    renderBitmapString(20, 160, GLUT_BITMAP_HELVETICA_18, str);
  }

  if(g_fast_forward) {
    sprintf(str, "Fast Forward");
    renderBitmapString(20, 180, GLUT_BITMAP_HELVETICA_18, str);
  }
  
  sprintf(str, "%d points", g_cloud->get_points_size());
  renderBitmapString(20, 80, GLUT_BITMAP_HELVETICA_18, str);
 
  sprintf(str, "Track %d of %d", g_track_num+1, g_tm.tracks_.size());
  renderBitmapString(20, 60, GLUT_BITMAP_HELVETICA_18, str);

  sprintf(str, "Cloud %d of %d", g_cloud_num+1, g_tm.tracks_[g_track_num]->clouds_.size());
  renderBitmapString(20, 40, GLUT_BITMAP_HELVETICA_18, str);

  if(g_wrap)
    sprintf(str, "Wrap mode: on");
  else
    sprintf(str, "Wrap mode: off ");
  renderBitmapString(20, 20, GLUT_BITMAP_HELVETICA_18, str);


  // -- Classifier related messages.
  if(!g_multibooster) {
    sprintf(str, "Classifier: none");
    renderBitmapString(20, gui3D.window_height - 20, GLUT_BITMAP_HELVETICA_18, str);
  }
  else {
    size_t start = g_mb_filename.find_last_of("/") + 1;
    sprintf(str, "Classifier: %s", g_mb_filename.substr(start).c_str());
    renderBitmapString(20, gui3D.window_height - 20, GLUT_BITMAP_HELVETICA_18, str);

    vector<SpectralAnalysis*> delete_me;
    vector<Descriptor3D*> desc = getFeatures(&delete_me);
    
    assert(g_cloud == g_tm.tracks_[g_track_num]->clouds_[g_cloud_num]);
    Object* obj = getDescriptorsForCloud(*g_cloud, desc);

    NameMapping class_map(getClassNames());
    int label = -2;
    string label_str = g_tm.tracks_[g_track_num]->label_;
    if(label_str.compare("background") == 0)
      label = -1;
    else if(label_str.compare("unlabeled") == 0)
      label = -2;
    else
      label = class_map.toId(label_str);

    //    cout << obj->status(class_map, NameMapping(getFeatureNames(desc))) << endl;
    //     cout << g_multibooster->class_map_.serialize() << endl;
    //     cout << g_multibooster->feature_map_.serialize() << endl;

    VectorXf result = g_multibooster->classify(*obj);
    for(size_t i=0; i<g_multibooster->class_map_.size(); ++i) {
      glColor3f(1, 1, 1);
      sprintf(str, "%s: ", g_multibooster->class_map_.toName(i).c_str());
      renderBitmapString(20, gui3D.window_height - 40 - 20*i, GLUT_BITMAP_HELVETICA_18, str);

      if(label == -2)
	glColor3f(1, 1, 1);
      else if((label == (int)i && result(i) > 0) || (label != (int)i && result(i) < 0))
	glColor3f(0, 1, 0);
      else
	glColor3f(1, 0, 0);
      
      sprintf(str, "%.2f", result(i));
      renderBitmapString(150, gui3D.window_height - 40 - 20*i, GLUT_BITMAP_HELVETICA_18, str);
      
      if(!g_cloud_classifications.empty()) {
	VectorXf sum = g_cloud_classifications[g_track_num].rowwise().sum();

	if(label == -2)
	  glColor3f(1, 1, 1);
	else if((label == (int)i && sum(i) > 0) || (label != (int)i && sum(i) < 0))
	  glColor3f(0, 1, 0);
	else
	  glColor3f(1, 0, 0);
      
	sprintf(str, "%.2f", sum(i));
	//sprintf(str, "%.2f", g_cloud_classifications[g_track_num](i, g_cloud_num));
	renderBitmapString(250, gui3D.window_height - 40 - 20*i, GLUT_BITMAP_HELVETICA_18, str);
      }
    }

    // -- Clean up.
    glColor3f(1, 1, 1);
    delete obj;
    for(size_t i=0; i<delete_me.size(); ++i)
      delete delete_me[i];
    for(size_t i=0; i<desc.size(); ++i)
      delete desc[i];
  }
}


void getCloudStats(const PointCloud& cloud, double* x, double* y, double* z, double* min_z = NULL, double* max_z = NULL) {
  if(min_z)
    *min_z = FLT_MAX;
  if(max_z)
    *max_z = FLT_MIN;
  for(size_t i=0; i<cloud.get_points_size(); ++i) {
    *x += cloud.points[i].x;
    *y += cloud.points[i].y;
    *z += cloud.points[i].z;
    if(min_z && cloud.points[i].z < *min_z)
      *min_z = cloud.points[i].z;
    if(max_z && cloud.points[i].z > *max_z)
      *max_z = cloud.points[i].z;
  }
  if(cloud.get_points_size() > 0) {
    *x /= cloud.get_points_size();
    *y /= cloud.get_points_size();
    *z /= cloud.get_points_size();
  }
  else {
    *x = 0;
    *y = 0;
    *z = 0;
  } 
}

void display(void)
{
  // -- Clear to black.
  glClearColor(0, 0, 0, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // -- Turn on smooth lines.
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);


  // -- If this track doesn't have any pointclouds associated with it, don't show anything.  
  if(!g_cloud)
    return;
  
  // -- Get the centroid and min z.
  shared_ptr<PointCloud> cloud_ptr;
  cloud_ptr = g_tm.tracks_[g_track_num]->clouds_[0];

  double x_init = 0;
  double y_init = 0;
  double z_init = 0;
  double min_z = 0;
  double max_z = 0;
  getCloudStats(*cloud_ptr.get(), &x_init, &y_init, &z_init, &min_z, &max_z);
  
  drawGrid(0,0);
  
  // -- Draw the point cloud.
  for(size_t i=0; i<g_cloud->get_points_size(); ++i) {
    assert(g_cloud->get_channels_size() > 0);
    assert(g_cloud->channels[0].get_values_size() == g_cloud->get_points_size());
      
    if(!g_display_intensity) { 
      double u = (g_cloud->points[i].z - min_z) / 3.0;
      if(u > 1)
	u = 1;
      else if(u < 0)
	u = 0;
      glColor3f(1 - u, u, 0);
    }
    else {
      double u = (double)g_cloud->channels[0].values[i] / 255.;
      glColor3f(u, 0.5+0.5*u, u);
    }

    glPushMatrix();
    glTranslatef(g_cloud->points[i].x - x_init,
		 g_cloud->points[i].y - y_init,
		 g_cloud->points[i].z - z_init);
    glutSolidCube(0.05);
    glPopMatrix();
  }

  draw_info_box();

  
//   double this_x_centroid = 0;
//   double this_y_centroid = 0;
//   double this_z_centroid = 0;
//   getCloudStats(*g_cloud.get(), &this_x_centroid, &this_y_centroid, &this_z_centroid, &min_z, &max_z);
//   cout << "x y z: " << this_x_centroid << "       " << this_y_centroid << "       " << this_z_centroid << endl;
}

void timer(int)
{
  // -- Increment the cloud we are looking at.
  if(!g_pause) {
    if(!g_fast_forward) 
      incrementCloud(1);
    else
      incrementCloud(5);
  }


  // -- Set g_cloud to point to the cloud, or NULL if there is none.
  if(g_tm.tracks_[g_track_num]->clouds_.empty()) {
    g_cloud.reset();
    assert(g_cloud.get() == NULL);
    cout << "Warning: track " << g_track_num << " has no point clouds." << endl;
  }
  else
    g_cloud = g_tm.tracks_[g_track_num]->clouds_[g_cloud_num];

  // -- Set the camera position.
  shared_ptr<PointCloud> cloud_ptr;
  cloud_ptr = g_tm.tracks_[g_track_num]->clouds_[0];
  
  double x_curr = 0;
  double y_curr = 0;
  double z_curr = 0;
  getCloudStats(*g_cloud.get(), &x_curr, &y_curr, &z_curr);
  double x_init = 0;
  double y_init = 0;
  double z_init = 0;
  getCloudStats(*cloud_ptr.get(), &x_init, &y_init, &z_init);

  gui3D.camera_pose.x_offset = x_curr - x_init;
  gui3D.camera_pose.y_offset = y_curr - y_init;
  gui3D.camera_pose.z_offset = z_curr - z_init;

  
  // -- Redraw.
  gui3D_forceRedraw();
  gui3D_add_timerFunc(100, timer, 0);

  if(g_record && getenv("VIDEO")) {
    dgc_videoout_add_opengl_frame(g_vo);
  }
}

string usageString() {
  ostringstream oss;
  oss << "usage: " << endl;
  oss << " track_visualizer TRACKS" << endl;
  oss << "   where TRACKS is a track file saved by the TrackManager class, usually with .tm extension." << endl;
  oss << " CLASSIFY= track_visualizer CLASSIFIER TRACKS [TRACKS...]" << endl;
  return oss.str();
}

void useClassifier(string filename) {
  g_mb_filename = filename;
  g_multibooster = shared_ptr<MultiBooster>(new MultiBooster(g_mb_filename));
  if(getenv("WC_LIMIT"))
    g_multibooster->wc_limiter_ = atoi(getenv("WC_LIMIT"));
  
  // -- Change the name mappings of the classifier to match the objects that will be produced.
  vector<SpectralAnalysis*> delete_me;
  vector<Descriptor3D*> desc = getFeatures(&delete_me);
  g_multibooster->applyNewMappings(g_class_map, getFeatureNames(desc));
  for(size_t i=0; i<delete_me.size(); ++i)
    delete delete_me[i];
  for(size_t i=0; i<desc.size(); ++i)
    delete desc[i];
  
  cout << g_multibooster->status(false) << endl;
}

void useMetaClassifier(string filename) {
  g_meta_multibooster = shared_ptr<MultiBooster>(new MultiBooster(filename));
  cout << g_meta_multibooster->status(false) << endl;
}
   
void useTrackManager(string filename) {
  cout << "Loading track manager " << filename << endl;
  g_tm_filename = string(filename);
  g_tm = TrackManager(filename);

  cout << "Loaded " << g_tm.tracks_.size() << " tracks." << endl;
      
  if(g_tm.tracks_.size() == 0) {
    cerr << "0 tracks.  Aborting." << endl;
    exit(1);
  }

  // -- Label filtering.
  int label_filter = (int)(bool)getenv("CAR_ONLY") + (int)(bool)getenv("PED_ONLY") + (int)(bool)getenv("BIKE_ONLY") + (int)(bool)getenv("UNLABELED_ONLY") + (int)(bool)getenv("BACKGROUND_ONLY");
  assert(label_filter < 2);
  if(label_filter == 1) { 
    vector< shared_ptr<Track> > target;
    target.reserve(g_tm.tracks_.size());
    for(size_t i=0; i<g_tm.tracks_.size(); ++i) {
      Track& track = *g_tm.tracks_[i];
      if(getenv("CAR_ONLY") && track.label_.compare("car") == 0 ||
	 getenv("PED_ONLY") && track.label_.compare("pedestrian") == 0 ||
	 getenv("BIKE_ONLY") && track.label_.compare("bicyclist") == 0 ||
	 getenv("BACKGROUND_ONLY") && track.label_.compare("background") == 0 ||
	 getenv("UNLABELED_ONLY") && track.label_.compare("unlabeled") == 0)
	target.push_back(g_tm.tracks_[i]);
    }
    if(target.empty()) {
      cout << "No objects of the specified type." << endl;
      exit(1);
    }
      
    g_tm = TrackManager(target);
  }
  
  // -- Occlusion filtering.
  if(getenv("FILTER_OUT_UNREASONABLE") || getenv("FILTER_OUT_REASONABLE")) {
    vector< shared_ptr<Track> > accept;
    vector< shared_ptr<Track> > reject;
    for(size_t i=0; i<g_tm.tracks_.size(); ++i) {
      Track& track = *g_tm.tracks_[i];
//       double mean_numpts = 0;
//       double mean_radius = 0; //Max distance from the centroid.
      
//       for(size_t j=0; j<track.clouds_.size(); ++j) {
// 	mean_numpts += track.clouds_[j]->get_points_size();
// 	mean_radius += computeRadius(*track.clouds_[j]);
//       }
//       mean_numpts /= (double)track.clouds_.size();
//       mean_radius /= (double)track.clouds_.size();

      // -- Class objects must have a reasonable size and numpts.
//       bool reasonable = (track.label_.compare("car") == 0 && mean_numpts > RSS_MIN_NUMPTS_CAR && mean_radius > RSS_MIN_RADIUS_CAR) ||
// 	(track.label_.compare("pedestrian") == 0 && mean_numpts > RSS_MIN_NUMPTS_PED && mean_radius > RSS_MIN_RADIUS_PED) ||
// 	(track.label_.compare("bicyclist") == 0 && mean_numpts > RSS_MIN_NUMPTS_BIKE && mean_radius > RSS_MIN_RADIUS_BIKE) ||
// 	track.label_.compare("background") == 0 || track.label_.compare("unlabeled") == 0;

      float mean_zheight = 0;
      float mean_xydist = 0;
      for(size_t j=0; j<track.clouds_.size(); ++j) { 
	mean_zheight += computeHeight(*track.clouds_[j]);
	mean_xydist += computeXYDist(*track.clouds_[j]);
      }
      mean_zheight /= (float)track.clouds_.size();
      mean_xydist /= (float)track.clouds_.size();
      
      bool reasonable = (track.label_.compare("car") == 0 && mean_zheight > RSS_MIN_HEIGHT_CAR && mean_xydist > RSS_MIN_XY_CAR) ||
	(track.label_.compare("pedestrian") == 0 && mean_zheight > RSS_MIN_HEIGHT_PEDESTRIAN && mean_xydist > RSS_MIN_XY_PEDESTRIAN) ||
	(track.label_.compare("bicyclist") == 0 && mean_zheight > RSS_MIN_HEIGHT_BICYCLIST && mean_xydist > RSS_MIN_XY_BICYCLIST) ||
	track.label_.compare("background") == 0 || track.label_.compare("unlabeled") == 0;
      
      if(getenv("FILTER_OUT_REASONABLE") && !reasonable || getenv("FILTER_OUT_UNREASONABLE") && reasonable)
	accept.push_back(g_tm.tracks_[i]);
      else
	reject.push_back(g_tm.tracks_[i]);
    }

    if(accept.empty()) {
      cout << "Filtered out all tracks.  Exiting... " << endl;
      exit(1);
    }
    
    g_tm = TrackManager(accept);
    cout << "Filtered down to " << g_tm.tracks_.size() << " tracks." << endl;

    // -- Save the rejected tracks for later analysis.
    if(getenv("FILTER_OUT_UNREASONABLE")) { 
      string basename = g_tm_filename.substr(0, g_tm_filename.size() - 3);
      string stem = basename.substr(basename.find_last_of("/")+1, basename.size());
      string savename = stem + "_occlusion_filtered.tm";
      TrackManager reject_tm(reject);
      reject_tm.save(savename);
    }
  }
}

int main(int argc, char **argv)
{
  if(argc < 2) {
    cout << usageString();
    return 1;
  }
  
  if(getenv("CLASSIFY")) {
    if(argc < 3) { 
      cout << usageString();
      return 1;
    }
    else  { 
      useClassifier(argv[1]);
      int tm_start_idx;
      if(string(argv[2]).find(".mb") != string::npos) { 
	useMetaClassifier(argv[2]);
	tm_start_idx = 3;
	assert(g_meta_multibooster);
      }
      else {
	assert(!g_meta_multibooster);
	tm_start_idx = 2;
      }
      
      PerfStats track_stats(g_class_map);
      PerfStats cloud_stats(g_class_map);
      PerfStats progressive_stats(g_class_map);
      PerfStats meta_stats(g_class_map);
      
      // -- Classify and collect statistics for all .tm files provided.
      for(int i=tm_start_idx; i<argc; ++i) {
	useTrackManager(argv[i]);
	classifyAll();
	incrementStats(&track_stats, &cloud_stats, &progressive_stats, &meta_stats);
      }

      cout << "******************************" << endl;
      cout << "********** Track Stats:" << endl;
      cout << "******************************" << endl << endl;
      cout << track_stats.statString() << endl;
      cout << "******************************" << endl;
      cout << "********** Cloud Stats:" << endl;
      cout << "******************************" << endl << endl;
      cout << cloud_stats.statString() << endl;
      cout << "******************************" << endl;
      cout << "********** Progressive Stats:" << endl;
      cout << "******************************" << endl << endl;
      cout << progressive_stats.statString() << endl;
      cout << "******************************" << endl;
      cout << "********** Meta Stats:" << endl;
      cout << "******************************" << endl << endl;
      cout << meta_stats.statString() << endl;
      
      return 0;
    }
  }

  string arg(argv[1]);
  if(arg.substr(arg.size() - 3).compare(".mb") == 0) {
    useClassifier(arg);
    arg = string(argv[2]);
  }
  useTrackManager(arg);

  g_tm.sortTracks();
  g_cloud = g_tm.tracks_[0]->clouds_[0];

  double num_clouds = 0;
  for(size_t i=0; i<g_tm.tracks_.size(); ++i) {
    num_clouds += g_tm.tracks_[i]->clouds_.size();
  }
  cout << "Total number of clouds: " << num_clouds << endl;

  
  
  /* setup GUI */
  gui3D_initialize(argc, argv, 10, 10, 720, 480, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_displayFunc(display);
  gui3D_set_motionFunc(motion);
  gui3D_set_mouseFunc(mouse);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_add_timerFunc(100, timer, 0);
  gui3D_setInitialCameraPos(0, -1, 15, 0, 0, 0);
  
  if(getenv("VIDEO")) { 
    string video_name = getNextSavename();
    g_vo  = dgc_videoout_init(video_name.c_str(), 4000000, 720, 480,
			      10, CODEC_ID_MPEG4, 0, PIX_FMT_YUV420P);
  }
  if(getenv("COLLECT_DATASET")) {
    collectDatasetForTracks();
    return 0;
  }

  if(getenv("COLLECT_META_DATASET")) {
    //useClassifier(argv[1]);
    //useTrackManager(argv[2]);
    classifyAll();
    MultiBoosterDataset* mbd = collectMetaDataset();

    for(int i=3; i<argc; ++i) {
      useTrackManager(argv[i]);
      classifyAll();
      MultiBoosterDataset* mbd_this_tm = collectMetaDataset();
      mbd->join(*mbd_this_tm);
      delete mbd_this_tm;
    }

    mbd->save("meta_multibooster_dataset.mbd");
    delete mbd;
    
    return 0;
  }
  
  gui3D_mainloop();
  return 0;
}

