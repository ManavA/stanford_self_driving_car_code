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
#include "multibooster_support2.h"
#include <cluster_descriptors/cluster_descriptors.h>

using namespace dgc;
using namespace std;
//using namespace std::tr1;
using boost::shared_ptr;
using namespace track_manager;
using namespace sensor_msgs;
using namespace vlr;
using namespace Eigen;
using namespace pipeline;

TrackManager g_tm;
shared_ptr<PointCloud> g_cloud;
shared_ptr<Track> g_track;
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
NameMapping g_class_map = NameMapping(getClassNames());
shared_ptr<MultiBooster> g_multibooster;
shared_ptr<DescriptorPipeline> g_dp;
string g_mb_filename;
bool g_display_intensity = true;


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

Object* getDescriptorsForCloud2(const PointCloud& roscloud, bool debug = false, bool timing = false) {
  shared_ptr<MatrixXf> cloud((MatrixXf*)NULL);
  shared_ptr<VectorXf> intensity((VectorXf*)NULL);
  rosToEigen(roscloud, &cloud, &intensity);
  if(debug)
    for(size_t i = 0; i < g_dp->pipeline_.nodes_.size(); ++i) 
      g_dp->pipeline_.nodes_[i]->debug_ = true;

  timeval start, end;
  gettimeofday(&start, NULL);
  Object* obj = g_dp->computeDescriptors(cloud, intensity);
  gettimeofday(&end, NULL);
  if(timing)
    cout << (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000. << " ms to compute descriptors." << endl;

  if(debug)
    for(size_t i = 0; i < g_dp->pipeline_.nodes_.size(); ++i) 
      g_dp->pipeline_.nodes_[i]->debug_ = false;
  
  return obj;
}


void collectDatasetForTrack(const Track& track, vector<Object*>* objects) {
  objects->reserve(track.clouds_.size()); //Doesn't shrink the capacity ever.

  // -- Compute all descriptors.
  NameMapping class_map(getClassNames());
  for(size_t i=0; i<track.clouds_.size(); ++i) {
    Object* obj = getDescriptorsForCloud2(*track.clouds_[i]);
    if(track.label_.compare("unlabeled") == 0)
      obj->label_ = -2;
    else if(track.label_.compare("background") == 0)
      obj->label_ = -1;
    else
      obj->label_ = class_map.toId(track.label_);

    objects->push_back(obj);
  }
}

void classifyAll() {
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

void keyboard(unsigned char key, int x, int y)
{
  switch(key) {
  case 'i':
    g_display_intensity = !g_display_intensity;
    break;
  case 'd':
    getDescriptorsForCloud2(*g_cloud, true);
    break;
  case 't':
    getDescriptorsForCloud2(*g_cloud, false, true);
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
  
  if(g_pause) { 
    sprintf(str, "Paused");
    renderBitmapString(20, 160, GLUT_BITMAP_HELVETICA_18, str);
  }

  if(g_fast_forward) {
    sprintf(str, "Fast Forward");
    renderBitmapString(20, 180, GLUT_BITMAP_HELVETICA_18, str);
  }

  // -- Show distance to velodyne.
  float dist = -1;
  if(g_track->serialization_version_ > 0) {
    double x = 0, y = 0, z = 0;
    getCloudStats(*g_cloud, &x, &y, &z);
    vector<float>& center = g_track->velodyne_centers_[g_cloud_num];
    dist = sqrt(pow(x - center[0], 2) + pow(y - center[1], 2) + pow(z - center[2], 2));
  }
  sprintf(str, "Distance: %f", dist);
  renderBitmapString(20, 100, GLUT_BITMAP_HELVETICA_18, str);
  
  sprintf(str, "%d points", g_cloud->get_points_size());
  renderBitmapString(20, 80, GLUT_BITMAP_HELVETICA_18, str);
 
  sprintf(str, "Track %d of %d", g_track_num+1, (int)g_tm.tracks_.size());
  renderBitmapString(20, 60, GLUT_BITMAP_HELVETICA_18, str);

  sprintf(str, "Cloud %d of %d", g_cloud_num+1, (int)g_tm.tracks_[g_track_num]->clouds_.size());
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

    assert(g_cloud == g_tm.tracks_[g_track_num]->clouds_[g_cloud_num]);
    Object* obj = getDescriptorsForCloud2(*g_cloud);

    NameMapping class_map(getClassNames());
    int label = -2;
    string label_str = g_tm.tracks_[g_track_num]->label_;
    if(label_str.compare("background") == 0)
      label = -1;
    else if(label_str.compare("unlabeled") == 0)
      label = -2;
    else
      label = class_map.toId(label_str);

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
	renderBitmapString(250, gui3D.window_height - 40 - 20*i, GLUT_BITMAP_HELVETICA_18, str);
      }
    }

    // -- Clean up.
    glColor3f(1, 1, 1);
    delete obj;
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

  g_track = g_tm.tracks_[g_track_num];

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
  g_multibooster->applyNewMappings(g_class_map, getDescriptorNames());
  
  cout << g_multibooster->status(false) << endl;
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
      if((getenv("CAR_ONLY") && track.label_.compare("car") == 0) ||
	 (getenv("PED_ONLY") && track.label_.compare("pedestrian") == 0) ||
	 (getenv("BIKE_ONLY") && track.label_.compare("bicyclist") == 0) ||
	 (getenv("BACKGROUND_ONLY") && track.label_.compare("background") == 0) ||
	 (getenv("UNLABELED_ONLY") && track.label_.compare("unlabeled") == 0))
	target.push_back(g_tm.tracks_[i]);
    }
    if(target.empty()) {
      cout << "No objects of the specified type." << endl;
      exit(1);
    }
      
    g_tm = TrackManager(target);
  }
}


int main(int argc, char **argv)
{
  if(argc < 2) {
    cout << usageString();
    return 1;
  }
  
  string arg(argv[1]);
  if(arg.substr(arg.size() - 3).compare(".mb") == 0) {
    useClassifier(arg);
    arg = string(argv[2]);
  }
  useTrackManager(arg);


  int num_threads = 1;
  g_dp = shared_ptr<DescriptorPipeline>(new DescriptorPipeline(num_threads));
  
  g_tm.sortTracks();
  g_cloud = g_tm.tracks_[0]->clouds_[0];
  g_track = g_tm.tracks_[0];

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
  
  gui3D_mainloop();
  return 0;
}


