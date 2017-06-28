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


#include "track_manager.h"

using namespace sensor_msgs;
using namespace std;
using boost::shared_ptr;

/************************************************************/
/******************** TrackManager ********************/
/************************************************************/

track_manager::TrackManager::TrackManager() :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(vector< shared_ptr<Track> >())
{
}

track_manager::TrackManager::TrackManager(std::istream& istrm)  :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(vector< shared_ptr<Track> >())
{
  bool success = deserialize(istrm);
  if(!success)
    throw 1;
}

track_manager::TrackManager::TrackManager(const std::vector< shared_ptr<Track> >& tracks)  :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(tracks)
{
}

track_manager::TrackManager::TrackManager(const string& filename) :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(vector< shared_ptr<Track> >())
{
  ifstream ifs(filename.c_str(), ios::in);
  bool success = deserialize(ifs);
  if(!success)
    throw 1;
}

bool track_manager::TrackManager::save(const string& filename) {
  ofstream ofs(filename.c_str(), ios::out);
  serialize(ofs);
  ofs.close();
  return true;
}

int track_manager::TrackManager::getTotalNumClouds() const {
  int num = 0;
  for(size_t i = 0; i < tracks_.size(); ++i)
    num += tracks_[i]->clouds_.size();
  return num;
}

void track_manager::TrackManager::serialize(ostream& out) {
  out << "TrackManager" << endl;
  out << "serialization_version_" << endl;
  out << serialization_version_ << endl;
//   out << "num_tracks" << endl;
//   out << tracks_.size() << endl;
  for(size_t i=0; i<tracks_.size(); ++i) {
    tracks_[i]->serialize(out);
  }
}

bool track_manager::TrackManager::deserialize(istream& istrm) {
  tracks_.clear();
  string line;

  getline(istrm, line);
  if(line.compare("TrackManager") != 0) {
    return false;
  }

  getline(istrm, line);
  if(line.compare("serialization_version_") != 0)
    return false;

  istrm >> serialization_version_;
  if(serialization_version_ != TRACKMANAGER_SERIALIZATION_VERSION) {
    cerr << "Expected TrackManager serialization_version_ == " << TRACKMANAGER_SERIALIZATION_VERSION;
    cerr << ".  This file is vs " << serialization_version_ << ", aborting." << endl;
    return false;
  }
  getline(istrm, line);
 
//   getline(istrm, line);
//   if(line.compare("num_tracks") != 0)
//     return false;

//   size_t num_tracks = 0;
//   istrm >> num_tracks;
//   getline(istrm, line);

//   tracks_.reserve(num_tracks);
//   for(size_t i=0; i<num_tracks; ++i) {
//     tracks_.push_back(shared_ptr<Track>(new Track(istrm)));
//   }
  
  while(true) {
    shared_ptr<Track> tr(new Track());
    if(tr->deserialize(istrm))
      tracks_.push_back(tr);
    else
      break;
  }
  
  return true;
}

bool track_manager::TrackManager::operator!=(const TrackManager& tm) {
  return !operator==(tm);
}

bool track_manager::TrackManager::operator==(const TrackManager& tm) {
  if(serialization_version_ != tm.serialization_version_)
    return false;
  if(tracks_.size() != tm.tracks_.size())
    return false;
  for(size_t i=0; i<tracks_.size(); ++i) {
    if(*tracks_[i] != *tm.tracks_[i])
      return false;
  }
  return true;
}

double track_manager::getTrackLength(const track_manager::Track& tr) {
  return (double) tr.clouds_.size();
}

void track_manager::TrackManager::sortTracks() {
  sortTracks(&getTrackLength);
}

void track_manager::TrackManager::sortTracks(double (*rateTrack)(const Track&)) {
  vector< pair<double, shared_ptr<Track> > > length_idx(tracks_.size());
  for(size_t i=0; i<tracks_.size(); ++i) {
    length_idx[i].first = rateTrack(*tracks_[i]);
    length_idx[i].second = tracks_[i];
  }
  greater< pair<double, shared_ptr<Track> > > emacs = greater< pair<double, shared_ptr<Track> > >();
  sort(length_idx.begin(), length_idx.end(), emacs); //Descending.

  for(size_t i=0; i<tracks_.size(); ++i) {
    tracks_[i] = length_idx[i].second;
  }
}

void track_manager::TrackManager::sortTracks(const vector<double>& track_ratings) {
  assert(track_ratings.size() == tracks_.size());
  vector< pair<double, shared_ptr<Track> > > length_idx(tracks_.size());
  for(size_t i=0; i<tracks_.size(); ++i) {
    length_idx[i].first = track_ratings[i];
    length_idx[i].second = tracks_[i];
  }
  greater< pair<double, shared_ptr<Track> > > emacs = greater< pair<double, shared_ptr<Track> > >();
  sort(length_idx.begin(), length_idx.end(), emacs); //Descending.

  for(size_t i=0; i<tracks_.size(); ++i) {
    tracks_[i] = length_idx[i].second;
  }
} 

/************************************************************/
/******************** Track ********************/
/************************************************************/

void track_manager::Track::serialize(ostream& out) const {
  out << "Track" << endl;
  out << "serialization_version_" << endl;
  out << serialization_version_ << endl;
  out << "label_" << endl;
  out << label_ << endl;
  out << "num_clouds" << endl;
  out << clouds_.size() << endl;
  assert(clouds_.size() == timestamps_.size());
  assert((size_t)velodyne_centers_.size() == clouds_.size());
  for(size_t i=0; i<clouds_.size(); ++i) {
    out << "timestamp" << endl;
    out << timestamps_[i] << endl;
    assert(velodyne_centers_[i].size() == 3);
    out << "velo_center" << endl;
    out << velodyne_centers_[i][0] << endl;
    out << velodyne_centers_[i][1] << endl;
    out << velodyne_centers_[i][2] << endl;
    serializePointCloudROS(*clouds_[i], out);
  }
}


bool track_manager::Track::deserialize(istream& istrm) {
  if(istrm.eof())
    return false;

  long begin = istrm.tellg();
  clouds_.clear();
  string line;

  getline(istrm, line);
  if(line.compare("Track") != 0) {
    istrm.seekg(begin);
    return false;
  }

  getline(istrm, line);
  if(line.compare("serialization_version_") != 0)
    return false;
  istrm >> serialization_version_;
  if(serialization_version_ != TRACK_SERIALIZATION_VERSION
     && serialization_version_ != 0) {
    cerr << "Track serialization version is wrong, aborting." << endl;
    return false;
  }
  getline(istrm, line);

  getline(istrm, line);
  if(line.compare("label_") != 0)
    return false;
  getline(istrm, label_);
  
  getline(istrm, line);
  if(line.compare("num_clouds") != 0)
    return false;
  size_t num_clouds = 0;
  istrm >> num_clouds;
  getline(istrm, line);

  clouds_.resize(num_clouds);
  timestamps_.resize(num_clouds);
  velodyne_centers_.resize(num_clouds, vector<float>(3));
  for(size_t i=0; i<num_clouds; ++i) {
    assert(!clouds_[i]);

    if(serialization_version_ == 0) {
      timestamps_[i] = -1;
      velodyne_centers_[i][0] = 0;
      velodyne_centers_[i][1] = 0;
      velodyne_centers_[i][2] = 0;
    }
    else { 
      getline(istrm, line);
      if(line.compare("timestamp") != 0)
	return false;
      double timestamp = 0;
      istrm >> timestamp;
      getline(istrm, line);
      timestamps_[i] = timestamp;

      getline(istrm, line);
      if(line.compare("velo_center") != 0)
	return false;
      double x = 0;
      double y = 0;
      double z = 0;
      istrm >> x;
      getline(istrm, line);
      istrm >> y;
      getline(istrm, line);
      istrm >> z;
      getline(istrm, line);
      velodyne_centers_[i][0] = x;
      velodyne_centers_[i][1] = y;
      velodyne_centers_[i][2] = z;
    }
    
    clouds_[i] = shared_ptr<PointCloud>(new PointCloud());
    clouds_[i]->header.stamp = ros::Time(1); //Avoid a warning about timestamps from ROS.  We aren't using them anyway.
    bool success = deserializePointCloudROS(istrm, clouds_[i].get());
    if(!success)
      return false;
  }
  
  return true;
}


track_manager::Track::Track(const string& label,
			    const std::vector< boost::shared_ptr<sensor_msgs::PointCloud> >& clouds,
			    const std::vector<double>& timestamps,
			    const std::vector< std::vector<float> >& velodyne_centers) :
  serialization_version_(TRACK_SERIALIZATION_VERSION),
  label_(label),
  clouds_(clouds),
  timestamps_(timestamps),
  velodyne_centers_(velodyne_centers)
{
  assert(timestamps_.size() == clouds_.size());
  assert(clouds_.size() == velodyne_centers_.size());
}
  
track_manager::Track::Track(const vector< shared_ptr<PointCloud> >& clouds) :
  serialization_version_(TRACK_SERIALIZATION_VERSION),
  label_("unlabeled"),
  clouds_(clouds),
  timestamps_(vector<double>(clouds.size(), -1)),
  velodyne_centers_(vector< vector<float> >(clouds.size()))
{
  // -- Initialize velodyne_centers_ to be all zeros.
  for(size_t i = 0; i < velodyne_centers_.size(); ++i) { 
    velodyne_centers_[i] = vector<float>(3);
  }

  assert(timestamps_.size() == clouds_.size());
  assert(clouds_.size() == velodyne_centers_.size());
}
		    
track_manager::Track::Track() :
  serialization_version_(TRACK_SERIALIZATION_VERSION),
  label_("unlabeled")
{
}

		    

track_manager::Track::Track(istream& istrm) :
  serialization_version_(TRACK_SERIALIZATION_VERSION),
  label_("unlabeled"),
  clouds_(vector< shared_ptr<PointCloud> >())
{
  long begin = istrm.tellg();
  istrm.seekg(begin);

  bool success = deserialize(istrm);
  if(!success)
    throw 1;
}

void track_manager::Track::reserve(size_t num) {
  clouds_.reserve(num);
  timestamps_.reserve(num);
  velodyne_centers_.reserve(num);
}

void track_manager::Track::insertCloud(boost::shared_ptr<sensor_msgs::PointCloud> cloud,
				       double timestamp,
				       const vector<float>& velodyne_center)
{
  clouds_.push_back(cloud);
  timestamps_.push_back(timestamp);
  velodyne_centers_.push_back(velodyne_center);
}

bool track_manager::Track::operator!=(const Track& tr) {
  return !operator==(tr);
}

bool track_manager::Track::operator==(const Track& tr) {
  if(tr.label_.compare(label_) != 0)
    return false;
  if(tr.clouds_.size() != clouds_.size())
    return false;
  for(size_t i=0; i<clouds_.size(); ++i) {
    if(!cloudsEqual(*clouds_[i], *tr.clouds_[i]))
      return false;
  }
  return true;
}


/************************************************************/
/******************** Helper Functions ********************/
/************************************************************/

void track_manager::serializePointCloudROS(const sensor_msgs::PointCloud& cloud, ostream& out) {
  out << "Cloud" << endl;
  out << "serialization_length" << endl;
  out << cloud.serializationLength() << endl;
  uint8_t data[cloud.serializationLength()];
  cloud.serialize(data, 0);
  assert(sizeof(char*) == sizeof(uint8_t*));
  out.write((char*)data, cloud.serializationLength());
}

bool track_manager::deserializePointCloudROS(std::istream& istrm, sensor_msgs::PointCloud* cloud) {
  string line;

  getline(istrm, line);
  if(line.compare("Cloud") != 0)
    return false;

  getline(istrm, line);
  if(line.compare("serialization_length") != 0)
    return false;

  uint32_t serialization_length = 0;
  istrm >> serialization_length;
  getline(istrm, line);
  
  uint8_t data[serialization_length];
  istrm.read((char*)data, serialization_length);
  cloud->deserialize(data);
  return true;
}


bool track_manager::deserializePointCloud(istream& istrm, PointCloud* cloud) {
  string line;

  getline(istrm, line);
  if(line.compare("Cloud") != 0)
    return false;

  getline(istrm, line);
  if(line.compare("serialization_version_") != 0)
    return false;

  int serialization_version = 0;
  istrm >> serialization_version;
  if(serialization_version != POINTCLOUD_SERIALIZATION_VERSION)
    return false;
  getline(istrm, line);
 
  getline(istrm, line);
  if(line.compare("num_points") != 0)
    return false;

  size_t num_points = 0;
  istrm >> num_points;
  getline(istrm, line);
  
  getline(istrm, line);
  if(line.compare("points") != 0)
    return false;

  float* buf = (float*)malloc(sizeof(float)*num_points*3);
  istrm.read((char*)buf, sizeof(float)*num_points*3);
  cloud->set_points_size(num_points);
  for(size_t i=0; i<num_points; ++i) {
    cloud->points[i].x = buf[i*3];
    cloud->points[i].y = buf[i*3+1];
    cloud->points[i].z = buf[i*3+2];
  }
  free(buf);
  return true;
}

void track_manager::serializePointCloud(const sensor_msgs::PointCloud& cloud, ostream& out) {
  out << "Cloud" << endl;
  out << "serialization_version_" << endl;
  out << POINTCLOUD_SERIALIZATION_VERSION << endl;
  out << "num_points" << endl;
  out << cloud.get_points_size() << endl;
  out << "points" << endl;

  float* buf = (float*)malloc(sizeof(float)*cloud.get_points_size()*3);
  for(size_t i=0; i<cloud.get_points_size(); ++i) {
    buf[i*3] = cloud.points[i].x;
    buf[i*3+1] = cloud.points[i].y;
    buf[i*3+2] = cloud.points[i].z;
  }
  out.write((char*)buf, sizeof(float)*cloud.get_points_size()*3);
  free(buf);
}


bool track_manager::cloudsEqual(const PointCloud& c1, const PointCloud& c2) {

  // -- Check the points.
  if(c1.get_points_size() != c2.get_points_size())
    return false;

  for(size_t i=0; i<c1.get_points_size(); ++i) {
    if(c1.points[i].x != c2.points[i].x)
      return false;
    if(c1.points[i].y != c2.points[i].y)
      return false;
    if(c1.points[i].z != c2.points[i].z)
      return false;
  }

  // -- Check the channels.
  if(c1.get_channels_size() != c2.get_channels_size())
    return false;

  for(size_t i=0; i<c1.get_channels_size(); ++i) {
    if(c1.channels[i].get_values_size() != c1.channels[i].get_values_size())
      return false;
    for(size_t j=0; j<c1.channels[i].get_values_size(); ++j) {
      if(c1.channels[i].values[j] != c2.channels[i].values[j])
	return false;
    }
  }

  return true;
}


bool track_manager::streamTrack(std::string track_manager_filename, const Track& tr) {
  // -- Stick the track on the end.
  ofstream out;
  out.open(track_manager_filename.c_str(), ios::out | ios::app | ios::binary); 
  tr.serialize(out);
  out.close();
  
  return true;
}
