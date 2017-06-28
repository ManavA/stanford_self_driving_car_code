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


#ifndef TRACK_MANAGER_H
#define TRACK_MANAGER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <sensor_msgs/PointCloud.h>
#include <tr1/memory>

#define POINTCLOUD_SERIALIZATION_VERSION 0
#define TRACK_SERIALIZATION_VERSION 1
#define TRACKMANAGER_SERIALIZATION_VERSION 1


namespace track_manager { 

  class Track {
  public:
    int serialization_version_;
    std::string label_;
    std::vector< boost::shared_ptr<sensor_msgs::PointCloud> > clouds_;
    //! Timestamps for each cloud.
    std::vector<double> timestamps_;
    //! velodyne_centers_[i][{0,1,2}] is the {x,y,z} coord of the velodyne for the ith cloud, in smooth coordinates.
    std::vector< std::vector<float> > velodyne_centers_;
    

    //! Initializes with label == "unknown", and that's it.
    Track();
    Track(std::istream& istrm);
    //! Initializes timestamps_ to be all -1s and velodyne_centers_ to be 0,0,0.
    Track(const std::vector< boost::shared_ptr<sensor_msgs::PointCloud> >& clouds);
    Track(const std::string& label,
	  const std::vector< boost::shared_ptr<sensor_msgs::PointCloud> >& clouds,
	  const std::vector<double>& timestamps,
	  const std::vector< std::vector<float> >& velodyne_centers);

    //! Reserves space in the vectors of velo centers, timestamps, and clouds.
    void reserve(size_t num);
    void insertCloud(boost::shared_ptr<sensor_msgs::PointCloud> cloud,
		     double timestamp,
		     const std::vector<float>& velodyne_center);
    bool operator==(const Track& tr);
    bool operator!=(const Track& tr);
    void serialize(std::ostream& out) const;
    bool deserialize(std::istream& istrm);
  };

  class TrackManager {
  public:
    int serialization_version_;
    std::vector< boost::shared_ptr<Track> > tracks_;

    int getTotalNumClouds() const;
    bool operator==(const TrackManager& tm);
    bool operator!=(const TrackManager& tm);
    bool save(const std::string& filename);
    void serialize(std::ostream& out);
    bool deserialize(std::istream& istrm);
    //! Put the tracks in descending order of track length.
    void sortTracks();
    //! Sort based on some other criteria.  Descending.
    void sortTracks(double (*rateTrack)(const Track&));
    void sortTracks(const std::vector<double>& track_ratings);
    
    TrackManager();
    TrackManager(const std::string& filename);
    TrackManager(std::istream& istrm);
    TrackManager(const std::vector< boost::shared_ptr<Track> >& tracks);
  };

  void serializePointCloud(const sensor_msgs::PointCloud& cloud, std::ostream& out);
  bool deserializePointCloud(std::istream& istrm, sensor_msgs::PointCloud* cloud);
  bool cloudsEqual(const sensor_msgs::PointCloud& c1, const sensor_msgs::PointCloud& c2);
  bool streamTrack(std::string track_manager_filename, const Track& tr); 
  double getTrackLength(const Track& tr);
  bool deserializePointCloudROS(std::istream& istrm, sensor_msgs::PointCloud* cloud);
  void serializePointCloudROS(const sensor_msgs::PointCloud& cloud, std::ostream& out);
}


   
#endif //TRACK_MANAGER_H
