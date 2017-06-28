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


#include <sys/stat.h>
#include <iostream> 
#include <fstream>
#include <sstream>
#include <track_manager.h>
#include <math.h>

using namespace std;
using namespace track_manager;
using namespace sensor_msgs;

string usageString() {
  ostringstream oss;
  oss << "Usage: " << endl;
  oss << "  track_stats TRACK_MANAGER [TRACK_MANAGER ...]" << endl;
  //  oss << "  track_stats STATFILE TRACK_MANAGER [TRACK_MANAGER ...]" << endl;
  //  oss << "  ... where STATFILE is the name of a file to write the statistics of the following track manager(s)." << endl;
  return oss.str();
}

void computeCentroid(const PointCloud& pc, float* x, float* y, float* z) {
  *x = 0;
  *y = 0;
  *z = 0;
  for(size_t i=0; i<pc.get_points_size(); ++i) {
    *x += pc.points[i].x;
    *y += pc.points[i].y;
    *z += pc.points[i].z;
  }
  *x /= (double)pc.get_points_size();
  *y /= (double)pc.get_points_size();
  *z /= (double)pc.get_points_size();
}
    

float computeSpeed(const PointCloud& pc, const PointCloud& prev) {
  float x = 0;
  float y = 0;
  float z = 0;
  computeCentroid(pc, &x, &y, &z);

  float x_prev = 0;
  float y_prev = 0;
  float z_prev = 0;
  computeCentroid(prev, &x_prev, &y_prev, &z_prev);

  float speed = sqrt(pow(x - x_prev, 2) + pow(y - y_prev, 2) + pow(z - z_prev, 2));
  return speed;
}
  
  
int main(int argc, char** argv) {
  if(argc < 2) {
    cout << usageString() << endl;
    return 1;
  }

  // -- Open file for saving stats.
//   struct stat file_info;
//   if(stat(argv[1], &file_info) == 0) { 
//     cerr << "File " << argv[1] << " exists.  Delete it first." << endl;
//     return 1;
//   }
//   ofstream file(argv[1]);

//   ofstream vbike("vel_bike.txt");
//   ofstream vped("vel_ped.txt");
//   ofstream vcar("vel_car.txt");
//   ofstream vbg("vel_bg.txt");

  map<string, int> num_clouds;
  map<string, int> num_tracks;
  for(int i=1; i<argc; ++i) { 
    // -- Load the Track Manager class.
    cout << "Working on " << argv[i] << endl;
    TrackManager tm(argv[i]);

    for(size_t j=0; j<tm.tracks_.size(); ++j) {
      Track& tr = *tm.tracks_[j];
      num_clouds[tr.label_]+= tr.clouds_.size();
      num_tracks[tr.label_]++;
    }
  }

  int total_tracks = 0;
  int total_clouds = 0;
  cout << endl << "Track Statistics: " << endl;
  for(map<string, int>::iterator it = num_tracks.begin(); it!=num_tracks.end(); ++it) {
    cout << it->first << " tracks: " << it->second << endl;
    total_tracks += it->second;
  }
  cout << "Total tracks: " << total_tracks << endl << endl;
  cout << "Cloud Statistics: " << endl;
  for(map<string, int>::iterator it = num_clouds.begin(); it!=num_clouds.end(); ++it) {
    cout << it->first << " clouds: " << it->second << endl;
    total_clouds += it->second;
  }
  cout << "Total clouds: " << total_clouds << endl << endl;
      
      // -- Choose which file to write to.
//       ofstream* ofs = &vbike;
//       if(tr.label_.compare("pedestrian") == 0)
// 	ofs = &vped;
//       else if(tr.label_.compare("bicyclist") == 0)
// 	ofs = &vbike;
//       else if(tr.label_.compare("car") == 0)
// 	ofs = &vcar;
//       else
// 	ofs = &vbg;
      
//       for(size_t k=1; k<tr.clouds_.size(); ++k) {
// 	PointCloud& prev = *tr.clouds_[k-1];
// 	PointCloud& pc = *tr.clouds_[k];
	
// 	*ofs << computeSpeed(pc, prev) << endl;
//       }
//     }
//   }
  
  
}
