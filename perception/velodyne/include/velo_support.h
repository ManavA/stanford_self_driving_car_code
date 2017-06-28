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


#ifndef DGC_VELO_SUPPORT_H
#define DGC_VELO_SUPPORT_H

#include <sys/types.h>
//#include <logio.h>
//#include <localize/LocalizePose.h>
#include <transform.h>
#include <comp_stdio.h>
#include <velodyne.h>
#include <velodyne/Block.h>
#include <applanix/ApplanixPose.h>

namespace velodyne {

#define      MAX_NUM_VELOSCANS	      5000

typedef struct {
  int scan_num;
  double smooth_x, smooth_y, smooth_z;
  double longitude, latitude, altitude;
  double v_east, v_north, v_up;
  double roll, pitch, yaw;
  double timestamp;
  double x_offset, y_offset;
} dgc_velodyne_index_pose;

typedef struct {
public:
  off64_t file_offset;
  int spin_start_i;
  int num_scans;
  int num_poses;
  dgc_velodyne_index_pose *pose;
} dgc_velodyne_index_entry;

class dgc_velodyne_index {
public:
  dgc_velodyne_index();
  ~dgc_velodyne_index();
  int load(char *filename);
  int save(char *filename);
  int num_spins;
  dgc_velodyne_index_entry *spin;
};

class Spin {
public:
  Spin();
  Spin(const Spin& copy);
  virtual ~Spin();

  size_t numBlocks() {return blocks_.size();}
  std::vector<velodyne::Block> blocks_;

  void copy(const Spin &copy);

  void save(const std::string& filename, double applanix_lat, double applanix_lon, double applanix_altitude);
  void load(const std::string& filename, double* applanix_lat, double* applanix_lon, double* applanix_altitude);
  void load(Velodyne::VelodyneFile& vlf, Velodyne& velodyne, dgc_velodyne_index* vindex, int which, double* applanix_lat, double* applanix_lon,
      double* applanit_alt);
};

typedef void (*spin_function)(Spin* spin, Config* config, applanix::ApplanixPose* applanix_pose);

//int read_applanix_message(cio::FILE* log_fp, dgc::LineBuffer* line_buffer, applanix::ApplanixPose* applanix_pose, localize::LocalizePose* localize_pose);
//int read_applanix_message(cio::FILE* log_fp, dgc::LineBuffer* line_buffer, applanix::ApplanixPose* applanix_pose);

void vlf_projector(const std::string& velo_log_filename, const std::string&  ipc_log_filename,
                   const std::string& cal_filename, const std::string& int_filename, dgc::dgc_transform_t velodyne_offset, spin_function f);

} // namespace velodyne

#endif
