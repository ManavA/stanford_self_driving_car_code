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


#ifndef VELODYNE_CONFIG_H_
#define VELODYNE_CONFIG_H_

#include <string>
#include <transform.h>

#define VELO_NUM_LASERS          64
#define VELO_NUM_TICKS           36000
#define VELO_SPIN_START          18000

namespace velodyne {

class Config {
public:
  Config();
  ~Config();

  void autoConfig();
  void recomputeAngles();
  void findBeamOrder();
  void integrateOffset(const dgc::dgc_transform_t offset);

  bool readCalibration(const std::string& filename);
  bool readIntensity(const std::string& filename);

  void printCalibrationData();

public:
  // variables that never change
  dgc::dgc_transform_t      offset_;
  double                    range_offset[VELO_NUM_LASERS];
  double                    range_offsetX[VELO_NUM_LASERS];
  double                    range_offsetY[VELO_NUM_LASERS];
  char                      laser_enabled[VELO_NUM_LASERS];
  double                    global_range_offset;
  double                    vert_angle[VELO_NUM_LASERS];
  double                    rot_angle[VELO_NUM_LASERS];
  double                    h_offset[VELO_NUM_LASERS];
  double                    v_offset[VELO_NUM_LASERS];
  double                    sin_vert_angle[VELO_NUM_LASERS];
  double                    cos_vert_angle[VELO_NUM_LASERS];
  double                    enc_rot_angle[VELO_NUM_TICKS][VELO_NUM_LASERS];
  double                    sin_rot_angle[VELO_NUM_TICKS][VELO_NUM_LASERS];
  double                    cos_rot_angle[VELO_NUM_TICKS][VELO_NUM_LASERS];
  double                    sin_enc_angle[VELO_NUM_TICKS];
  double                    cos_enc_angle[VELO_NUM_TICKS];
  double                    enc_angle[VELO_NUM_TICKS];
  double                    range_multiplier;
  int                       min_intensity;
  int                       max_intensity;
  double                    intensity_map[VELO_NUM_LASERS][256];
  int                       beam_order[VELO_NUM_LASERS];
  int                       inv_beam_order[VELO_NUM_LASERS];
  int                       spin_start;
};

} // namespace velodyne
#endif
