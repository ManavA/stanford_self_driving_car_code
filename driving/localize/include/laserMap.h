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


#ifndef VLR_LASER_MAP_H_
#define VLR_LASER_MAP_H_

#define TERRAIN_TILE_SIZE       500
#define TERRAIN_TILE_RESOLUTION 0.15
#define TERRAIN_GRID_NUM_TILES  3

namespace vlr {

typedef struct {
  float z;
  uint32_t z_count;
  float intensity;
  uint32_t i_count;
  float current_highest;
  float z_response;
  int32_t n;
  float xa;
  float sxi2;
  float stdev;
} TerrainTileCell;


class TerrainTile {
public:
  TerrainTile(int32_t rows, int32_t cols);
  ~TerrainTile();
  bool load(std::string& filename);
  void save(std::string& filename);
  void do_average();

  inline std::string& fileName() {return filename_;}
  inline double& utmX0() {return utm_x0_;}
  inline double& utmY0() {return utm_y0_;}
  inline double& resolution() {return resolution_;}
  inline std::string& utmZone() {return utm_zone_;}
  inline TerrainTileCell** cell() {return cell_;}

private:
  int32_t rows_, cols_;
  double utm_x0_, utm_y0_, resolution_, min_z_;
  std::string utm_zone_;
  TerrainTileCell** cell_;
  uint32_t dl_;
  uint32_t num_pixels_;
  std::string filename_;
  uint8_t* rgb_buf_;
};

} // namespace vlr

#endif
