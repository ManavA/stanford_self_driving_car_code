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


#ifndef TERRAINMAP_H_
#define TERRAINMAP_H_

#define TERRAIN_TILE_SIZE       500
#define TERRAIN_TILE_RESOLUTION 0.15
#define TERRAIN_GRID_NUM_TILES  3

namespace vlr {

typedef struct {
  float z;
  unsigned int z_count;
  float intensity;
  unsigned int i_count;
  float current_highest;
  float z_response;
  int n;
  float xa;
  float sxi2;
  float stdev;
} terrain_tile_cell;

typedef struct {
  float curb;
  float line_lat;
  float blur;
  float orig;
} vision_tile_cell;

class vision_tile {
public:
  vision_tile(int rows, int cols);
  ~vision_tile();
  int load(char *filename);
  int save(char *filename);

  int rows, cols;
  double utm_x0, utm_y0, resolution, min_z;
  char utmzone[10], filename[200];
  vision_tile_cell **cell;
  unsigned int dl;
};

class terrain_tile {
public:
  terrain_tile(int rows, int cols);
  ~terrain_tile();
  int load(char *filename);
  int save(char *filename);
  void do_average(void);

  int rows, cols;
  double utm_x0, utm_y0, resolution, min_z;
  char utmzone[10], filename[200];
  terrain_tile_cell **cell;
  unsigned int dl;
};

} // namespace vlr

#endif
