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


#ifndef BIL_H
#define BIL_H

namespace vlr {

typedef struct {
  double min_lat, min_lon, resolution, resolution_degrees;
  int rows, cols;
  unsigned char **map;
} charmap_t, *charmap_p;

typedef struct {
  double min_x, min_y;
  char utmzone[3];
  double x_resolution, y_resolution;
  double min_lat, min_lon, resolution_degrees;
  int rows, cols;
  short int **map;
} shortmap_t, *shortmap_p;

typedef struct {
  float x, y, z;
} vector_t, *vector_p;

typedef struct {
  float x1, y1, x2, y2;
} marchinggrid_line_t, *marchinggrid_line_p;

typedef struct {
  int num_lines;
  marchinggrid_line_p line;
} marchinggrid_results_t, *marchinggrid_results_p;

void marching_squares(shortmap_p map, float isolevel, 
                      marchinggrid_results_p results);

shortmap_p read_shortmap_bil(const std::string& filename);

void shortmap_to_pgm(shortmap_p map, const std::string& filename);

charmap_p read_charmap_bil(const std::string& filename);

void charmap_to_ppm(charmap_p map, const std::string& filename);

vector_t **compute_map_normals(char *map_filename, shortmap_p map);

void free_shortmap(shortmap_p *map);

} // namespace vlr

#endif
