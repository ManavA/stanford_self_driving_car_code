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


#ifndef DGC_FACELIST_H
#define DGC_FACELIST_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double x, y, z;
  float r, g, b;
  float range;
} dgc_vertex_t, *dgc_vertex_p;

typedef struct {
  int fill;
  dgc_vertex_t v1, v2, v3, v4;
} dgc_polygon_t, *dgc_polygon_p;

typedef struct {
  dgc_polygon_p points;
  int num_points;
  int mark, full;
} dgc_facelist_t, *dgc_facelist_p;

void dgc_facelist_new(dgc_facelist_p points, int num_points);

void dgc_facelist_add_color_points(dgc_facelist_p points, dgc_polygon_p poly,
                                   int n, double max_size);

void dgc_facelist_reset(dgc_facelist_p points);

void dgc_facelist_draw_mesh(dgc_facelist_p points, double origin_x, 
                            double origin_y, double origin_z);

void dgc_facelist_draw_2D(dgc_facelist_p points, 
                          double origin_x, double origin_y);

#ifdef __cplusplus
}
#endif

#endif
