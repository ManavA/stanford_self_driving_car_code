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


#ifndef DGC_POINTLIST_H
#define DGC_POINTLIST_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double x, y, z;
  float r, cost;
} dgc_pl_point3D_t, *dgc_pl_point3D_p;

typedef struct {
  dgc_pl_point3D_p points;
  int num_points;
  int mark, full;
} dgc_pointlist_t, *dgc_pointlist_p;

void dgc_pointlist_new(dgc_pointlist_p points, int num_points);

void dgc_pointlist_add_point(dgc_pointlist_p points, double x, double y,
                             double z, double r);

void dgc_pointlist_add_point_wcost(dgc_pointlist_p points, double x, double y,
                                   double z, double cost);

void dgc_pointlist_add_points(dgc_pointlist_p points, double *x, double *y,
                              double *z, int n);

void dgc_pointlist_add_clipped_points(dgc_pointlist_p points, 
                                      double *x, double *y, double *z,
                                      float *range, int n, double min_range, 
                                      double max_range);

void dgc_pointlist_reset(dgc_pointlist_p points);

void dgc_pointlist_draw_circles(dgc_pointlist_p points, double r, double g, 
                                double b, double origin_x, double origin_y,
                                double origin_z);

void dgc_pointlist_draw_circles2D(dgc_pointlist_p points, double r, double g, 
                                  double b, double origin_x, double origin_y);

void dgc_pointlist_draw(dgc_pointlist_p points, double r, double g, double b,
                        double origin_x, double origin_y, double origin_z);

void dgc_pointlist_draw_partial(dgc_pointlist_p points, int mark);

void dgc_pointlist_draw_costs(dgc_pointlist_p points,
                              double r1, double g1, double b1,
                              double r2, double g2, double b2);

#ifdef __cplusplus
}
#endif

#endif
