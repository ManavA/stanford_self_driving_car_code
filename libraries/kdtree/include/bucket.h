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


#ifndef DGC_BUCKET_H
#define DGC_BUCKET_H

typedef struct {
  double x, y;
  int id;
} bucket_point;

typedef struct dgc_neighbor_list_struct {
  float distance;
  double x, y;
  int id;
  struct dgc_neighbor_list_struct *next;
} dgc_neighbor_list_t, *dgc_neighbor_list_p;

struct bucket_t {
  int num_points, max_points;
  bucket_point *point;
};

class bucket_grid {
public:
  bucket_grid(double resolution, double width, double height);
  ~bucket_grid();

  void clear(void);
  void build_grid(double *x, double *y, int num_points);
  void build_grid(double *x, double *y, int num_points,
		  double min_x, double min_y);
  void nearest_neighbor(double x, double y, double max_range,
			double *closest_x, double *closest_y, int *which,
			double *distance);
  dgc_neighbor_list_p range_search(double x, double y, double r);
  void range_search(double x, double y, double range, 
		    bucket_point **neighbor_list,
		    int *max_n);

  void print_stats(void);

private:
  double min_x, min_y, resolution;
  int x_size, y_size;
  bucket_t **grid;
};

void dgc_neighbor_list_free(dgc_neighbor_list_p *list);

#endif 
