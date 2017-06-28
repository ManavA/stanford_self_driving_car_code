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


#include <global.h>
#include "bucket.h"

using namespace dgc;

bucket_grid::bucket_grid(double resolution, double width, double height)
{
  int i, j;

  this->resolution = resolution;
  x_size = (int)ceil(width / resolution);
  y_size = (int)ceil(height / resolution);
  grid = (bucket_t **)calloc(x_size, sizeof(bucket_t *));
  dgc_test_alloc(grid);
  for(i = 0; i < x_size; i++) {
    grid[i] = (bucket_t *)calloc(y_size, sizeof(bucket_t));
    dgc_test_alloc(grid[i]);
    for(j = 0; j < y_size; j++) {
      grid[i][j].num_points = 0;
      grid[i][j].max_points = 0;
      grid[i][j].point = NULL;
    }
  }
}

bucket_grid::~bucket_grid()
{
  int x, y;

  for(x = 0; x < x_size; x++) {
    for(y = 0; y < y_size; y++) 
      if(grid[x][y].point != NULL)
        free(grid[x][y].point);
    free(grid[x]);
  }
  free(grid);
}

void bucket_grid::clear(void)
{
  int x, y;

  for(x = 0; x < x_size; x++) 
    for(y = 0; y < y_size; y++) 
      grid[x][y].num_points = 0;
}

void bucket_grid::build_grid(double *x, double *y, int num_points)
{
  int i, xi, yi;
  bucket_t *bucket;
  bucket_point *p;

  clear();

  if(num_points == 0) {
    this->min_x = 0;
    this->min_y = 0;
  }
  else {
    this->min_x = x[0];
    this->min_y = y[0];
    for(i = 0; i < num_points; i++) {
      if(x[i] < this->min_x)
	this->min_x = x[i];
      if(y[i] < this->min_y)
	this->min_y = y[i];
    }
    for(i = 0; i < num_points; i++) {
      xi = (int)floor((x[i] - min_x) / resolution);
      yi = (int)floor((y[i] - min_y) / resolution);
      if(xi >= 0 && yi >= 0 && xi < x_size && yi < y_size) {
	bucket = grid[xi] + yi;
	if(bucket->num_points == bucket->max_points) {
	  bucket->max_points += 100;
	  bucket->point = (bucket_point *)
	    realloc(bucket->point, bucket->max_points * 
		    sizeof(bucket_point));
	}
	p = bucket->point + bucket->num_points;
	p->x = x[i];
	p->y = y[i];
	p->id = i;
	bucket->num_points++;
      }
    }
  }
}

void bucket_grid::build_grid(double *x, double *y, int num_points,
			     double min_x, double min_y)
{
  int i, xi, yi;
  bucket_t *bucket;
  bucket_point *p;

  clear();

  this->min_x = min_x;
  this->min_y = min_y;

  for(i = 0; i < num_points; i++) {
    xi = (int)floor((x[i] - min_x) / resolution);
    yi = (int)floor((y[i] - min_y) / resolution);
    if(xi >= 0 && yi >= 0 && xi < x_size && yi < y_size) {
      bucket = grid[xi] + yi;
      if(bucket->num_points == bucket->max_points) {
	bucket->max_points += 100;
	bucket->point = (bucket_point *)
	  realloc(bucket->point, bucket->max_points * 
		  sizeof(bucket_point));
      }
      p = bucket->point + bucket->num_points;
      p->x = x[i];
      p->y = y[i];
      p->id = i;
      bucket->num_points++;
    }
  }
}

#ifdef blah
void bucket_grid::sync_to_map(dgc_grid_p obstacle_grid)
{
  int n, i, xi, yi;
  dgc_perception_map_grid_p cell;
  bucket_t *bucket;

  if(obstacle_grid != NULL) {
    min_x = obstacle_grid->map_c0 * obstacle_grid->resolution;
    min_y = obstacle_grid->map_r0 * obstacle_grid->resolution;

    cell = (dgc_perception_map_grid_p)obstacle_grid->cell;
    n = obstacle_grid->rows * obstacle_grid->cols;
    for(i = 0; i < n; i++) {
      if(cell->type != PERCEPTION_MAP_OBSTACLE_FREE &&
         (cell->type == PERCEPTION_MAP_OBSTACLE_HIGH ||
          cell->type == PERCEPTION_MAP_OBSTACLE_LOW)) {
        xi = (int)floor((cell->x - min_x) / resolution);
	yi = (int)floor((cell->y - min_y) / resolution);
        if(xi >= 0 && yi >= 0 && xi < x_size && yi < y_size) {
	  bucket = grid[xi] + yi;
	  if(bucket->num_cells == bucket->max_cells) {
	    bucket->max_cells += 100;
	    bucket->cell = (dgc_perception_map_grid_p *)
	      realloc(bucket->cell, bucket->max_cells * 
		      sizeof(dgc_perception_map_grid_p));
	  }
	  bucket->cell[bucket->num_cells] = cell;
	  bucket->num_cells++;
	}
      }
      cell++;
    }
  }
}
#endif

inline void dgc_neighbor_list_add(dgc_neighbor_list_p *list, 
				  double x, double y, int id, double distance)
{
  dgc_neighbor_list_p temp;

  temp = (dgc_neighbor_list_p)calloc(1, sizeof(dgc_neighbor_list_t));
  temp->x = x;
  temp->y = y;
  temp->id = id;
  temp->distance = distance;
  temp->next = *list;
  *list = temp;
}

void dgc_neighbor_list_free(dgc_neighbor_list_p *list)
{
  dgc_neighbor_list_p temp;
  
  while(*list != NULL) {
    temp = *list;
    *list = (*list)->next;
    free(temp);
  }
}

inline int dgc_neighbor_list_length(dgc_neighbor_list_p list)
{
  int count = 0;
  
  while(list != NULL) {
    count++;
    list = list->next;
  }
  return count;
}

void dgc_neighbor_list_print(dgc_neighbor_list_p list)
{
  int i = 0;

  fprintf(stderr, "Point list : \n");
  while(list != NULL) {
    fprintf(stderr, "%d : %.2f, %.2f  %d - %.2f\n", i, 
	    list->x, list->y, list->id, list->distance);
    list = list->next;
    i++;
  }
  fprintf(stderr, "\n");
}

void bucket_grid::nearest_neighbor(double x, double y, double max_range,
				   double *closest_x, double *closest_y,
				   int *which, double *distance)
{
  double range_sq = max_range * max_range, d_sq, min_d_sq;
  int xi1, xi2, yi1, yi2, r, c, i, n;
  bucket_t *bucket;
  bucket_point *p;
  
  xi1 = (int)floor((x - max_range - min_x) / resolution);
  yi1 = (int)floor((y - max_range - min_y) / resolution);
  xi2 = (int)floor((x + max_range - min_x) / resolution);
  yi2 = (int)floor((y + max_range - min_y) / resolution);

  min_d_sq = range_sq;
  *which = -1;

  for(c = xi1; c <= xi2; c++)
    for(r = yi1; r <= yi2; r++) {
      if(c < 0 || r < 0 || c >= x_size || r >= y_size)
	continue;
      bucket = grid[c] + r;
      n = bucket->num_points;
      for(i = 0; i < n; i++) {
	p = bucket->point + i;
	d_sq = (p->x - x) * (p->x - x) + (p->y - y) * (p->y - y);
	if(d_sq <= min_d_sq) {
	  min_d_sq = d_sq;
	  *closest_x = p->x;
	  *closest_y = p->y;
	  *which = p->id;
	}
      }
    }
  if(*which != -1)
    *distance = sqrt(min_d_sq);
}

dgc_neighbor_list_p bucket_grid::range_search(double x, double y, double range)
{
  double range_sq = range * range, d_sq;
  int xi1, xi2, yi1, yi2, r, c, i, n;
  dgc_neighbor_list_p neighbor_list = NULL;
  bucket_point *p;
  bucket_t *bucket;
  
  xi1 = (int)floor((x - range - min_x) / resolution);
  yi1 = (int)floor((y - range - min_y) / resolution);
  xi2 = (int)floor((x + range - min_x) / resolution);
  yi2 = (int)floor((y + range - min_y) / resolution);

  for(c = xi1; c <= xi2; c++)
    for(r = yi1; r <= yi2; r++) {
      if(c < 0 || r < 0 || c >= x_size || r >= y_size)
	continue;
      bucket = grid[c] + r;
      n = bucket->num_points;
      for(i = 0; i < n; i++) {
	p = bucket->point + i;
	d_sq = (p->x - x) * (p->x - x) + (p->y - y) * (p->y - y);
	if(d_sq <= range_sq)
	  dgc_neighbor_list_add(&neighbor_list, p->x, p->y, p->id, sqrt(d_sq));
      }
    }
  return neighbor_list;
}

void bucket_grid::range_search(double x, double y, double range, 
			       bucket_point **neighbor_list,
			       int *max_n)
{
  double range_sq = range * range;
  
  int xi1 = (int)floor((x - range - min_x) / resolution);
  int yi1 = (int)floor((y - range - min_y) / resolution);
  int xi2 = (int)floor((x + range - min_x) / resolution);
  int yi2 = (int)floor((y + range - min_y) / resolution);

  int list_n = 0;
  for(int c = xi1; c <= xi2; c++) {
    for(int r = yi1; r <= yi2; r++) {
      if(c < 0 || r < 0 || c >= x_size || r >= y_size)
	continue;
      bucket_t *bucket = grid[c] + r;
      int n = bucket->num_points;

      for (int i = 0; i < n && list_n < *max_n; i++) {
	bucket_point *p = bucket->point + i;
	double d_sq = (p->x - x) * (p->x - x) + (p->y - y) * (p->y - y);
	if (d_sq <= range_sq)
	  neighbor_list[list_n++] = p;
      }
    }
  }
  *max_n = list_n;
}


void bucket_grid::print_stats(void)
{
  int x, y;
  int max_points = 0, total_points = 0;
  bucket_t *bucket;

  for(x = 0; x < x_size; x++) 
    for(y = 0; y < y_size; y++) {
      bucket = grid[x] + y;
      if(bucket->num_points > max_points) 
	max_points = bucket->num_points;
      total_points += bucket->num_points;
    }
  fprintf(stderr, "Num buckets : %d\n", x_size * y_size);
  fprintf(stderr, "Total points in buckets : %d\n", total_points);
  fprintf(stderr, "Maximum number of points per cell : %d\n", max_points);
  fprintf(stderr, "Average number of points per cell : %.2f\n", 
	  total_points / (double)(x_size * y_size));
}

