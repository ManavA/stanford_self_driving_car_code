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


#include <grid.h>
#include <string.h>
#include <stdlib.h>

namespace dgc {

inline int int_floor(double  x) { return ((int)(x + 100000.0)) - 100000; }

//inline int wrap(int x, int max)
//{
//  if(x >= max) {
//    while(x >= max)
//      x -= max;
//  }
//  else if(x < 0) {
//    while(x < 0)
//      x += max;
//  }
//  return x;
//}

dgc_grid_p dgc_grid_initialize(double map_resolution, int map_rows, 
                               int map_cols, int bytes_per_cell, 
			       void *default_value)
{
  dgc_grid_p grid;
  int r, c;
  void *cell;

  grid = (dgc_grid_p)calloc(1, sizeof(dgc_grid_t));
  dgc_test_alloc(grid);
  grid->resolution = map_resolution;
  grid->rows = map_rows;
  grid->cols = map_cols;
  grid->bytes_per_cell = bytes_per_cell;
  grid->map_r0 = 0;
  grid->map_c0 = 0;
  grid->array_r0 = 0;
  grid->array_c0 = 0;
  grid->cell = calloc(map_rows * map_cols, bytes_per_cell);
  dgc_test_alloc(grid->cell);
  
  if(default_value == NULL) {
    grid->default_value = (void *)calloc(bytes_per_cell, 1);
    dgc_test_alloc(grid->default_value);
  }
  else
    grid->default_value = default_value;

  /* fill everything with default values */
  for(r = 0; r < grid->rows; r++) 
    for(c = 0; c < grid->cols; c++) { 
      cell = (void *)dgc_grid_get_rc_local(grid, r, c);
      memcpy(cell, grid->default_value, grid->bytes_per_cell);
    }
  grid->clear_handler = NULL;
  return grid;
}

void dgc_grid_set_clear_handler(dgc_grid_p grid, clear_handler_t handler)
{
  grid->clear_handler = handler;
}

void *dgc_grid_get_rc_local_unsafe(dgc_grid_p grid, int r, int c)
{
  r = wrap(r + grid->array_r0, grid->rows);
  c = wrap(c + grid->array_c0, grid->cols);
  return (void *)((char*)grid->cell + grid->bytes_per_cell * (r * grid->cols + c));
}

void *dgc_grid_get_rc_local(dgc_grid_p grid, int r, int c)
{
  if(grid == NULL)
    return NULL;
  if(r < 0 || c < 0 || r >= grid->rows || c >= grid->cols)
    return NULL;
  else {
    r = wrap(r + grid->array_r0, grid->rows);
    c = wrap(c + grid->array_c0, grid->cols);
    return (void *)((char*)grid->cell + 
		    grid->bytes_per_cell * (r * grid->cols + c));
  }
}

void dgc_grid_xy_to_rc_local(dgc_grid_p grid, double x, double y,
                             int *r, int *c)
{
  if(grid == NULL)
    return;
  *c = int_floor(x / grid->resolution) - grid->map_c0;
  *r = int_floor(y / grid->resolution) - grid->map_r0;
}

void dgc_grid_rc_local_to_xy(dgc_grid_p grid, int r, int c, 
			     double *x, double *y)
{
  if(grid == NULL)
    return;
  *x = (grid->map_c0 + c + 0.5) * grid->resolution;
  *y = (grid->map_r0 + r + 0.5) * grid->resolution;
}

void dgc_grid_cell_to_rc_local(dgc_grid_p grid, void *cell, 
			       int *r, int *c)
{
  int n;
  
  if(grid == NULL)
    return;

  n = ((char*)(cell) - (char*)(grid->cell)) / grid->bytes_per_cell;
  *r = n / grid->cols;
  *c = n - *r * grid->cols;
  
  *r -= grid->array_r0;
  *c -= grid->array_c0;
  if(*r < 0) 
    *r += grid->rows;
  if(*c < 0) 
    *c += grid->cols;
}

void dgc_grid_cell_to_xy(dgc_grid_p grid, void *cell, 
			 double *x, double *y)
{
  int n, r, c;

  if(grid == NULL)
    return;

  n = ((char*)(cell) - (char*)(grid->cell)) / grid->bytes_per_cell;
  r = n / grid->cols;
  c = n - r * grid->cols;
  
  r -= grid->array_r0;
  c -= grid->array_c0;
  if(r < 0) 
    r += grid->rows;
  if(c < 0) 
    c += grid->cols;

  *x = (grid->map_c0 + c + 0.5) * grid->resolution;
  *y = (grid->map_r0 + r + 0.5) * grid->resolution;
}

void dgc_grid_clear(dgc_grid_p grid)
{
  int i;
  void *cell;

  if(grid->clear_handler) {
    for(i = 0; i < grid->rows * grid->cols; i++) {
      cell = grid->cell + grid->bytes_per_cell * i;
      grid->clear_handler(cell);
      memcpy(cell, grid->default_value, grid->bytes_per_cell);
    }
  }
  else {
    for(i = 0; i < grid->rows * grid->cols; i++)
      memcpy((char*)grid->cell + grid->bytes_per_cell * i,
	     grid->default_value, grid->bytes_per_cell);
  }
}

inline void dgc_grid_add_column_east(dgc_grid_p grid)
{
  int r;
  void *cell;

  if(grid->clear_handler) {
    for(r = 0; r < grid->rows; r++) {
      cell = grid->cell + grid->bytes_per_cell * 
	(r * grid->cols + grid->array_c0);
      grid->clear_handler(cell);
      memcpy(cell, grid->default_value, grid->bytes_per_cell);
    }
  }
  else {
    for(r = 0; r < grid->rows; r++)
      memcpy(grid->cell + grid->bytes_per_cell * 
	     (r * grid->cols + grid->array_c0), grid->default_value, 
	     grid->bytes_per_cell);
  }
  grid->map_c0++;
  grid->array_c0++;
  if(grid->array_c0 == grid->cols)
    grid->array_c0 = 0;
}

inline void dgc_grid_add_column_west(dgc_grid_p grid)
{
  int r, new_array_c0;
  void *cell;

  new_array_c0 = grid->array_c0 - 1;
  if(new_array_c0 < 0)
    new_array_c0 = grid->cols - 1;

  if(grid->clear_handler) {
    for(r = 0; r < grid->rows; r++) {
      cell = grid->cell + grid->bytes_per_cell * 
	(r * grid->cols + new_array_c0);
      grid->clear_handler(cell);
      memcpy(cell, grid->default_value, grid->bytes_per_cell);
    }
  }
  else {
    for(r = 0; r < grid->rows; r++)
      memcpy(grid->cell + grid->bytes_per_cell * 
	     (r * grid->cols + new_array_c0), grid->default_value, 
	     grid->bytes_per_cell);
  }

  grid->map_c0--;
  grid->array_c0 = new_array_c0;
}

inline void dgc_grid_add_row_north(dgc_grid_p grid)
{
  int c;
  void *cell;

  if(grid->clear_handler) {
    for(c = 0; c < grid->cols; c++) {
      cell = grid->cell + grid->bytes_per_cell * 
	(grid->array_r0 * grid->cols + c);
      grid->clear_handler(cell);
      memcpy(cell, grid->default_value, grid->bytes_per_cell);
    }
  }
  else {
    for(c = 0; c < grid->cols; c++) 
      memcpy((char*)grid->cell + grid->bytes_per_cell * 
	     (grid->array_r0 * grid->cols + c),
	     grid->default_value, grid->bytes_per_cell);
  }
  grid->map_r0++;
  grid->array_r0++;
  if(grid->array_r0 == grid->rows)
    grid->array_r0 = 0;
}

inline void dgc_grid_add_row_south(dgc_grid_p grid)
{
  int c, new_array_r0;
  void *cell;

  new_array_r0 = grid->array_r0 - 1;
  if(new_array_r0 < 0)
    new_array_r0 = grid->rows - 1;

  if(grid->clear_handler) {
    for(c = 0; c < grid->cols; c++) {
      cell = grid->cell + grid->bytes_per_cell *
	(new_array_r0 * grid->cols + c);
      grid->clear_handler(cell);
      memcpy(cell, grid->default_value, grid->bytes_per_cell);
    }
  }
  else {
    for(c = 0; c < grid->cols; c++)
      memcpy((char*)grid->cell + grid->bytes_per_cell *
	     (new_array_r0 * grid->cols + c), grid->default_value, 
	     grid->bytes_per_cell);
  }

  grid->map_r0--;
  grid->array_r0 = new_array_r0;
}

int dgc_grid_recenter_grid(dgc_grid_p grid, double x, double y)
{
  int i, corner_r, corner_c, dr, dc;
  
  corner_r = int_floor(y / grid->resolution) - grid->rows / 2;
  corner_c = int_floor(x / grid->resolution) - grid->cols / 2;

  dr = corner_r - grid->map_r0;
  dc = corner_c - grid->map_c0;
  
  if(dr == 0 && dc == 0)
    return 0;
  if(abs(dr) >= grid->rows || abs(dc) >= grid->cols) {
    dgc_grid_clear(grid);
    grid->map_r0 = corner_r;
    grid->map_c0 = corner_c;
    grid->array_r0 = 0;
    grid->array_c0 = 0;
  }
  else {
    if(dr > 0)
      for(i = 0; i < dr; i++)
        dgc_grid_add_row_north(grid);
    else if(dr < 0)
      for(i = 0; i < abs(dr); i++)
        dgc_grid_add_row_south(grid);
    if(dc > 0)
      for(i = 0; i < dc; i++)
        dgc_grid_add_column_east(grid);
    else if(dc < 0)
      for(i = 0; i < abs(dc); i++)
        dgc_grid_add_column_west(grid);
  }
  return 1;
}

void dgc_grid_free(dgc_grid_p grid)
{
  free(grid->cell);
  free(grid);
}

void dgc_grid_copy(dgc_grid_p src, dgc_grid_p dst)
{
  // realloc memory if necessary
  if(dst->rows != src->rows || dst->cols != src->cols)
    dst->cell = realloc(dst->cell, src->rows * src->cols * src->bytes_per_cell);
  
  // copy grid properties
  dst->resolution = src->resolution;
  dst->rows = src->rows;
  dst->cols = src->cols;
  dst->map_r0 = src->map_r0;
  dst->map_c0 = src->map_c0;
  dst->array_r0 = src->array_r0;
  dst->array_c0 = src->array_c0;
  dst->bytes_per_cell = src->bytes_per_cell;
  dst->default_value = src->default_value;
  
  // copy grid cells
  memcpy(dst->cell, src->cell, dst->rows * dst->cols * dst->bytes_per_cell);
}

} //namespace dgc
