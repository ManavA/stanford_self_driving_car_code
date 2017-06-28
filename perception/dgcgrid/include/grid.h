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


#ifndef DGC_GRID_H
#define DGC_GRID_H

#include <cmath>
#include <global.h>

namespace dgc {

typedef void (*clear_handler_t)(void *cell);

typedef struct {
  double resolution;   /* map resolution in meters                     */
  int rows, cols;      /* size of grid                                 */
  int map_r0, map_c0;  /* grid coordinates of lower left corner of map */
  int array_r0, array_c0;    /* position of lower left corner in array */
  int bytes_per_cell;  /* number of bytes per grid cell                */
  void *cell;          /* actual map data                              */
  void *default_value; /* default value for new cells                  */
  clear_handler_t clear_handler;
} dgc_grid_t, *dgc_grid_p;

#define WRAP(x, max) (((x)>(max))?((x)-(max)):(((x)<0)?((x)+(max)):(x)))

dgc_grid_p dgc_grid_initialize(double map_resolution, int map_rows, 
                               int map_cols, int bytes_per_cell, 
			       void *default_value);


// might be too long for inline...
inline int wrap(int x, int max)
{
  if(x >= max) {
    while(x >= max)
      x -= max;
  }
  else if(x < 0) {
    while(x < 0)
      x += max;
  }
  return x;
}

inline void *dgc_grid_get_xy(dgc_grid_p grid, double x, double y)
{
  int r, c;
  
  if(grid == NULL)
    return NULL;
  c = (int)floor(x / grid->resolution) - grid->map_c0;
  r = (int)floor(y / grid->resolution) - grid->map_r0;
  if(r < 0 || c < 0 || r >= grid->rows || c >= grid->cols)
    return NULL;
  else {
    r = wrap(r + grid->array_r0, grid->rows);
    c = wrap(c + grid->array_c0, grid->cols);
    return (void *)((char*)grid->cell + 
		    grid->bytes_per_cell * (r * grid->cols + c));
  }
}

inline void *dgc_grid_get_rc_global(dgc_grid_p grid, int r, int c)
{
  if(grid == NULL)
    return NULL;
  r -= grid->map_r0;
  c -= grid->map_c0;
  if(r < 0 || c < 0 || r >= grid->rows || c >= grid->cols)
    return NULL;
  else {
    r = wrap(r + grid->array_r0, grid->rows);
    c = wrap(c + grid->array_c0, grid->cols);
    return (void *)((char*)grid->cell + 
		    grid->bytes_per_cell * (r * grid->cols + c));
  }
}

void *dgc_grid_get_rc_local(dgc_grid_p grid, int r, int c);

void *dgc_grid_get_rc_local_unsafe(dgc_grid_p grid, int r, int c);

void dgc_grid_xy_to_rc_local(dgc_grid_p grid, double x, double y,
                             int *r, int *c);

void dgc_grid_rc_local_to_xy(dgc_grid_p grid, int r, int c, 
                             double *x, double *y);

void dgc_grid_cell_to_rc_local(dgc_grid_p grid, void *cell, 
			       int *r, int *c);

void dgc_grid_cell_to_xy(dgc_grid_p grid, void *cell, 
			 double *x, double *y);

void dgc_grid_clear(dgc_grid_p grid);

int dgc_grid_recenter_grid(dgc_grid_p grid, double x, double y);

void dgc_grid_free(dgc_grid_p grid);

void dgc_grid_copy(dgc_grid_p src, dgc_grid_p dst);

void dgc_grid_set_clear_handler(dgc_grid_p grid, clear_handler_t handler);

} //namespace dgc

#endif
