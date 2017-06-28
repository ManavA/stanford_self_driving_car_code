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


#include "perception.h"

using namespace dgc;

restricted_2d_area_t    ibeo_box[NUM_IBEO_LASERS] = 
  { { -5.0,  0.8,  -2.0,  0.5, -1.3, 1.15 },
    { -5.0,  0.8,  -0.5,  2.0, -1.3, 1.15 } };

dgc_transform_t                 ibeo_offset[NUM_IBEO_LASERS];

#define    NUM_BINS_PER_REV             180
#define    MAX_BEAMS_IN_BIN             20
#define    MAX_DIST_SAME_OBSTACLE       1.0

typedef struct {
  IbeoLaserPoint     *point;
  double                    dist;
  double                    angle;
  short                     valid;
} ibeo_point_t;

typedef struct {
  int                    num_beams;
  ibeo_point_t           beam[MAX_BEAMS_IN_BIN];
} ibeo_bin_t;

double
ibeo_beam_dist_2d( IbeoLaserPoint *p1, IbeoLaserPoint *p2 )
{
  return(hypot( p1->x-p2->x, p1->y-p2->y )); 
}

void 
integrate_ibeo( IbeoLaser *ibeo, dgc_transform_t *ibeo_offset, 
		restricted_2d_area_t *area, int nr, unsigned short counter )
{
  static ibeo_bin_t            ibbin[NUM_BINS_PER_REV][NUM_IBEO_LASERS];
  int                          i;
  dgc_transform_t              t;
  double                       dist, angle, px, py, pz;
  PerceptionCell*    cell = NULL, terrain_cell = NULL;
  ApplanixPose               * pose;

  for (i=0; i<NUM_BINS_PER_REV; i++) {
    ibbin[i][nr].num_beams = 0;
  }
  
  for(i = 0; i < ibeo->num_points; i++) {
    if( ibeo->point[i].status == DGC_IBEO_STATUS_OK &&
	ibeo->point[i].level>=0 && 
	ibeo->point[i].level<NUM_IBEO_BEAMS ) {
      px = ibeo->point[i].x;
      py = ibeo->point[i].y;
      pz = ibeo->point[i].z;
      dist = hypot( px, py );
      angle = atan2(py, px );
      if ( dist>settings.ibeo_min_dist && 
	   dist<settings.ibeo_max_dist &&
	   angle>settings.ibeo_min_angle &&
	   angle<settings.ibeo_max_angle &&
	   angle>area->min_angle && angle<area->max_angle &&
	   !(px>area->x1 && px<area->x2 &&
	     py>area->y1 && py<area->y2) ) {
	
	pose = applanix_pose(ibeo->timestamp);
	dgc_transform_copy( t, *ibeo_offset );
	dgc_transform_rotate_x( t, pose->roll );
	dgc_transform_rotate_y( t, pose->pitch );
	dgc_transform_rotate_z( t, pose->yaw );
	dgc_transform_translate( t, 
				 pose->smooth_x, 
				 pose->smooth_y, 
				 pose->smooth_z );
	dgc_transform_point(&px, &py, &pz, t);
	cell = (PerceptionCell*)grid_get_xy(grid, px, py );
	if (cell!=NULL) {
	  if (cell->last_use!=counter) {
	    obstacles_s->cell[obstacles_s->num] = cell;
	    if (obstacles_s->num<MAX_NUM_POINTS)
	      obstacles_s->num++;
	    cell->last_use = counter;
	  }
	  terrain_cell = 
	    (PerceptionCell*)grid_get_xy(terrain_grid, px, py );
	  set_cell_min( cell, pz, counter );
	  set_cell_max( cell, pz, counter );
	  set_cell_min( terrain_cell, pz, counter );
	  set_cell_max( terrain_cell, pz, counter );
	  sync_with_terrain( terrain_cell, cell );
	  cell->hits ++;
	  include_cell(cell,counter);
	}
      }
    }
  }

}
  
