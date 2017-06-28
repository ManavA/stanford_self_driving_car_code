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


#include <rndf.h>
#include "rndf_lookup.h"

using namespace vlr;

RndfLookup::RndfLookup(rndf_file *rndf)
{
  rndf_wp = NULL;
  build_rndf_kdtree(rndf);
}

RndfLookup::~RndfLookup()
{
  free(rndf_wp);
  dgc_kdtree_free(&rndf_kdtree);
}	

double RndfLookup::road_angle(double x, double y)
{
  rndf_waypoint *w;
  double dist;
  int which;
  
  dgc_kdtree_nearest_neighbor(rndf_kdtree, x, y, &which, &dist);
  w = rndf_wp[which];
  return w->heading();
}

void RndfLookup::resize_wp_memory(int num_waypoints)
{
  if(num_waypoints > max_waypoints) {
    max_waypoints = num_waypoints + 10000;
    rndf_wp = (rndf_waypoint **)realloc(rndf_wp, max_waypoints *
					sizeof(rndf_waypoint *));
    dgc_test_alloc(rndf_wp);
    rndf_x = (double *)realloc(rndf_x, max_waypoints * sizeof(double));
    dgc_test_alloc(rndf_x);
    rndf_y = (double *)realloc(rndf_y, max_waypoints * sizeof(double));
    dgc_test_alloc(rndf_y);
  }
}

void RndfLookup::build_rndf_kdtree(rndf_file *rndf)
{
  int num_waypoints = 0;
  int i, j, k, l, l2, n;
  rndf_waypoint *w;
  double alpha;

  rndf_x = NULL; 
  rndf_y = NULL;
  rndf_wp = NULL;
  max_waypoints = 0;
  for(i = 0; i < rndf->num_segments(); i++)
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
	w = rndf->segment(i)->lane(j)->waypoint(k);
	if(w->next() != NULL) {
	  n = (int)floor(w->length() / 1.0);
	  for(l = 0; l <= n; l++) {
	    resize_wp_memory(num_waypoints + 1);
	    alpha = l / (double)(n + 1);
	    rndf_wp[num_waypoints] = w;
	    rndf_x[num_waypoints] = w->utm_x() + 
	      alpha * (w->next()->utm_x() - w->utm_x());
	    rndf_y[num_waypoints] = w->utm_y() + 
	      alpha * (w->next()->utm_y() - w->utm_y());
	    num_waypoints++;
	  }
	}
	else {
	  resize_wp_memory(num_waypoints + 1);
	  rndf_wp[num_waypoints] = w;
	  rndf_x[num_waypoints] = w->utm_x();
	  rndf_y[num_waypoints] = w->utm_y();
	  num_waypoints++;
	}
	for(l = 0; l < w->num_exits(); l++) {
	  n = (int)floor(w->exit_length(l) / 1.0);
	  for(l2 = 0; l2 <= n; l2++) {
	    resize_wp_memory(num_waypoints + 1);
	    alpha = l2 / (double)(n + 1);
	    rndf_wp[num_waypoints] = w;
	    rndf_x[num_waypoints] = w->utm_x() + 
	      alpha * (w->exit(l)->utm_x() - w->utm_x());
	    rndf_y[num_waypoints] = w->utm_y() + 
	      alpha * (w->exit(l)->utm_y() - w->utm_y());
	    num_waypoints++;
	  }
	}
      }
  rndf_kdtree = dgc_kdtree_build_balanced(rndf_x, rndf_y, num_waypoints);
  free(rndf_x);
  free(rndf_y);
}

