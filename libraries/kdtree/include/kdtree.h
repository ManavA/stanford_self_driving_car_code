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


#ifndef DGC_KDTREE_H
#define DGC_KDTREE_H

/* internal kdtree structures */

typedef struct {
  double min_x, min_y, max_x, max_y;
} dgc_hyperrect;

typedef struct {
  int id;
  double x, y;
} dgc_tagged_point;

typedef struct dgc_kdtree_struct {
  char axis;
  double axis_split;
  struct dgc_kdtree_struct *left, *right;
  int num_points, max_points;
  dgc_tagged_point *data;
} dgc_kdtree_t, *dgc_kdtree_p;

/* external structures */

typedef struct dgc_dlist_node_struct {
  double distance;
  int id;
  struct dgc_dlist_node_struct *next;
} dgc_dlist_node_t, *dgc_dlist_node_p;

typedef struct dgc_priority_queue_node {
  int id;
  double distance;
  struct dgc_priority_queue_node *prev, *next;
} dgc_priority_queue_node_t, *dgc_priority_queue_node_p;

typedef struct {
  int length, max_length;
  dgc_priority_queue_node_p first, last;
} dgc_priority_queue_t, *dgc_priority_queue_p;

#ifdef __cplusplus
extern "C" {
#endif

/* linked list functions */

void dgc_dlist_add(dgc_dlist_node_p *list, int id, double distance);

void dgc_dlist_free(dgc_dlist_node_p *list);       /* necessary functions are from
                                              here down */
int dgc_dlist_length(dgc_dlist_node_p list);

void dgc_dlist_print(dgc_dlist_node_p list);

/* kd-tree functions */

dgc_kdtree_p dgc_kdtree_build_even(double max_x, double max_y, 
				   double min_size);

void dgc_even_kdtree_clear(dgc_kdtree_p kdtree);

void dgc_even_kdtree_add_points(dgc_kdtree_p kdtree, double *x, double *y, 
				int num_points);

dgc_kdtree_p dgc_kdtree_build_balanced(double *x, double *y, int num_points);

void dgc_kdtree_nearest_neighbor(dgc_kdtree_p kdtree, double x, double y, 
                                 int *which, double *distance);

dgc_dlist_node_p dgc_kdtree_range_search(dgc_kdtree_p kdtree, 
                                         double x, double y, double max_range);
  
dgc_priority_queue_p dgc_kdtree_k_nearest_neighbors(dgc_kdtree_p kdtree, 
                                                    double x, double y, int k);

void dgc_kdtree_free(dgc_kdtree_p *kdtree);

/* priority queue functions */

void dgc_priority_queue_print(dgc_priority_queue_p queue, int backward);

void dgc_priority_queue_free(dgc_priority_queue_p queue);

#ifdef __cplusplus
}
#endif

#endif
