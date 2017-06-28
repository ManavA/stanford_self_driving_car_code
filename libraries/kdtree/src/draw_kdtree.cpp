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


#include "kdtree.h"

#define       N                    100
#define       WINDOW_SIZE          800


point2D random_point(void)
{
  point2D temp;
  
  temp.x = carmen_uniform_random(0, 100);
  temp.y = carmen_uniform_random(0, 100);
  return temp;
}

void random_points(point2D *points, int num_points)
{
  int i;

  for(i = 0; i < num_points; i++) {
    points[i].x = carmen_uniform_random(0, 100);
    points[i].y = carmen_uniform_random(0, 100);
  }
}

void nearest_neighbor_slow(point2D *points, int num_points,
			   point2D target, int *which, float *distance)
{
  int i, min_index = 0;
  float d, min_distance = hypot(points[0].x - target.x,
				points[0].y - target.y);
  
  for(i = 1; i < num_points; i++) {
    d = hypot(points[i].x - target.x, points[i].y - target.y);
    if(d < min_distance) {
      min_distance = d;
      min_index = i;
    }
  }
  *which = min_index;
  *distance = min_distance;
}

void test_nearest_neighbor(kdtree_p kdtree, point2D *points,
			   int num_points)
{
  float distance, distance_true;
  int i, nearest, nearest_true;
  point2D target;
  
  fprintf(stderr, "Testing nearest neighbor algorithm\n");
  for(i = 0; i < num_points; i++) {
    target = random_point();
    kdtree_nearest_neighbor(kdtree, target, &nearest, &distance);
    nearest_neighbor_slow(points, N, target, &nearest_true, &distance_true);
    if(nearest != nearest_true)
      fprintf(stderr, "Mistake on iteration %d\n", i);
  }
  fprintf(stderr, "Done testing nearest neighbor\n");
}

dlist_node_p range_search_slow(point2D *points, int num_points, 
			       point2D target, float max_range)
{
  dlist_node_p list = NULL;
  float distance;
  int i;

  for(i = 0; i < num_points; i++) {
    distance = hypot(target.x - points[i].x, target.y - points[i].y);
    if(distance < max_range)
      dlist_add(&list, i, distance);
  }
  return list;
}

void test_range_search(kdtree_p kdtree, point2D *points, int num_points)
{
  dlist_node_p point_list = NULL, true_point_list = NULL;
  point2D target;
  float range = 10.0;

  fprintf(stderr, "Testing range search algorithm\n");
  target = random_point();
  carmen_time_code(point_list = kdtree_range_search(kdtree, target, range);,
		 "range search");
  dlist_print(point_list);

  true_point_list = range_search_slow(points, num_points, target, range);
  dlist_print(true_point_list);

  dlist_free(&point_list);
  dlist_free(&true_point_list);

  fprintf(stderr, "Done testing range search.\n");
}

int main(void)
{
  //  point2D points[N];
  dgc_kdtree_p kdtree;

  dgc_randomize(argc

  /* pick set of random 2D points */
  random_points(points, N);

  /* build KD tree */
  kdtree = kdtree_build_balanced(points, N);

  /* draw picture of tree to file */
  initialize_graphics();
  draw_kdtree(points, N, kdtree);

  /* test algorithms */
  test_nearest_neighbor(kdtree, points, N);
  test_range_search(kdtree, points, N);

  /* free KD tree */
  kdtree_free(&kdtree);
  return 0;
}
