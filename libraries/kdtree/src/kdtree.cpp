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
#include <global.h>
#include <malloc.h>
#include <string.h>

using namespace dgc;

#define     HUGE_DISTANCE       1.0e10
#define     SQR(x)              ((x)*(x))

#define ELEM_SWAP(a,b) { dgc_tagged_point t=(a);(a)=(b);(b)=t; }

dgc_tagged_point quick_select_median(dgc_tagged_point *arr, int n, 
                                     char dimension) 
{
  int low, high, median, middle, ll, hh;
  
  low = 0; high = n - 1; median = (low + high) / 2;
  for(;;) {
    if(high <= low)        /* One element only */
      return arr[median];
    if(dimension == 'x') {
      if(high == low + 1) {  /* Two elements only */
        if(arr[low].x > arr[high].x)
          ELEM_SWAP(arr[low], arr[high]);
        return arr[median];
      }
    }
    else {
      if(high == low + 1) {  /* Two elements only */
        if(arr[low].y > arr[high].y)
          ELEM_SWAP(arr[low], arr[high]);
        return arr[median];
      }
    }

    /* Find median of low, middle and high items; swap into position low */
    middle = (low + high) / 2;
    if(dimension == 'x') {
      if(arr[middle].x > arr[high].x)    ELEM_SWAP(arr[middle], arr[high]);
      if(arr[low].x > arr[high].x)       ELEM_SWAP(arr[low], arr[high]);
      if(arr[middle].x > arr[low].x)     ELEM_SWAP(arr[middle], arr[low]);
    }
    else {
      if(arr[middle].y > arr[high].y)    ELEM_SWAP(arr[middle], arr[high]);
      if(arr[low].y > arr[high].y)       ELEM_SWAP(arr[low], arr[high]);
      if(arr[middle].y > arr[low].y)     ELEM_SWAP(arr[middle], arr[low]);
    }

    /* Swap low item (now in position middle) into position (low+1) */
    ELEM_SWAP(arr[middle], arr[low + 1]);

    /* Nibble from each end towards middle, swapping items when stuck */
    ll = low + 1;
    hh = high;
    for(;;) {
      if(dimension == 'x') {
        do ll++; while(arr[low].x > arr[ll].x);
        do hh--; while(arr[hh].x > arr[low].x);
      }
      else {
        do ll++; while(arr[low].y > arr[ll].y);
        do hh--; while(arr[hh].y > arr[low].y);
      }
      if(hh < ll)
        break;
      ELEM_SWAP(arr[ll], arr[hh]);
    }

    /* Swap middle item (in position low) back into correct position */
    ELEM_SWAP(arr[low], arr[hh]);
    
    /* Re-set active partition */
    if(hh <= median)
      low = ll;
    if(hh >= median)
      high = hh - 1;
  }
}

inline char opp_dimension(char dimension)
{
  if(dimension == 'x')
    return 'y';
  else
    return 'x';
}

dgc_kdtree_p dgc_kdtree_build_balanced_helper(dgc_tagged_point *points, 
                                              dgc_tagged_point *work,
                                              int min_i, int max_i,
                                              char dimension)
{
  dgc_kdtree_p temp;
  dgc_tagged_point median_value;
  int i, mark, middle;

  temp = (dgc_kdtree_p)calloc(1, sizeof(dgc_kdtree_t));
  dgc_test_alloc(temp);
  temp->axis = dimension;

  if(max_i - min_i <= 50) {
    temp->num_points = max_i - min_i + 1;
    temp->max_points = temp->num_points;
    temp->data = 
      (dgc_tagged_point *)calloc(temp->num_points, sizeof(dgc_tagged_point));
    dgc_test_alloc(temp->data);
    memcpy(temp->data, points + min_i, temp->num_points * 
	   sizeof(dgc_tagged_point));
    temp->left = NULL;
    temp->right = NULL;
    return temp;
  }
  else {
    for(i = min_i; i <= max_i; i++)
      work[i] = points[i];
    median_value = quick_select_median(work + min_i,
                                       max_i - min_i + 1, dimension);
    mark = min_i;
    if(dimension == 'x') {
      temp->axis_split = median_value.x;
      for(i = min_i; i <= max_i; i++)
        if(work[i].x < median_value.x) {
          points[mark] = work[i];
          mark++;
        }
      middle = mark;
      for(i = min_i; i <= max_i; i++) 
        if(work[i].x >= median_value.x) {
          points[mark] = work[i];
          mark++;
        }
    }
    else {
      temp->axis_split = median_value.y;
      for(i = min_i; i <= max_i; i++)
        if(work[i].y < median_value.y) {
          points[mark] = work[i];
          mark++;
        }
      middle = mark;
      for(i = min_i; i <= max_i; i++) 
        if(work[i].y >= median_value.y) {
          points[mark] = work[i];
          mark++;
        }
    }

    temp->data = (dgc_tagged_point *)calloc(1, sizeof(dgc_tagged_point));
    dgc_test_alloc(temp->data);
    temp->data = NULL;
    temp->num_points = 0;
    temp->max_points = 0;

    if(middle - 1 >= min_i) 
      temp->left = dgc_kdtree_build_balanced_helper(points, work,
                                                    min_i, middle - 1,
                                                    opp_dimension(dimension));
    else 
      temp->left = NULL;
    if(max_i >= middle + 1) 
       temp->right = 
         dgc_kdtree_build_balanced_helper(points, work, middle, max_i,
                                          opp_dimension(dimension));
    else
      temp->right = NULL;
    return temp;
  }
}

dgc_kdtree_p dgc_kdtree_build_balanced(double *x, double *y, int num_points)
{
  dgc_tagged_point *points, *work;
  dgc_kdtree_p result;
  int i;

  if(num_points > 0) {
    points = (dgc_tagged_point *)calloc(num_points, sizeof(dgc_tagged_point));
    dgc_test_alloc(points);
    work = (dgc_tagged_point *)calloc(num_points, sizeof(dgc_tagged_point));
    dgc_test_alloc(work);
    for(i = 0; i < num_points; i++) {
      points[i].x = x[i];
      points[i].y = y[i];
      points[i].id = i;
    }
    result = dgc_kdtree_build_balanced_helper(points, work, 0,
                                              num_points - 1, 'x');
    free(points);
    free(work);
    return result;
  }
  else
    return NULL;
}

dgc_kdtree_p dgc_kdtree_build_even_helper(double min_x, double min_y,
					  double max_x, double max_y, 
					  double min_size, char dimension)
{
  dgc_kdtree_p temp;

  temp = (dgc_kdtree_p)calloc(1, sizeof(dgc_kdtree_t));
  dgc_test_alloc(temp);
  temp->axis = dimension;

  temp->num_points = 0;
  temp->max_points = 0;

  if(dimension == 'x' && max_x - min_x > min_size) {
    temp->axis_split  = 0.5 * (max_x + min_x);
    temp->left = dgc_kdtree_build_even_helper(min_x, min_y, 
					      temp->axis_split, max_y, 
					      min_size, 'y');
    temp->right = dgc_kdtree_build_even_helper(temp->axis_split, min_y, 
					       max_x, max_y, min_size, 'y');
  }
  else if(dimension == 'y' && max_y - min_y > min_size) {
    temp->axis_split  = 0.5 * (max_y + min_y);
    temp->left = dgc_kdtree_build_even_helper(min_x, min_y, 
					      max_x, temp->axis_split, 
					      min_size, 'x');
    temp->right = dgc_kdtree_build_even_helper(min_x, temp->axis_split, 
					       max_x, max_y, min_size, 'x');
  }
  else {
    temp->left = NULL;
    temp->right = NULL;
  }
  return temp;
}

dgc_kdtree_p dgc_kdtree_build_even(double max_x, double max_y, double min_size)
{
  return dgc_kdtree_build_even_helper(0, 0, max_x, max_y, min_size, 'x');
}

void dgc_even_kdtree_clear(dgc_kdtree_p kdtree)
{
  if(kdtree != NULL) {
    kdtree->num_points = 0;
    if(kdtree->left)
      dgc_even_kdtree_clear(kdtree->left);
    if(kdtree->right)
      dgc_even_kdtree_clear(kdtree->right);
  }
}

void dgc_even_kdtree_add_points(dgc_kdtree_p kdtree, double *x, double *y, 
				int num_points)
{
  int i;
  dgc_kdtree_p p;

  for(i = 0; i < num_points; i++) {
    p = kdtree;
    while(p->left != NULL || p->right != NULL) {
      if(p->axis == 'x') {
	if(x[i] < p->axis_split)
	  p = p->left;
	else
	  p = p->right;
      }
      else {
	if(y[i] < p->axis_split)
	  p = p->left;
	else
	  p = p->right;
      }
    }
    if(p->num_points == p->max_points) {
      p->max_points += 500;
      //      fprintf(stderr, "realloc cell to %d\n", p->max_points);
      p->data = (dgc_tagged_point *)realloc(p->data, p->max_points *
					    sizeof(dgc_tagged_point));
      dgc_test_alloc(p->data);
    }
    p->data[p->num_points].x = x[i];
    p->data[p->num_points].y = y[i];
    p->data[p->num_points].id = i;
    p->num_points++;
  }
}

void dgc_kdtree_nearest_neighbor_helper(dgc_kdtree_p kdtree, 
                                        double x, double y,
                                        dgc_hyperrect *hr, 
                                        double max_dist_squared,
                                        dgc_tagged_point *nearest, 
                                        double *dist_squared)
{
  dgc_hyperrect left_hr, right_hr, nearer_hr, further_hr;
  dgc_kdtree_p nearer_kd, further_kd;
  double target_in_left;
  double x_dist, y_dist, d;
  dgc_tagged_point temp_nearest;
  double temp_dist_squared;
  int i;

  if(kdtree == NULL) {
    *dist_squared = HUGE_DISTANCE * HUGE_DISTANCE;
    return;
  }
  if(kdtree->left == NULL && kdtree->right == NULL) {
    *dist_squared = max_dist_squared;
    for(i = 0; i < kdtree->num_points; i++) {
      d = SQR(kdtree->data[i].x - x) + SQR(kdtree->data[i].y - y);
      if(d < *dist_squared) {
        *dist_squared = d;
        *nearest = kdtree->data[i];
      }
    }
    return;
  }

  left_hr = *hr;
  right_hr = *hr;
  if(kdtree->axis == 'x') {
    left_hr.max_x = kdtree->axis_split;
    right_hr.min_x = kdtree->axis_split;
    target_in_left = (x < kdtree->axis_split);
  }
  else {
    left_hr.max_y = kdtree->axis_split;
    right_hr.min_y = kdtree->axis_split;
    target_in_left = (y < kdtree->axis_split);
  }

  if(target_in_left) {
    nearer_kd = kdtree->left;
    nearer_hr = left_hr;
    further_kd = kdtree->right;
    further_hr = right_hr;
  }
  else {
    nearer_kd = kdtree->right;
    nearer_hr = right_hr;
    further_kd = kdtree->left;
    further_hr = left_hr;
  }

  dgc_kdtree_nearest_neighbor_helper(nearer_kd, x, y, &nearer_hr, 
                                     max_dist_squared, nearest, dist_squared);
  
  max_dist_squared = std::min(max_dist_squared, *dist_squared);

  if(x > further_hr.max_x)
    x_dist = x - further_hr.max_x;
  else if(x < further_hr.min_x)
    x_dist = further_hr.min_x - x;
  else
    x_dist = 0;

  if(y > further_hr.max_y)
    y_dist = y - further_hr.max_y;
  else if(y < further_hr.min_y)
    y_dist = further_hr.min_y - y;
  else
    y_dist = 0;

  if(x_dist * x_dist < max_dist_squared &&
     y_dist * y_dist < max_dist_squared) {
    temp_dist_squared = HUGE_DISTANCE * HUGE_DISTANCE;
    dgc_kdtree_nearest_neighbor_helper(further_kd, x, y, &further_hr, 
                                       max_dist_squared, &temp_nearest,
                                       &temp_dist_squared);
    if(temp_dist_squared < *dist_squared) {
      *nearest = temp_nearest;
      *dist_squared = temp_dist_squared;
    }
  }
}

void dgc_kdtree_nearest_neighbor(dgc_kdtree_p kdtree, double x, double y, 
                                 int *which, double *distance)
{
  dgc_hyperrect hr = {-HUGE_DISTANCE, -HUGE_DISTANCE,
                      HUGE_DISTANCE, HUGE_DISTANCE};
  dgc_tagged_point nearest;
  
  *distance = HUGE_DISTANCE * HUGE_DISTANCE;
  dgc_kdtree_nearest_neighbor_helper(kdtree, x, y, &hr, HUGE_DISTANCE * 
                                     HUGE_DISTANCE,
                                     &nearest, distance);
  *which = nearest.id;
  *distance = sqrt(*distance);
}
        
void dgc_kdtree_free(dgc_kdtree_p *kdtree)
{
  if(*kdtree == NULL)
    return;
  if((*kdtree)->data != NULL)
    free((*kdtree)->data);
  dgc_kdtree_free(&((*kdtree)->left));
  dgc_kdtree_free(&((*kdtree)->right));
  free(*kdtree);
  *kdtree = NULL;
}

inline void dgc_dlist_add(dgc_dlist_node_p *list, int id, double distance)
{
  dgc_dlist_node_p temp;

  temp = (dgc_dlist_node_p)calloc(1, sizeof(dgc_dlist_node_t));
  temp->id = id;
  temp->distance = distance;
  temp->next = *list;
  *list = temp;
}

void dgc_dlist_free(dgc_dlist_node_p *list)
{
  dgc_dlist_node_p temp;
  
  while(*list != NULL) {
    temp = *list;
    *list = (*list)->next;
    free(temp);
  }
}

inline int dgc_dlist_length(dgc_dlist_node_p list)
{
  int count = 0;
  
  while(list != NULL) {
    count++;
    list = list->next;
  }
  return count;
}

void dgc_dlist_print(dgc_dlist_node_p list)
{
  fprintf(stderr, "Point list : ");
  while(list != NULL) {
    fprintf(stderr, "(%d %.2f) ", list->id, list->distance);
    list = list->next;
  }
  fprintf(stderr, "\n");
}

void dgc_kdtree_range_search_helper(dgc_kdtree_p kdtree, double x, double y, 
                                    dgc_hyperrect *hr, 
                                    double max_range_squared, 
                                    dgc_dlist_node_p *list)
{
  dgc_hyperrect left_hr, right_hr, nearer_hr, further_hr;
  dgc_kdtree_p nearer_kd, further_kd;
  double target_in_left;
  double x_dist, y_dist, d;
  int i;

  if(kdtree == NULL)
    return;
  if(kdtree->left == NULL && kdtree->right == NULL) {
    for(i = 0; i < kdtree->num_points; i++) {
      d = SQR(kdtree->data[i].x - x) + SQR(kdtree->data[i].y - y);
      if(d < max_range_squared)
        dgc_dlist_add(list, kdtree->data[i].id, sqrt(d));
    }
    return;
  }

  left_hr = *hr;
  right_hr = *hr;
  if(kdtree->axis == 'x') {
    left_hr.max_x = kdtree->axis_split;
    right_hr.min_x = kdtree->axis_split;
    target_in_left = (x < kdtree->axis_split);
  }
  else {
    left_hr.max_y = kdtree->axis_split;
    right_hr.min_y = kdtree->axis_split;
    target_in_left = (y < kdtree->axis_split);
  }

  if(target_in_left) {
    nearer_kd = kdtree->left;
    nearer_hr = left_hr;
    further_kd = kdtree->right;
    further_hr = right_hr;
  }
  else {
    nearer_kd = kdtree->right;
    nearer_hr = right_hr;
    further_kd = kdtree->left;
    further_hr = left_hr;
  }

  dgc_kdtree_range_search_helper(nearer_kd, x, y, &nearer_hr, 
                             max_range_squared, list);
  
  if(x > further_hr.max_x)
    x_dist = x - further_hr.max_x;
  else if(x < further_hr.min_x)
    x_dist = further_hr.min_x - x;
  else
    x_dist = 0;

  if(y > further_hr.max_y)
    y_dist = y - further_hr.max_y;
  else if(y < further_hr.min_y)
    y_dist = further_hr.min_y - y;
  else
    y_dist = 0;

  if(x_dist * x_dist < max_range_squared &&
     y_dist * y_dist < max_range_squared) 
    dgc_kdtree_range_search_helper(further_kd, x, y, &further_hr, 
				   max_range_squared, list);
}

dgc_dlist_node_p dgc_kdtree_range_search(dgc_kdtree_p kdtree, 
                                         double x, double y, 
                                         double max_range)
{
  dgc_hyperrect hr = {-HUGE_DISTANCE, -HUGE_DISTANCE, 
                      HUGE_DISTANCE, HUGE_DISTANCE};
  dgc_dlist_node_p list = NULL;
  
  dgc_kdtree_range_search_helper(kdtree, x, y, &hr, SQR(max_range), &list);
  return list;
}

dgc_priority_queue_p dgc_priority_queue_initialize(int max_length)
{
  dgc_priority_queue_p queue;

  queue = (dgc_priority_queue_p)calloc(1, sizeof(dgc_priority_queue_t));
  dgc_test_alloc(queue);
  queue->length = 0;
  queue->max_length = max_length;
  queue->first = NULL;
  queue->last = NULL;
  return queue;
}

inline void dgc_priority_queue_add(dgc_priority_queue_p queue, int id, 
                                   double distance)
{
  dgc_priority_queue_node_p temp, mark;

  if(queue->length == queue->max_length && distance > queue->last->distance)
    return;
  else if(queue->length == 0) {
    temp = 
      (dgc_priority_queue_node_p)calloc(1, sizeof(dgc_priority_queue_node_t));
    dgc_test_alloc(temp);
    temp->id = id;
    temp->distance = distance;
    temp->prev = NULL;
    temp->next = NULL;
    queue->first = temp;
    queue->last = temp;
    queue->length = 1;
  }
  else if(distance > queue->last->distance) {
    temp = 
      (dgc_priority_queue_node_p)calloc(1, sizeof(dgc_priority_queue_node_t));
    dgc_test_alloc(temp);
    temp->id = id;
    temp->distance = distance;
    temp->prev = queue->last;
    temp->prev->next = temp;
    temp->next = NULL;
    queue->last = temp;
    queue->length++;
    return;
  }
  else {
    mark = queue->first;
    while(distance > mark->distance)
      mark = mark->next;
    temp =
      (dgc_priority_queue_node_p)calloc(1, sizeof(dgc_priority_queue_node_t));
    dgc_test_alloc(temp);
    temp->id = id;
    temp->distance = distance;
    temp->prev = mark->prev;
    if(temp->prev != NULL)
      temp->prev->next = temp;
    temp->next = mark;
    temp->next->prev = temp;
    if(mark == queue->first)
      queue->first = temp;
    queue->length++;
    
    if(queue->length > queue->max_length) {
      mark = queue->last;
      queue->last = queue->last->prev;
      queue->last->next = NULL;
      free(mark);
      queue->length--;
    }
  }
}

void dgc_priority_queue_free(dgc_priority_queue_p queue)
{
  dgc_priority_queue_node_p mark;

  while(queue->first != NULL) {
    mark = queue->first;
    queue->first = queue->first->next;
    free(mark);
  }
  free(queue);
}

void dgc_priority_queue_print(dgc_priority_queue_p queue, int backward)
{
  dgc_priority_queue_node_p mark;

  if(backward) {
    fprintf(stderr, "BACKWARD: ");
    mark = queue->last;
    while(mark != NULL) {
      fprintf(stderr, "%6.2f ",  mark->distance);
      mark = mark->prev;
    }
    fprintf(stderr, "\n");
  }
  else {
    fprintf(stderr, "FORWARD:  ");
    mark = queue->first;
    while(mark != NULL) {
      fprintf(stderr, "%6.2f ", mark->distance);
      mark = mark->next;
    }
    fprintf(stderr, "\n");
  }
}

void dgc_kdtree_k_nearest_neighbors_helper(dgc_kdtree_p kdtree, 
                                           double x, double y,
                                           dgc_hyperrect *hr, 
                                           dgc_priority_queue_p queue)
{
  dgc_hyperrect left_hr, right_hr, nearer_hr, further_hr;
  dgc_kdtree_p nearer_kd, further_kd;
  double target_in_left, x_dist, y_dist, d, max_range_squared;
  int i;

  if(kdtree == NULL)
    return;

  if(kdtree->left == NULL && kdtree->right == NULL) {
    for(i = 0; i < kdtree->num_points; i++) {
      d = SQR(kdtree->data[i].x - x) + SQR(kdtree->data[i].y - y);
      if(queue->length < queue->max_length || d < queue->last->distance)
        dgc_priority_queue_add(queue, kdtree->data[i].id, d);
    }
    return;
  }
  
  left_hr = *hr;
  right_hr = *hr;
  if(kdtree->axis == 'x') {
    left_hr.max_x = kdtree->axis_split;
    right_hr.min_x = kdtree->axis_split;
    target_in_left = (x < kdtree->axis_split);
  }
  else {
    left_hr.max_y = kdtree->axis_split;
    right_hr.min_y = kdtree->axis_split;
    target_in_left = (y < kdtree->axis_split);
  }

  if(target_in_left) {
    nearer_kd = kdtree->left;
    nearer_hr = left_hr;
    further_kd = kdtree->right;
    further_hr = right_hr;
  }
  else {
    nearer_kd = kdtree->right;
    nearer_hr = right_hr;
    further_kd = kdtree->left;
    further_hr = left_hr;
  }

  dgc_kdtree_k_nearest_neighbors_helper(nearer_kd, x, y, &nearer_hr, queue);
  
  if(x > further_hr.max_x)
    x_dist = x - further_hr.max_x;
  else if(x < further_hr.min_x)
    x_dist = further_hr.min_x - x;
  else
    x_dist = 0;

  if(y > further_hr.max_y)
    y_dist = y - further_hr.max_y;
  else if(y < further_hr.min_y)
    y_dist = further_hr.min_y - y;
  else
    y_dist = 0;

  max_range_squared = HUGE_DISTANCE;
  if(queue->length == queue->max_length)
    max_range_squared = queue->last->distance;
  if(x_dist * x_dist < max_range_squared &&
     y_dist * y_dist < max_range_squared) 
    dgc_kdtree_k_nearest_neighbors_helper(further_kd, x, y, 
                                          &further_hr, queue);
}

dgc_priority_queue_p dgc_kdtree_k_nearest_neighbors(dgc_kdtree_p kdtree, 
                                                    double x, double y, int k)
{
  dgc_priority_queue_p queue;
  dgc_priority_queue_node_p mark;
  dgc_hyperrect hr = {-HUGE_DISTANCE, -HUGE_DISTANCE, 
                  HUGE_DISTANCE, HUGE_DISTANCE};

  queue = dgc_priority_queue_initialize(k);
  dgc_kdtree_k_nearest_neighbors_helper(kdtree, x, y, &hr, queue);
  mark = queue->first;
  while(mark != NULL) {
    mark->distance = sqrt(mark->distance);
    mark = mark->next;
  }
  return queue;
}
