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


#include <gl_support.h>
#include <paw2Sensor.h>

#define     MAX_INTENSITY              128.0
#define     MIN_DIST_BETWEEN_SCANS     0.1
#define     MAX_FILL_SIZE              5.0
#define     NUM_LINES                  150

namespace vlr {

projected_scan_p scan_alloc(int num_points)
{
  projected_scan_p scan = NULL;

  scan = (projected_scan_p)calloc(1, sizeof(projected_scan_t));
  dgc_test_alloc(scan);
  scan->num_points = num_points;
  scan->point = (dgc_point3D_t*)calloc(num_points, sizeof(dgc_point3D_t));
  dgc_test_alloc(scan->point);
  scan->range = (float *)calloc(num_points, sizeof(float));
  dgc_test_alloc(scan->range);
  scan->intensity = (float *)calloc(num_points, sizeof(float));
  dgc_test_alloc(scan->intensity);
  return scan;
}

void scan_copy(projected_scan_p dest, projected_scan_p src)
{
  int i;

  if(src->num_points != dest->num_points)
    dgc_die("Error: source and destination scans are different sizes.\n");

  for(i = 0; i < src->num_points; i++) {
    dest->point[i] = src->point[i];
    dest->range[i] = src->range[i];
    dest->intensity[i] = src->intensity[i];
  }
}

void add_points(projected_scan_p scan, dgc_pointlist_p pointlist, int start_index, int num_points)
{
  static double *x = NULL, *y = NULL, *z = NULL;
  static float *range = NULL;
  static int n = 0;
  int i;

  if(n < num_points) {
    n = num_points;
    x = (double *)realloc(x, n * sizeof(double));
    dgc_test_alloc(x);
    y = (double *)realloc(y, n * sizeof(double));
    dgc_test_alloc(y);
    z = (double *)realloc(z, n * sizeof(double));
    dgc_test_alloc(z);
    range = (float *)realloc(range, n * sizeof(float));
    dgc_test_alloc(range);
  }

  for(i = start_index; i < (num_points + start_index); i++) {
    x[i - start_index] = scan->point[i].x;
    y[i - start_index] = scan->point[i].y;
    z[i - start_index] = scan->point[i].z;
    range[i - start_index] = scan->range[i];
  }
  dgc_pointlist_add_clipped_points(pointlist, x, y, z, range, num_points, 0.5, 40.0);
}

void add_polygons(projected_scan_p old_scan, projected_scan_p new_scan,
                  dgc_facelist_p facelist, int start_index, int num_points)
{
  static dgc_polygon_p poly = NULL;
  static int n = 0;
  int i;

  if ((num_points > new_scan->num_points) || (num_points > old_scan->num_points))
    dgc_die("Error: add_new_polygons called with wrong sized scans.\n");

  if(n < num_points) {
    n = num_points;
    poly = (dgc_polygon_p)realloc(poly, n * sizeof(dgc_polygon_t));
    dgc_test_alloc(poly);
  }

  for(i = start_index; i < (start_index + num_points - 1); i++) {
    poly[i-start_index].v1.x = new_scan->point[i].x;
    poly[i-start_index].v1.y = new_scan->point[i].y;
    poly[i-start_index].v1.z = new_scan->point[i].z;
    poly[i-start_index].v1.range = new_scan->range[i];
    poly[i-start_index].v1.r = poly[i-start_index].v1.g = poly[i-start_index].v1.b = new_scan->intensity[i];

    poly[i-start_index].v2.x = new_scan->point[i + 1].x;
    poly[i-start_index].v2.y = new_scan->point[i + 1].y;
    poly[i-start_index].v2.z = new_scan->point[i + 1].z;
    poly[i-start_index].v2.range = new_scan->range[i + 1];
    poly[i-start_index].v2.r = poly[i-start_index].v2.g = poly[i-start_index].v2.b = new_scan->intensity[i + 1];

    poly[i-start_index].v3.x = old_scan->point[i + 1].x;
    poly[i-start_index].v3.y = old_scan->point[i + 1].y;
    poly[i-start_index].v3.z = old_scan->point[i + 1].z;
    poly[i-start_index].v3.range = old_scan->range[i + 1];
    poly[i-start_index].v3.r = poly[i-start_index].v3.g = poly[i-start_index].v3.b = old_scan->intensity[i + 1];

    poly[i-start_index].v4.x = old_scan->point[i].x;
    poly[i-start_index].v4.y = old_scan->point[i].y;
    poly[i-start_index].v4.z = old_scan->point[i].z;
    poly[i-start_index].v4.range = old_scan->range[i];
    poly[i-start_index].v4.r = poly[i-start_index].v4.g = poly[i-start_index].v4.b = old_scan->intensity[i];
  }
  dgc_facelist_add_color_points(facelist, poly, num_points, MAX_FILL_SIZE);
}

} // namespace vlr
