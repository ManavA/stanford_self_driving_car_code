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


#include <string.h>
#include <cmath>
#include <GL/gl.h>
#include <GL/glut.h>
#include <facelist.h>

void dgc_facelist_new(dgc_facelist_p points, int num_points) {
  points->num_points = num_points;
  points->points = new dgc_polygon_t[points->num_points];
  memset(points->points, 0, points->num_points*sizeof(dgc_polygon_t));
  points->full = 0;
  points->mark = 0;
}

void dgc_facelist_add_color_points(dgc_facelist_p points, dgc_polygon_p poly, int n, double max_size) {
  int i;
  double d1, d2, d3, d4;

  for (i = 0; i < n; i++)
    if (poly[i].v1.range < 40.0 && poly[i].v2.range < 40.0 && poly[i].v3.range < 40.0 && poly[i].v4.range < 40.0) {
      points->points[points->mark] = poly[i];

      d1 = hypot(poly[i].v1.x - poly[i].v2.x, poly[i].v1.y - poly[i].v2.y);
      d2 = hypot(poly[i].v2.x - poly[i].v3.x, poly[i].v2.y - poly[i].v3.y);
      d3 = hypot(poly[i].v3.x - poly[i].v4.x, poly[i].v3.y - poly[i].v4.y);
      d4 = hypot(poly[i].v4.x - poly[i].v1.x, poly[i].v4.y - poly[i].v1.y);
      if (d1 < max_size && d2 < max_size && d3 < max_size && d4 < max_size) points->points[points->mark].fill = 1;
      else points->points[points->mark].fill = 0;

      points->mark++;
      if (points->mark == points->num_points) {
        points->mark = 0;
        points->full = 1;
      }
    }
}

void dgc_facelist_reset(dgc_facelist_p points) {
  points->mark = 0;
  points->full = 0;
}

void dgc_facelist_draw_mesh(dgc_facelist_p points, double origin_x, double origin_y, double origin_z) {
  int i, max;

  if (points->full || points->mark > 0) {
    max = points->full ? points->num_points : points->mark;

    for (i = 0; i < max; i++)
      if (points->points[i].fill) {
        glBegin(GL_QUADS);
        glColor3f(points->points[i].v1.r, points->points[i].v1.g, points->points[i].v1.b);
        glVertex3f(points->points[i].v1.x - origin_x, points->points[i].v1.y - origin_y, points->points[i].v1.z - origin_z);

        glColor3f(points->points[i].v2.r, points->points[i].v2.g, points->points[i].v2.b);
        glVertex3f(points->points[i].v2.x - origin_x, points->points[i].v2.y - origin_y, points->points[i].v2.z - origin_z);

        glColor3f(points->points[i].v3.r, points->points[i].v3.g, points->points[i].v3.b);
        glVertex3f(points->points[i].v3.x - origin_x, points->points[i].v3.y - origin_y, points->points[i].v3.z - origin_z);

        glColor3f(points->points[i].v4.r, points->points[i].v4.g, points->points[i].v4.b);
        glVertex3f(points->points[i].v4.x - origin_x, points->points[i].v4.y - origin_y, points->points[i].v4.z - origin_z);

        glEnd();
      }

  }
}

void dgc_facelist_draw_2D(dgc_facelist_p points, double origin_x, double origin_y) {
  int i, max;

  if (points->full || points->mark > 0) {
    max = points->full ? points->num_points : points->mark;

    for (i = 0; i < max; i++)
      if (points->points[i].fill) {
        glBegin(GL_QUADS);
        glColor3f(points->points[i].v1.r, points->points[i].v1.g, points->points[i].v1.b);
        glVertex2f(points->points[i].v1.x - origin_x, points->points[i].v1.y - origin_y);

        glColor3f(points->points[i].v2.r, points->points[i].v2.g, points->points[i].v2.b);
        glVertex2f(points->points[i].v2.x - origin_x, points->points[i].v2.y - origin_y);

        glColor3f(points->points[i].v3.r, points->points[i].v3.g, points->points[i].v3.b);
        glVertex2f(points->points[i].v3.x - origin_x, points->points[i].v3.y - origin_y);

        glColor3f(points->points[i].v4.r, points->points[i].v4.g, points->points[i].v4.b);
        glVertex2f(points->points[i].v4.x - origin_x, points->points[i].v4.y - origin_y);
        glEnd();
      }

  }
}

