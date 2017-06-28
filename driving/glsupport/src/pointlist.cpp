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
#include <pointlist.h>

void dgc_pointlist_new(dgc_pointlist_p points, int num_points) {
  points->num_points = num_points;
  points->points = new dgc_pl_point3D_t[points->num_points];
  memset(points->points, 0, points->num_points*sizeof(dgc_pl_point3D_t));
  points->full = 0;
  points->mark = 0;
}

void dgc_pointlist_add_point(dgc_pointlist_p points, double x, double y, double z, double r) {
  points->points[points->mark].x = x;
  points->points[points->mark].y = y;
  points->points[points->mark].z = z;
  points->points[points->mark].r = r;
  points->mark++;
  if (points->mark == points->num_points) {
    points->mark = 0;
    points->full = 1;
  }
}

void dgc_pointlist_add_point_wcost(dgc_pointlist_p points, double x, double y, double z, double cost) {
  points->points[points->mark].x = x;
  points->points[points->mark].y = y;
  points->points[points->mark].z = z;
  points->points[points->mark].cost = cost;
  points->mark++;
  if (points->mark == points->num_points) {
    points->mark = 0;
    points->full = 1;
  }
}

void dgc_pointlist_add_points(dgc_pointlist_p points, double *x, double *y, double *z, int n) {
  int i;

  for (i = 0; i < n; i++) {
    points->points[points->mark].x = x[i];
    points->points[points->mark].y = y[i];
    points->points[points->mark].z = z[i];
    points->mark++;
    if (points->mark == points->num_points) {
      points->mark = 0;
      points->full = 1;
    }
  }
}

void dgc_pointlist_add_clipped_points(dgc_pointlist_p points, double *x, double *y, double *z, float *range, int n, double min_range, double max_range) {
  int i;

  for (i = 0; i < n; i++)
    if (range[i] > min_range && range[i] < max_range) {
      points->points[points->mark].x = x[i];
      points->points[points->mark].y = y[i];
      points->points[points->mark].z = z[i];
      points->mark++;
      if (points->mark == points->num_points) {
        points->mark = 0;
        points->full = 1;
      }
    }
}

void dgc_pointlist_reset(dgc_pointlist_p points) {
  points->mark = 0;
  points->full = 0;
}

void dgc_pointlist_draw_circles(dgc_pointlist_p points, double r, double g, double b, double origin_x, double origin_y, double origin_z) {
  int i, j, max;
  double angle;

  if (points->full || points->mark > 0) {
    max = points->full ? points->num_points : points->mark;
    glColor3f(r, g, b);
    for (i = 0; i < max; i++) {
      glBegin(GL_LINE_LOOP);
      for (j = 0; j < 10; j++) {
        angle = j / 10.0 * 2 * M_PI;
        glVertex3f(points->points[i].x + points->points[i].r * cos(angle) - origin_x, points->points[i].y + points->points[i].r * sin(angle) - origin_y,
            points->points[i].z - origin_z);
      }
      glEnd();
    }
  }
}

void dgc_pointlist_draw_circles2D(dgc_pointlist_p points, double r, double g, double b, double origin_x, double origin_y) {
  int i, j, max;
  double angle;

  if (points->full || points->mark > 0) {
    max = points->full ? points->num_points : points->mark;
    glColor3f(r, g, b);
    for (i = 0; i < max; i++) {
      glBegin(GL_LINE_LOOP);
      for (j = 0; j < 10; j++) {
        angle = j / 10.0 * 2 * M_PI;
        glVertex2f(points->points[i].x + points->points[i].r * cos(angle) - origin_x, points->points[i].y + points->points[i].r * sin(angle) - origin_y);
      }
      glEnd();
    }
  }
}

void dgc_pointlist_draw(dgc_pointlist_p points, double r, double g, double b, double origin_x, double origin_y, double origin_z) {
  int i, max;

  if (points->full || points->mark > 0) {
    max = points->full ? points->num_points : points->mark;
    glColor3f(r, g, b);
    glBegin(GL_POINTS);
    for (i = 0; i < max; i++)
      glVertex3f(points->points[i].x - origin_x, points->points[i].y - origin_y, points->points[i].z - origin_z);
    glEnd();
  }
}

void dgc_pointlist_draw_costs(dgc_pointlist_p points, double r1, double g1, double b1, double r2, double g2, double b2) {
  int i, max;
  double frac, r, g, b;

  if (points->full || points->mark > 0) {
    max = points->full ? points->num_points : points->mark;
    glBegin(GL_POINTS);
    for (i = 0; i < max; i++) {
      frac = points->points[i].cost;
      r = r1 + frac * (r2 - r1);
      g = g1 + frac * (g2 - g1);
      b = b1 + frac * (b2 - b1);
      glColor3f(r, g, b);
      glVertex3f(points->points[i].x, points->points[i].y, points->points[i].z);
    }
    glEnd();
  }
}

void dgc_pointlist_draw_partial(dgc_pointlist_p points, int mark) {
  int i;

  if (points->full) {
    i = points->mark;
    glBegin(GL_POINTS);
    while (i != mark) {
      glVertex3f(points->points[i].x, points->points[i].y, points->points[i].z);
      i++;
      if (i == points->num_points) i = 0;
    }
    glEnd();
  }
  else {
    glBegin(GL_POINTS);
    for (i = 0; i < mark; i++)
      glVertex3f(points->points[i].x, points->points[i].y, points->points[i].z);
    glEnd();
  }
}

