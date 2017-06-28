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


#include <global.h>
#include <passat_constants.h>
#include <gl_support.h>
#include <passatmodel.h>
#include <applanix/ApplanixPose.h>
#include <driving_common/RadarTarget.h>
#include <driving_common/RadarTargetLRR3.h>
//#include <facelist.h>

#include "graphics.h"

using namespace dgc;

namespace vlr {

RGB crgb[NUM_COLORS] = {
  { 1.0, 1.0, 1.0 },
  { 1.0, 0.0, 0.0 },
  { 0.0, 1.0, 0.0 },
  { 0.0, 0.0, 1.0 },
  { 1.0, 1.0, 0.0 },
  { 0.0, 1.0, 1.0 },
  { 1.0, 0.0, 1.0 },
  { 0.0, 0.0, 0.0 }
};

char* _numf(float v, const char* unit)
{
  static char nstr[20];
  int  val = (int)fabs(v*100);
  snprintf( nstr, 20, "%c%02.2f %s", v<0?'-':'+', val/100.0, unit );
  return(nstr);
}

//void generate_rndf() {
//  rndf_dl = generate_rndf_display_list(*rn, 0.5);
//  rndf_valid = 1;
//  rndf_generate_display_lists = FALSE;
//}
//
//void display_check_camera(ApplanixPose *pose) {
//  static double pan_offset = 0;
//  static double last_x = 0, last_y = 0;
//  static int use_pan_offset = 0;
//  static double last_pan = 0;
//
//  if (show_inside_view) {
//
//    /* INSIDE VIEW */
//
//    if (use_pan_offset) {
//      pan_offset -= last_pan - gui3D.camera_pose.pan;
//    }
//    else {
//      use_pan_offset = 1;
//      pan_offset = 0;
//    }
//    if (pan_offset < -180.0) pan_offset = -180.0;
//    if (pan_offset > 180.0) pan_offset = 180.0;
//    gui3D.camera_pose.pan = 180.0 + dgc_r2d(pose->yaw) + pan_offset;
//    gui3D.camera_pose.tilt = dgc_r2d(pose->pitch);
//    gui3D.camera_pose.distance = 0.10;
//    gui3D.camera_pose.x_offset = 1.80 * cos(pose->yaw) - 0.30 * sin(pose->yaw);
//    gui3D.camera_pose.y_offset = 1.80 * sin(pose->yaw) + 0.30 * cos(pose->yaw);
//    gui3D.camera_pose.z_offset = -0.3;
//    last_pan = gui3D.camera_pose.pan;
//  }
//  else {
//
//    /* REGULAR VIEW */
//
//    pan_offset = 0;
//    use_pan_offset = 0;
//    if (follow_mode) {
//      gui3D.camera_pose.pan = dgc_r2d(pose->yaw) + camera_rot_angle;
//      gui3D.camera_pose.x_offset = 0.0;
//      gui3D.camera_pose.y_offset = 0.0;
//      gui3D.camera_pose.z_offset = 0.0;
//    }
//    else if (!camera_lock) {
//      gui3D.camera_pose.x_offset -= (pose->smooth_x - last_x);
//      gui3D.camera_pose.y_offset -= (pose->smooth_y - last_y);
//    }
//  }
//  last_x = pose->smooth_x;
//  last_y = pose->smooth_y;
//  display_redraw();
//}
//
//void draw_marker(float x, float y, float z, float size) {
//  int i;
//  float f, angle;
//
//  glPushMatrix();
//  glBegin(GL_POLYGON);
//  f = 1.5 * sqrt(gui3D.camera_pose.distance);
//  for (i = 0; i < 24; i++) {
//    angle = i / 24.0 * M_PI * 2;
//    glVertex3f(x + f * size * cos(angle), y + f * size * sin(angle), z);
//  }
//  glEnd();
//  glPopMatrix();
//}

bool drawCircle(double cx, double cy, double cz, double r) {
  double angle;
  glBegin(GL_LINE_LOOP);
  for (int32_t j = 0; j < 100; j++) {
    angle = j / 100.0 * M_PI * 2;
    glVertex3f(cx + r * cos(angle), cy + r * sin(angle), cz);
  }
  glEnd();

  return true;
}

void drawGrid(double center_x, double center_y) {
#define DEPTH_OFFSET -1
int grid_x, grid_y;

glLineWidth(0.5);
glEnable(GL_BLEND);
glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

glEnable(GL_LINE_SMOOTH);
glColor3f(0.4, 0.4, 0.4);
glBegin(GL_LINES);
for (grid_x = -100; grid_x < 100; grid_x++)
  {
  glVertex3f(grid_x - center_x, -100 - center_y, DEPTH_OFFSET);
  glVertex3f(grid_x - center_x, 100 - center_y, DEPTH_OFFSET);
  }
for (grid_y = -100; grid_y < 100; grid_y++)
  {
  glVertex3f(-100 - center_x, grid_y - center_y, DEPTH_OFFSET);
  glVertex3f( 100 - center_x, grid_y - center_y, DEPTH_OFFSET);
  }
glEnd();

glDisable(GL_LINE_SMOOTH);
glDisable(GL_BLEND);
}


void draw_disc(float x, float y, float z, float size) {
  int i;
  float angle;

  glPushMatrix();
  glBegin(GL_POLYGON);
  for (i = 0; i < 24; i++) {
    angle = i / 24.0 * M_PI * 2;
    glVertex3f(x + size * cos(angle), y + size * sin(angle), z);
  }
  glEnd();
  glPopMatrix();
}

void draw_bicyclist( float x, float y, float z, float scale, float direction,
		     float magnitude, RGB rgb, float occ)
{
  float head_size = 0.1*scale;
  float height = 0.8*scale;
  float width = 0.2*scale;

  GLUquadricObj *quadratic;
  quadratic = gluNewQuadric();

  glColor4f( rgb.r, rgb.g, rgb.b, occ );

  glPushMatrix();
  {
    /* Body */
    glTranslatef(x, y, z);
    gluCylinder(quadratic, width, 0.0, height, 32, 32);

    /* Head */
    glPushMatrix();
    glTranslatef(0.0, 0.0, height);
    gluSphere(quadratic, head_size, 32, 32);
    glPopMatrix();

    /* Wheels */
    glPushMatrix();
    glTranslatef(0.0, 0.0, 0.0*scale);
    glRotatef(90, 1, 0, 0);
    glRotatef(dgc_r2d(direction), 0, 1, 0);
    glTranslatef(0.5*scale, 0.0, 0.0);
    glutWireTorus(0.05*scale, 0.25*scale, 50, 50);
    glTranslatef(-0.8*scale, 0.0, 0.0);
    glutWireTorus(0.05*scale, 0.25*scale, 50, 50);
    glPopMatrix();

    /* Handlebars. */
    glPushMatrix();
    glRotatef(dgc_r2d(direction), 0, 0, 1); // x is bike-forward, z is up.
    glTranslatef(0.3*scale, 0.4*scale, 0.35*scale);
    glRotatef(90, 1, 0, 0);
    gluCylinder(quadratic, 0.03*scale, 0.03*scale, 0.8*scale, 32, 32);
    glPopMatrix();

    /* Direction and magnitude of travel */
    glRotatef(dgc_r2d(direction), 0, 0, 1);
    glTranslatef(0.3, 0.0, 0.0);
    glLineWidth(2.0);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(magnitude, 0, 0);
    glEnd();
  }
  glPopMatrix();

  gluDeleteQuadric(quadratic);
}


void drawPedestrian(float x, float y, float z1, float height, float size, float direction, float magnitude,
    RGB rgb, float occ) {
  float head_size = 0.6 * size;

  GLUquadricObj *quadratic;
  quadratic = gluNewQuadric();

  glColor4f(rgb.r, rgb.g, rgb.b, occ);

  glPushMatrix();
  {
    /* Body */
    glTranslatef(x, y, z1);
    gluCylinder(quadratic, size, 0.0, height, 32, 32);

    /* Head */
    glTranslatef(0.0, 0.0, height);
    gluSphere(quadratic, head_size, 32, 32);

    /* Direction and magnitude of travel */
    glRotatef(dgc_r2d(direction), 0, 0, 1);
    glLineWidth(2.0);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(magnitude, 0, 0);
    glEnd();
  }
  glPopMatrix();

  gluDeleteQuadric(quadratic);
}

void draw_obstacle_frame(float x, float y, float z, float l, float w, float direction, float magnitude, RGB rgb) {
  glColor3f(rgb.r, rgb.g, rgb.b);
  glLineWidth(2.0);

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(dgc_r2d(direction), 0, 0, 1);

  glBegin(GL_LINE_LOOP);
  glVertex3f(l / 2, w / 2, 0);
  glVertex3f(l / 2, -w / 2, 0);
  glVertex3f(-l / 2, -w / 2, 0);
  glVertex3f(-l / 2, w / 2, 0);
  glVertex3f(l / 2, 0, 0);
  glVertex3f(l / 2 + magnitude, 0, 0);
  glVertex3f(l / 2, 0, 0);
  glVertex3f(-l / 2, -w / 2, 0);
  glVertex3f(-l / 2, w / 2, 0);
  glEnd();

  glPopMatrix();
}

void draw_cage(double l, double w, double h) {
  glLineWidth(1.0);

  glColor4f(0, 1, 0, 0.25);
  glBegin(GL_TRIANGLES);
  glVertex3f(l * 0.1 - l / 4, -w / 2, 0);
  glVertex3f(l * 0.1 + l / 4, 0, 0);
  glVertex3f(l * 0.1 - l / 4, w / 2, 0);
  glEnd();

  glColor3f(1, 1, 1);

  /* draw car outline */
  glBegin(GL_LINE_LOOP);
  glVertex3f(l / 2, w / 2, 0);
  glVertex3f(l / 2, -w / 2, 0);
  glVertex3f(-l / 2, -w / 2, 0);
  glVertex3f(-l / 2, w / 2, 0);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3f(l / 2, w / 2, h);
  glVertex3f(l / 2, -w / 2, h);
  glVertex3f(-l / 2, -w / 2, h);
  glVertex3f(-l / 2, w / 2, h);
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(l / 2, w / 2, 0);
  glVertex3f(l / 2, w / 2, h);
  glVertex3f(l / 2, -w / 2, 0);
  glVertex3f(l / 2, -w / 2, h);
  glVertex3f(-l / 2, -w / 2, 0);
  glVertex3f(-l / 2, -w / 2, h);
  glVertex3f(-l / 2, w / 2, 0);
  glVertex3f(-l / 2, w / 2, h);
  glEnd();

}

void draw_nline_flag(double x, double y, double w, double h, int num_lines, char **line) {
  int i;
  float dh;

  glPushMatrix();
  glTranslatef(x, y, 0.5);
  glRotatef(0 + 90, 0, 0, 1); // TODO: deal with pan
//  glRotatef(gui3D.camera_pose.pan + 90, 0, 0, 1);
  glBegin(GL_LINE_STRIP);
  glColor3f(0, 0, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 2.0);
  glEnd();
  glTranslatef(-w / 2, 0, 2);
  glBegin(GL_POLYGON);
  glColor4f(1, 1, 1, 0.7);
  glVertex3f(-w / 2.0, 0, 0);
  glVertex3f(-w / 2.0, 0, h);
  glVertex3f(w / 2.0, 0, h);
  glVertex3f(w / 2.0, 0, 0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glColor3f(0, 0, 0);
  glVertex3f(-w / 2.0, 0, 0);
  glVertex3f(-w / 2.0, 0, h);
  glVertex3f(w / 2.0, 0, h);
  glVertex3f(w / 2.0, 0, 0);
  glEnd();

  glColor3f(0, 0, 0);
  glTranslatef(0, -0.1, 0);
  glRotatef(90, 1, 0, 0);

  dh = h / (double) num_lines;
  for (i = 0; i < num_lines; i++)
    if (line[num_lines - i - 1] != NULL) render_stroke_text_2D(-w / 2.0 + 0.20 * dh, (i + 0.2) * dh, GLUT_STROKE_ROMAN,
        0.6 * dh, line[num_lines - i - 1]);
  glPopMatrix();
}

void draw_car_box(double l, double w) {
  glBegin(GL_QUADS);
  glVertex3f(l / 2, w / 2, 0);
  glVertex3f(l / 2, -w / 2, 0);
  glVertex3f(-l / 2, -w / 2, 0);
  glVertex3f(-l / 2, w / 2, 0);
  glEnd();
}

void draw_empty_box(double l, double w) {
  glBegin(GL_LINE_LOOP);
  glVertex3f(l / 2, w / 2, 0);
  glVertex3f(l / 2, -w / 2, 0);
  glVertex3f(-l / 2, -w / 2, 0);
  glVertex3f(-l / 2, w / 2, 0);
  glEnd();
}

void draw_vehicle_cage(double x, double y, double theta, double w, double l, int id, double v, double range,
    int draw_flag) {
  char line1[100], line2[100], line3[100], line4[100];
  char *text[4];

  glLineWidth(2.0);
  glPushMatrix();

  glTranslatef(x, y, 0.3);
  glRotatef(dgc_r2d(theta), 0, 0, 1);

  /* draw car outline */
  glBegin(GL_LINE_LOOP);
  glVertex3f(l / 2, w / 2, -DGC_PASSAT_HEIGHT);
  glVertex3f(l / 2, -w / 2, -DGC_PASSAT_HEIGHT);
  glVertex3f(-l / 2, -w / 2, -DGC_PASSAT_HEIGHT);
  glVertex3f(-l / 2, w / 2, -DGC_PASSAT_HEIGHT);
  glEnd();

  if (draw_flag) {
    glBegin(GL_LINE_LOOP);
    glVertex3f(l / 2, w / 2, 0);
    glVertex3f(l / 2, -w / 2, 0);
    glVertex3f(-l / 2, -w / 2, 0);
    glVertex3f(-l / 2, w / 2, 0);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(l / 2, w / 2, -DGC_PASSAT_HEIGHT);
    glVertex3f(l / 2, w / 2, 0);
    glVertex3f(l / 2, -w / 2, -DGC_PASSAT_HEIGHT);
    glVertex3f(l / 2, -w / 2, 0);
    glVertex3f(-l / 2, -w / 2, -DGC_PASSAT_HEIGHT);
    glVertex3f(-l / 2, -w / 2, 0);
    glVertex3f(-l / 2, w / 2, -DGC_PASSAT_HEIGHT);
    glVertex3f(-l / 2, w / 2, 0);

    glVertex3f(-l / 2, -w / 2, 0);
    glVertex3f(l / 2, 0, 0);
    glVertex3f(-l / 2, w / 2, 0);
    glVertex3f(l / 2, 0, 0);
    glEnd();
  }

  glPopMatrix();

  /* draw info flag */
  if (draw_flag) {
    sprintf(line1, "ID:    %d", id);
    sprintf(line2, "VEL:   %.1f MPH", v);
    sprintf(line3, "RANGE: %.1f m\n", range);
    sprintf(line4, "GAP:   %.1f CL\n", (range - l / 2.0 - DGC_PASSAT_IMU_TO_FA_DIST - DGC_PASSAT_FA_TO_BUMPER_DIST)
        / DGC_PASSAT_LENGTH);
    text[0] = line1;
    text[1] = line2;
    text[2] = line3;
    text[3] = line4;
    draw_nline_flag(x, y, 2.5, 1.0, 2, text);
  }

  glLineWidth(1.0);
}

void draw_cube(float x, float y, float zm, float zp, float w, RGB rgb1, RGB rgb2, float occ) {
  float w2 = w / 2.0;
  float xp = x + w2;
  float xm = x - w2;
  float yp = y + w2;
  float ym = y - w2;

  // top
  glBegin(GL_QUADS);
  glColor4f(rgb2.r, rgb2.g, rgb2.b, occ);
  glVertex3f(xm, yp, zp);
  glVertex3f(xp, yp, zp);
  glVertex3f(xp, ym, zp);
  glVertex3f(xm, ym, zp);
  glEnd();

  // Draw the side faces
  glBegin(GL_TRIANGLE_STRIP);

  glColor4f(rgb2.r, rgb2.g, rgb2.b, occ);
  glVertex3f(xm, yp, zp);

  glColor4f(rgb1.r, rgb1.g, rgb1.b, occ);
  glVertex3f(xm, yp, zm);

  glColor4f(rgb2.r, rgb2.g, rgb2.b, occ);
  glVertex3f(xp, yp, zp);

  glColor4f(rgb1.r, rgb1.g, rgb1.b, occ);
  glVertex3f(xp, yp, zm);

  glColor4f(rgb2.r, rgb2.g, rgb2.b, occ);
  glVertex3f(xp, ym, zp);

  glColor4f(rgb1.r, rgb1.g, rgb1.b, occ);
  glVertex3f(xp, ym, zm);

  glColor4f(rgb2.r, rgb2.g, rgb2.b, occ);
  glVertex3f(xm, ym, zp);

  glColor4f(rgb1.r, rgb1.g, rgb1.b, occ);
  glVertex3f(xm, ym, zm);

  glColor4f(rgb2.r, rgb2.g, rgb2.b, occ);
  glVertex3f(xm, yp, zp);

  glColor4f(rgb1.r, rgb1.g, rgb1.b, occ);
  glVertex3f(xm, yp, zm);

  glEnd();

}

void draw_cube2(float x, float y, float z1, float z2, float w, RGB rgb) {
  float w2 = w / 2.0;

  glColor3f(rgb.r, rgb.g, rgb.b);

  glVertex3f(x - w2, y - w2, z1);
  glVertex3f(x + w2, y - w2, z1);
  glVertex3f(x + w2, y + w2, z1);
  glVertex3f(x - w2, y + w2, z1);

  glVertex3f(x - w2, y - w2, z1);
  glVertex3f(x + w2, y - w2, z1);
  glVertex3f(x + w2, y - w2, z2);
  glVertex3f(x - w2, y - w2, z2);

  glVertex3f(x + w2, y - w2, z1);
  glVertex3f(x + w2, y + w2, z1);
  glVertex3f(x + w2, y + w2, z2);
  glVertex3f(x + w2, y - w2, z2);

  glVertex3f(x + w2, y + w2, z1);
  glVertex3f(x - w2, y + w2, z1);
  glVertex3f(x - w2, y + w2, z2);
  glVertex3f(x + w2, y + w2, z2);

  glVertex3f(x - w2, y - w2, z1);
  glVertex3f(x - w2, y + w2, z1);
  glVertex3f(x - w2, y + w2, z2);
  glVertex3f(x - w2, y - w2, z2);

  glVertex3f(x - w2, y - w2, z2);
  glVertex3f(x + w2, y - w2, z2);
  glVertex3f(x + w2, y + w2, z2);
  glVertex3f(x - w2, y + w2, z2);

  /*
   glColor3f( rgb.r*0.5, rgb.g*0.5, rgb.b*0.5 );
   glLineWidth(0.05);

   glBegin(GL_LINE_LOOP);
   glVertex3f(x-w2, y-w2, z1);
   glVertex3f(x+w2, y-w2, z1);
   glVertex3f(x+w2, y+w2, z1);
   glVertex3f(x-w2, y+w2, z1);
   glEnd();

   glBegin(GL_LINE_LOOP);
   glVertex3f(x-w2, y-w2, z2);
   glVertex3f(x+w2, y-w2, z2);
   glVertex3f(x+w2, y+w2, z2);
   glVertex3f(x-w2, y+w2, z2);
   glEnd();

   glBegin(GL_LINE_LOOP);
   glVertex3f(x-w2, y-w2, z1);
   glVertex3f(x-w2, y-w2, z2);
   glVertex3f(x+w2, y-w2, z2);
   glVertex3f(x+w2, y-w2, z1);
   glEnd();

   glBegin(GL_LINE_LOOP);
   glVertex3f(x+w2, y-w2, z1);
   glVertex3f(x+w2, y-w2, z2);
   glVertex3f(x+w2, y+w2, z2);
   glVertex3f(x+w2, y+w2, z1);
   glEnd();

   glBegin(GL_LINE_LOOP);
   glVertex3f(x+w2, y+w2, z1);
   glVertex3f(x-w2, y+w2, z1);
   glVertex3f(x-w2, y+w2, z2);
   glVertex3f(x+w2, y+w2, z2);
   glEnd();

   glBegin(GL_LINE_LOOP);
   glVertex3f(x-w2, y-w2, z1);
   glVertex3f(x-w2, y+w2, z1);
   glVertex3f(x-w2, y+w2, z2);
   glVertex3f(x-w2, y-w2, z2);
   glEnd();
   */

}

void draw_narrow_arrow(float x, float y, float z, float size) {
  float s_2 = size / 2.0;
  float s_4 = size / 4.0;
  float s_n = size / 20.0;

  glBegin(GL_POLYGON);

  glVertex3f(x + s_2, y + 0.0, z);
  glVertex3f(x + 0.0, y + s_4, z);
  glVertex3f(x + 0.0, y + s_n, z);
  glVertex3f(x - s_2, y + s_n, z);
  glVertex3f(x - s_2, y - s_n, z);
  glVertex3f(x + 0.0, y - s_n, z);
  glVertex3f(x + 0.0, y - s_4, z);
  glVertex3f(x + s_2, y + 0.0, z);

  glEnd();

}

void draw_arrow(float x, float y, float z, float size) {
  float s_2 = size / 2.0;
  float s_4 = size / 4.0;

  glBegin(GL_POLYGON);

  glVertex3f(x + s_2, y + 0.0, z);
  glVertex3f(x + 0.0, y + s_2, z);
  glVertex3f(x + 0.0, y + s_4, z);
  glVertex3f(x - s_2, y + s_4, z);
  glVertex3f(x - s_2, y - s_4, z);
  glVertex3f(x + 0.0, y - s_4, z);
  glVertex3f(x + 0.0, y - s_2, z);
  glVertex3f(x + s_2, y + 0.0, z);

  glEnd();

}

void draw_cross_hair(float x, float y, float z, float size) {
  float s_2 = size / 2.0;
  glBegin(GL_LINES);
  glColor3f(1.0, 1.0, 1.0);
  glVertex3f(x + s_2, y, z);
  glVertex3f(x - s_2, y, z);
  glVertex3f(x, y + s_2, z);
  glVertex3f(x, y - s_2, z);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(x + s_2, y, z);
  glVertex3f(x, y - s_2, z);
  glVertex3f(x - s_2, y, z);
  glVertex3f(x, y + s_2, z);
  glEnd();
  glColor4f(1.0, 0.0, 0.0, 0.3);
  glBegin(GL_POLYGON);
  glVertex3f(x + s_2, y, z);
  glVertex3f(x, y - s_2, z);
  glVertex3f(x - s_2, y, z);
  glVertex3f(x, y + s_2, z);
  glEnd();
}

void draw_star(float x, float y, float z, float size) {
  float s_2 = size / 2.0;
  glBegin(GL_TRIANGLES);
  glVertex3f(x - size, y, z);
  glVertex3f(x + s_2, y - s_2, z);
  glVertex3f(x + s_2, y + s_2, z);
  glVertex3f(x + size, y, z);
  glVertex3f(x - s_2, y - s_2, z);
  glVertex3f(x - s_2, y + s_2, z);
  glEnd();
}

void draw_radar_spot(float x, float y, float z, float size, float height, RGB rgb1, RGB rgb2, float occ) {
  float s_1 = size; // 3.5
  float s_2 = s_1 * .56888; // 1.9911
  float s_3 = s_1 * .70731; // 2.4756
  float s_4 = s_1 * .24254; // 0.8489

  glColor4f(rgb1.r, rgb1.g, rgb1.b, occ);

  glBegin(GL_QUADS);

  glVertex3f(x - s_3, y + s_3, z);
  glVertex3f(x - s_2, y + s_4, z);
  glVertex3f(x - s_2, y + s_4, z + height);
  glVertex3f(x - s_3, y + s_3, z + height);

  glVertex3f(x - s_4, y + s_2, z);
  glVertex3f(x - s_3, y + s_3, z);
  glVertex3f(x - s_3, y + s_3, z + height);
  glVertex3f(x - s_4, y + s_2, z + height);

  glVertex3f(x + 0.0, y + s_1, z);
  glVertex3f(x - s_4, y + s_2, z);
  glVertex3f(x - s_4, y + s_2, z + height);
  glVertex3f(x + 0.0, y + s_1, z + height);

  glVertex3f(x + s_4, y + s_2, z);
  glVertex3f(x + 0.0, y + s_1, z);
  glVertex3f(x + 0.0, y + s_1, z + height);
  glVertex3f(x + s_4, y + s_2, z + height);

  glVertex3f(x + s_3, y + s_3, z);
  glVertex3f(x + s_4, y + s_2, z);
  glVertex3f(x + s_4, y + s_2, z + height);
  glVertex3f(x + s_3, y + s_3, z + height);

  glVertex3f(x + s_2, y + s_4, z);
  glVertex3f(x + s_3, y + s_3, z);
  glVertex3f(x + s_3, y + s_3, z + height);
  glVertex3f(x + s_2, y + s_4, z + height);

  glVertex3f(x + s_1, y + 0.0, z);
  glVertex3f(x + s_2, y + s_4, z);
  glVertex3f(x + s_2, y + s_4, z + height);
  glVertex3f(x + s_1, y + 0.0, z + height);

  glVertex3f(x + s_2, y - s_4, z);
  glVertex3f(x + s_1, y + 0.0, z);
  glVertex3f(x + s_1, y + 0.0, z + height);
  glVertex3f(x + s_2, y - s_4, z + height);

  glVertex3f(x + s_3, y - s_3, z);
  glVertex3f(x + s_2, y - s_4, z);
  glVertex3f(x + s_2, y - s_4, z + height);
  glVertex3f(x + s_3, y - s_3, z + height);

  glVertex3f(x + s_4, y - s_2, z);
  glVertex3f(x + s_3, y - s_3, z);
  glVertex3f(x + s_3, y - s_3, z + height);
  glVertex3f(x + s_4, y - s_2, z + height);

  glVertex3f(x + 0.0, y - s_1, z);
  glVertex3f(x + s_4, y - s_2, z);
  glVertex3f(x + s_4, y - s_2, z + height);
  glVertex3f(x + 0.0, y - s_1, z + height);

  glVertex3f(x - s_4, y - s_2, z);
  glVertex3f(x + 0.0, y - s_1, z);
  glVertex3f(x + 0.0, y - s_1, z + height);
  glVertex3f(x - s_4, y - s_2, z + height);

  glVertex3f(x - s_3, y - s_3, z);
  glVertex3f(x - s_4, y - s_2, z);
  glVertex3f(x - s_4, y - s_2, z + height);
  glVertex3f(x - s_3, y - s_3, z + height);

  glVertex3f(x - s_2, y - s_4, z);
  glVertex3f(x - s_3, y - s_3, z);
  glVertex3f(x - s_3, y - s_3, z + height);
  glVertex3f(x - s_2, y - s_4, z + height);

  glVertex3f(x - s_1, y + 0.0, z);
  glVertex3f(x - s_2, y - s_4, z);
  glVertex3f(x - s_2, y - s_4, z + height);
  glVertex3f(x - s_1, y + 0.0, z + height);

  glVertex3f(x - s_2, y + s_4, z);
  glVertex3f(x - s_1, y + 0.0, z);
  glVertex3f(x - s_1, y + 0.0, z + height);
  glVertex3f(x - s_2, y + s_4, z + height);

  glEnd();

  glColor4f(rgb2.r, rgb2.g, rgb2.b, occ);
  glBegin(GL_POLYGON);
  glVertex3f(x - s_2, y + s_4, z + height);
  glVertex3f(x - s_3, y + s_3, z + height);
  glVertex3f(x - s_4, y + s_2, z + height);
  glVertex3f(x + 0.0, y + s_1, z + height);
  glVertex3f(x + s_4, y + s_2, z + height);
  glVertex3f(x + s_3, y + s_3, z + height);
  glVertex3f(x + s_2, y + s_4, z + height);
  glVertex3f(x + s_1, y + 0.0, z + height);
  glVertex3f(x + s_2, y - s_4, z + height);
  glVertex3f(x + s_3, y - s_3, z + height);
  glVertex3f(x + s_4, y - s_2, z + height);
  glVertex3f(x + 0.0, y - s_1, z + height);
  glVertex3f(x - s_4, y - s_2, z + height);
  glVertex3f(x - s_3, y - s_3, z + height);
  glVertex3f(x - s_2, y - s_4, z + height);
  glVertex3f(x - s_1, y + 0.0, z + height);
  glVertex3f(x - s_2, y + s_4, z + height);
  glEnd();
}

void draw_radar_cal(float x, float y, float z, float size, applanix::ApplanixPose *pose, int radar_num, driving_common::RadarTarget target) {
  char str[4096];
  glPushMatrix();
  {
    glTranslatef(x, y, z);
    glRotatef(270 + dgc_r2d(pose->yaw), 0, 0, 1);
    draw_cross_hair(0, 0, 0, size);
    glColor3f(1.0, 1.0, 1.0);
    sprintf(str, "RADAR: %d", radar_num);
    render_stroke_text_2D(-0.7, -2.5, GLUT_STROKE_ROMAN, 0.3, str);
    sprintf(str, "VEL: %s", _numf(dgc_ms2mph(target.relative_velocity + pose->speed), "mph"));
    render_stroke_text_2D(-1.4, -3.0, GLUT_STROKE_ROMAN, 0.3, str);
    sprintf(str, "M:%s - H:%s", target.measured ? "yes" : "no", target.historical ? "yes" : "no");
    render_stroke_text_2D(-1.2, -3.5, GLUT_STROKE_ROMAN, 0.3, str);
  }
  glPopMatrix();

}
void draw_radar_lrr3_cal(float x, float y, float z, float size, applanix::ApplanixPose* pose, int radar_num,
    driving_common::RadarTargetLRR3 target) {
  char str[4096];
  glPushMatrix();
  {
    glTranslatef(x, y, z);
    glRotatef(270 + dgc_r2d(pose->yaw), 0, 0, 1);
    draw_cross_hair(0, 0, 0, size);
    glColor3f(1.0, 1.0, 1.0);
    sprintf(str, "RADAR: %d", radar_num);
    render_stroke_text_2D(-0.7, -2.5, GLUT_STROKE_ROMAN, 0.3, str);
    sprintf(str, "VEL: %s", _numf(dgc_ms2mph(target.long_relative_velocity), "mph"));
    render_stroke_text_2D(-1.4, -3.0, GLUT_STROKE_ROMAN, 0.3, str);
    sprintf(str, "M:%s - H:%s", target.measured ? "yes" : "no", target.historical ? "yes" : "no");
    render_stroke_text_2D(-1.2, -3.5, GLUT_STROKE_ROMAN, 0.3, str);
  }
  glPopMatrix();

}

void draw_left_turn_signal_outline(float x, float y, float r) {
  double r2 = r / 2.0;
  glPushMatrix();
  glTranslatef(x, y, 0);
  glBegin(GL_LINE_LOOP);
  glVertex2f(-r2, 0);
  glVertex2f(0, r2);
  glVertex2f(0, 0.4 * r2);
  glVertex2f(r2, 0.4 * r2);
  glVertex2f(r2, -0.4 * r2);
  glVertex2f(0, -0.4 * r2);
  glVertex2f(0, -r2);
  glEnd();
  glPopMatrix();
}

void draw_left_turn_signal(float x, float y, float r) {
  double r2 = r / 2.0;
  glPushMatrix();
  glTranslatef(x, y, 0);
  glBegin(GL_POLYGON);
  glVertex2f(-r2, 0);
  glVertex2f(0, r2);
  glVertex2f(0, 0.4 * r2);
  glVertex2f(r2, 0.4 * r2);
  glVertex2f(r2, -0.4 * r2);
  glVertex2f(0, -0.4 * r2);
  glVertex2f(0, -r2);
  glEnd();
  glPopMatrix();
}

void draw_right_turn_signal_outline(float x, float y, float r) {
  double r2 = r / 2.0;
  glPushMatrix();
  glTranslatef(x, y, 0);
  glBegin(GL_LINE_LOOP);
  glVertex2f(r2, 0);
  glVertex2f(0, r2);
  glVertex2f(0, 0.4 * r2);
  glVertex2f(-r2, 0.4 * r2);
  glVertex2f(-r2, -0.4 * r2);
  glVertex2f(0, -0.4 * r2);
  glVertex2f(0, -r2);
  glEnd();
  glPopMatrix();
}

void draw_right_turn_signal(float x, float y, float r) {
  double r2 = r / 2.0;
  glPushMatrix();
  glTranslatef(x, y, 0);
  glBegin(GL_POLYGON);
  glVertex2f(r2, 0);
  glVertex2f(0, r2);
  glVertex2f(0, 0.4 * r2);
  glVertex2f(-r2, 0.4 * r2);
  glVertex2f(-r2, -0.4 * r2);
  glVertex2f(0, -0.4 * r2);
  glVertex2f(0, -r2);
  glEnd();
  glPopMatrix();
}

void draw_pedals(float x, float y, float w, float h, double throttle, double brake) {
  int i;

  if (throttle < 0.0) throttle = 0.0;
  if (throttle > 1.0) throttle = 1.0;
  if (brake < 0.0) brake = 0.0;
  if (brake > 100.0) brake = 100.0;

  glLineWidth(1.0);

  glBegin(GL_LINES);
  for (i = 1; i <= 9; i++) {
    glVertex2f(x, y + h * i / 10.0);
    glVertex2f(x + w, y + h * i / 10.0);
    glVertex2f(x + 2 * w, y + h * i / 10.0);
    glVertex2f(x + 3 * w, y + h * i / 10.0);
  }
  glEnd();

  glLineWidth(2.0);
  glBegin(GL_LINE_LOOP);
  glVertex2f(x, y);
  glVertex2f(x + w, y);
  glVertex2f(x + w, y + h);
  glVertex2f(x, y + h);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex2f(x + 2 * w, y);
  glVertex2f(x + 3 * w, y);
  glVertex2f(x + 3 * w, y + h);
  glVertex2f(x + 2 * w, y + h);
  glEnd();

  glColor3f(0, 1, 0);
  glBegin(GL_POLYGON);
  glVertex2f(x, y);
  glVertex2f(x + w, y);
  glVertex2f(x + w, y + throttle * h);
  glVertex2f(x, y + throttle * h);
  glEnd();
  glColor3f(1, 0, 0);
  glBegin(GL_POLYGON);
  glVertex2f(x + 2 * w, y);
  glVertex2f(x + 3 * w, y);
  glVertex2f(x + 3 * w, y + brake / 100.0 * h);
  glVertex2f(x + 2 * w, y + brake / 100.0 * h);
  glEnd();

}

void draw_steering_wheel(float x, float y, float r, float steering_angle) {
  int i;
  double angle;

  glLineWidth(2.0);
  glBegin(GL_LINE_LOOP);
  for (i = 0; i < 24; i++) {
    angle = i / 24.0 * M_PI * 2;
    glVertex2f(x + 1.1 * r * cos(angle), y + 1.1 * r * sin(angle));
  }
  glEnd();
  glBegin(GL_LINE_LOOP);
  for (i = 0; i < 24; i++) {
    angle = i / 24.0 * M_PI * 2;
    glVertex2f(x + 0.9 * r * cos(angle), y + 0.9 * r * sin(angle));
  }
  glEnd();
  glBegin(GL_LINE_LOOP);
  for (i = 0; i < 24; i++) {
    angle = i / 24.0 * M_PI * 2;
    glVertex2f(x + r / 3.0 * cos(angle), y + r / 3.0 * sin(angle));
  }
  glEnd();
  glBegin(GL_LINES);
  glVertex2f(x + r / 3.0 * cos(steering_angle), y + r / 3.0 * sin(steering_angle));
  glVertex2f(x + 0.9 * r * cos(steering_angle), y + 0.9 * r * sin(steering_angle));
  glVertex2f(x - r / 3.0 * cos(steering_angle), y - r / 3.0 * sin(steering_angle));
  glVertex2f(x - 0.9 * r * cos(steering_angle), y - 0.9 * r * sin(steering_angle));
  glColor3f(1, 0, 0);
  glVertex2f(x + 0.9 * r * cos(steering_angle + M_PI_2), y + 0.9 * r * sin(steering_angle + M_PI_2));
  glVertex2f(x + 1.1 * r * cos(steering_angle + M_PI_2), y + 1.1 * r * sin(steering_angle + M_PI_2));
  glEnd();
  glLineWidth(1.0);
}

//void draw_stickered_passat(dgc_passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle, double wheel_rot_angle, double velodyne_angle, int draw_body) {
//
//  if (draw_body) {
//    glPushMatrix();
//    glRotatef(180.0, 0, 0, 1);
//    glScalef(5.0, 5.0, 5.0);
//    vehicle_gl_models.draw(DGC_STICKERED_PASSAT_MODEL_ID);
//    glPopMatrix();
//  }
//
//  /* draw the license plates */
//  // Front Plae
//  glPushMatrix();
//  //  glRotatef(180.0, 0, 0, 1);
//  glTranslatef(2.5, -0.255 * 0.784, -0.45);
//  glRotatef(-7.0, 0, 1, 0);
//  passatmodel_license_plate2(passatwagonmodel);
//  glPopMatrix();
//
//  // Rear plate
//  glPushMatrix();
//  glRotatef(180.0, 0, 0, 1);
//  glTranslatef(2.4, -0.255 * 0.784, -0.265);
//  glRotatef(-7.0, 0, 1, 0);
//  passatmodel_license_plate2(passatwagonmodel);
//  glPopMatrix();
//
//  glScalef(0.65, 0.65, 0.65);
//
//  // Front Right Tire
//  glPushMatrix();
//  glTranslatef(2.27, -1.17, -0.86);
//  glRotatef(dgc_r2d(wheel_dir_angle), 0, 0, 1);
//  glRotatef(-dgc_r2d(wheel_rot_angle), 0, 1, 0);
//  glCallList(passatwagonmodel->tire);
//  glPopMatrix();
//
//  // Front Left Tire
//  glPushMatrix();
//  glTranslatef(2.27, 1.17, -0.86);
//  glRotatef(180.0 + dgc_r2d(wheel_dir_angle), 0, 0, 1);
//  glRotatef(dgc_r2d(wheel_rot_angle), 0, 1, 0);
//  glCallList(passatwagonmodel->tire);
//  glPopMatrix();
//
//  // Rear Right Tire
//  glPushMatrix();
//  glTranslatef(-2.05, -1.17, -0.86);
//  glRotatef(-dgc_r2d(wheel_rot_angle), 0, 1, 0);
//  glCallList(passatwagonmodel->tire);
//  glPopMatrix();
//
//  // Rear Left Tire
//  glPushMatrix();
//  glTranslatef(-2.05, 1.17, -0.86);
//  glRotatef(180, 0, 0, 1);
//  glRotatef(dgc_r2d(wheel_rot_angle), 0, 1, 0);
//  glCallList(passatwagonmodel->tire);
//  glPopMatrix();
//
//  if (draw_body) {
//    glScalef(0.35, 0.35, 0.35);
//    // Velodyne laser
//    glPushMatrix();
//    glTranslatef(-0.34, 0, 3.6);
//    glRotatef(dgc_r2d(velodyne_angle) + 180, 0, 0, 1);
//    glCallList(passatwagonmodel->velodyne);
//    glPopMatrix();
//  }
//
//}

//void draw_actuators(float steering_angle, float throttle, float brake, char signal, float speed, int p_cte, float cte,
//    int offset_x, int offset_y, float t) {
//  static HSV turn_color = {
//  0.333, 1.0, 1.0
//  };
//  static float turn_color_move = -0.175;
//  RGB rgb1 = {
//  0.0, 0.0, 0.0
//  };
//  char str[4096];
//
//  glColor4f(1, 1, 1, t);
//  draw_steering_wheel(gui3D.window_width - (offset_x + 44), gui3D.window_height - (offset_y + 40), 30, dgc_d2r(steering_angle));
//  glColor4f(1, 1, 1, t);
//  draw_pedals(gui3D.window_width - (offset_x + 122), gui3D.window_height - (offset_y + 105), 12, 100, throttle, brake);
//  if (signal == (char) DGC_PASSAT_TURN_SIGNAL_LEFT) {
//    turn_color.v += turn_color_move;
//    if (turn_color.v < 0.3 || turn_color.v > 1.0) turn_color_move *= -1;
//    rgb1 = dgc_hsv_to_rgb(turn_color);
//    glColor4f(rgb1.r, rgb1.g, rgb1.b, t);
//    draw_left_turn_signal(gui3D.window_width - (offset_x + 63), gui3D.window_height - (offset_y + 90), 30);
//  }
//  else if (signal == (char) DGC_PASSAT_TURN_SIGNAL_RIGHT) {
//    turn_color.v += turn_color_move;
//    if (turn_color.v < 0.3 || turn_color.v > 1.0) turn_color_move *= -1;
//    rgb1 = dgc_hsv_to_rgb(turn_color);
//    glColor4f(rgb1.r, rgb1.g, rgb1.b, t);
//    draw_right_turn_signal(gui3D.window_width - (offset_x + 26), gui3D.window_height - (offset_y + 90), 30);
//  }
//
//  glColor4f(1, 1, 1, t);
//
//  draw_left_turn_signal_outline(gui3D.window_width - (offset_x + 63), gui3D.window_height - (offset_y + 90), 30);
//  draw_right_turn_signal_outline(gui3D.window_width - (offset_x + 26), gui3D.window_height - (offset_y + 90), 30);
//
//  glColor4f(0, 0, 1, 0.7 * t);
//  glBegin(GL_QUADS);
//  glVertex2f(gui3D.window_width - (offset_x + 5), gui3D.window_height - (offset_y + 115));
//  glVertex2f(gui3D.window_width - (offset_x + 130), gui3D.window_height - (offset_y + 115));
//  glVertex2f(gui3D.window_width - (offset_x + 130), gui3D.window_height - (offset_y + 146));
//  glVertex2f(gui3D.window_width - (offset_x + 5), gui3D.window_height - (offset_y + 146));
//  glEnd();
//
//  if (p_cte) {
//    glColor4f(1, 0, 0, 0.7 * t);
//    glBegin(GL_QUADS);
//    glVertex2f(gui3D.window_width - (offset_x + 5), gui3D.window_height - (offset_y + 155));
//    glVertex2f(gui3D.window_width - (offset_x + 130), gui3D.window_height - (offset_y + 155));
//    glVertex2f(gui3D.window_width - (offset_x + 130), gui3D.window_height - (offset_y + 186));
//    glVertex2f(gui3D.window_width - (offset_x + 5), gui3D.window_height - (offset_y + 186));
//    glEnd();
//  }
//
//  glColor4f(1, 1, 1, t);
//  glBegin(GL_LINE_LOOP);
//  glVertex2f(gui3D.window_width - (offset_x + 5), gui3D.window_height - (offset_y + 115));
//  glVertex2f(gui3D.window_width - (offset_x + 130), gui3D.window_height - (offset_y + 115));
//  glVertex2f(gui3D.window_width - (offset_x + 130), gui3D.window_height - (offset_y + 146));
//  glVertex2f(gui3D.window_width - (offset_x + 5), gui3D.window_height - (offset_y + 146));
//  glEnd();
//
//  if (p_cte) {
//    glBegin(GL_LINE_LOOP);
//    glVertex2f(gui3D.window_width - (offset_x + 5), gui3D.window_height - (offset_y + 155));
//    glVertex2f(gui3D.window_width - (offset_x + 130), gui3D.window_height - (offset_y + 155));
//    glVertex2f(gui3D.window_width - (offset_x + 130), gui3D.window_height - (offset_y + 186));
//    glVertex2f(gui3D.window_width - (offset_x + 5), gui3D.window_height - (offset_y + 186));
//    glEnd();
//  }
//  glColor4f(1, 1, 1, t);
//  sprintf(str, "%s", numf(speed, "m/h"));
//  renderBitmapString(gui3D.window_width - (offset_x + 105), gui3D.window_height - (offset_y + 137),
//      GLUT_BITMAP_HELVETICA_18, str);
//  if (p_cte) {
//    sprintf(str, "CTE: %s", _numf(cte, "m"));
//    renderBitmapString(gui3D.window_width - (offset_x + 122), gui3D.window_height - (offset_y + 177),
//        GLUT_BITMAP_HELVETICA_18, str);
//  }
//}

#define G_METER_HISTORY_LENGTH  30

void drawGMeter(float x, float y, float size, float x_acc, float y_acc) {
  static float h[2 * G_METER_HISTORY_LENGTH];
  static int firsttime = 1;

  float s_2 = size / 2.0;
  float s_4 = size / 4.0;
  float s_m = size / 20.0;
  float fac = 1.15;
  int i;

  if (firsttime) {
    for (i = 0; i < G_METER_HISTORY_LENGTH; i++) {
      h[i * 2] = 0.0;
      h[i * 2 + 1] = 0.0;
    }
    firsttime = 0;
  }
  glColor4f(0, 0, 0, 0.6);
  glBegin(GL_QUADS);
  glVertex2f(x - (fac * s_2), y - (fac * s_2));
  glVertex2f(x - (fac * s_2), y + (fac * s_2));
  glVertex2f(x + (fac * s_2), y + (fac * s_2));
  glVertex2f(x + (fac * s_2), y - (fac * s_2));
  glEnd();

  glColor3f(0.7, 0.7, 0.7);
  glLineWidth(2.0);
  glBegin(GL_LINES);
  glVertex2f(x, y - s_2);
  glVertex2f(x, y + s_2);
  glVertex2f(x - s_2, y);
  glVertex2f(x + s_2, y);

  glVertex2f(x - s_m, y + s_2);
  glVertex2f(x + s_m, y + s_2);
  glVertex2f(x - s_m, y + s_4);
  glVertex2f(x + s_m, y + s_4);
  glVertex2f(x - s_m, y - s_2);
  glVertex2f(x + s_m, y - s_2);
  glVertex2f(x - s_m, y - s_4);
  glVertex2f(x + s_m, y - s_4);

  glVertex2f(x + s_4, y - s_m);
  glVertex2f(x + s_4, y + s_m);
  glVertex2f(x + s_2, y - s_m);
  glVertex2f(x + s_2, y + s_m);
  glVertex2f(x - s_4, y - s_m);
  glVertex2f(x - s_4, y + s_m);
  glVertex2f(x - s_2, y - s_m);
  glVertex2f(x - s_2, y + s_m);
  glEnd();
  glPointSize(7.0);
  glBegin(GL_POINTS);
  {
    glColor3f(1, 1, 0);
    glVertex2f(x + y_acc * s_2, y + x_acc * s_2);
    for (i = 0; i < G_METER_HISTORY_LENGTH; i++) {
      glVertex2f(x + h[i * 2 + 1] * s_2, y + h[i * 2] * s_2);
    }
  }
  glEnd();
  glPointSize(13.0);
  glBegin(GL_POINTS);
  {
    glColor3f(1, 0, 0);
    glVertex2f(x + y_acc * s_2, y + x_acc * s_2);
  }
  glEnd();
  for (i = G_METER_HISTORY_LENGTH - 1; i > 0; i--) {
    h[i * 2] = h[(i - 1) * 2];
    h[i * 2 + 1] = h[(i - 1) * 2 + 1];
  }
  h[0] = x_acc;
  h[1] = y_acc;
}

void draw_cte(float error) {
  draw_cross_hair(DGC_PASSAT_IMU_TO_FA_DIST, error, 0.2, 0.2);
}

} // namespace vlr
