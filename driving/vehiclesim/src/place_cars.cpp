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


  // TODO: remove or finish porting :-(
#include <ros/ros.h>
#include <lltransform.h>
#include <gui3D.h>
#include <imagery.h>
#include <aw_roadNetwork.h>
#include <rndfgl.h>
#include <multicentral.h>
#include <passat_constants.h>
#include "rndf_lookup.h"

using namespace vlr;
using namespace vlr::rndf;

char *rndf_filename = NULL;
RoadNetwork* rn = NULL, *display_rn = NULL;
RndfLookup *rndf_lookup = NULL;

double origin_x, origin_y;
char utmzone[3];

char *imagery_root;

int last_car_selected = 0;

typedef struct {
  double x, y, lat, lon, theta;
  char utmzone[10];
} pose_t, *pose_p;

pose_p pose = NULL;

inline void draw_vehicle_cage(double x, double y, double theta, int id)
{
  char line[100];
  double w = DGC_PASSAT_WIDTH;
  double l = DGC_PASSAT_LENGTH;

  glLineWidth(2.0);
  glPushMatrix();

  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(theta), 0, 0, 1);

  /* draw car outline */
  glBegin(GL_POLYGON);
  glVertex2f(l / 2, w / 2);
  glVertex2f(l / 2, -w / 2);
  glVertex2f(-l / 2, -w / 2);
  glVertex2f(-l / 2, w / 2);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glColor3f(0, 0, 0);
  glVertex2f(l / 2, w / 2);
  glVertex2f(l / 2, -w / 2);
  glVertex2f(-l / 2, -w / 2);
  glVertex2f(-l / 2, w / 2);
  glEnd();

  glBegin(GL_LINES);
  glVertex2f(-l / 2, -w / 2);
  glVertex2f(l / 2, 0);
  glVertex2f(-l / 2, w / 2);
  glVertex2f(l / 2, 0);
  glEnd();

  glRotatef(-90, 0, 0, 1);
  glColor3f(0, 0, 0);
  sprintf(line, "%d", id + 1);
  render_stroke_text_centered_2D(0, 0, GLUT_STROKE_ROMAN, 1, line);

  glPopMatrix();
  glLineWidth(1.0);
}

void display(void)
{
  int i;
  
  glLineWidth(2);

    // turn on smooth lines
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
  /* clear window */
  glClearColor(1, 1, 1, 1);
  glClear(GL_COLOR_BUFFER_BIT);

  /* draw the aerial imagery */
  dgc_imagery_draw_2D(imagery_root, gui3D.window_width, gui3D.window_height, 
		      gui3D.camera_pose.zoom, origin_x, origin_y, 
		      gui3D.camera_pose.x_offset_2D,
		      gui3D.camera_pose.y_offset_2D, utmzone, 1);

  /* draw the GPS trails */
  glColor3f(1, 0, 0);

  /* draw the RNDF file */
  draw_rndf(*display_rn, 0, 1, 0, 1, 0, 1, origin_x, origin_y, 1.0);

  /* draw the car locations */
  for(i = 0; i < mc->NumCentrals(); i++) {
    glPushMatrix();
    glTranslatef(pose[i].x - origin_x,
                 pose[i].y - origin_y, 0);
    glRotatef(dgc_r2d(pose[i].theta), 0, 0, 1);
    //    glTranslatef(-1.6, 0, 0);
    if(i % 4 == 0)
      glColor3f(1, 0, 0);
    else if(i % 4 == 1)
      glColor3f(1, 1, 1);
    else if(i % 4 == 2)
      glColor3f(0.5, 0.5, 0.5);
    else if(i % 4 == 3)
      glColor3f(0, 0.7, 0);
    draw_vehicle_cage(0, 0, 0, i);
    glPopMatrix();
  }

  /* draw the compass rose - have to do this last */
  dgc_imagery_draw_compass_rose(gui3D.window_width, gui3D.window_height, 
                                gui3D.camera_pose.rotation_2D,
				gui3D.camera_pose.zoom, BOTTOM_LEFT);
}

void timer(int)
{
  int i;

  /* handle IPC messages */
  mc->ReconnectCentrals();
  for(i = 0; i < mc->NumCentrals(); i++) 
    if(mc->Connected(i)) {
      ipc->SetContext(mc->Context(i));
      IPC_handleMessage(0);
      IPC_handleMessage(0);
      IPC_handleMessage(0);
    }

  glutSetWindow(gui3D.window_id);
  if(dgc_imagery_update())
    gui3D_forceRedraw();
  gui3D_add_timerFunc(100, timer, 0);
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"imagery", "root", DGC_PARAM_FILENAME, &imagery_root, 0, NULL},
    {"rndf", "rndf_file", DGC_PARAM_FILENAME, &rndf_filename, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

void get_start_state(ParamInterface *pint, int argc, char **argv, 
		     double *lat, double *lon, double *theta)
{
  Param params[] = {
    {"sim", "vehicle_start_latitude", DGC_PARAM_DOUBLE, lat, 0, NULL},
    {"sim", "vehicle_start_longitude", DGC_PARAM_DOUBLE, lon, 0, NULL},
    {"sim", "vehicle_start_theta", DGC_PARAM_DOUBLE, theta, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

void set_car_position(ParamInterface *pint, int i, double lat, double lon, double theta)
{
  ipc->SetContext(mc->Context(i));
  pint->SetDouble("sim", "vehicle_start_latitude", lat, NULL);
  pint->SetDouble("sim", "vehicle_start_longitude", lon, NULL);
  pint->SetDouble("sim", "vehicle_start_theta", theta, NULL);
  pose[i].lat = lat;
  pose[i].lon = lon;
  pose[i].theta = theta;
  vlr::latLongToUtm(pose[i].lat, pose[i].lon, &pose[i].x, &pose[i].y,
	      pose[i].utmzone);
}

void keyboard(unsigned char key, int x, int y)
{
  double x2, y2, x3, y3;
  double lat, lon, angle;
  char temputmzone[10];

  gui3D_get_2D_position(x, y, &x2, &y2);
  vlr::utmToLatLong(origin_x + x2, origin_y + y2, utmzone, &lat, &lon);

  switch(key) {
  case 'i': case 'I':
    dgc_imagery_cycle_imagery_type();
    break;
  case '1': 
  case '2': 
  case '3': 
  case '4': 
  case '5': 
  case '6': 
  case '7': 
  case '8': 
  case '9': 
    if(key - '1' < mc->NumCentrals()) {
      set_car_position(pint, key - '1', lat, lon, 
		       rndf_lookup->road_angle(origin_x + x2, origin_y + y2));
      last_car_selected = key - '1';
    }
    break;
  case 'd': case 'D': case 'o': case 'O':
    vlr::latLongToUtm(pose[last_car_selected].lat, 
		pose[last_car_selected].lon, &x3, &y3, temputmzone);
    angle = atan2((y2 + origin_y) - y3, (x2 + origin_x) - x3);
    set_car_position(pint, last_car_selected, pose[last_car_selected].lat,
		     pose[last_car_selected].lon, angle);
    break;
  case 27: case 'Q':
    exit(0);
    break;
  }
  gui3D_forceRedraw();
  
  x3 = 0;
  y3 = 0;
  angle = 0;
}

void ipc_exit_handler(void)
{

}

int main(int argc, char **argv)
{
  char temp_filename[200];
  int i;

  /* connect to all IPC servers */
  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);
  mc = new MultiCentral(ipc, true);      // allow zero centrals
  if (mc->Connect(argc, argv) < 0)
    dgc_fatal_error("Could not start multi-central connection.");
  mc->StartMonitoringCentrals(NULL);
  read_parameters(pint, argc, argv);

  /* load the RNDF file */
  rn = new RoadNetwork;
  if(!rn->loadRNDF(rndf_filename))
    dgc_die("Error: could not read RNDF file %s\n", rndf_filename);
  rndf->average_waypoint(&origin_x, &origin_y, utmzone);
  rndf_lookup = new RndfLookup(rndf);

  /*  if(strcmp(rndf_filename + strlen(rndf_filename) - 10, ".srndf.txt") == 0) {
    strcpy(temp_filename, rndf_filename);
    strcpy(temp_filename + strlen(temp_filename) - 10, ".txt");
  }
  else*/
    strcpy(temp_filename, rndf_filename);
  display_rn = new RoadNetwork;
  if(!display_rn->loadRNDF(temp_filename))
    dgc_die("Error: could not read RNDF file %s\n", temp_filename);

  /* get initial positions of the vehicles */
  pose = (pose_p)calloc(mc->NumCentrals(), sizeof(pose_t));
  dgc_test_alloc(pose);
  for(i = 0; i < mc->NumCentrals(); i++) {
    ipc->SetContext(mc->Context(i));
    get_start_state(pint, argc, argv, &pose[i].lat, &pose[i].lon, &pose[i].theta);
    latLongToUtm(pose[i].lat, pose[i].lon, &pose[i].x, &pose[i].y,
		pose[i].utmzone);
  }

  /* prepare graphics */
  gui3D_initialize(argc, argv, 10, 10, 800, 800, 10.0);
  gui3D_setCameraParams(0.2, 0.5, 0.001, 0.5, 30, 1, 400);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_add_timerFunc(100, timer, 0);
  gui3D_set_displayFunc(display);
  gui3D_set_2D_mode();
  gui3D_mainloop();
  return 0;
}
