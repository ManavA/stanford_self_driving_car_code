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
#include <textures.h>
#include <passatgl.h>
#include <passat_constants.h>
#include "passatmodel.h"
#include "junior-plate.h"
#include "junior2-plate.h"

namespace vlr {
passatmodel_t* passatmodel_load(double r, double g, double b, double t) {
  passatmodel_t* passatmodel;

  passatmodel = (passatmodel_t*) calloc(1, sizeof(passatmodel_t));
  dgc::dgc_test_alloc(passatmodel);
  passatmodel->transparency = t;
  passatmodel->passat_dl = generate_passat(r, g, b, t);
  passatmodel->license_texture.loadFromBytes(junior_plate, 1024, 1);
  return passatmodel;
}

void passatwagonmodel_color(float r, float g, float b);

passatwagonmodel_t* passatwagonmodel_load(double r, double g, double b, double t) {
  passatwagonmodel_t* passatwagonmodel;

  passatwagonmodel = (passatwagonmodel_t*) calloc(1, sizeof(passatwagonmodel_t));
  dgc::dgc_test_alloc(passatwagonmodel);

  passatwagonmodel->transparency = t;
  passatwagonmodel_color(r, g, b);
  passatwagonmodel->passatwagon = generate_passatwagon(t);
  passatwagonmodel->tire = generate_tire(t);
  passatwagonmodel->velodyne = generate_velodyne(t);
  passatwagonmodel->windows = generate_passat_windshield(t);
  passatwagonmodel->license_texture.loadFromBytes(junior2_plate, 1024, 1);
  return passatwagonmodel;
}

void passatmodel_license_plate(passatmodel_t* passatmodel) {
  glDisable(GL_LIGHTING);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, passatmodel->license_texture.glTextureId());
  glBegin(GL_QUADS);
  glColor4f(1, 1, 1, passatmodel->transparency);
  glTexCoord2f(0, 0);
  glVertex3f(0, 0, 0);
  glTexCoord2f(passatmodel->license_texture.maxU(), 0);
  glVertex3f(0, 0.51, 0);
  glTexCoord2f(passatmodel->license_texture.maxU(), passatmodel->license_texture.maxV());
  glVertex3f(0, 0.51, 0.095);
  glTexCoord2f(0, passatmodel->license_texture.maxV());
  glVertex3f(0, 0, 0.095);
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_LIGHTING);
}

void passatmodel_license_plate2(passatwagonmodel_t* passatwagonmodel) {
  glDisable(GL_LIGHTING);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, passatwagonmodel->license_texture.glTextureId());
  glBegin(GL_QUADS);
  glColor4f(1, 1, 1, passatwagonmodel->transparency);
  glTexCoord2f(0, 0);
  glVertex3f(0, 0, 0);
  glTexCoord2f(passatwagonmodel->license_texture.maxU(), 0);
  glVertex3f(0, 0.51 * 0.784, 0);
  glTexCoord2f(passatwagonmodel->license_texture.maxU(), passatwagonmodel->license_texture.maxV());
  glVertex3f(0, 0.51 * 0.784, 0.095);
  glTexCoord2f(0, passatwagonmodel->license_texture.maxV());
  glVertex3f(0, 0, 0.095);
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_LIGHTING);
}

void passatmodel_draw(passatmodel_t* passatmodel) {
  /* draw the license plates */
  glPushMatrix();
  glScalef(5.0, 5.0, 5.0);
  //  glRotatef(90.0, 0, 0, 1);
  glCallList(passatmodel->passat_dl);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(2.51, -0.255, -0.3325);
  passatmodel_license_plate(passatmodel);
  glPopMatrix();
  glPushMatrix();
  glRotatef(180.0, 0, 0, 1);
  glTranslatef(2.41, -0.255, 0.075);
  passatmodel_license_plate(passatmodel);
  glPopMatrix();
}

void passatwagonmodel_draw_model(passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle,
    double wheel_rot_angle, double velodyne_angle, int draw_body, int draw_windows) {

  /* draw the license plates */
  if (draw_body) {
    glPushMatrix();
    glScalef(5.0, 5.0, 5.0);
    glRotatef(180, 0, 0, 1);
    glCallList(passatwagonmodel->passatwagon);

    if (draw_windows) glCallList(passatwagonmodel->windows);

    glPopMatrix();

    // Front Plae
    glPushMatrix();
    //  glRotatef(180.0, 0, 0, 1);
    glTranslatef(2.5, -0.255 * 0.784, -0.45);
    glRotatef(-7.0, 0, 1, 0);
    passatmodel_license_plate2(passatwagonmodel);
    glPopMatrix();

    // Rear plate
    glPushMatrix();
    glRotatef(180.0, 0, 0, 1);
    glTranslatef(2.4, -0.255 * 0.784, -0.265);
    glRotatef(-7.0, 0, 1, 0);
    passatmodel_license_plate2(passatwagonmodel);
    glPopMatrix();

  }

  glScalef(0.65, 0.65, 0.65);

  // Front Right Tire
  glPushMatrix();
  glTranslatef(2.27, -1.17, -0.86);
  glRotatef(dgc::dgc_r2d(wheel_dir_angle), 0, 0, 1);
  glRotatef(-dgc::dgc_r2d(wheel_rot_angle), 0, 1, 0);
  glCallList(passatwagonmodel->tire);
  glPopMatrix();

  // Front Left Tire
  glPushMatrix();
  glTranslatef(2.27, 1.17, -0.86);
  glRotatef(180.0 + dgc::dgc_r2d(wheel_dir_angle), 0, 0, 1);
  glRotatef(dgc::dgc_r2d(wheel_rot_angle), 0, 1, 0);
  glCallList(passatwagonmodel->tire);
  glPopMatrix();

  // Rear Right Tire
  glPushMatrix();
  glTranslatef(-2.05, -1.17, -0.86);
  glRotatef(-dgc::dgc_r2d(wheel_rot_angle), 0, 1, 0);
  glCallList(passatwagonmodel->tire);
  glPopMatrix();

  // Rear Left Tire
  glPushMatrix();
  glTranslatef(-2.05, 1.17, -0.86);
  glRotatef(180, 0, 0, 1);
  glRotatef(dgc::dgc_r2d(wheel_rot_angle), 0, 1, 0);
  glCallList(passatwagonmodel->tire);
  glPopMatrix();

  if (draw_body) {
    glScalef(0.35, 0.35, 0.35);
    // Velodyne laser
    glPushMatrix();
    glTranslatef(-0.34, 0, 3.6);
    glRotatef(dgc::dgc_r2d(velodyne_angle) + 180, 0, 0, 1);
    glCallList(passatwagonmodel->velodyne);
    glPopMatrix();
  }

}

void passatwagonmodel_draw(passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle, double wheel_rot_angle,
    double velodyne_angle) {
  passatwagonmodel_draw_model(passatwagonmodel, wheel_dir_angle, wheel_rot_angle, velodyne_angle, 1, 1);
}

void passatwagonmodel_draw_no_windows(passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle,
    double wheel_rot_angle, double velodyne_angle) {
  passatwagonmodel_draw_model(passatwagonmodel, wheel_dir_angle, wheel_rot_angle, velodyne_angle, 1, 0);
}

void passatwagonwheels_draw(passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle, double wheel_rot_angle) {
  passatwagonmodel_draw_model(passatwagonmodel, wheel_dir_angle, wheel_rot_angle, 0.0, 0, 0);
}

} // namespace vlr
