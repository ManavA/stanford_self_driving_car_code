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
#include <gl_support.h>
#include <textures.h>

#include "bluebk.h"
#include "bluedn.h"
#include "blueup.h"
#include "blueft.h"
#include "bluelt.h"
#include "bluert.h"

namespace vlr {

static int skybox_initialized = 0;
static GLint skybox_lid;

void generate_skybox(void) {
  static Texture skybox_texture[6];

  skybox_lid = glGenLists(1);
  glNewList(skybox_lid, GL_COMPILE);

  skybox_texture[0].loadFromBytes(blueup_data, 256, 1);
  skybox_texture[1].loadFromBytes(bluedn_data, 256, 1);
  skybox_texture[2].loadFromBytes(bluert_data, 256, 1);
  skybox_texture[3].loadFromBytes(bluelt_data, 256, 1);
  skybox_texture[4].loadFromBytes(blueft_data, 256, 1);
  skybox_texture[5].loadFromBytes(bluebk_data, 256, 1);

  // djoubert187 _at_ hotmail.com
  glEnable(GL_TEXTURE_2D);
  glColor3f(1, 1, 1);

  // If you have border issues change this to 1.005f
  float r = -1.0;

  // Common Axis Z - FRONT Side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[4].glTextureId());
  glBegin(GL_QUADS);
  glTexCoord2f(1, 0);
  glVertex3f(-r, 1.0, -r);
  glTexCoord2f(1, -1);
  glVertex3f(-r, 1.0, r);
  glTexCoord2f(0, -1);
  glVertex3f(r, 1.0, r);
  glTexCoord2f(0, 0);
  glVertex3f(r, 1.0, -r);
  glEnd();

  // Common Axis Z - BACK side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[5].glTextureId());
  glBegin(GL_QUADS);
  glTexCoord2f(1, 0);
  glVertex3f(-r, -1.0, -r);
  glTexCoord2f(1, -1);
  glVertex3f(-r, -1.0, r);
  glTexCoord2f(0, -1);
  glVertex3f(r, -1.0, r);
  glTexCoord2f(0, 0);
  glVertex3f(r, -1.0, -r);
  glEnd();

  // Common Axis X - Left side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[3].glTextureId());
  glBegin(GL_QUADS);
  glTexCoord2f(1, -1);
  glVertex3f(-1.0, -r, r);
  glTexCoord2f(0, -1);
  glVertex3f(-1.0, r, r);
  glTexCoord2f(0, 0);
  glVertex3f(-1.0, r, -r);
  glTexCoord2f(1, 0);
  glVertex3f(-1.0, -r, -r);
  glEnd();

  // Common Axis X - Right side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[2].glTextureId());
  glBegin(GL_QUADS);
  glTexCoord2f(1, -1);
  glVertex3f(1.0, -r, r);
  glTexCoord2f(0, -1);
  glVertex3f(1.0, r, r);
  glTexCoord2f(0, 0);
  glVertex3f(1.0, r, -r);
  glTexCoord2f(1, 0);
  glVertex3f(1.0, -r, -r);
  glEnd();

  // Common Axis Y - Draw Up side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[0].glTextureId());
  glBegin(GL_QUADS);
  glTexCoord2f(0, 0);
  glVertex3f(r, -r, 1);
  glTexCoord2f(-1, 0);
  glVertex3f(r, r, 1);
  glTexCoord2f(-1, -1);
  glVertex3f(-r, r, 1);
  glTexCoord2f(0, -1);
  glVertex3f(-r, -r, 1);
  glEnd();

  // Common Axis Y - Down side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[1].glTextureId());
  glBegin(GL_QUADS);
  glTexCoord2f(0, 0);
  glVertex3f(r, -r, -1);
  glTexCoord2f(1, 0);
  glVertex3f(r, r, -1);
  glTexCoord2f(1, -1);
  glVertex3f(-r, r, -1);
  glTexCoord2f(0, -1);
  glVertex3f(-r, -r, -1);
  glEnd();
  glDisable(GL_TEXTURE_2D);

  glEndList();
}

void draw_skybox(double r, double z_flat) {
  if (!skybox_initialized) {
    generate_skybox();
    skybox_initialized = 1;
  }

  glPushMatrix();
  glScalef(r, r, r);
  glDisable(GL_LIGHTING);
  glCallList(skybox_lid);
  glPopMatrix();

  glColor3f(0.5, 0.5, 0.5);
  glBegin(GL_QUADS);
  glVertex3f(-r, -r, z_flat);
  glVertex3f(r, -r, z_flat);
  glVertex3f(r, r, z_flat);
  glVertex3f(-r, r, z_flat);
  glEnd();
}

} // namespace vlr
