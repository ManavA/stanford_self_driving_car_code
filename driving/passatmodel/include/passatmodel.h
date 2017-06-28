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


#ifndef PASSATMODEL_H
#define PASSATMODEL_H

#include <GL/gl.h>
#include <GL/glu.h>

#include <global.h>
#include <textures.h>

namespace vlr {

typedef struct {
  GLint passat_dl;
  Texture license_texture;
  double transparency;
} passatmodel_t;

typedef struct {
  GLint passatwagon, tire, velodyne, windows;
  Texture license_texture;
  double transparency;
} passatwagonmodel_t;

passatmodel_t* passatmodel_load(double r, double g, double b, double t);

void passatmodel_draw(passatmodel_t* passatmodel);

passatwagonmodel_t* passatwagonmodel_load(double r, double g, double b, double t);

void passatwagonmodel_draw(passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle, double wheel_rot_angle,
    double velodyne_angle);

void passatwagonwheels_draw(passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle, double wheel_rot_angle);

void passatmodel_license_plate(passatmodel_t* passatmodel);
void passatmodel_license_plate2(passatwagonmodel_t* passatwagonmodel);

void
passatwagonmodel_draw_no_windows(passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle,
    double wheel_rot_angle, double velodyne_angle);

} // namespace vlr

#endif
