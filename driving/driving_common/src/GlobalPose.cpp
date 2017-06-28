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


#include <stdio.h>
#include <GlobalPose.h>

namespace driving_common {

GlobalPose::GlobalPose() : smooth_x_(0), smooth_y_(0), smooth_z_(0),
               offset_x_(0), offset_y_(0), offset_z_(0),
               yaw_(0), pitch_(0), roll_(0),
               v_(0), v_lat_(0), v_z_(0), a_(0), a_lat_(0), a_z_(0) {
}

GlobalPose::GlobalPose(double smooth_x, double smooth_y, double yaw) :
               smooth_x_(smooth_x), smooth_y_(smooth_y), smooth_z_(0),
               offset_x_(0), offset_y_(0), offset_z_(0),
               yaw_(yaw), pitch_(0), roll_(0),
               v_(0), v_lat_(0), v_z_(0), a_(0), a_lat_(0), a_z_(0) {
}

GlobalPose::GlobalPose(double smooth_x, double smooth_y, double smooth_z, double yaw, double pitch, double roll) :
               smooth_x_(smooth_x), smooth_y_(smooth_y), smooth_z_(smooth_z),
               offset_x_(0), offset_y_(0), offset_z_(0),
               yaw_(yaw), pitch_(pitch), roll_(roll),
               v_(0), v_lat_(0), v_z_(0), a_(0), a_lat_(0), a_z_(0) {
}

GlobalPose::GlobalPose(double smooth_x, double smooth_y, double offset_x, double offset_y, double yaw) :
               smooth_x_(smooth_x), smooth_y_(smooth_y), smooth_z_(0),
               offset_x_(offset_x), offset_y_(offset_y), offset_z_(0),
               yaw_(yaw), pitch_(0), roll_(0),
               v_(0), v_lat_(0), v_z_(0), a_(0), a_lat_(0), a_z_(0) {
}

GlobalPose::GlobalPose(double smooth_x, double smooth_y, double offset_x, double offset_y,
     double yaw, double pitch, double roll,  double v, double v_lat, double v_z,
     double a, double a_lat, double a_z) :
     smooth_x_(smooth_x), smooth_y_(smooth_y), smooth_z_(0),
     offset_x_(offset_x), offset_y_(offset_y), offset_z_(0),
     yaw_(yaw), pitch_(pitch), roll_(roll),
     v_(v), v_lat_(v_lat), v_z_(v_z), a_(a), a_lat_(a_lat), a_z_(a_z) {
}

GlobalPose::~GlobalPose() {

}

void GlobalPose::dump() {
  printf("smooth: %f, %f\n", smooth_x_, smooth_y_);
  printf("utm: %f, %f\n", smooth_x_+offset_x_, smooth_y_+offset_y_);
  printf("offset: %f, %f\n", offset_x_, offset_y_);
  printf("yaw: %f, pitch: %f, roll: %f\n", yaw_, pitch_, roll_);
  printf("v: %f, v (lat): %f, v (z): %f\n", v_, v_lat_, v_z_);
  printf("a: %f, a (lat): %f, a (z): %f\n", a_, a_lat_, a_z_);
}
} // namespace driving_common
