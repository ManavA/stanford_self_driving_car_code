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


#ifndef DRIVING_COMMON_GLOBAL_POSE_H_
#define DRIVING_COMMON_GLOBAL_POSE_H_

namespace driving_common {

class GlobalPose {
public:
  GlobalPose();
//  GlobalPose(double smooth_x, double smooth_y);
  GlobalPose(double smooth_x, double smooth_y, double yaw);
  GlobalPose(double smooth_x, double smooth_y, double smooth_z, double yaw, double pitch, double roll);
  GlobalPose(double smooth_x, double smooth_y, double offset_x, double offset_y, double yaw);
  GlobalPose(double smooth_x, double smooth_y, double offset_x, double offset_y,
       double yaw, double pitch, double roll,  double v, double v_lat, double v_z,
       double a, double a_lat, double a_z);
//  GlobalPose(double smooth_x, double smooth_y, double smooth_z);

  virtual ~GlobalPose();

  inline double x() const {return smooth_x_;}
  inline double& x() {return smooth_x_;}
  inline double y() const {return smooth_y_;}
  inline double& y() {return smooth_y_;}
  inline double z() const {return smooth_z_;}
  inline double& z() {return smooth_z_;}
  inline double utmX() const {return smooth_x_ + offset_x_;}
  inline double utmY() const {return smooth_y_ + offset_y_;}
  inline double offsetX() const {return offset_x_;}
  inline double& offsetX() {return offset_x_;}
  inline double offsetY() const {return offset_y_;}
  inline double& offsetY() {return offset_y_;}
  inline double offsetZ() const {return offset_z_;}
  inline double& offsetZ() {return offset_z_;}
  inline double yaw() const {return yaw_;}
  inline double& yaw() {return yaw_;}
  inline double pitch() const {return pitch_;}
  inline double& pitch() {return pitch_;}
  inline double roll() const {return roll_;}
  inline double& roll() {return roll_;}
  inline double v() const {return v_;}
  inline double& v() {return v_;}
  inline double vLateral() const {return v_lat_;}
  inline double& vLateral() {return v_lat_;}
  inline double vZ() const {return v_z_;}
  inline double& vZ() {return v_z_;}
  inline double a() const {return a_;}
  inline double& a() {return a_;}
  inline double aLateral() const {return a_lat_;}
  inline double& aLateral() {return a_lat_;}
  inline double aZ() const {return a_z_;}
  inline double& aZ() {return a_z_;}
  void dump();

protected:
  double smooth_x_, smooth_y_, smooth_z_;
  double offset_x_, offset_y_, offset_z_;
  double yaw_, pitch_, roll_;
  double v_, v_lat_, v_z_;
  double a_, a_lat_, a_z_;
};

} // namespace driving_common
#endif
