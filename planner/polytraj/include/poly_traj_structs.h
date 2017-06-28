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


#ifndef POLY_TRAJ_STRUCTS_H_
#define POLY_TRAJ_STRUCTS_H_

#include <inttypes.h>
//#include <driving_common/Trajectory2D.h>

namespace vlr {

struct movement_state  {
    double x;
    double x_der;
    double x_dder;
};

struct trajectory_point_1D {
    double arg;     // argument (e.g. time t or arc length s)
    double x;       // value
    double x_der;   // first derivative with respect to arg
    double x_dder;  // second derivative
    double x_ddder; // third
};

struct velocity_params {
  double t_horizon;
  double a_max;
  double a_min;
  double time_res;
  double time_max;
  double arclength_res;
  double v_res;
  double v_offset_max;
  double v_offset_min;
  double k_t;
  double k_j;
  double k_v;
  double time_sample_res;
};

struct tracking_params {
    double t_horizon;
    double s_res;
    double s_offset_max;
    double s_offset_min;
    double a_max;
    double a_min;
    double time_res;
    double time_max;
    double k_t;
    double k_j;
    double k_s;
    double time_sample_res;
};

struct lanekeeping_params {
    double t_horizon;
    double d_res;
    double d_offset_max;
    double d_offset_min;

    struct {
        double d_ddot_max;
        double d_ddot_min;
        double time_res;
        double time_max;
        double k_t;
        double k_j;
        double k_d;
        double time_sample_res;
    } holon;
    struct {
        double d_pprime_max;
        double d_pprime_min;
        double s_res;
        double s_max;
        double k_s;
        double k_j;
        double k_d;
        double arclength_sample_res;
    } nonhol;
};

struct PolyTraj2D_params {
    double max_curvature;
    double max_center_line_offset;
    double max_center_line_angular_offset;
    double max_lat_acceleration;
    double max_lon_acceleration;
    double min_lon_acceleration;
    double time_delay;
};

struct CurvePoint {
      double s;
      double x;
      double y;
      double theta;
      double kappa;
      double kappa_prime;
      double v;
      double t;
};

class Circle {
public:
  Circle() {}
  Circle(double x_, double y_, double r_) : x(x_), y(y_), r(r_) {}
  virtual ~Circle() {}

  double x, y, r;
};

struct MovingBox {
    double width;
    double length;
    double ref_offset;
    double t;
    double x;
    double y;
    double psi;

    static const uint32_t num_circles_=4;
    Circle circles_[num_circles_];
    Circle circum_circle_;
};

struct InitialStateFakePerception {
      double phi;
      double v;
      double r;
      double x; // center of circle
      double y; // center of circle
};

typedef enum { time_based = 1, arclength_based = 2 } GenerationMode;

} // namespace vlr

#endif // POLY_TRAJ_STRUCTS_H_
