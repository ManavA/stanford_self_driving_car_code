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


#ifndef POLY_TRAJ_H_
#define POLY_TRAJ_H_

#include <vector>

#include <poly_traj_structs.h>

namespace vlr {

//------------------- Definition of a polynomial trajectory ---------------
class PolyTraj{
    std::vector<double> coeff_;
    double te;      // zero-based, ts := 0
    double horizon_;   // zero-based sampling horizon, time or arg-length based
    double x0_;
    //double x_at_sample_horizon_;
    //double t0;      // absolute time
    movement_state end_state; // between 5th order and 3rd order polynomial for fast calcualtion
    double total_cost;
    double jerk_integral;
    double a_min;
    double a_max;

    // Debug info:
    double cost_arg_, cost_jerk_, cost_deviation_;
    double arg_, jerk_, deviation_;

    //double eval_poly(int derivative, double t);
    int calc_end_state();
    //double eval_trajectory(int derivative, double t);
    int calc_min_max_acceleration_of_1D_traj();
    void sample(const double& t_current, double time_resolution);
    void calc_jerk_integral();
    bool operator()(PolyTraj pt1, PolyTraj pt2);

public:
    std::vector<std::vector<trajectory_point_1D> > discrete_traj_points_;
    
    PolyTraj() : coeff_(6) {}

    int generate_5th_order( double x0, double x0_dot, double x0_ddot,
            double x1, double x1_dot, double x1_ddot,
            double t_current, double t, double horizon);
    int generate_4th_order( double x0, double x0_dot, double x0_ddot,
            double x1_dot, double x1_ddot, 
            double t_current, double t, double horizon);
    int eval_trajectory(const double& t, std::vector<double>& eval) const;
    double get_te()                     {return te;}
    double get_dder_min()                  {return a_min;}
    double get_dder_max()                  {return a_max;}
    double get_jerk_integral()          {return jerk_integral;}
    double get_total_cost()  const      {return total_cost;}
    void set_total_cost(double cost)    {total_cost = cost;}
    void set_debug_info( const double& arg, const double& jerk, const double& deviation, const double& cost_arg, const double& cost_jerk, const double& cost_deviation);
    void get_debug_info( double& arg, double& jerk, double& deviation, double& cost_arg, double& cost_jerk, double& cost_deviation) const;
    movement_state get_end_state()      {return end_state;}

    void sampleArgumentEquidistant(const double& t_current, double time_resolution);
    void sampleArgumentSynchronized(const std::vector<std::vector<double> >& sample_ref );

    class CompPolyTraj { // in order to sort the trajectories
    public:
        bool operator()(const PolyTraj &pt1, const PolyTraj &pt2) {
            if(pt1.total_cost < pt2.total_cost )
                return true;
            else
                return false;
        }
    };
};

} // namespace vlr

#endif // POLY_TRAJ_H_
