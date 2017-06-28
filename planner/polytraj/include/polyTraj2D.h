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


#ifndef POLYTRAJ2D_H_
#define POLYTRAJ2D_H_

#include <vector>
#include <iostream>

#include <driving_common/Trajectory2D.h>
#include <polyTraj.h>
#include <polyTraj2D.h>
#include <rootTraj.h>

#include <poly_traj_structs.h>

namespace vlr {

class PolyTraj2D {
    GenerationMode generation_mode_;
    double  total_cost_;
    double  max_curvature_;
    double min_velocity_;
    double  max_distance_; // to center line
    double  max_angle_;
    double  max_a_lat_;
    double  max_a_lon_;
    double  min_a_lon_;

    // Debug info:
    int index_lat_;
    int index_lon_;

    driving_common::TrajectoryPoint2D latLongTrajectories2GlobalCordinates(const trajectory_point_1D& lat_traj,
            const trajectory_point_1D& long_traj, const CurvePoint& foot_curve_point ) const;
    static void projectionCondition(const std::vector<CurvePoint>& center_line, const double& s_foot,
        const double& x1, const double& x2, double& f);  

public:
    const PolyTraj* pt_traj_lat_ ;
    const RootTraj* pt_traj_root_ ;
    std::vector<driving_common::TrajectoryPoint2D> trajectory2D_;
    PolyTraj2D( const std::vector<CurvePoint>& center_line, const PolyTraj& traj_lat,
            const RootTraj& root_traj, const GenerationMode mode,
            std::vector<std::vector<trajectory_point_1D> >::const_iterator it_lat_sampled_trajectory,
            int& index_lat, int& index_lon);
    double getInitialJerk() const;
    void calc_extreme_values(const double t_delay);
    //void calc_max_curvature();
    //void calc_min_velocity();
    //void calc_max_dist_to_centerline();
    //void calc_max_angle_to_centerline();
    double get_max_curvature() const    {return max_curvature_;}
    double get_min_velocity() const     {return min_velocity_;}
    double get_max_dist_to_centerline() const     {return max_distance_;}
    double get_max_angle_to_centerline() const     {return max_angle_;}
    double get_max_lon_acceleration() const     {return max_a_lon_;}
    double get_min_lon_acceleration() const     {return min_a_lon_;}
    double get_max_lat_acceleration() const     {return max_a_lat_;}
    double get_total_cost()  const      {return total_cost_;}
    int	   get_index_lat()  const		{return index_lat_;}
    int	   get_index_lon()  const		{return index_lon_;}
    void set_total_cost(double cost)    {total_cost_ = cost;}				// for debug
    void calculateNextInitialStates(const double& t_to_next_step,
        movement_state& next_lat_state,
            movement_state& next_long_state) const;
    driving_common::TrajectoryPoint2D evaluateTrajectoryAtT(const double& t) const;
    driving_common::TrajectoryPoint2D calculateNextStartTrajectoryPoint(const double& t_next) const;
    void evalLatLongTrajectoriesTimeBased(const double& t_act, const double& delta_t,
        trajectory_point_1D& lat_state,
        trajectory_point_1D& long_state) const;
    void evalLatLongTrajectoriesArclengthBased(const double& t_act, 
            const double& delta_t, const double& delta_s,
            trajectory_point_1D& lat_state,
            trajectory_point_1D& long_state) const;
    
    static void globalCordinates2LatLong(const driving_common::TrajectoryPoint2D& tp_2d,
        const std::vector<CurvePoint>& center_line, const GenerationMode& generation_mode,
        movement_state& lat_state, movement_state& lon_state);
    
    void generateNewControlTrajectory(const double& t_current, const double& t_control_horizon,
                                                  const double& control_t_res, std::vector<driving_common::TrajectoryPoint2D>& trajectory) const;

    class CompPolyTraj2D { // in order to sort trajectories
    public:
        bool operator()(const PolyTraj2D &pt1, const PolyTraj2D &pt2) {
            if(pt1.total_cost_ < pt2.total_cost_ )
                return true;
            else
                return false;
        }
    };
    friend std::ostream &operator<< (std::ostream &ostr, const PolyTraj2D &traj2d);
private:
};

} // namespace vlr

#endif // POLYTRAJ2D_H_
