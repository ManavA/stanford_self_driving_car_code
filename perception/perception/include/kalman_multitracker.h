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


#ifndef KALMAN_MULTITRACKER_H
#define KALMAN_MULTITRACKER_H

#include <vector>
#include <list>
#include <float.h>
#include <obstacle.h>
#include <tracked_obstacle.h>
#include <linear_kalman_filter/linear_kalman_filter.h>

namespace perception {
//! Expand to include cluster descriptors?
/*class MultiTrackerMeasurement {
 public:
  Eigen::VectorXd centroid_;
  double timestamp_;

  MultiTrackerMeasurement();
};*/


/*class MultiTrackerTrack {
  public:
    MultiTrackerTrack( LinearKalmanFilter kf, boost::shared_ptr<TrackedObstacle> obs);
    LinearKalmanFilter filter;
    boost::shared_ptr<TrackedObstacle> obstacle;
};*/

class KalmanMultiTracker {
 private:
  //! The the next id_ value to assign to a LinearKalmanFilter.
  int next_id_;
  //! Minimum value of the filter's prediction Gaussian (unnormalized) to allow when making correspondences.
  double correspondence_thresh_;
  //! Maximum allowable position uncertainty before deleting a filter.
  double pruning_thresh_;
  //! Measurement matrix common to all tracks.
  Eigen::MatrixXd measurement_matrix_;
  //! Transition matrix common to all tracks.  Modified every update step based on delta_time.
  Eigen::MatrixXd transition_matrix_;
  //! Initial state covariance common to all tracks.
  Eigen::MatrixXd initial_sigma_;
  //! Transition covariance common to all tracks.
  Eigen::MatrixXd transition_covariance_;
  //! Measurement covariance common to all tracks.
  Eigen::MatrixXd measurement_covariance_;
  double current_timestamp_;
  double prev_timestamp_;
  Perception& perception_;

  //! Prune out filters that have high uncertainty.
  void prune( double timestamp );
  void update(const std::vector< boost::shared_ptr<Obstacle> >& measurements, double timestamp);

 public:
  std::list<boost::shared_ptr<TrackedObstacle> >tracks_;

  //! @param initial_position_variance is the variance for both x and y in the initial sigma for new filters.
  KalmanMultiTracker(double correspondence_thresh, double pruning_thresh,
		     double measurement_variance, double position_variance,
		     double velocity_variance, double initial_position_variance,
		     double initial_velocity_variance, Perception& perception);

  //! Computes scores, assigns correspondences, runs prediction and update steps for all filters, spawns new filters, prunes bad ones.
  //! @param timestamp is the time at which the update was made; this can be offset arbitrarily from the measurement timestamps.
  void step(const std::vector<boost::shared_ptr <Obstacle> >& measurements, double timestamp);
};

} // namespace perception
#endif //KALMAN_MULTITRACKER_H
