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


#ifndef LINEAR_KALMAN_FILTER_H
#define LINEAR_KALMAN_FILTER_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <boost/shared_ptr.hpp>

//! Variable names reflect symbols used in the Kalman filter in Probabilistic Robotics.
class LinearKalmanFilter {
 public:
  int id_;
  //! The time associated with the most recent prediction or update of the filter.
  double timestamp_;
  //! The state mean.
  Eigen::VectorXd mu_;
  //! The state covariance.
  Eigen::MatrixXd sigma_;
  Eigen::MatrixXd measurement_matrix_;
  Eigen::MatrixXd transition_covariance_;
  Eigen::MatrixXd measurement_covariance_;
  Eigen::MatrixXd kalman_gain_;

  LinearKalmanFilter(int id, double timestamp,
		     const Eigen::VectorXd& initial_state,
		     const Eigen::MatrixXd& initial_sigma,
		     const Eigen::MatrixXd& measurement_matrix,		     
		     const Eigen::MatrixXd& transition_covariance,
		     const Eigen::MatrixXd& measurement_covariance);

  //! Returns the product of the x and y position variances.
  double getPositionUncertainty();
  
  //! Estimates the next state of the tracked object based on the transition matrix.
  //! Lines 2 and 3 of Kalman_filter algorithm in Probabilistic Robotics, Chapter 3.
  void predict(const Eigen::MatrixXd& transition_matrix, double timestamp);

  //! Estimates the state of the tracked object based on a given measurement.
  //! Lines 4-7 of Kalman_filter algorithm in Probabilistic Robotics, Chapter 3.
  void update(const Eigen::VectorXd& measurement, double timestamp);
};
  
#endif //LINEAR_KALMAN_FILTER_H
