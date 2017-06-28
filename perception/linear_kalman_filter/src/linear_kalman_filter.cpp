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


#include <linear_kalman_filter/linear_kalman_filter.h>

using namespace Eigen;
using boost::shared_ptr;

LinearKalmanFilter::LinearKalmanFilter(int id, double timestamp,
				       const Eigen::VectorXd& initial_state,
				       const Eigen::MatrixXd& initial_sigma,
				       const Eigen::MatrixXd& measurement_matrix,		     
				       const Eigen::MatrixXd& transition_covariance,
				       const Eigen::MatrixXd& measurement_covariance) :
  id_(id),
  timestamp_(timestamp),
  mu_(initial_state),
  sigma_(initial_sigma),
  measurement_matrix_(measurement_matrix),
  transition_covariance_(transition_covariance),
  measurement_covariance_(measurement_covariance),
  kalman_gain_(MatrixXd::Zero(initial_state.rows(), measurement_covariance.cols()))
{
}


void LinearKalmanFilter::predict(const Eigen::MatrixXd& transition_matrix, double timestamp) {
  mu_ = transition_matrix * mu_;
  sigma_ = transition_matrix * sigma_ * transition_matrix.transpose() + transition_covariance_;
  timestamp_ = timestamp;
}

void LinearKalmanFilter::update(const Eigen::VectorXd& measurement, double timestamp) {
  MatrixXd tmp = measurement_matrix_ * sigma_ * measurement_matrix_.transpose() + measurement_covariance_;
  kalman_gain_ = sigma_ * measurement_matrix_.transpose() * tmp.inverse();
  mu_ = mu_ + kalman_gain_ * (measurement - measurement_matrix_ * mu_);
  sigma_ = (MatrixXd::Identity(mu_.rows(), mu_.rows()) - kalman_gain_ * measurement_matrix_) * sigma_;
  timestamp_ = timestamp;
}

double LinearKalmanFilter::getPositionUncertainty() {
  return sigma_(0, 0) * sigma_(1, 1);
}
