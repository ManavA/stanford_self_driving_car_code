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


#include <kalman_multitracker/kalman_multitracker.h>

using namespace Eigen;
using namespace std;
using boost::shared_ptr;

MultiTrackerMeasurement::MultiTrackerMeasurement() :
  centroid_(VectorXd::Zero(2)),
  timestamp_(-1)
{
}

KalmanMultiTracker::KalmanMultiTracker(double correspondence_thresh, double pruning_thresh,
				       double measurement_variance, double position_variance,
				       double velocity_variance, double initial_position_variance,
				       double initial_velocity_variance) :
  next_id_(0),
  correspondence_thresh_(correspondence_thresh),
  pruning_thresh_(pruning_thresh),
  current_timestamp_(0),
  prev_timestamp_(0)
{
  measurement_matrix_ = MatrixXd::Identity(2, 4);
  transition_matrix_ = MatrixXd::Identity(4, 4);
  measurement_covariance_ = MatrixXd::Identity(2, 2) * measurement_variance;
 
  initial_sigma_ = MatrixXd::Zero(4, 4);
  initial_sigma_(0, 0) = initial_position_variance;
  initial_sigma_(1, 1) = initial_position_variance;
  initial_sigma_(2, 2) = initial_velocity_variance;
  initial_sigma_(3, 3) = initial_velocity_variance;
  
  transition_covariance_ = MatrixXd::Zero(4, 4);
  transition_covariance_(0, 0) = position_variance;
  transition_covariance_(1, 1) = position_variance;
  transition_covariance_(2, 2) = velocity_variance;
  transition_covariance_(3, 3) = velocity_variance;
}

void KalmanMultiTracker::prune() {
  // -- Prune filters with high position uncertainty.
  list<LinearKalmanFilter>::iterator it;
  for(it = filters_.begin(); it != filters_.end(); ++it) {
    // If the position variance is too high, get rid of the track.
    if(it->getPositionUncertainty() > pruning_thresh_) {
      it = filters_.erase(it);
      it--;
    }
  }
}

void KalmanMultiTracker::step(const vector<MultiTrackerMeasurement>& measurements, double timestamp) {
  update(measurements, timestamp);
  prune();
}

void KalmanMultiTracker::update(const vector<MultiTrackerMeasurement>& measurements, double timestamp) {
  prev_timestamp_ = current_timestamp_;
  current_timestamp_ = timestamp;

  vector<bool> matched_measurements(measurements.size(), false);
  vector<bool> matched_filters(filters_.size(), false);
  if(!filters_.empty() && !measurements.empty()) { 
  
    // -- For all filter / measurement pairs, compute the score.
    MatrixXd scores((int)filters_.size(), (int)measurements.size());
    list<LinearKalmanFilter>::iterator it = filters_.begin();
    for(int i = 0; i < (int)filters_.size(); ++i, ++it) {
      LinearKalmanFilter& filter = *it;
      for(int j = 0; j < (int)measurements.size(); ++j) {
	const MultiTrackerMeasurement& measurement = measurements[j];

	// Compute the prediction for the timestamp of this measurement.
	transition_matrix_(0, 2) = measurement.timestamp_ - filter.timestamp_;
	transition_matrix_(1, 3) = measurement.timestamp_ - filter.timestamp_;
	VectorXd mu_bar = (transition_matrix_ * filter.mu_).segment(0, 2);
	MatrixXd sigma_bar = transition_matrix_ * filter.sigma_ * transition_matrix_.transpose() + filter.transition_covariance_;

	// Compute the score for this prediction / measurement pair.
	VectorXd innovation = mu_bar - measurement.centroid_;
	MatrixXd sigma_bar_inv_pos = sigma_bar.block(0, 0, 2, 2).inverse(); // The covariance matrix for the marginal of a Gaussian is just that block of the full covariance matrix.
	scores(i, j) = exp(-0.5 * (innovation.transpose() * sigma_bar_inv_pos * innovation)[0]);
      }
    }
    
    // -- Assigning correspondence in order of decreasing score, and run prediction and update for filters with matched measurements.
    while(true) {
      int f = 0;
      int m = 0;
      double max_score = scores.maxCoeff(&f, &m);
      if(max_score < correspondence_thresh_)
	break;

      // Find the filter with the corresponding measurement.
      // This has bad asymptotic complexity but probably small constant.
      it = filters_.begin();
      advance(it, f);

      // Run the prediction and update step for this delta_time.
      double delta_time = measurements[m].timestamp_ - it->timestamp_;
      transition_matrix_(0, 2) = delta_time;
      transition_matrix_(1, 3) = delta_time;
      it->predict(transition_matrix_, measurements[m].timestamp_);
      it->update(measurements[m].centroid_, measurements[m].timestamp_);

      // Clear the scores from the matched filter and measurement.
      scores.row(f) = VectorXd::Zero(scores.cols());
      scores.col(m) = VectorXd::Zero(scores.rows());
      matched_filters[f] = true;
      matched_measurements[m] = true;
    }
  }

  list<LinearKalmanFilter>::iterator it = filters_.begin();
  for(size_t i = 0; i < matched_filters.size(); ++i, ++it) {
    if(matched_filters[i])
      continue;

    transition_matrix_(0, 2) = current_timestamp_ - prev_timestamp_;
    transition_matrix_(1, 3) = current_timestamp_ - prev_timestamp_;
    it->predict(transition_matrix_, it->timestamp_ + current_timestamp_ - prev_timestamp_);
  }    
     
  // -- For all remaining measurements, spawn new filters.
  for(size_t i = 0; i < matched_measurements.size(); ++i) {
    if(matched_measurements[i])
      continue;

    VectorXd initial_state = VectorXd::Zero(4);
    initial_state.segment(0, 2) = measurements[i].centroid_;
    LinearKalmanFilter new_kf(next_id_, measurements[i].timestamp_, initial_state, initial_sigma_, measurement_matrix_,
			      transition_covariance_, measurement_covariance_);
    filters_.push_front(new_kf);
    ++next_id_;
  }
}
