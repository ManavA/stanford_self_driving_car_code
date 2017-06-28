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


#ifndef TRACKED_OBSTACLE_H_
#define TRACKED_OBSTACLE_H_

#include <global.h>
#include <grid.h>
#include <queue>
#include <vector>
#include <tr1/memory>

#include <obstacle.h>
#include <linear_kalman_filter/linear_kalman_filter.h>

namespace perception {

class Perception;

class TrackedObstacle : public Obstacle {
private:
  int pedestrian_label_count;

//  int observed_, occluded_;
  int num_observations_;

//  double max_speed_;

  double timestamp_first_;       // time of earliest observation
  double timestamp_prediction_;  // time of most recent prediction
  double timestamp_observation_; // time of most recent observation

  double x_velocity_;
  double y_velocity_;
  Eigen::VectorXf prior_log_odds_; //Prior log odds for each class. The indices here correspond to those in booster->class_map_.  
  Eigen::VectorXf log_odds_; //Log odds for each class, updated via incorporateBoostingResponse.

protected:
  virtual void populatePoints();

public:
  boost::shared_ptr<LinearKalmanFilter> filter;
  boost::shared_ptr<Obstacle> lastObservation_;
  TrackedObstacle(int id, boost::shared_ptr<Obstacle> observation, double timestamp, Perception& perception);
  TrackedObstacle(const TrackedObstacle& o);
  virtual ~TrackedObstacle();

  void update(boost::shared_ptr<Obstacle>, double timestamp);
  void update(double timestamp);
//  double x_var;
//  double y_var;
//  double xy_cov;
  int getNumObservations() const {return num_observations_;}
  double timestampObservation() const {return timestamp_observation_;}

  double getXVel() const;
  double getYVel() const;
//  double getXVar() const;
//  double getYVar() const;
//  double getXYCov() const;
  double getVelocity() const;
//  double getMaxSpeed() const { return max_speed_; }
//  int    getObserved() const { return observed_; }
//  int    getOccluded() const { return occluded_; }
  Eigen::VectorXf getLogOdds() { return log_odds_; }
  boost::shared_ptr<Obstacle> getLastObservation() { return lastObservation_; }

  virtual int  getSize() { return 0; }

  virtual void markDynamic(const Grid<PerceptionCell>& grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter);
  void incorporateClassification(int type, const Eigen::VectorXf& response);
  void estimateModel();

  float maxHeight();
};

//bool compareTrackedObstaclesConfidence(const boost::shared_ptr<TrackedObstacle> o1, const boost::shared_ptr<TrackedObstacle> o2);

} // namespace perception

#endif // PERCEPTION_OBSTACLE_H_
