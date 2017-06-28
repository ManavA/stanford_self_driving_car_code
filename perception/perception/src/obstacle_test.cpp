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


#include <roadrunner.h>
#include <vector>

#include "perception.h"
#include "obstacle.h"
#include "tracked_obstacle.h"
#include "utils.h"

#include <gtest/gtest.h>

using namespace dgc;

TEST(Obstacle, Sanity) {
  Obstacles obstacles;

  GridObstacle* obs = new GridObstacle(1, NULL);
  obstacles.obstacles.push_back(obs);

  Obstacle* o = obstacles.obstacles[0];
  EXPECT_EQ(1, o->id);

  delete obs;
}

KalmanFilter buildSimpleFilter() {
  KalmanFilter filter(4);

  gsl_matrix *transition_mat_, *transition_cov_;
  gsl_matrix *observation_mat_, *observation_cov_;

  transition_mat_ = matrix_init(4, 4);
  transition_cov_ = matrix_init(4, 4);
  observation_mat_ = matrix_init(2, 4);
  observation_cov_ = matrix_init(2, 2);

  gsl_matrix_set_identity(transition_mat_);
  gsl_matrix_set_identity(transition_cov_);
  gsl_matrix_set_identity(observation_mat_);
  gsl_matrix_set_identity(observation_cov_);

  double dt = 0.1;
  gsl_matrix_set(transition_mat_, 0, 2, dt);
  gsl_matrix_set(transition_mat_, 1, 3, dt);

  gsl_vector* mean = filter.getMean();
  gsl_vector_set_zero(mean);

  // set initial covariance values
  gsl_matrix* covariance = filter.getCovariance();
  double l_cov = 0.1 * 0.1;
  double v_cov = 0.1 * 0.1;
  matrix_set(covariance, 0, 0, l_cov);
  matrix_set(covariance, 1, 1, l_cov);
  matrix_set(covariance, 2, 2, v_cov);
  matrix_set(covariance, 3, 3, v_cov);

  filter.setTransitionMatrix(transition_mat_);
  filter.setTransitionCovariance(transition_cov_);

  filter.setObservationMatrix(observation_mat_);
  filter.setObservationCovariance(observation_cov_);

  return filter;
}

KalmanFilter buildMovingFilter(double x_vel, double y_vel) {
  KalmanFilter filter = buildSimpleFilter();
  gsl_vector* mean = filter.getMean();
  vector_set(mean, 2, x_vel);
  vector_set(mean, 3, y_vel);
  return filter;
}

void freeFilterMatrices(KalmanFilter filter) {
  matrix_free(filter.getTransitionMatrix());
  matrix_free(filter.getTransitionCovariance());
  matrix_free(filter.getObservationMatrix());
  matrix_free(filter.getObservationCovariance());
}

TEST(KalmanFilter, SimpleFilter) {
  KalmanFilter filter = buildSimpleFilter();

  double x = vector_get(filter.getMean(), 0);
  double y = vector_get(filter.getMean(), 1);
  double x_vel = vector_get(filter.getMean(), 2);
  double y_vel = vector_get(filter.getMean(), 3);

  EXPECT_DOUBLE_EQ(0.0, x);
  EXPECT_DOUBLE_EQ(0.0, y);
  EXPECT_DOUBLE_EQ(0.0, x_vel);
  EXPECT_DOUBLE_EQ(0.0, y_vel);

  freeFilterMatrices(filter);
}

TEST(KalmanFilter, SimpleFilterTransition) {
  KalmanFilter filter = buildSimpleFilter();
  filter.transitionUpdate();

  double x = vector_get(filter.getMean(), 0);
  double y = vector_get(filter.getMean(), 1);
  double x_vel = vector_get(filter.getMean(), 2);
  double y_vel = vector_get(filter.getMean(), 3);

  EXPECT_DOUBLE_EQ(0.0, x);
  EXPECT_DOUBLE_EQ(0.0, y);
  EXPECT_DOUBLE_EQ(0.0, x_vel);
  EXPECT_DOUBLE_EQ(0.0, y_vel);

  freeFilterMatrices(filter);
}

TEST(KalmanFilter, MovingFilterTransition) {
  KalmanFilter filter = buildMovingFilter(1.0, 1.0);
  filter.transitionUpdate();

  double x = vector_get(filter.getMean(), 0);
  double y = vector_get(filter.getMean(), 1);
  double x_vel = vector_get(filter.getMean(), 2);
  double y_vel = vector_get(filter.getMean(), 3);

  EXPECT_DOUBLE_EQ(0.1, x);
  EXPECT_DOUBLE_EQ(0.1, y);
  EXPECT_DOUBLE_EQ(1.0, x_vel);
  EXPECT_DOUBLE_EQ(1.0, y_vel);

  freeFilterMatrices(filter);
}

TEST(KalmanFilter, SimpleFilterObservation) {
  KalmanFilter filter = buildMovingFilter(0.0, 0.0);

  gsl_vector* innovation= vector_init(2);
  vector_set(innovation, 0, 0.0);
  vector_set(innovation, 1, 0.0);
  for (int i=0; i<10; i++) {
    filter.transitionUpdate();
    filter.observationUpdate(innovation);
  }

  double x = vector_get(filter.getMean(), 0);
  double y = vector_get(filter.getMean(), 1);
  double x_vel = vector_get(filter.getMean(), 2);
  double y_vel = vector_get(filter.getMean(), 3);

  EXPECT_DOUBLE_EQ(0.0, x);
  EXPECT_DOUBLE_EQ(0.0, y);
  EXPECT_DOUBLE_EQ(0.0, x_vel);
  EXPECT_DOUBLE_EQ(0.0, y_vel);

  freeFilterMatrices(filter);
}

TEST(KalmanFilter, MovingFilterObservation) {
  KalmanFilter filter = buildMovingFilter(1.45, 0.0);

  double dt = 0.1;

  gsl_vector* innovation= vector_init(2);
  vector_set(innovation, 0, 0.0);
  vector_set(innovation, 1, 0.0);
  for (int i=1; i<100; i++) {
    float x = i * dt * 1.5; // actual position
    filter.transitionUpdate();
    gsl_vector* mean = filter.getMean();
    double predicted_x = vector_get(mean, 0);
    vector_set(innovation, 0, x - predicted_x);
    filter.observationUpdate(innovation);
  }
  filter.transitionUpdate();

  double x = vector_get(filter.getMean(), 0);
  double y = vector_get(filter.getMean(), 1);
  double x_vel = vector_get(filter.getMean(), 2);
  double y_vel = vector_get(filter.getMean(), 3);

  EXPECT_LT(fabs(15 - x), 0.001);
  EXPECT_LT(fabs(1.5 - x_vel), 0.001);
  EXPECT_DOUBLE_EQ(0.0, y);
  EXPECT_DOUBLE_EQ(0.0, y_vel);

  freeFilterMatrices(filter);
}

TEST(KalmanFilter, MovingFilterFromStandstillObservation) {
  KalmanFilter filter = buildMovingFilter(0.0, 0.0);

  double dt = 0.1;

  gsl_vector* innovation= vector_init(2);
  vector_set(innovation, 0, 0.0);
  vector_set(innovation, 1, 0.0);
  for (int i=1; i<100; i++) {
    float x = i * dt * 1.5; // actual position
    float y = i * dt * 1.0;
    filter.transitionUpdate();
    gsl_vector* mean = filter.getMean();
    double predicted_x = vector_get(mean, 0);
    double predicted_y = vector_get(mean, 1);
    vector_set(innovation, 0, x - predicted_x);
    vector_set(innovation, 1, y - predicted_y);
    filter.observationUpdate(innovation);
  }
  filter.transitionUpdate();

  double x = vector_get(filter.getMean(), 0);
  double y = vector_get(filter.getMean(), 1);
  double x_vel = vector_get(filter.getMean(), 2);
  double y_vel = vector_get(filter.getMean(), 3);

  EXPECT_LT(fabs(15 - x), 0.001);
  EXPECT_LT(fabs(1.5 - x_vel), 0.001);
  EXPECT_LT(fabs(10 - y), 0.001);
  EXPECT_LT(fabs(1.0 - y_vel), 0.001);

  freeFilterMatrices(filter);
}

TEST(KalmanFilter, MovingFilterNoisyObservation) {
  KalmanFilter filter = buildMovingFilter(0.0, 0.0);

  double dt = 0.1;

  gsl_vector* innovation= vector_init(2);
  vector_set(innovation, 0, 0.0);
  vector_set(innovation, 1, 0.0);
  for (int i=1; i<1000; i++) {
    float x = i * dt * 1.5 + sample(0.1);
    float y = i * dt * 1.0 + sample(0.1);
    filter.transitionUpdate();
    gsl_vector* mean = filter.getMean();
    double predicted_x = vector_get(mean, 0);
    double predicted_y = vector_get(mean, 1);
    vector_set(innovation, 0, x - predicted_x);
    vector_set(innovation, 1, y - predicted_y);
    filter.observationUpdate(innovation);
  }
  filter.transitionUpdate();

  double x = vector_get(filter.getMean(), 0);
  double y = vector_get(filter.getMean(), 1);
  double x_vel = vector_get(filter.getMean(), 2);
  double y_vel = vector_get(filter.getMean(), 3);

  EXPECT_LT(fabs(150 - x), 0.1);
  EXPECT_LT(fabs(1.5 - x_vel), 0.1);
  EXPECT_LT(fabs(100 - y), 0.10);
  EXPECT_LT(fabs(1.0 - y_vel), 0.10);

  freeFilterMatrices(filter);
}
