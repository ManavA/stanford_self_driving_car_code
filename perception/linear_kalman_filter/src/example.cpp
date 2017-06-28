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
#include <matplotlib_interface/matplotlib_interface.h> 

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  
  double initial_variance = 1;
  double velocity_variance = 1;
  double measurement_variance = 1;

  VectorXd initial_state = VectorXd::Zero(4);
  MatrixXd initial_sigma = MatrixXd::Identity(4, 4) * initial_variance;
  MatrixXd measurement_matrix = MatrixXd::Identity(2, 4);
  MatrixXd transition_covariance = MatrixXd::Zero(4, 4);
  transition_covariance(2, 2) = velocity_variance;
  transition_covariance(3, 3) = velocity_variance;
  MatrixXd measurement_covariance = MatrixXd::Identity(2, 2) * measurement_variance;
  
  LinearKalmanFilter kf(0, 0, initial_state, initial_sigma, measurement_matrix, transition_covariance, measurement_covariance);

  double delta_time = 0.1;
  MatrixXd transition_matrix = MatrixXd::Identity(4, 4);
  transition_matrix(0, 2) = delta_time;
  transition_matrix(1, 3) = delta_time;

  mpliBegin();
  mpli("from pylab import *");
  mpli("delta = 0.01");
  mpli("x = arange(0, 3, delta)");
  mpli("y = arange(-1.5, 1.5, delta)");
  mpli("X, Y = meshgrid(x, y)");
  
  VectorXd measurement(2);
  double t = 0;
  while(true) {
    measurement(0) = t;
    measurement(1) = sin(t);
    t += 0.1;

    mpliNamedExport("mu", kf.mu_);
    mpli("scatter(mu[0], mu[1])");
    kf.predict(transition_matrix, t);
    mpliNamedExport("sigma_bar", kf.sigma_);
    mpliNamedExport("mu_bar", kf.mu_);
    mpli("Z = bivariate_normal(X, Y, sigma_bar[0, 0], sigma_bar[1, 1], mu_bar[0], mu_bar[1], sigma_bar[0, 1])");
    mpli("CS = contour(X, Y, Z, colors=('r'))");
    mpli("clabel(CS, inline=1, fontsize=10)");
    mpli("draw()");
    mpli("waitforbuttonpress()");
    mpli("clf()");
    
    kf.update(measurement, t);
    mpliNamedExport("sigma", kf.sigma_);
    mpliNamedExport("mu", kf.mu_);
    mpli("scatter(mu[0], mu[1])");
    mpli("Z = bivariate_normal(X, Y, sigma[0, 0], sigma[1, 1], mu[0], mu[1], sigma[0, 1])");
    //mpli("CS = contour(X, Y, Z, levels=[0.6, 0.5, 0.3, 0.2, 0.1, 0.001], colors=('r', 'g', 'b', 'y'))");
    mpli("CS = contour(X, Y, Z, colors=('g'))");
    mpli("clabel(CS, inline=1, fontsize=10)");
    mpli("draw()");
    mpli("waitforbuttonpress()");
    mpli("clf()");

  }

  mpliEnd();
}
