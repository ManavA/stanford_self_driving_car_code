/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: sac_model_circle.cpp 23527 2009-09-01 11:36:15Z juergensturm $
 *
 */

/** \author Radu Bogdan Rusu */

#include <point_cloud_mapping/sample_consensus/sac_model_circle.h>
#include <point_cloud_mapping/geometry/nearest.h>

#include <cminpack.h>

#define SQR(a) ((a)*(a))

namespace sample_consensus
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get 3 random non-collinear points as data samples and return them as point indices.
    * \param iterations the internal number of iterations used by SAC methods
    * \param samples the resultant model samples
    * \note assumes unique points!
    */
  void
    SACModelCircle2D::getSamples (int &iterations, std::vector<int> &samples)
  {
    samples.resize (3);
    double trand = indices_.size () / (RAND_MAX + 1.0);

    // Get a random number between 1 and max_indices
    int idx = (int)(rand () * trand);
    // Get the index
    samples[0] = indices_.at (idx);

    // Get a second point which is different than the first
    do
    {
      idx = (int)(rand () * trand);
      samples[1] = indices_.at (idx);
      iterations++;
    } while (samples[1] == samples[0]);
    iterations--;

    double Dx1, Dy1, Dz1, Dx2, Dy2, Dz2, Dy1Dy2;
    // Compute the segment values (in 3d) between XY
    Dx1 = cloud_->points[samples[1]].x - cloud_->points[samples[0]].x;
    Dy1 = cloud_->points[samples[1]].y - cloud_->points[samples[0]].y;
    Dz1 = cloud_->points[samples[1]].z - cloud_->points[samples[0]].z;

    int iter = 0;
    do
    {
      // Get the third point, different from the first two
      do
      {
        idx = (int)(rand () * trand);
        samples[2] = indices_.at (idx);
        iterations++;
      } while ( (samples[2] == samples[1]) || (samples[2] == samples[0]) );
      iterations--;

      // Compute the segment values (in 3d) between XZ
      Dx2 = cloud_->points[samples[2]].x - cloud_->points[samples[0]].x;
      Dy2 = cloud_->points[samples[2]].y - cloud_->points[samples[0]].y;
      Dz2 = cloud_->points[samples[2]].z - cloud_->points[samples[0]].z;

      Dy1Dy2 = Dy1 / Dy2;
      iter++;

      if (iter > MAX_ITERATIONS_COLLINEAR )
      {
        ROS_WARN ("[SACModelCircle2D::getSamples] WARNING: Could not select 3 non collinear points in %d iterations!",  MAX_ITERATIONS_COLLINEAR);
        break;
      }
      iterations++;
    }
    // Use Zoli's method for collinearity check
    while (((Dx1 / Dx2) == Dy1Dy2) && (Dy1Dy2 == (Dz1 / Dz2)));
    iterations--;

    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Select all the points which respect the given model coefficients as inliers.
    * \param model_coefficients the coefficients of a circle model that we need to compute distances to
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    * \param inliers the resultant model inliers
    * \note: To get the refined inliers of a model, use:
    * ANNpoint refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
    */
  void
    SACModelCircle2D::selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers)
  {
    int nr_p = 0;
    inliers.resize (indices_.size ());

    // Iterate through the 3d points and calculate the distances from them to the circle
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      // Calculate the distance from the point to the circle as the difference between
      //dist(point,circle_origin) and circle_radius
      double distance_to_circle = fabs (sqrt (
                                              ( cloud_->points.at (indices_[i]).x - model_coefficients.at (0) ) *
                                              ( cloud_->points.at (indices_[i]).x - model_coefficients.at (0) ) +

                                              ( cloud_->points.at (indices_[i]).y - model_coefficients.at (1) ) *
                                              ( cloud_->points.at (indices_[i]).y - model_coefficients.at (1) )
                                             ) - model_coefficients.at (2));
      if (distance_to_circle < threshold)
      {
        // Returns the indices of the points whose distances are smaller than the threshold
        inliers[nr_p] = indices_[i];
        nr_p++;
      }
    }
    inliers.resize (nr_p);
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute all distances from the cloud data to a given 2D circle model.
    * \param model_coefficients the coefficients of a 2D circle model that we need to compute distances to
    * \param distances the resultant estimated distances
    */
  void
    SACModelCircle2D::getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances)
  {
    distances.resize (indices_.size ());

    // Iterate through the 3d points and calculate the distances from them to the circle
    for (unsigned int i = 0; i < indices_.size (); i++)
      // Calculate the distance from the point to the circle as the difference between
      //dist(point,circle_origin) and circle_radius
      distances[i] = fabs (sqrt (
                                 ( cloud_->points.at (indices_[i]).x - model_coefficients.at (0) ) *
                                 ( cloud_->points.at (indices_[i]).x - model_coefficients.at (0) ) +

                                 ( cloud_->points.at (indices_[i]).y - model_coefficients.at (1) ) *
                                 ( cloud_->points.at (indices_[i]).y - model_coefficients.at (1) )
                                ) - model_coefficients.at (2));
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a new point cloud with inliers projected onto the 2D circle model.
    * \param inliers the data inliers that we want to project on the 2D circle model
    * \param model_coefficients the coefficients of a 2D circle model
    * \param projected_points the resultant projected points
    * \todo implement this.
    */
  void
    SACModelCircle2D::projectPoints (const std::vector<int> &inliers, const std::vector<double> &model_coefficients,
                                     sensor_msgs::PointCloud &projected_points)
  {
//    std::cerr << "[SACModelCircle2D::projecPoints] Not implemented yet." << std::endl;
    projected_points = *cloud_;

	double cx = model_coefficients[0];
	double cy = model_coefficients[1];
	double r =  model_coefficients[2];
	for(size_t i=0;i<cloud_->get_points_size();i++) {
		double dx = cloud_->points[i].x - cx;
		double dy = cloud_->points[i].y - cy;
		double a = sqrt( SQR(r) / ( SQR(dx) + SQR(dy) ));
		projected_points.points[i].x = a*dx + cx;
		projected_points.points[i].y = a*dy + cy;
	}
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Project inliers (in place) onto the given 2D circle model.
    * \param inliers the data inliers that we want to project on the 2D circle model
    * \param model_coefficients the coefficients of a 2D circle model
    * \todo implement this.
    */
  void
    SACModelCircle2D::projectPointsInPlace (const std::vector<int> &inliers, const std::vector<double> &model_coefficients)
  {
    std::cerr << "[SACModelCircle2D::projecPointsInPlace] Not implemented yet." << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Check whether the given index samples can form a valid 2D circle model, compute the model coefficients from
    * these samples and store them internally in model_coefficients_.
    * \param samples the point indices found as possible good candidates for creating a valid model
    */
  bool
    SACModelCircle2D::computeModelCoefficients (const std::vector<int> &samples)
  {
    model_coefficients_.resize (3);

    geometry_msgs::Point32 u, v, m;
    u.x = ( cloud_->points.at (samples.at (0)).x + cloud_->points.at (samples.at (1)).x ) / 2;
    u.y = ( cloud_->points.at (samples.at (1)).x + cloud_->points.at (samples.at (2)).x ) / 2;

    v.x = ( cloud_->points.at (samples.at (0)).y + cloud_->points.at (samples.at (1)).y ) / 2;
    v.y = ( cloud_->points.at (samples.at (1)).y + cloud_->points.at (samples.at (2)).y ) / 2;

    m.x = -( cloud_->points.at (samples.at (1)).x - cloud_->points.at (samples.at (0)).x ) /
           ( cloud_->points.at (samples.at (1)).y - cloud_->points.at (samples.at (0)).y );
    m.y = -( cloud_->points.at (samples.at (2)).x - cloud_->points.at (samples.at (1)).x ) /
           ( cloud_->points.at (samples.at (2)).y - cloud_->points.at (samples.at (1)).y );

    // Center (x, y)
    model_coefficients_[0] = (m.x * u.x -  m.y * u.y  - (v.x - v.y) )           / (m.x - m.y);
    model_coefficients_[1] = (m.x * m.y * (u.x - u.y) +  m.x * v.y - m.y * v.x) / (m.x - m.y);

    // Radius
    model_coefficients_[2] = sqrt (
                                   ( model_coefficients_[0] - cloud_->points.at (samples.at (0)).x ) *
                                   ( model_coefficients_[0] - cloud_->points.at (samples.at (0)).x ) +

                                   ( model_coefficients_[1] - cloud_->points.at (samples.at (0)).y ) *
                                   ( model_coefficients_[1] - cloud_->points.at (samples.at (0)).y )
                                  );
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recompute the plane coefficients using the given inlier set and return them to the user.
    * @note: these are the coefficients of the circle model after refinement (eg. after SVD)
    * \param inliers the data inliers found as supporting the model
    * \param refit_coefficients the resultant recomputed coefficients after non-linear optimization
    */
  void
    SACModelCircle2D::refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients)
  {
    if (inliers.size () == 0)
    {
      ROS_ERROR ("[SACModelCircle2D::RefitModel] Cannot re-fit 0 inliers!");
      refit_coefficients = model_coefficients_;
      return;
    }
    if (model_coefficients_.size () == 0)
    {
      ROS_WARN ("[SACModelCircle2D::RefitModel] Initial model coefficients have not been estimated yet - proceeding without an initial solution!");
      best_inliers_ = indices_;
    }

    tmp_inliers_ = &inliers;
    
    int m = inliers.size ();

    double *fvec = new double[m];

    int n = 3;      // 3 unknowns
    int iwa[n];

    int lwa = m * n + 5 * n + m;
    double *wa = new double[lwa];

    // Set the initial solution
    double x[3] = {0.0, 0.0, 0.0};
    if ((int)model_coefficients_.size () == n)
      for (int d = 0; d < n; d++)
        x[d] = model_coefficients_.at (d);

    // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
    double tol = sqrt (dpmpar (1));

    // Optimize using forward-difference approximation LM
    int info = lmdif1 (&sample_consensus::SACModelCircle2D::functionToOptimize, this, m, n, x, fvec, tol, iwa, wa, lwa);

    // Compute the L2 norm of the residuals
    ROS_DEBUG  ("LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g \nFinal solution: %g %g %g",
                info, enorm (m, fvec), model_coefficients_.at (0), model_coefficients_.at (1), model_coefficients_.at (2), x[0], x[1], x[2]);

    refit_coefficients.resize (n);
    for (int d = 0; d < n; d++)
      refit_coefficients[d] = x[d];

    free (wa); free (fvec);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////(
  /** \brief Cost function to be minimized
    * \param p a pointer to our data structure array
    * \param m the number of functions
    * \param n the number of variables
    * \param x a pointer to the variables array
    * \param fvec a pointer to the resultant functions evaluations
    * \param iflag set to -1 inside the function to terminate execution
    */
  int
    SACModelCircle2D::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
  {
    SACModelCircle2D *model = (SACModelCircle2D*)p;

    for (int i = 0; i < m; i ++)
    {
      // Compute the difference between the center of the circle and the datapoint X_i
      double xt = model->cloud_->points[model->tmp_inliers_->at (i)].x - x[0];
      double yt = model->cloud_->points[model->tmp_inliers_->at (i)].y - x[1];

      // g = sqrt ((x-a)^2 + (y-b)^2) - R
      fvec[i] = sqrt (xt * xt + yt * yt) - x[2];
    }
    return (0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Verify whether a subset of indices verifies the internal circle model coefficients.
    * \param indices the data indices that need to be tested against the circle model
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    */
  bool
    SACModelCircle2D::doSamplesVerifyModel (const std::set<int> &indices, double threshold)
  {
    // Iterate through the 3d points and calculate the distances from them to the circle
    for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
    {
      // Calculate the distance from the point to the circle as the difference between
      //dist(point,circle_origin) and circle_radius
      double distance_to_circle = fabs (sqrt (
                                              ( cloud_->points.at (*it).x - model_coefficients_.at (0) ) *
                                              ( cloud_->points.at (*it).x - model_coefficients_.at (0) ) +

                                              ( cloud_->points.at (*it).y - model_coefficients_.at (1) ) *
                                              ( cloud_->points.at (*it).y - model_coefficients_.at (1) )
                                             ) - model_coefficients_.at (2));
      if (distance_to_circle > threshold)
        return (false);
    }
    return (true);
  }
}
