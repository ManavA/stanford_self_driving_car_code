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
 * $Id: sac_model_cylinder.cpp 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu */

#include <stdlib.h>
#include <point_cloud_mapping/sample_consensus/sac_model_cylinder.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>

#include <cminpack.h>

namespace sample_consensus
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get 4 random points (3 non-collinear) as data samples and return them as point indices.
    * \param iterations the internal number of iterations used by SAC methods
    * \param samples the resultant model samples
    * \note assumes unique points!
    */
  void
    SACModelCylinder::getSamples (int &iterations, std::vector<int> &samples)
  {
    samples.resize (4);
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
        ROS_WARN ("[SACModelCylinder::getSamples] WARNING: Could not select 3 non collinear points in %d iterations!", MAX_ITERATIONS_COLLINEAR);
        break;
      }
      iterations++;
    }
    // Use Zoli's method for collinearity check
    while (((Dx1 / Dx2) == Dy1Dy2) && (Dy1Dy2 == (Dz1 / Dz2)));
    iterations--;

    // Need to improve this: we need 4 points, 3 non-collinear always, and the 4th should not be in the same plane as the other 3
    // otherwise we can encounter degenerate cases
    do
    {
      samples[3] = (int)(rand () * trand);
      iterations++;
    } while ( (samples[3] == samples[2]) || (samples[3] == samples[1]) || (samples[3] == samples[0]) );
    iterations--;

    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Select all the points which respect the given model coefficients as inliers.
    * \param model_coefficients the coefficients of a cylinder model that we need to compute distances to
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    * \param inliers the resultant model inliers
    * \note: To get the refined inliers of a model, use:
    * ANNpoint refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
    */
  void
    SACModelCylinder::selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers)
  {
    int nr_p = 0;
    inliers.resize (indices_.size ());

    // Model coefficients: [point_on_axis axis_direction radius]
    // Iterate through the 3d points and calculate the distances from them to the cylinder
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      // Aproximate the distance from the point to the cylinder as the difference between
      //dist(point,cylinder_axis) and cylinder radius
      // NOTE: need to revise this.
      if (fabs (
                cloud_geometry::distances::pointToLineDistance (cloud_->points.at (indices_[i]), model_coefficients) - model_coefficients[6]
               ) < threshold)
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
  /** \brief Compute all distances from the cloud data to a given cylinder model.
    * \param model_coefficients the coefficients of a cylinder model that we need to compute distances to
    * \param distances the resultant estimated distances
    */
  void
    SACModelCylinder::getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances)
  {
    distances.resize (indices_.size ());

    // Iterate through the 3d points and calculate the distances from them to the cylinder
    for (unsigned int i = 0; i < indices_.size (); i++)
      // Aproximate the distance from the point to the cylinder as the difference between
      //dist(point,cylinder_axis) and cylinder radius
      // NOTE: need to revise this.
      distances[i] = fabs (cloud_geometry::distances::pointToLineDistance (cloud_->points.at (indices_[i]), model_coefficients) - model_coefficients[6]);
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a new point cloud with inliers projected onto the cylinder model.
    * \param inliers the data inliers that we want to project on the cylinder model
    * \param model_coefficients the coefficients of a cylinder model
    * \param projected_points the resultant projected points
    * \todo implement this.
    */
  void
    SACModelCylinder::projectPoints (const std::vector<int> &inliers, const std::vector<double> &model_coefficients,
                                     sensor_msgs::PointCloud &projected_points)
  {
    std::cerr << "[SACModelCylinder::projecPoints] Not implemented yet." << std::endl;
    projected_points = *cloud_;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Project inliers (in place) onto the given cylinder model.
    * \param inliers the data inliers that we want to project on the cylinder model
    * \param model_coefficients the coefficients of a cylinder model
    * \todo implement this.
    */
  void
    SACModelCylinder::projectPointsInPlace (const std::vector<int> &inliers, const std::vector<double> &model_coefficients)
  {
    std::cerr << "[SACModelCylinder::projecPointsInPlace] Not implemented yet." << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Check whether the given index samples can form a valid cylinder model, compute the model coefficients from
    * these samples and store them internally in model_coefficients_. The cylinder coefficients are: x, y, z, R.
    * \param samples the point indices found as possible good candidates for creating a valid model
    */
  bool
    SACModelCylinder::computeModelCoefficients (const std::vector<int> &samples)
  {
    model_coefficients_.resize (7);

    // Save the nx/ny/nz channel indices the first time we run this
    if (nx_idx_ == -1)
    {
      nx_idx_ = cloud_geometry::getChannelIndex (*cloud_, "nx");
      if (nx_idx_ == -1) return (false);
    }
    if (ny_idx_ == -1)
    {
      ny_idx_ = cloud_geometry::getChannelIndex (*cloud_, "ny");
      if (ny_idx_ == -1) return (false);
    }
    if (nz_idx_ == -1)
    {
      nz_idx_ = cloud_geometry::getChannelIndex (*cloud_, "nz");
      if (nz_idx_ == -1) return (false);
    }

    geometry_msgs::Point32 u, v, w;

    u.x = cloud_->channels[nx_idx_].values.at (samples.at (0));
    u.y = cloud_->channels[ny_idx_].values.at (samples.at (0));
    u.z = cloud_->channels[nz_idx_].values.at (samples.at (0));

    v.x = cloud_->channels[nx_idx_].values.at (samples.at (1));
    v.y = cloud_->channels[ny_idx_].values.at (samples.at (1));
    v.z = cloud_->channels[nz_idx_].values.at (samples.at (1));

    w.x = (u.x + cloud_->points.at (samples.at (0)).x) - cloud_->points.at (samples.at (1)).x;
    w.y = (u.y + cloud_->points.at (samples.at (0)).y) - cloud_->points.at (samples.at (1)).y;
    w.z = (u.z + cloud_->points.at (samples.at (0)).z) - cloud_->points.at (samples.at (1)).z;

    double a = cloud_geometry::dot (u, u);
    double b = cloud_geometry::dot (u, v);
    double c = cloud_geometry::dot (v, v);
    double d = cloud_geometry::dot (u, w);
    double e = cloud_geometry::dot (v, w);
    double denominator = a*c - b*b;
    double sc, tc;
    // Compute the line parameters of the two closest points
    if (denominator < 1e-8)          // The lines are almost parallel
    {
      sc = 0.0;
      tc = (b > c ? d / b : e / c);  // Use the largest denominator
    }
    else
    {
      sc = (b*e - c*d) / denominator;
      tc = (a*e - b*d) / denominator;
    }

    // point_on_axis, axis_direction
    model_coefficients_[0] = cloud_->points.at (samples.at (0)).x + cloud_->channels[nx_idx_].values.at (samples.at (0)) + (sc * u.x);
    model_coefficients_[1] = cloud_->points.at (samples.at (0)).y + cloud_->channels[ny_idx_].values.at (samples.at (0)) + (sc * u.y);
    model_coefficients_[2] = cloud_->points.at (samples.at (0)).z + cloud_->channels[nz_idx_].values.at (samples.at (0)) + (sc * u.z);
    model_coefficients_[3] = cloud_->points.at (samples.at (1)).x + (tc * v.x) - model_coefficients_[0];
    model_coefficients_[4] = cloud_->points.at (samples.at (1)).y + (tc * v.y) - model_coefficients_[1];
    model_coefficients_[5] = cloud_->points.at (samples.at (1)).z + (tc * v.z) - model_coefficients_[2];

    double norm = sqrt (
                        (model_coefficients_[3] * model_coefficients_[3]) +
                        (model_coefficients_[4] * model_coefficients_[4]) +
                        (model_coefficients_[5] * model_coefficients_[5])
                      );
    model_coefficients_[3] /= norm;
    model_coefficients_[4] /= norm;
    model_coefficients_[5] /= norm;

    // cylinder radius
    model_coefficients_[6] = cloud_geometry::distances::pointToLineDistance (cloud_->points.at (samples.at (0)), model_coefficients_);

    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recompute the cylinder coefficients using the given inlier set and return them to the user.
    * @note: these are the coefficients of the cylinder model after refinement (eg. after SVD)
    * \param inliers the data inliers found as supporting the model
    * \param refit_coefficients the resultant recomputed coefficients after non-linear optimization
    */
  void
    SACModelCylinder::refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients)
  {
    if (inliers.size () == 0)
    {
      ROS_ERROR ("[SACModelCylinder::RefitModel] Cannot re-fit 0 inliers!");
      refit_coefficients = model_coefficients_;
      return;
    }
    if (model_coefficients_.size () == 0)
    {
      ROS_WARN ("[SACModelCylinder::RefitModel] Initial model coefficients have not been estimated yet - proceeding without an initial solution!");
      best_inliers_ = indices_;
    }

    tmp_inliers_ = &inliers;

    int m = inliers.size ();

    double *fvec = new double[m];

    int n = 7;      // 7 unknowns
    int iwa[n];

    int lwa = m * n + 5 * n + m;
    double *wa = new double[lwa];

    // Set the initial solution
    double x[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if ((int)model_coefficients_.size () == n)
      for (int d = 0; d < n; d++)
        x[d] = model_coefficients_.at (d);

    // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
    double tol = sqrt (dpmpar (1));

    // Optimize using forward-difference approximation LM
    int info = lmdif1 (&sample_consensus::SACModelCylinder::functionToOptimize, this, m, n, x, fvec, tol, iwa, wa, lwa);

    // Compute the L2 norm of the residuals
    ROS_DEBUG ("LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g %g",
               info, enorm (m, fvec), model_coefficients_.at (0), model_coefficients_.at (1), model_coefficients_.at (2), model_coefficients_.at (3),
               model_coefficients_.at (4), model_coefficients_.at (5), model_coefficients_.at (6), x[0], x[1], x[2], x[3], x[4], x[5], x[6]);

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
    SACModelCylinder::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
  {
    SACModelCylinder *model = (SACModelCylinder*)p;

    std::vector<double> line_coefficients (6);
    for (unsigned int d = 0; d < 6; d++)
      line_coefficients[d] = x[d];

    for (int i = 0; i < m; i++)
      // dist = f - r
      fvec[i] = cloud_geometry::distances::pointToLineDistance (model->cloud_->points[model->tmp_inliers_->at (i)], line_coefficients) - x[6];

    return (0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Verify whether a subset of indices verifies the internal cylinder model coefficients.
    * \param indices the data indices that need to be tested against the cylinder model
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    */
  bool
    SACModelCylinder::doSamplesVerifyModel (const std::set<int> &indices, double threshold)
  {
    for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
      // Aproximate the distance from the point to the cylinder as the difference between
      //dist(point,cylinder_axis) and cylinder radius
      // NOTE: need to revise this.
      if (fabs (
                cloud_geometry::distances::pointToLineDistance (cloud_->points.at (*it), model_coefficients_) - model_coefficients_[6]
               ) > threshold)
        return (false);

    return (true);
  }
}
