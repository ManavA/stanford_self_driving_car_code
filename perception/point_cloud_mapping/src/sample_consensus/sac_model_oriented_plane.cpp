/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: sac_model_oriented_plane.cpp 10961 2009-02-11 00:20:50Z tfoote $
 *
 */

/** \author Radu Bogdan Rusu */

#include <point_cloud_mapping/sample_consensus/sac_model_oriented_plane.h>
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/nearest.h>

namespace sample_consensus
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Select all the points which respect the given model coefficients as inliers.
    * \param model_coefficients the coefficients of a plane model that we need to compute distances to
    * \param inliers the resultant model inliers
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    * \note: To get the refined inliers of a model, use:
    * ANNpoint refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
    */
  void
    SACModelOrientedPlane::selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers)
  {
    int nr_p = 0;

    // Obtain the plane normal
    geometry_msgs::Point32 n;
    n.x = model_coefficients.at (0);
    n.y = model_coefficients.at (1);
    n.z = model_coefficients.at (2);

    double angle_error = cloud_geometry::angles::getAngle3D (axis_, n);

    // Check whether the current plane model satisfies our angle threshold criterion with respect to the given axis
    if ( (angle_error > eps_angle_) && ( (M_PI - angle_error) > eps_angle_ ) )
    {
      inliers.resize (0);
      return;
    }

    inliers.resize (indices_.size ());
    // Iterate through the 3d points and calculate the distances from them to the plane
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      // Calculate the distance from the point to the plane normal as the dot product
      // D = (P-A).N/|N|
      if (fabs (model_coefficients.at (0) * cloud_->points.at (indices_.at (i)).x +
                model_coefficients.at (1) * cloud_->points.at (indices_.at (i)).y +
                model_coefficients.at (2) * cloud_->points.at (indices_.at (i)).z +
                model_coefficients.at (3)) < threshold)
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
  /** \brief Compute all distances from the cloud data to a given plane model.
    * \param model_coefficients the coefficients of a plane model that we need to compute distances to
    * \param distances the resultant estimated distances
    */
  void
    SACModelOrientedPlane::getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances)
  {
    // Obtain the plane normal
    geometry_msgs::Point32 n;
    n.x = model_coefficients.at (0);
    n.y = model_coefficients.at (1);
    n.z = model_coefficients.at (2);

    double angle_error = cloud_geometry::angles::getAngle3D (axis_, n);

    // Check whether the current plane model satisfies our angle threshold criterion with respect to the given axis
    // TODO: check this
    if ( (angle_error > eps_angle_) && ( (M_PI - angle_error) > eps_angle_ ) )
    {
      distances.resize (0);
      return;
    }

    distances.resize (indices_.size ());
    // Iterate through the 3d points and calculate the distances from them to the plane
    for (unsigned int i = 0; i < indices_.size (); i++)
      // Calculate the distance from the point to the plane normal as the dot product
      // D = (P-A).N/|N|
      distances[i] = fabs (model_coefficients.at (0) * cloud_->points.at (indices_[i]).x +
                           model_coefficients.at (1) * cloud_->points.at (indices_[i]).y +
                           model_coefficients.at (2) * cloud_->points.at (indices_[i]).z +
                           model_coefficients.at (3));
    return;
  }

}
