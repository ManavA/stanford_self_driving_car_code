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
 * $Id: sac_model_oriented_line.cpp 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu
  *
  * \todo Change the internal representation of the line model from 2 points to 1 point + direction.
  */

#include <point_cloud_mapping/sample_consensus/sac_model_oriented_line.h>
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/nearest.h>

namespace sample_consensus
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Select all the points which respect the given model coefficients as inliers.
    * \param model_coefficients the coefficients of a line model that we need to compute distances to
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    * \param inliers the resultant model inliers
    * \note: To get the refined inliers of a model, use:
    * ANNpoint refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
    */
  void
    SACModelOrientedLine::selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers)
  {
    double sqr_threshold = threshold * threshold;

    // Obtain the line direction
    geometry_msgs::Point32 p3, p4;
    p3.x = model_coefficients.at (3) - model_coefficients.at (0);
    p3.y = model_coefficients.at (4) - model_coefficients.at (1);
    p3.z = model_coefficients.at (5) - model_coefficients.at (2);

    double angle_error = cloud_geometry::angles::getAngle3D (axis_, p3);

    // Check whether the current line model satisfies our angle threshold criterion with respect to the given axis
    if (angle_error >  eps_angle_)
    {
      inliers.resize (0);
      return;
    }

    int nr_p = 0;
    inliers.resize (indices_.size ());
    // Iterate through the 3d points and calculate the distances from them to the plane
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      // Calculate the distance from the point to the line
      // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
      // P1, P2 = line points, P0 = query point
      // P1P2 = <x2-x1, y2-y1, z2-z1> = <x3, y3, z3>
      // P1P0 = < x-x1,  y-y1, z-z1 > = <x4, y4, z4>
      // P1P2 x P1P0 = < y3*z4 - z3*y4, -(x3*z4 - x4*z3), x3*y4 - x4*y3 >
      //             = < (y2-y1)*(z-z1) - (z2-z1)*(y-y1), -[(x2-x1)*(z-z1) - (x-x1)*(z2-z1)], (x2-x1)*(y-y1) - (x-x1)*(y2-y1) >
      p4.x = model_coefficients.at (3) - cloud_->points.at (indices_.at (i)).x;
      p4.y = model_coefficients.at (4) - cloud_->points.at (indices_.at (i)).y;
      p4.z = model_coefficients.at (5) - cloud_->points.at (indices_.at (i)).z;

      // P1P2 = sqrt (x3^2 + y3^2 + z3^2)
      // a = sqrt [(y3*z4 - z3*y4)^2 + (x3*z4 - x4*z3)^2 + (x3*y4 - x4*y3)^2]
      //double distance = SQR_NORM (cANN::cross (p4, p3)) / SQR_NORM (p3);
      geometry_msgs::Point32 c = cloud_geometry::cross (p4, p3);
      double sqr_distance = (c.x * c.x + c.y * c.y + c.z * c.z) / (p3.x * p3.x + p3.y * p3.y + p3.z * p3.z);

      if (sqr_distance < sqr_threshold)
      {
        // Returns the indices of the points whose squared distances are smaller than the threshold
        inliers[nr_p] = indices_[i];
        nr_p++;
      }
    }
    inliers.resize (nr_p);
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute all distances from the cloud data to a given line model.
    * \param model_coefficients the coefficients of a line model that we need to compute distances to
    * \param distances the resultant estimated distances
    */
  void
    SACModelOrientedLine::getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances)
  {
    // Obtain the line direction
    geometry_msgs::Point32 p3, p4;
    p3.x = model_coefficients.at (3) - model_coefficients.at (0);
    p3.y = model_coefficients.at (4) - model_coefficients.at (1);
    p3.z = model_coefficients.at (5) - model_coefficients.at (2);

    double angle_error = cloud_geometry::angles::getAngle3D (axis_, p3);

    // Check whether the current line model satisfies our angle threshold criterion with respect to the given axis
    if (angle_error >  eps_angle_)
    {
      distances.resize (0);
      return;
    }

    distances.resize (indices_.size ());
    // Iterate through the 3d points and calculate the distances from them to the plane
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      // Calculate the distance from the point to the line
      // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
      p4.x = model_coefficients.at (3) - cloud_->points.at (indices_.at (i)).x;
      p4.y = model_coefficients.at (4) - cloud_->points.at (indices_.at (i)).y;
      p4.z = model_coefficients.at (5) - cloud_->points.at (indices_.at (i)).z;

      geometry_msgs::Point32 c = cloud_geometry::cross (p4, p3);
      distances[i] = sqrt (c.x * c.x + c.y * c.y + c.z * c.z) / (p3.x * p3.x + p3.y * p3.y + p3.z * p3.z);
    }
    return;
  }

}
