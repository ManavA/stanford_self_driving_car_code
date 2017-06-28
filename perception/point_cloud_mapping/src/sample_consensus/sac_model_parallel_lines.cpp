/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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
 * $Id: sac_model_parallel_lines.cpp 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Caroline Pantofaru */

#include <point_cloud_mapping/sample_consensus/sac_model_parallel_lines.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/nearest.h>

using namespace std;

namespace sample_consensus
{

  //////////////////////////////////////////////////////////////////////////
  /**
   * \brief Compute the square distance from a point to a line (as defined by two points).
   * \param line_point1 One point on the line.
   * \param line_point2 Another point on the line.
   * \param point The point
   */
  double
    SACModelParallelLines::pointToLineSquareDistance (const geometry_msgs::Point32 &line_point1, const geometry_msgs::Point32 &line_point2, const geometry_msgs::Point32 &point)
  {
    geometry_msgs::Point32 v12, v1p;
    v12.x = line_point2.x - line_point1.x;
    v12.y = line_point2.y - line_point1.y;
    v12.z = line_point2.z - line_point1.z;
    v1p.x = point.x - line_point1.x;
    v1p.y = point.y - line_point1.y;
    v1p.z = point.z - line_point1.z;
    geometry_msgs::Point32 c = cloud_geometry::cross (v12, v1p);
    double sqr_distance = (c.x * c.x + c.y * c.y + c.z * c.z) / (v12.x * v12.x + v12.y * v12.y + v12.z * v12.z);
    return (sqr_distance);
  }


  /////////////////////////////////////////////////////////////////////////
  /**
   * \brief Compute the closest distance between each of a set of points and a pair of parallel lines. Also returns the closest line for each point.
   * \param indices The indices of the points in the point cloud.
   * \param model_coefficients The 3d parallel lines model (length 9). The model is (point on line 1, another point on line 1, point on line 2).
   * \param closest_line The closest line for each point. 0 means the first line (the one with 2 points listed), 1 means the second line.
   * \param closest_dist The distance from each point to its closest line.
   */
  void
    SACModelParallelLines::closestLine (const std::vector<int> &indices, const std::vector<double> &model_coefficients,
                                        std::vector<int> *closest_line, std::vector<double> *closest_dist)
  {
    int end = indices.size ();
    geometry_msgs::Point32 d1, d2, l1, c1, c2;
    l1.x = model_coefficients[3] - model_coefficients[0];
    l1.y = model_coefficients[4] - model_coefficients[1];
    l1.z = model_coefficients[5] - model_coefficients[2];
    double l_sqr_length = (l1.x * l1.x + l1.y * l1.y + l1.z * l1.z);
    double sqr_distance1, sqr_distance2;
    for (int i = 0; i < end; i++)
    {
      // Calculate the distance from the point to both lines
      d1.x = cloud_->points[indices[i]].x - model_coefficients[0];
      d1.y = cloud_->points[indices[i]].y - model_coefficients[1];
      d1.z = cloud_->points[indices[i]].z - model_coefficients[2];
      d2.x = cloud_->points[indices[i]].x - model_coefficients[6];
      d2.y = cloud_->points[indices[i]].y - model_coefficients[7];
      d2.z = cloud_->points[indices[i]].z - model_coefficients[8];

      c1 = cloud_geometry::cross (l1, d1);
      sqr_distance1 = (c1.x * c1.x + c1.y * c1.y + c1.z * c1.z);
      c2 = cloud_geometry::cross (l1, d2);
      sqr_distance2 = (c2.x * c2.x + c2.y * c2.y + c2.z * c2.z);

      if (sqr_distance1 < sqr_distance2)
      {
        (*closest_line)[i] = 0;
        (*closest_dist)[i] = sqrt (sqr_distance1 / l_sqr_length);
      }
      else
      {
        (*closest_line)[i] = 1;
        (*closest_dist)[i] = sqrt (sqr_distance2 / l_sqr_length);
      }
    }
  }

  void
    SACModelParallelLines::closestLine (const std::set<int> &indices, const std::vector<double> &model_coefficients,
                                        std::vector<int> *closest_line, std::vector<double> *closest_dist)
  {
    std::set<int>::iterator end = indices.end ();
    geometry_msgs::Point32 d1, d2, l1, c1, c2;
    l1.x = model_coefficients[3] - model_coefficients[0];
    l1.y = model_coefficients[4] - model_coefficients[1];
    l1.z = model_coefficients[5] - model_coefficients[2];
    double l_sqr_length = (l1.x * l1.x + l1.y * l1.y + l1.z * l1.z);
    double sqr_distance1, sqr_distance2;
    int i = 0;
    for (std::set<int>::iterator it = indices.begin (); it != end; ++it)
    {
      // Calculate the distance from the point to the line
      d1.x = cloud_->points[*it].x - model_coefficients[0];
      d1.y = cloud_->points[*it].y - model_coefficients[1];
      d1.z = cloud_->points[*it].z - model_coefficients[2];
      d2.x = cloud_->points[*it].x - model_coefficients[6];
      d2.y = cloud_->points[*it].y - model_coefficients[7];
      d2.z = cloud_->points[*it].z - model_coefficients[8];

      c1 = cloud_geometry::cross (l1, d1);
      sqr_distance1 = (c1.x * c1.x + c1.y * c1.y + c1.z * c1.z);
      c2 = cloud_geometry::cross (l1, d2);
      sqr_distance2 = (c2.x * c2.x + c2.y * c2.y + c2.z * c2.z);

      if (sqr_distance1 < sqr_distance2)
      {
        (*closest_line)[i] = 0;
        (*closest_dist)[i] = sqrt (sqr_distance1 / l_sqr_length);
      }
      else
      {
        (*closest_line)[i] = 1;
        (*closest_dist)[i] = sqrt (sqr_distance2 / l_sqr_length);
      }
      ++i;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get 3 random points as data samples and return them as point indices.
    * \param iterations The internal number of iterations used by the SAC methods
    * \param samples the resultant model samples
    * \note Ensures that the 3 points are unique and not co-linear.
    */
  void
    SACModelParallelLines::getSamples (int &iterations, std::vector<int> &samples)
  {
    std::vector<int> random_idx (3);
    double trand = indices_.size () / (RAND_MAX + 1.0);

    // Get a random number between 1 and max_indices
    int idx = (int)(rand()*trand);
    // Get the index
    random_idx[0] = indices_[idx];

    // Get two other points that are different and are not colinear.
    int iter = 0;
    int total_points = 1;

    double sqr_min_line_sep_m = min_line_sep_m_ * min_line_sep_m_;
    double sqr_max_line_sep_m = max_line_sep_m_ * max_line_sep_m_;
    double sqr_distance;
    int r0 = 0, r1 = 1, r2 = 2;

    while (total_points < 3)
    {
      iter++;

      if (iter > MAX_ITERATIONS_UNIQUE)
      {
        ROS_WARN ("[SACModelParallelLines::getSamples] WARNING: Could not select 3 unique, non-colinear points in %d iterations!", MAX_ITERATIONS_UNIQUE);
        break;
      }

      idx = (int)(rand ()*trand);
      random_idx[total_points] = indices_[idx];
      // If we already have this point, continue looking.
      if ( (random_idx[total_points] == random_idx[0]) || ((total_points==2) && random_idx[total_points] == random_idx[1]) )
      {
        continue;
      }
      // If this is the 3rd point and it's colinear with the other two, or not within the distance bounds, continue looking.
      // Try all combinations of the 3 pts into ((l1,l2),p)
      if (total_points == 2)
      {

        r1 = 0;
        for (r0 = 0; r0 < 3; r0++)
        {
          r1 = (r0+1)%3;
          r2 = (r1+1)%3;
          sqr_distance = pointToLineSquareDistance (cloud_->points[random_idx[r0]], cloud_->points[random_idx[r1]], cloud_->points[random_idx[r2]]);

          if (sqr_distance == 0.0 || sqr_distance < sqr_min_line_sep_m || sqr_distance > sqr_max_line_sep_m)
            continue;

          break;
        }
        if (r0 == 3)
          continue;
      }

      // This point is ok.
      total_points++;
    }

    iterations += iter-1;

    samples.resize (3);
    samples[0] = random_idx[r0];
    samples[1] = random_idx[r1];
    samples[2] = random_idx[r2];
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Select all the points which respect the given model coefficients as inliers.
    * \param model_coefficients The coefficients of the parallel lines model.
    * \param threshold The maximum distance from an inlier to its closest line.
    * \param inliers the resultant model inliers
    * \note: To get the refined inliers of a model, use:
    * refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
    */
  void
    SACModelParallelLines::selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers)
  {
    int nr_p = 0;
    inliers.resize (indices_.size ());

    // Get all of the point-to-closest-line distances.
    std::vector<int> closest_line (indices_.size ());
    std::vector<double> closest_dist (indices_.size ());
    closestLine (indices_, model_coefficients, &closest_line, &closest_dist);

    // Find the inliers
    for (unsigned int i = 0; i < closest_dist.size (); i++)
    {
      if (closest_dist[i] < threshold)
      {
        inliers[nr_p] = indices_[i];
        nr_p++;
      }
    }
    inliers.resize (nr_p);
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute all distances from the cloud data to a given parallel lines model.
    * \param model_coefficients The coefficients of a line model that we need to compute distances to. The order is (point on line 1, another point on line 1, point on line 2).
    * \param distances the resultant estimated distances
    */
  void
    SACModelParallelLines::getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances)
  {
    distances.resize (indices_.size ());

    std::vector<int> closest_line (indices_.size ());
    closestLine (indices_, model_coefficients, &closest_line, &distances);

    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a new point cloud with inliers projected onto the line model.
    * \param inliers The indices of the inliers into the data cloud.
    * \param model_coefficients The coefficients of the parallel lines model.
    * \param projected_points the resultant projected points
    */
  void
    SACModelParallelLines::projectPoints (const std::vector<int> &inliers, const std::vector<double> &model_coefficients,
                                          sensor_msgs::PointCloud &projected_points)
  {
    // Allocate enough space
    projected_points.points.resize (inliers.size ());

    // Create the channels
    projected_points.set_channels_size (cloud_->get_channels_size ());
    for (unsigned int d = 0; d < projected_points.get_channels_size (); d++)
    {
      projected_points.channels[d].name = cloud_->channels[d].name;
      projected_points.channels[d].values.resize (inliers.size ());
    }

    // Compute the closest distances from the pts to the lines.
    std::vector<int> closest_line (inliers.size ());
    std::vector<double> closest_dist (inliers.size ());
    closestLine (inliers, model_coefficients, &closest_line, &closest_dist);

    geometry_msgs::Point32 l1;
    l1.x = model_coefficients[3] - model_coefficients[0];
    l1.y = model_coefficients[4] - model_coefficients[1];
    l1.z = model_coefficients[5] - model_coefficients[2];
    double l_sqr_length = (l1.x * l1.x + l1.y * l1.y + l1.z * l1.z);

    // Iterate through the 3d points and project them onto their closest line.
    for (unsigned int i = 0; i < inliers.size (); i++)
    {
      double mx,my,mz;
      if (closest_line[i] == 0)
      {
        mx = model_coefficients[0];
        my = model_coefficients[1];
        mz = model_coefficients[2];
      }
      else
      {
        mx = model_coefficients[6];
        my = model_coefficients[7];
        mz = model_coefficients[8];
      }
      double k = (
                  ( cloud_->points[inliers[i]].x * l1.x + cloud_->points[inliers[i]].y * l1.y + cloud_->points[inliers[i]].z * l1.z ) -
                  ( mx * l1.x + my * l1.y + mz * l1.z )
                 ) / l_sqr_length;
      // Calculate the projection of the point on the line (pointProj = A + k * B)
      projected_points.points[i].x = mx + k * l1.x;
      projected_points.points[i].y = my + k * l1.y;
      projected_points.points[i].z = mz + k * l1.z;

      // Copy the other attributes
      for (unsigned int d = 0; d < projected_points.get_channels_size (); d++)
        projected_points.channels[d].values[i] = cloud_->channels[d].values[inliers[i]];
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Project the inliers onto their closest lines in place.
    * \param inliers The indices of the inliers into the data cloud.
    * \param model_coefficients The coefficients of the parallel lines model.
    */
  void
    SACModelParallelLines::projectPointsInPlace (const std::vector<int> &inliers, const std::vector<double> &model_coefficients)
  {

    // Compute the closest distances from the pts to the lines.
    std::vector<int> closest_line (inliers.size ());
    std::vector<double> closest_dist (inliers.size ());
    closestLine (inliers, model_coefficients, &closest_line, &closest_dist);

    geometry_msgs::Point32 l1;
    l1.x = model_coefficients[3] - model_coefficients[0];
    l1.y = model_coefficients[4] - model_coefficients[1];
    l1.z = model_coefficients[5] - model_coefficients[2];
    double l_sqr_length = (l1.x * l1.x + l1.y * l1.y + l1.z * l1.z);

    // Iterate through the 3d points and project them onto their closest line.
    for (unsigned int i = 0; i < inliers.size(); i++)
    {
      double mx,my,mz;
      if (closest_line[i] == 0)
      {
        mx = model_coefficients[0];
        my = model_coefficients[1];
        mz = model_coefficients[2];
      }
      else
      {
        mx = model_coefficients[6];
        my = model_coefficients[7];
        mz = model_coefficients[8];
      }
      double k = (
                  ( cloud_->points[inliers[i]].x * l1.x + cloud_->points[inliers[i]].y * l1.y + cloud_->points[inliers[i]].z * l1.z ) -
                  ( mx * l1.x + my * l1.y + mz * l1.z )
                 ) / l_sqr_length;
      // Calculate the projection of the point on the line (pointProj = A + k * B)
      cloud_->points[inliers[i]].x = mx + k * l1.x;
      cloud_->points[inliers[i]].y = my + k * l1.y;
      cloud_->points[inliers[i]].z = mz + k * l1.z;
    }
  }



  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute the model coefficients from the samples and store them internally in model_coefficients_. The line coefficients are represented by the points themselves.
    * \param samples The point indices found as possible good candidates for creating a valid model.
    */
  bool
    SACModelParallelLines::computeModelCoefficients (const std::vector<int> &samples)
  {
    model_coefficients_.resize (9);
    model_coefficients_[0] = cloud_->points[samples[0]].x;
    model_coefficients_[1] = cloud_->points[samples[0]].y;
    model_coefficients_[2] = cloud_->points[samples[0]].z;
    model_coefficients_[3] = cloud_->points[samples[1]].x;
    model_coefficients_[4] = cloud_->points[samples[1]].y;
    model_coefficients_[5] = cloud_->points[samples[1]].z;
    model_coefficients_[6] = cloud_->points[samples[2]].x;
    model_coefficients_[7] = cloud_->points[samples[2]].y;
    model_coefficients_[8] = cloud_->points[samples[2]].z;

    return (true);
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recompute the parallel lines coefficients using the given inlier set and return them to the user.
    * @note: These are the coefficients of the line model after refinement.
    * \param inliers The data inliers found as supporting the model.
    * \param refit_coefficients the resultant recomputed coefficients after non-linear optimization
    */
  void
    SACModelParallelLines::refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients)
  {
    if (inliers.size () == 0)
    {
      ROS_ERROR ("[SACModelParallelLines::RefitModel] Cannot re-fit 0 inliers!");
      refit_coefficients = model_coefficients_;
      return;
    }

    // Get the distances from the inliers to their closest line
    std::vector<int> closest_line (inliers.size ());
    std::vector<double> closest_dist (inliers.size ());
    closestLine (inliers, model_coefficients_, &closest_line, &closest_dist);

    refit_coefficients.resize (9);

    // Compute the centroids of the two sets of samples
    geometry_msgs::Point32 centroid1, centroid2, centroid;
    std::vector<int> inliers1, inliers2;
    int end = inliers.size ();
    for (int i=0; i < end; ++i)
    {
      (closest_line[i] == 0) ? inliers1.push_back (inliers[i]) : inliers2.push_back (inliers[i]);
    }
    cloud_geometry::nearest::computeCentroid (*cloud_, inliers1, centroid1);
    cloud_geometry::nearest::computeCentroid (*cloud_, inliers2, centroid2);

    // Remove the centroids from the two sets of inlier samples to center everything at (0,0)
    sensor_msgs::PointCloud zero_cloud;
    zero_cloud.points.resize (inliers.size ());
    geometry_msgs::Point32 tpoint;
    for (unsigned int i = 0; i < inliers1.size (); i++)
    {
      tpoint.x = cloud_->points[inliers1[i]].x - centroid1.x;
      tpoint.y = cloud_->points[inliers1[i]].y - centroid1.y;
      tpoint.z = cloud_->points[inliers1[i]].z - centroid1.z;
      zero_cloud.points.push_back (tpoint);
    }
    for (unsigned int i = 0; i < inliers2.size (); i++)
    {
      tpoint.x = cloud_->points[inliers2[i]].x - centroid2.x;
      tpoint.y = cloud_->points[inliers2[i]].y - centroid2.y;
      tpoint.z = cloud_->points[inliers2[i]].z - centroid2.z;
      zero_cloud.points.push_back (tpoint);
    }

    // Compute the 3x3 covariance matrix
    Eigen::Matrix3d covariance_matrix;
    std::vector<int> zero_inliers (zero_cloud.points.size ());
    for (unsigned int i = 0; i < zero_cloud.points.size (); i++)
    {
      zero_inliers[i] = i;
    }
    cloud_geometry::nearest::computeCovarianceMatrix (zero_cloud, zero_inliers, covariance_matrix, centroid);

    // Be careful about the new centroids! I'm using centroid 1 and 2 b/c there is no guarantee that there are the same number
    // of points on both lines. Originally, for the 1-line case, this was set to the refit centroid from the covariance matrix.

    refit_coefficients[0] = centroid1.x;
    refit_coefficients[1] = centroid1.y;
    refit_coefficients[2] = centroid1.z;

    refit_coefficients[6] = centroid2.x;
    refit_coefficients[7] = centroid2.y;
    refit_coefficients[8] = centroid2.z;

    // Extract the eigenvalues and eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
    Eigen::Vector3d eigen_values  = ei_symm.eigenvalues ();
    Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors ();

    refit_coefficients[3] = eigen_vectors(0, 2) + refit_coefficients[0];
    refit_coefficients[4] = eigen_vectors(1, 2) + refit_coefficients[1];
    refit_coefficients[5] = eigen_vectors(2, 2) + refit_coefficients[2];
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Verify whether a subset of indices verifies the internal line model coefficients.
    * \param indices The data indices that need to be tested against the line model.
    * \param threshold A maximum admissible distance threshold for determining the inliers from the outliers.
    */
  bool
    SACModelParallelLines::doSamplesVerifyModel (const std::set<int> &indices, double threshold)
  {
    std::vector<int> closest_line (indices.size ());
    std::vector<double> closest_dist (indices.size ());
    closestLine (indices, model_coefficients_, &closest_line, &closest_dist);

    for (unsigned int i = 0; i < closest_dist.size (); ++i)
    {
      if (closest_dist[i] > threshold)
        return (false);
    }
    return (true);
  }
}

