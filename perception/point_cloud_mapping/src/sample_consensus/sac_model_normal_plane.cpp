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

/** \author Radu Bogdan Rusu, Jared Glover */

#include <point_cloud_mapping/sample_consensus/sac_model_normal_plane.h>
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/nearest.h>

namespace sample_consensus
{
  int get_channel_index(sensor_msgs::PointCloud *cloud, std::string name)
  {
    for (unsigned int i = 0; i < cloud->channels.size(); i++)
      if (cloud->channels[i].name.compare(name) == 0)
	return i;
    return -1;

  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Select all the points which respect the given model coefficients as inliers.
    * \param model_coefficients the coefficients of a plane model that we need to compute distances to
    * \param inliers the resultant model inliers
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    * \note: To get the refined inliers of a model, use:
    * ANNpoint refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
    */
  void
    SACModelNormalPlane::selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers)
  {
    int nr_p = 0;
    double w = normal_distance_weight_;

    // Obtain the plane normal
    geometry_msgs::Point32 n;
    n.x = model_coefficients.at (0);
    n.y = model_coefficients.at (1);
    n.z = model_coefficients.at (2);
    double nw = model_coefficients.at (3);

    //ROS_INFO("coeff = (%.2f, %.2f, %.2f, %.2f)", n.x, n.y, n.z, nw);

    // check against template, if given
    if (eps_angle_ > 0.0)
      {
	double angle_diff = fabs(cloud_geometry::angles::getAngle3D (axis_, n));
	angle_diff = fmin(angle_diff, M_PI - angle_diff);
	if (angle_diff > eps_angle_)
	  {
	    inliers.resize (0);
	    return;
	  }
      }
    if (eps_dist_ > 0.0)
      {
	double d = -nw;
	if (fabs(d - dist_) > eps_dist_)
	  {
	    inliers.resize (0);
	    return;
	  }
      }

    // Get the point cloud's normal channels (fail if they don't exist)
    int nx_idx = get_channel_index(cloud_, "nx");
    int ny_idx = get_channel_index(cloud_, "ny");
    int nz_idx = get_channel_index(cloud_, "nz");

    //ROS_INFO("normal channels = (%d, %d, %d)", nx_idx, ny_idx, nz_idx);

    if (nx_idx < 0 || ny_idx < 0 || nz_idx < 0)
      {
	inliers.resize (0);
	return;
      }

    inliers.resize (indices_.size ());
    // Iterate through the 3d points and calculate the distances from them to the plane
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      // Calculate the Euclidean distance from the point to the plane normal as the dot product
      // D = (P-A).N/|N|
      geometry_msgs::Point32 p = cloud_->points[indices_.at (i)];
      double d_euclid = fabs(n.x*p.x + n.y*p.y + n.z*p.z + nw);

      // Calculate the angular distance between the point normal and the plane normal
      geometry_msgs::Point32 pn;
      pn.x = cloud_->channels[nx_idx].values[indices_.at (i)];
      pn.y = cloud_->channels[ny_idx].values[indices_.at (i)];
      pn.z = cloud_->channels[nz_idx].values[indices_.at (i)];
      double d_normal = fabs(cloud_geometry::angles::getAngle3D (pn, n));
      d_normal = fmin(d_normal, M_PI - d_normal);

      if (fabs (w*d_normal + (1-w)*d_euclid) < threshold)
      {
	//ROS_INFO("n = (%.2f, %.2f, %.2f), pn = (%.2f, %.2f, %.2f), d_normal = %.2f  -->  *** INLIER ***", n.x, n.y, n.z, pn.x, pn.y, pn.z, d_normal);

        // Returns the indices of the points whose distances are smaller than the threshold
        inliers[nr_p] = indices_[i];
        nr_p++;
      }
      //else
      //  ROS_INFO("n = (%.2f, %.2f, %.2f), pn = (%.2f, %.2f, %.2f), d_normal = %.2f", n.x, n.y, n.z, pn.x, pn.y, pn.z, d_normal);
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
    SACModelNormalPlane::getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances)
  {
    double w = normal_distance_weight_;

    // Obtain the plane normal
    geometry_msgs::Point32 n;
    n.x = model_coefficients.at (0);
    n.y = model_coefficients.at (1);
    n.z = model_coefficients.at (2);
    double nw = model_coefficients.at (3);

    // check against template, if given
    if (eps_angle_ > 0.0)
      {
	double angle_diff = fabs(cloud_geometry::angles::getAngle3D (axis_, n));
	angle_diff = fmin(angle_diff, M_PI - angle_diff);
	if (angle_diff > eps_angle_)
	  {
	    distances.resize (0);
	    return;
	  }
      }
    if (eps_dist_ > 0.0)
      {
	double d = -nw;
	if (fabs(d - dist_) > eps_dist_)
	  {
	    distances.resize (0);
	    return;
	  }
      }

    // Get the point cloud's normal channels (fail if they don't exist)
    int nx_idx = get_channel_index(cloud_, "nx");
    int ny_idx = get_channel_index(cloud_, "ny");
    int nz_idx = get_channel_index(cloud_, "nz");

    //ROS_INFO("nx = %d, ny = %d, nz = %d", nx_idx, ny_idx, nz_idx);  //dbug

    if (nx_idx < 0 || ny_idx < 0 || nz_idx < 0)
      {
	distances.resize (0);
	return;
      }

    distances.resize (indices_.size ());
    // Iterate through the 3d points and calculate the distances from them to the plane
    for (unsigned int i = 0; i < indices_.size (); i++)
      {
	// Calculate the distance from the point to the plane normal as the dot product
	// D = (P-A).N/|N|
	geometry_msgs::Point32 p = cloud_->points[indices_.at (i)];
	double d_euclid = fabs(n.x*p.x + n.y*p.y + n.z*p.z + nw);

	// Calculate the angular distance between the point normal and the plane normal
	geometry_msgs::Point32 pn;
	pn.x = cloud_->channels[nx_idx].values[indices_.at (i)];
	pn.y = cloud_->channels[ny_idx].values[indices_.at (i)];
	pn.z = cloud_->channels[nz_idx].values[indices_.at (i)];
	double d_normal = fabs(cloud_geometry::angles::getAngle3D (pn, n));
	d_normal = fmin(d_normal, M_PI - d_normal);

	distances[i] = fabs (w*d_normal + (1-w)*d_euclid);
      }

    return;
  }

}
