/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <descriptors_3d/bounding_box_spectral.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
BoundingBoxSpectral::BoundingBoxSpectral(double bbox_radius, SpectralAnalysis& spectral_information)
{
  result_size_ = 3;
  result_size_defined_ = true;

  neighborhood_radius_ = bbox_radius;
  neighborhood_radius_defined_ = true;

  eig_vecs_min_ = NULL;
  eig_vecs_mid_ = NULL;
  eig_vecs_max_ = NULL;
  spectral_information_ = &spectral_information;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void BoundingBoxSpectral::clearShared()
{
  spectral_information_->clearSpectral();
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
std::string BoundingBoxSpectral::getName() const
{
  ostringstream oss;
  oss << "BoundingBoxSpectral_radius" << neighborhood_radius_ << "_spectralRadius" << spectral_information_->getRadius();
  return oss.str();
}


// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int BoundingBoxSpectral::precompute(const sensor_msgs::PointCloud& data,
                                    cloud_kdtree::KdTree& data_kdtree,
                                    const vector<const geometry_msgs::Point32*>& interest_pts)
{
  // Compute spectral information if not already done
  if (spectral_information_->isSpectralComputed() == false)
  {
    if (spectral_information_->analyzeInterestPoints(data, data_kdtree, interest_pts) < 0)
    {
      return -1;
    }
  }

  // Retrieve necessary spectral information this class needs to compute features
  eig_vecs_min_ = &(spectral_information_->getNormals());
  eig_vecs_mid_ = &(spectral_information_->getMiddleEigenVectors());
  eig_vecs_max_ = &(spectral_information_->getTangents());

  // verify the eigenvectors are for the interest points
  if (eig_vecs_min_->size() != interest_pts.size())
  {
    ROS_ERROR("BoundingBoxSpectral::precompute() inconsistent number of points and spectral info");
    eig_vecs_min_ = NULL;
    eig_vecs_mid_ = NULL;
    eig_vecs_max_ = NULL;
    return -1;
  }

  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int BoundingBoxSpectral::precompute(const sensor_msgs::PointCloud& data,
                                    cloud_kdtree::KdTree& data_kdtree,
                                    const vector<const std::vector<int>*>& interest_region_indices)
{
  // Compute spectral information if not already done
  if (spectral_information_->isSpectralComputed() == false)
  {
    if (spectral_information_->analyzeInterestRegions(data, data_kdtree, interest_region_indices) < 0)
    {
      return -1;
    }
  }

  // Retrieve necessary spectral information this class needs to compute features
  eig_vecs_min_ = &(spectral_information_->getNormals());
  eig_vecs_mid_ = &(spectral_information_->getMiddleEigenVectors());
  eig_vecs_max_ = &(spectral_information_->getTangents());

  // Verify the eigenvectors are for the interest regions
  if (eig_vecs_min_->size() != interest_region_indices.size())
  {
    ROS_ERROR("BoundingBoxSpectral::precompute() inconsistent number of regions and spectral info");
    eig_vecs_min_ = NULL;
    eig_vecs_mid_ = NULL;
    eig_vecs_max_ = NULL;
    return -1;
  }

  return 0;
}

// --------------------------------------------------------------
/* See function definition
 * Invariant: interest_sample_idx is in bounds of field containers.
 * Invariant: Eigenvectors are unit */
// --------------------------------------------------------------
void BoundingBoxSpectral::computeNeighborhoodFeature(const sensor_msgs::PointCloud& data,
                                                     const vector<int>& neighbor_indices,
                                                     const unsigned int interest_sample_idx,
                                                     vector<float>& result) const
{
  // Verify the principle components were extracted
  const Eigen::Vector3d* eig_vec_max = (*eig_vecs_max_)[interest_sample_idx];
  const Eigen::Vector3d* eig_vec_mid = (*eig_vecs_mid_)[interest_sample_idx];
  const Eigen::Vector3d* eig_vec_min = (*eig_vecs_min_)[interest_sample_idx];
  if (eig_vec_max == NULL)
  {
    ROS_DEBUG("BoundingBoxSpectral::computeNeighborhoodFeature() No spectral information for sample %u", interest_sample_idx);
    return;
  }

  result.resize(result_size_);
  const unsigned int nbr_neighbors = neighbor_indices.size();

  // --------------------------
  // Check for special case when no points in the bounding box as will initialize
  // the min/max extremas using the first point below
  if (nbr_neighbors == 0)
  {
    ROS_INFO("BoundingBoxSpectral::computeNeighborhoodFeature() No points to form bounding box");
    for (size_t i = 0 ; i < result_size_ ; i++)
    {
      result[i] = 0.0;
    }
    return;
  }

  // Initialize extrema values of the first point's scalar projections
  // onto the principle components
  const geometry_msgs::Point32& first_sensor_point = data.points.at(neighbor_indices[0]);
  Eigen::Vector3d curr_pt;
  curr_pt[0] = first_sensor_point.x;
  curr_pt[1] = first_sensor_point.y;
  curr_pt[2] = first_sensor_point.z;
  float min_v1 = curr_pt.dot(*eig_vec_max);
  float min_v2 = curr_pt.dot(*eig_vec_mid);
  float min_v3 = curr_pt.dot(*eig_vec_min);
  float max_v1 = min_v1;
  float max_v2 = min_v2;
  float max_v3 = min_v3;

  // Loop over remaining points in region and update projection extremas
  for (unsigned int i = 1 ; i < nbr_neighbors ; i++)
  {
    const geometry_msgs::Point32& curr_sensor_pt = data.points.at(neighbor_indices[i]);
    curr_pt[0] = curr_sensor_pt.x;
    curr_pt[1] = curr_sensor_pt.y;
    curr_pt[2] = curr_sensor_pt.z;

    // extrema along biggest eigenvector
    float curr_projection = curr_pt.dot(*eig_vec_max);
    if (curr_projection < min_v1)
    {
      min_v1 = curr_projection;
    }
    if (curr_projection > max_v1)
    {
      max_v1 = curr_projection;
    }
    // extrema along middle eigenvector
    curr_projection = curr_pt.dot(*eig_vec_mid);
    if (curr_projection < min_v2)
    {
      min_v2 = curr_projection;
    }
    if (curr_projection > max_v2)
    {
      max_v2 = curr_projection;
    }
    // extrema along smallest eigenvector
    curr_projection = curr_pt.dot(*eig_vec_min);
    if (curr_projection < min_v3)
    {
      min_v3 = curr_projection;
    }
    if (curr_projection > max_v3)
    {
      max_v3 = curr_projection;
    }
  }

  // --------------------------
  result[0] = max_v1 - min_v1;
  result[1] = max_v2 - min_v2;
  result[2] = max_v3 - min_v3;
}
