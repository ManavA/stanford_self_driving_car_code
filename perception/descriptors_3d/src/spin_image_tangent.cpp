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

#include <descriptors_3d/spin_image_tangent.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
SpinImageTangent::SpinImageTangent(const double row_res,
                                   const double col_res,
                                   const unsigned int nbr_rows,
                                   const unsigned int nbr_cols,
                                   const bool use_interest_regions_only,
                                   SpectralAnalysis& spectral_information)
{
  row_res_ = row_res;
  col_res_ = col_res;
  nbr_rows_ = nbr_rows;
  nbr_cols_ = nbr_cols;
  spectral_information_ = &spectral_information;

  result_size_ = nbr_rows * nbr_cols;
  result_size_defined_ = true;

  if (use_interest_regions_only)
  {
    neighborhood_radius_ = -1.0;
  }
  else
  {
    neighborhood_radius_ = sqrt(pow(row_res_ * nbr_rows_, 2.0) + pow(col_res_ * nbr_cols_, 2.0));
  }
  neighborhood_radius_defined_ = true;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SpinImageTangent::clearShared()
{
  spectral_information_->clearSpectral();
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
std::string SpinImageTangent::getName() const
{
  return string("TODO: Add a name to this feature.");
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SpinImageTangent::precompute(const sensor_msgs::PointCloud& data,
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
  spin_axes_ = &(spectral_information_->getTangents());

  // Verify the tangents are for the interest points
  size_t nbr_interest_pts = interest_pts.size();
  if (spin_axes_->size() != nbr_interest_pts)
  {
    ROS_ERROR("SpinImageTangent::precompute() inconsistent number of points and spectral info");
    spin_axes_ = NULL;
    return -1;
  }

  // Copy the center of the spin images as "Eigen" vectors
  spin_image_centers_.resize(nbr_interest_pts);
  for (size_t i = 0 ; i < nbr_interest_pts ; i++)
  {
    // Will be handled in NeighborhoodFeature::doComputation() if interest point is NULL
    if (interest_pts[i] != NULL)
    {
      spin_image_centers_[i][0] = (interest_pts[i])->x;
      spin_image_centers_[i][1] = (interest_pts[i])->y;
      spin_image_centers_[i][2] = (interest_pts[i])->z;
    }
  }

  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SpinImageTangent::precompute(const sensor_msgs::PointCloud& data,
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
  spin_axes_ = &(spectral_information_->getTangents());

  // Verify the tangents are for the interest regions
  size_t nbr_interest_regions = interest_region_indices.size();
  if (spin_axes_->size() != nbr_interest_regions)
  {
    ROS_ERROR("SpinImageTangent::precompute() inconsistent number of regions and spectral info");
    spin_axes_ = NULL;
    return -1;
  }

  // Copy the center (centroid of regions) of the spin images as "Eigen" vectors
  spin_image_centers_.resize(nbr_interest_regions);
  for (size_t i = 0 ; i < nbr_interest_regions ; i++)
  {
    // Will be handled in NeighborhoodFeature::doComputation() if region_indices is NULL
    // Will be handled in SpinImageGeneric::computeNeighborhoodFeature() if spin axis is NULL
    if (interest_region_indices[i] != NULL && (*spin_axes_)[i] != NULL)
    {
      geometry_msgs::Point32 region_centroid;
      cloud_geometry::nearest::computeCentroid(data, *(interest_region_indices[i]), region_centroid);
      spin_image_centers_[i][0] = region_centroid.x;
      spin_image_centers_[i][1] = region_centroid.y;
      spin_image_centers_[i][2] = region_centroid.z;
    }
  }

  return 0;
}

