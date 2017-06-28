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

#include <descriptors_3d/spin_image_custom.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
SpinImageCustom::SpinImageCustom(const double ref_x,
                                 const double ref_y,
                                 const double ref_z,
                                 const double row_res,
                                 const double col_res,
                                 const unsigned int nbr_rows,
                                 const unsigned int nbr_cols,
                                 const bool use_interest_regions_only)
{
  custom_axis_[0] = ref_x;
  custom_axis_[1] = ref_y;
  custom_axis_[2] = ref_z;
  row_res_ = row_res;
  col_res_ = col_res;
  nbr_rows_ = nbr_rows;
  nbr_cols_ = nbr_cols;

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
void SpinImageCustom::clearShared()
{
}



// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
std::string SpinImageCustom::getName() const
{
  ostringstream oss;
  oss << "SpinImageCustom_axis" << custom_axis_[0] << "," << custom_axis_[1] << "," << custom_axis_[2];
  oss << "_rowRes" << row_res_ << "_colRes" << col_res_ << "_numRows" << nbr_rows_ << "_numCols" << nbr_cols_;
  oss << "_radius" << neighborhood_radius_; 
  return oss.str();
}


// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SpinImageCustom::precompute(const sensor_msgs::PointCloud& data,
                                cloud_kdtree::KdTree& data_kdtree,
                                const vector<const geometry_msgs::Point32*>& interest_pts)
{
  // Point spin_axis_ to duplicate copies of the custom spin axis
  size_t nbr_interest_pts = interest_pts.size();
  custom_axis_duplicated_.assign(nbr_interest_pts, &custom_axis_);
  spin_axes_ = &custom_axis_duplicated_;

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
int SpinImageCustom::precompute(const sensor_msgs::PointCloud& data,
                                cloud_kdtree::KdTree& data_kdtree,
                                const vector<const std::vector<int>*>& interest_region_indices)
{
  // Point spin_axis_ to duplicate copies of the custom spin axis
  size_t nbr_interest_regions = interest_region_indices.size();
  custom_axis_duplicated_.assign(nbr_interest_regions, &custom_axis_);
  spin_axes_ = &custom_axis_duplicated_;

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

