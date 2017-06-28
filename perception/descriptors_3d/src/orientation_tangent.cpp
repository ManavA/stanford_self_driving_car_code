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

#include <descriptors_3d/orientation_tangent.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
OrientationTangent::OrientationTangent(const double ref_x,
                                       const double ref_y,
                                       const double ref_z,
                                       SpectralAnalysis& spectral_information)
{
  reference_direction_[0] = ref_x;
  reference_direction_[1] = ref_y;
  reference_direction_[2] = ref_z;
  reference_direction_flipped_[0] = -reference_direction_[0];
  reference_direction_flipped_[1] = -reference_direction_[1];
  reference_direction_flipped_[2] = -reference_direction_[2];

  spectral_information_ = &spectral_information;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
string OrientationTangent::getName() const
{
  return string("TODO: Add a name to this feature.");
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void OrientationTangent::clearShared()
{
  spectral_information_->clearSpectral();
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int OrientationTangent::precompute(const sensor_msgs::PointCloud& data,
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
  local_directions_ = &(spectral_information_->getTangents());

  // verify the normals are for the interest points
  if (local_directions_->size() != interest_pts.size())
  {
    ROS_ERROR("OrientationTangent::precompute() inconsistent number of points and spectral info");
    local_directions_ = NULL;
    return -1;
  }

  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int OrientationTangent::precompute(const sensor_msgs::PointCloud& data,
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
  local_directions_ = &(spectral_information_->getTangents());

  // verify the normals are for the interest regions
  if (local_directions_->size() != interest_region_indices.size())
  {
    ROS_ERROR("OrientationTangent::precompute() inconsistent number of regions and spectral info");
    local_directions_ = NULL;
    return -1;
  }

  return 0;
}

