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

#include <descriptors_3d/channel.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
Channel::Channel(const string& channel_name)
{
  channel_name_ = channel_name;
  result_size_ = 1;
  result_size_defined_ = true;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Channel::clearShared()
{
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
std::string Channel::getName() const
{
  return string("TODO: Add a name to this feature.");
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int Channel::precompute(const sensor_msgs::PointCloud& data,
                        cloud_kdtree::KdTree& data_kdtree,
                        const vector<const geometry_msgs::Point32*>& interest_pts)
{
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int Channel::precompute(const sensor_msgs::PointCloud& data,
                        cloud_kdtree::KdTree& data_kdtree,
                        const vector<const std::vector<int>*>& interest_region_indices)
{
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Channel::doComputation(const sensor_msgs::PointCloud& data,
                            cloud_kdtree::KdTree& data_kdtree,
                            const vector<const geometry_msgs::Point32*>& interest_pts,
                            vector<vector<float> >& results)
{
  // ----------------------------------------------
  // Lookup the channel index corresponding to the given name
  int channel_idx = cloud_geometry::getChannelIndex(data, channel_name_);
  if (channel_idx < 0)
  {
    ROS_ERROR("Channel::doComputation() could not find channel %s in given point cloud", channel_name_.c_str());
    return;
  }
  const std::vector<sensor_msgs::ChannelFloat32>& pt_cloud_chans = data.channels;

  // ----------------------------------------------
  // For each interest point, look up its index in the point cloud and then copy the
  // specified channel value
  int nbr_interest_pts = interest_pts.size();
#pragma omp parallel for
  for (int i = 0 ; i < nbr_interest_pts ; i++)
  {
    // --------------------------
    if (interest_pts[static_cast<size_t> (i)] == NULL)
    {
      ROS_WARN("Channel::doComputation() passed NULL interest point %u", i);
      continue;
    }
    const geometry_msgs::Point32& curr_interest_pt = *(interest_pts[static_cast<size_t> (i)]);

    // --------------------------
    // Look for the interest point in the point cloud
    vector<int> neighbor_indices;
    vector<float> neighbor_distances; // unused
    data_kdtree.nearestKSearch(curr_interest_pt, 1, neighbor_indices, neighbor_distances);
    unsigned int nearest_pt_idx = neighbor_indices.at(0);
    const geometry_msgs::Point32& nearest_pt = data.points[nearest_pt_idx];

    // --------------------------
    // Verify the nearest point is the same as the given interest point
    if (cloud_geometry::distances::pointToPointDistanceSqr(curr_interest_pt, nearest_pt) > 1e-8)
    {
      ROS_WARN("Channel::doComputation() could not find point (%f,%f,%f) in the given point cloud",
          curr_interest_pt.x, curr_interest_pt.y, curr_interest_pt.z);
      continue;
    }

    // --------------------------
    // Copy the channel value
    results[static_cast<size_t> (i)].resize(result_size_);
    results[static_cast<size_t> (i)][0] = pt_cloud_chans[channel_idx].values[nearest_pt_idx];
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Channel::doComputation(const sensor_msgs::PointCloud& data,
                            cloud_kdtree::KdTree& data_kdtree,
                            const vector<const vector<int>*>& interest_region_indices,
                            vector<vector<float> >& results)
{
  // ----------------------------------------------
  // Lookup the channel index corresponding to the given name
  int channel_idx = cloud_geometry::getChannelIndex(data, channel_name_);
  if (channel_idx < 0)
  {
    ROS_ERROR("Channel::doComputation() could not find channel %s in given point cloud", channel_name_.c_str());
    return;
  }
  const std::vector<sensor_msgs::ChannelFloat32>& pt_cloud_chans = data.channels;

  // ----------------------------------------------
  // For each interest region, look up each of its points index in the point cloud
  // and then copy the region's average specified channel value
  int nbr_interest_regions = interest_region_indices.size();
#pragma omp parallel for
  for (int i = 0 ; i < nbr_interest_regions ; i++)
  {
    // --------------------------
    if (interest_region_indices[static_cast<size_t> (i)] == NULL)
    {
      ROS_WARN("Channel::doComputation passed NULL interest region %u", i);
      continue;
    }
    const vector<int>& curr_interest_region = *(interest_region_indices[static_cast<size_t> (i)]);

    // --------------------------
    // Initialize average to 0
    results[static_cast<size_t> (i)].resize(result_size_);
    results[static_cast<size_t> (i)][0] = 0.0;

    // --------------------------
    // Sum each interest points' channel value
    unsigned int nbr_pts_in_region = curr_interest_region.size();
    for (unsigned int j = 0 ; j < nbr_pts_in_region ; j++)
    {
      unsigned int curr_interest_pt_idx = static_cast<unsigned int> (curr_interest_region[j]);
      results[static_cast<size_t> (i)][0] += pt_cloud_chans[channel_idx].values.at(
          curr_interest_pt_idx);
    }

    // --------------------------
    // Average
    results[static_cast<size_t> (i)][0] /= static_cast<float> (nbr_pts_in_region);
  }
}
