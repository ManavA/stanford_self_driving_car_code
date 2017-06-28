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

#include <descriptors_3d/shared/spectral_analysis.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
SpectralAnalysis::SpectralAnalysis(double support_radius)
{
  support_radius_ = support_radius;
  spectral_computed_ = false;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
SpectralAnalysis::~SpectralAnalysis()
{
  clearSpectral();
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SpectralAnalysis::clearSpectral()
{
  unsigned int nbr_data = normals_.size();
  for (unsigned int i = 0 ; i < nbr_data ; i++)
  {
    if (normals_[i] != NULL)
    {
      delete normals_[i];
      delete middle_eig_vecs_[i];
      delete tangents_[i];
      delete eigenvalues_[i];
    }
  }
  normals_.clear();
  middle_eig_vecs_.clear();
  tangents_.clear();
  eigenvalues_.clear();
  spectral_computed_ = false;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SpectralAnalysis::analyzeInterestPoints(const sensor_msgs::PointCloud& data,
                                            cloud_kdtree::KdTree& data_kdtree,
                                            const vector<const geometry_msgs::Point32*>& interest_pts)
{
  if (spectral_computed_)
  {
    ROS_ERROR("SpectralAnalysis::analyzeInterestPoints() spectral info already exists");
    return -1;
  }

  // ----------------------------------------
  // Ensure some regions are provided
  unsigned int nbr_interest_pts = interest_pts.size();

  // ----------------------------------------
  // Ensure radius is valid
  if (support_radius_ < 1e-6)
  {
    ROS_ERROR("SpectralAnalysis::analyzeInterestPoints() support radius must be set to a positive value");
    return -1;
  }

  // ----------------------------------------
  // Allocate accordingly
  normals_.assign(nbr_interest_pts, NULL);
  middle_eig_vecs_.assign(nbr_interest_pts, NULL);
  tangents_.assign(nbr_interest_pts, NULL);
  eigenvalues_.assign(nbr_interest_pts, NULL);

  // ----------------------------------------
  // Find neighboring points within radius for each interest point
  int int_nbr_interest_pts = static_cast<int> (nbr_interest_pts);
#pragma omp parallel for schedule(dynamic)
  for (int i = 0 ; i < int_nbr_interest_pts ; i++)
  {
    // ---------------------
    // Retrieve next interest point
    const geometry_msgs::Point32* curr_interest_pt = interest_pts[static_cast<size_t> (i)];
    if (curr_interest_pt == NULL)
    {
      ROS_WARN("SpectralAnalysis::analyzeInterestPoints() passed NULL interest point");
    }
    else
    {
      // ---------------------
      // Retrieve neighboring points around the interest point
      vector<int> neighbor_indices;
      vector<float> neighbor_distances; // unused
      // radiusSearch returning false (0 neighbors) is okay
      data_kdtree.radiusSearch(*curr_interest_pt, support_radius_, neighbor_indices, neighbor_distances);

      // ---------------------
      // Compute spectral information for interest point
      computeSpectralInfo(data, neighbor_indices, static_cast<size_t> (i));
    }
  }

  spectral_computed_ = true;
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SpectralAnalysis::analyzeInterestRegions(const sensor_msgs::PointCloud& data,
                                             cloud_kdtree::KdTree& data_kdtree,
                                             const vector<const vector<int>*>& interest_region_indices)
{
  if (spectral_computed_)
  {
    ROS_ERROR("SpectralAnalysis::analyzeInterestRegions() spectral info already exists");
    return -1;
  }

  // ----------------------------------------
  // Ensure some regions are provided
  unsigned int nbr_regions = interest_region_indices.size();

  // ----------------------------------------
  // Allocate accordingly
  normals_.assign(nbr_regions, NULL);
  middle_eig_vecs_.assign(nbr_regions, NULL);
  tangents_.assign(nbr_regions, NULL);
  eigenvalues_.assign(nbr_regions, NULL);

  // ----------------------------------------
  // For each interest region, either:
  //   Use the region itself as the support volume
  //   Find a support volume within a radius from the region's centroid
  int int_nbr_regions = static_cast<int> (nbr_regions);
#pragma omp parallel for schedule(dynamic)
  for (int i = 0 ; i < int_nbr_regions ; i++)
  {
    // ---------------------
    // Retrieve next interest region
    // (By default, use the interest region as the support volume)
    const vector<int>* curr_interest_region = interest_region_indices[static_cast<size_t> (i)];
    if (curr_interest_region == NULL)
    {
      ROS_WARN("SpectralAnalysis::analyzeInterestRegions() passed NULL interest region");
    }
    else
    {
      // Do a range search around the interest region's CENTROID if indicated
      vector<int> neighbor_indices;
      if (support_radius_ > 1e-6)
      {
        // Compute centroid of interest region
        geometry_msgs::Point32 region_centroid;
        cloud_geometry::nearest::computeCentroid(data, *curr_interest_region, region_centroid);

        vector<float> neighbor_distances; // unused
        // radiusSearch returning false (0 neighbors) is okay
        data_kdtree.radiusSearch(region_centroid, support_radius_, neighbor_indices, neighbor_distances);

        // Now point to the neighboring points from radiusSearch
        curr_interest_region = &neighbor_indices;
      }

      // ---------------------
      // Compute spectral information for interest region
      computeSpectralInfo(data, *curr_interest_region, static_cast<size_t> (i));
    }
  }

  // ----------------------------------------
  spectral_computed_ = true;
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SpectralAnalysis::computeSpectralInfo(const sensor_msgs::PointCloud& data,
                                           const vector<int>& support_volume_indices,
                                           const size_t idx)
{
  // ----------------------------------------
  // Need 3-by-3 matrix to have full rank
  if (support_volume_indices.size() < 3)
  {
    ROS_DEBUG("SpectralAnalysis::computeSpectralInfo() not enough neighbors for interest sample %u", idx);
    return;
  }

  // ----------------------------------------
  // Allocate for new data
  Eigen::Vector3d* new_normal = new Eigen::Vector3d();
  Eigen::Vector3d* new_middle_eigvec = new Eigen::Vector3d();
  Eigen::Vector3d* new_tangent = new Eigen::Vector3d();
  Eigen::Vector3d* new_eig_vals = new Eigen::Vector3d();

  // ----------------------------------------
  // Eigen-analysis of support volume
  // smallest eigenvalue = index 0
  geometry_msgs::Point32 centroid;
  Eigen::Matrix3d eigen_vectors;
  cloud_geometry::nearest::computePatchEigenNormalized(data, support_volume_indices, eigen_vectors,
      *(new_eig_vals), centroid);

  // ----------------------------------------
  // Populate containers
  for (unsigned int j = 0 ; j < 3 ; j++)
  {
    (*(new_normal))[j] = eigen_vectors(j, 0);
    (*(new_middle_eigvec))[j] = eigen_vectors(j, 1);
    (*(new_tangent))[j] = eigen_vectors(j, 2);
  }

  // Make unit length
  new_normal->normalize();
  new_middle_eigvec->normalize();
  new_tangent->normalize();

  normals_[idx] = new_normal;
  middle_eig_vecs_[idx] = new_middle_eigvec;
  tangents_[idx] = new_tangent;
  eigenvalues_[idx] = new_eig_vals;

}

