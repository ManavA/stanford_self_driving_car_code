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

#include <descriptors_3d/curvature.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
Curvature::Curvature(SpectralAnalysis& spectral_information)
{
  result_size_ = 1;
  result_size_defined_ = true;

  spectral_information_ = &spectral_information;
  eig_vals_ = NULL;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Curvature::clearShared()
{
  spectral_information_->clearSpectral();
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
std::string Curvature::getName() const
{
  return string("TODO: Add a name to this feature.");
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int Curvature::precompute(const sensor_msgs::PointCloud& data,
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
  eig_vals_ = &(spectral_information_->getEigenValues());

  // verify the eigenvectors are for the interest points
  if (eig_vals_->size() != interest_pts.size())
  {
    ROS_ERROR("Curvature::precompute() inconsistent number of points and spectral info");
    eig_vals_ = NULL;
    return -1;
  }

  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int Curvature::precompute(const sensor_msgs::PointCloud& data,
                          cloud_kdtree::KdTree& data_kdtree,
                          const vector<const std::vector<int>*>& interest_region_indices)
{
  // Compute spectral information if not already done
  if (spectral_information_->isSpectralComputed() == false)
  {
    if (spectral_information_->analyzeInterestRegions(data, data_kdtree, interest_region_indices)
        < 0)
    {
      return -1;
    }
  }

  // Retrieve necessary spectral information this class needs to compute features
  eig_vals_ = &(spectral_information_->getEigenValues());

  // verify the eigenvectors are for the interest regions
  if (eig_vals_->size() != interest_region_indices.size())
  {
    ROS_ERROR("Curvature::precompute() inconsistent number of regions and spectral info");
    eig_vals_ = NULL;
    return -1;
  }

  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Curvature::doComputation(const sensor_msgs::PointCloud& data,
                              cloud_kdtree::KdTree& data_kdtree,
                              const vector<const geometry_msgs::Point32*>& interest_pts,
                              vector<vector<float> >& results)
{
  // ----------------------------------------
  // Compute the curvature for each interest point
  int nbr_interest_pts = interest_pts.size();
#pragma omp parallel for
  for (int i = 0 ; i < nbr_interest_pts ; i++)
  {
    computeCurvature(i, results[static_cast<size_t> (i)]);
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Curvature::doComputation(const sensor_msgs::PointCloud& data,
                              cloud_kdtree::KdTree& data_kdtree,
                              const vector<const std::vector<int>*>& interest_region_indices,
                              vector<vector<float> >& results)
{
  // ----------------------------------------
  // Compute the curvature for each interest region
  int nbr_interest_regions = interest_region_indices.size();
#pragma omp parallel for
  for (int i = 0 ; i < nbr_interest_regions ; i++)
  {
    computeCurvature(i, results[static_cast<size_t> (i)]);
  }
}

// --------------------------------------------------------------
/* See function definition
 * Invariant: interest_sample_idx is within bounds */
// --------------------------------------------------------------
void Curvature::computeCurvature(const unsigned int interest_sample_idx, vector<float>& result) const
{
  // Retrieve the eigenvalues for current interest point/region
  const Eigen::Vector3d* curr_eigen_vals = (*eig_vals_)[interest_sample_idx];

  // NULL indicates couldnt compute spectral information for interest sample
  if (curr_eigen_vals != NULL)
  {
    result.resize(result_size_);
    result[0] = static_cast<float> ((*curr_eigen_vals)[0] / ((*curr_eigen_vals)[0]
        + (*curr_eigen_vals)[1] + (*curr_eigen_vals)[2]));
  }
}
