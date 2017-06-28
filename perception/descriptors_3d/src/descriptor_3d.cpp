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

#include <descriptors_3d/descriptor_3d.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
Descriptor3D::Descriptor3D()
{
  result_size_ = 0;
  result_size_defined_ = false;
  debug_ = false;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
Descriptor3D::~Descriptor3D()
{
}


// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Descriptor3D::compute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const vector<const geometry_msgs::Point32*>& interest_pts,
                           vector<vector<float> >& results)
{
  // ----------------------------------------
  // Allocate the results to be 0 vectors for each interest point
  unsigned int nbr_interest_pts = interest_pts.size();
  results.clear();
  results.resize(nbr_interest_pts);

  if (result_size_defined_ == false)
  {
    ROS_ERROR("Descriptor3D::compute result size not defined yet");
    return;
  }

  if (precompute(data, data_kdtree, interest_pts) < 0)
  {
    return;
  }

  return doComputation(data, data_kdtree, interest_pts, results);
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Descriptor3D::compute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const vector<const vector<int>*>& interest_region_indices,
                           vector<vector<float> >& results)
{
  // ----------------------------------------
  // Allocate the results to be 0 vectors for each interest region
  unsigned int nbr_interest_regions = interest_region_indices.size();
  results.clear();
  results.resize(nbr_interest_regions);

  if (result_size_defined_ == false)
  {
    ROS_ERROR("Descriptor3D::compute result size not defined yet");
    return;
  }

  if (precompute(data, data_kdtree, interest_region_indices) < 0)
  {
    return;
  }

  return doComputation(data, data_kdtree, interest_region_indices, results);
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Descriptor3D::concatenateFeatures(const vector<vector<vector<float> > >& all_descriptor_results,
                                       const unsigned int nbr_samples,
                                       const unsigned int nbr_concatenated_vals,
                                       vector<boost::shared_array<const float> >& concatenated_features)
{
  concatenated_features.assign(nbr_samples, boost::shared_array<const float>(NULL));
  unsigned int nbr_descriptors = all_descriptor_results.size();

  // ----------------------------------------------
  // Iterate over each interest point and compute all feature descriptors
  // If all descriptor computations are successful, then concatenate all values into 1 array
  int int_nbr_samples = static_cast<int> (nbr_samples);
#pragma omp parallel for
  for (int i = 0 ; i < int_nbr_samples ; i++)
  {
    // --------------------------------
    // Verify all features for the point were computed successfully
    bool all_features_success = true; // flag if all descriptors were computed correctly
    for (unsigned int j = 0 ; all_features_success && j < nbr_descriptors ; j++)
    {
      // Get the computed features vector for all interest samples
      const vector<vector<float> >& curr_descriptor_for_cloud = all_descriptor_results[j];

      // Get the computed feature for current sample
      const vector<float>& curr_feature_vals =
          curr_descriptor_for_cloud[static_cast<size_t> (i)];

      // non-zero descriptor length indicates computed successfully
      unsigned int curr_nbr_feature_vals = curr_feature_vals.size();
      all_features_success = curr_nbr_feature_vals != 0;
    }

    // --------------------------------
    // If all successful, then concatenate feature values into one vector
    if (all_features_success)
    {
      // allocate
      float* curr_sample_concat_feats = new float[nbr_concatenated_vals];

      // concatenate each descriptors' values together
      unsigned int prev_val_idx = 0; // offset when copying into concat_features
      for (unsigned int j = 0 ; j < nbr_descriptors ; j++)
      {
        // retrieve descriptor values for current point
        const vector<vector<float> >& curr_descriptor_for_cloud = all_descriptor_results[j];
        const vector<float>& curr_sample_descriptor_vals =
            curr_descriptor_for_cloud[static_cast<size_t> (i)];
        unsigned int curr_nbr_feature_vals = curr_sample_descriptor_vals.size();

        // copy descriptor values into concatenated vector at correct location
        memcpy(curr_sample_concat_feats + prev_val_idx, &(*(curr_sample_descriptor_vals.begin())),
            sizeof(float) * curr_nbr_feature_vals);

        // skip over all computed features so far
        prev_val_idx += curr_nbr_feature_vals;
      }

      // Save it
      concatenated_features[i].reset(static_cast<const float*> (curr_sample_concat_feats));
    }
    // Otherwise features not successful
    else
    {
      ROS_DEBUG("Descriptor3D::concatenateFeatures() skipping sample %u", i);
    }
  }

  return;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
unsigned int Descriptor3D::computeAndConcatFeatures(const sensor_msgs::PointCloud& data,
                                                    cloud_kdtree::KdTree& data_kdtree,
                                                    const vector<const geometry_msgs::Point32*>& interest_pts,
                                                    vector<Descriptor3D*>& descriptors_3d,
                                                    vector<boost::shared_array<const float> >& concatenated_features)

{
  // ----------------------------------------------
  // Clear out any shared information from previous compute() calls
  unsigned int nbr_descriptors = descriptors_3d.size();
  for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
  {
    if (descriptors_3d[i] == NULL)
    {
      ROS_ERROR("Descriptor3D::computeAndConcatFeatures() gave NULL descriptor for interest points");
      return 0;
    }
    descriptors_3d[i]->clearShared();
  }

  // ----------------------------------------------
  // Iterate over each descriptor and compute features for each point in the point cloud
  vector<vector<vector<float> > > all_descriptor_results(nbr_descriptors);
  unsigned int nbr_concatenated_vals = 0;
  for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
  {
    descriptors_3d[i]->compute(data, data_kdtree, interest_pts, all_descriptor_results[i]);

    nbr_concatenated_vals += (descriptors_3d[i])->getResultSize();

    ROS_DEBUG("Interest point descriptor has %u values", descriptors_3d[i]->getResultSize());
  }

  concatenateFeatures(all_descriptor_results, interest_pts.size(), nbr_concatenated_vals,
      concatenated_features);
  return nbr_concatenated_vals;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
unsigned int Descriptor3D::computeAndConcatFeatures(const sensor_msgs::PointCloud& data,
                                                    cloud_kdtree::KdTree& data_kdtree,
                                                    const vector<const vector<int>*>& interest_region_indices,
                                                    vector<Descriptor3D*>& descriptors_3d,
                                                    vector<boost::shared_array<const float> >& concatenated_features)

{
  // ----------------------------------------------
  // Clear out any shared information from previous compute() calls
  unsigned int nbr_descriptors = descriptors_3d.size();
  for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
  {
    if (descriptors_3d[i] == NULL)
    {
      ROS_ERROR("Descriptor3D::computeAndConcatFeatures() gave NULL descriptor for interest points");
      return 0;
    }
    descriptors_3d[i]->clearShared();
  }

  // ----------------------------------------------
  // Iterate over each descriptor and compute features for each point in the point cloud
  vector<vector<vector<float> > > all_descriptor_results(nbr_descriptors);
  unsigned int nbr_concatenated_vals = 0;
  for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
  {
    descriptors_3d[i]->compute(data, data_kdtree, interest_region_indices,
        all_descriptor_results[i]);

    nbr_concatenated_vals += (descriptors_3d[i])->getResultSize();

    ROS_DEBUG("Interest region descriptor has %u values", descriptors_3d[i]->getResultSize());
  }

  concatenateFeatures(all_descriptor_results, interest_region_indices.size(),
      nbr_concatenated_vals, concatenated_features);
  return nbr_concatenated_vals;
}

