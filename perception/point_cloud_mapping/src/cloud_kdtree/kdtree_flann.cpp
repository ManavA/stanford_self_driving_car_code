/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: kdtree_flann.cpp 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu */

#include "point_cloud_mapping/kdtree/kdtree_flann.h"

namespace cloud_kdtree
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Search for k-nearest neighbors for the given query point.
   * \param p_q the given query point
   * \param k the number of neighbors to search for
   * \param k_indices the resultant point indices
   * \param k_distances the resultant point distances
   */
  void
    KdTreeFLANN::nearestKSearch (const geometry_msgs::Point32 &p_q, int k, std::vector<int> &k_indices, std::vector<float> &k_distances)
  {
    k_indices.resize (k);
    k_distances.resize (k);

    float* p = (float*)malloc (3 * sizeof (float));
    p[0] = p_q.x; p[1] = p_q.y; p[2] = p_q.z;

//    std::cerr << p[0] <<  " " << p[1] << " " << p[2] << std::endl;
    m_lock_.lock ();
//    int* nn_idx_ = (int*) malloc (k * sizeof (int));
//    float* nn_dists_ = (float*) malloc (k * sizeof (float));
    flann_find_nearest_neighbors_index (index_id_, p, 1, &k_indices[0], &k_distances[0], k, flann_param_.checks, &flann_param_);
//    flann_find_nearest_neighbors_index (index_id_, p, 1, nn_idx_, nn_dists_, k, flann_param_.checks, &flann_param_);

//    EXPECT_EQ (indices[0], 0);
//    EXPECT_EQ (indices[1], 12);
//    EXPECT_EQ (indices[2], 198);
//    EXPECT_EQ (indices[3], 1);
//    EXPECT_EQ (indices[4], 127);
//    EXPECT_EQ (indices[5], 18);
//    EXPECT_EQ (indices[6], 132);
//    EXPECT_EQ (indices[7], 10);
//    EXPECT_EQ (indices[8], 11);
//    EXPECT_EQ (indices[9], 197);

    /// 0 12 198 1 127 18 132 10 11 197

    /// 0 12 198 1 2 125 26 278 42 248
    /// 0 12 198 1 18 132 10 197 16 9
    /// 0 12 198 1 18 132 10 197 16 9
    /// 0 12 198 1 18 132 10 197 16 9

    for (int i = 0 ; i < 10; i++)
      std::cerr << k_indices[i] << " ";
    std::cerr << std::endl;

//    free (nn_idx_);
//    free (nn_dists_);
    m_lock_.unlock ();

    free (p);
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Search for k-nearest neighbors for the given query point.
   * \param points the point cloud data
   * \param index the index in \a points representing the query point
   * \param k the number of neighbors to search for
   * \param k_indices the resultant point indices
   * \param k_distances the resultant point distances
   */
  void
    KdTreeFLANN::nearestKSearch (const sensor_msgs::PointCloud &points, int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances)
  {
    if (index >= (int)points.points.size ())
      return;

    k_indices.resize (k);
    k_distances.resize (k);

    float* p = (float*)malloc (3 * sizeof (float));
    p[0] = points.points.at (index).x; p[1] = points.points.at (index).y; p[2] = points.points.at (index).z;

    m_lock_.lock ();
    flann_find_nearest_neighbors_index (index_id_, p, 1, &k_indices[0], &k_distances[0], k, flann_param_.checks, &flann_param_);
    m_lock_.unlock ();

    free (p);
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Search for all the nearest neighbors of the query point in a given radius.
   * \param p_q the given query point
   * \param radius the radius of the sphere bounding all of p_q's neighbors
   * \param k_indices the resultant point indices
   * \param k_distances the resultant point distances
   * \param max_nn if given, bounds the maximum returned neighbors to this value
   */
  bool
    KdTreeFLANN::radiusSearch (const geometry_msgs::Point32 &p_q, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances,
                               int max_nn)
  {
    float* p = (float*)malloc (3 * sizeof (float));
    p[0] = p_q.x; p[1] = p_q.y; p[2] = p_q.z;
    radius *= radius;

    int neighbors_in_radius_ = flann_param_.checks;
//    m_lock_.lock ();
//    int neighbors_in_radius_ = ann_kd_tree_->annkFRSearch (p, radius, 0, NULL, NULL, epsilon_);
//    m_lock_.unlock ();

    if (neighbors_in_radius_  > max_nn) neighbors_in_radius_ = max_nn;
    k_indices.resize (neighbors_in_radius_);
    k_distances.resize (neighbors_in_radius_);

    m_lock_.lock ();
    int neighbors_found = flann_radius_search (index_id_, p, &k_indices[0], &k_distances[0], neighbors_in_radius_, radius, flann_param_.checks, &flann_param_);
    m_lock_.unlock ();
    free (p);

    if (neighbors_found == 0) {
      return (false);
    }

    k_indices.resize(neighbors_found);
    k_distances.resize(neighbors_found);

    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Search for all the nearest neighbors of the query point in a given radius.
   * \param points the point cloud data
   * \param index the index in \a points representing the query point
   * \param radius the radius of the sphere bounding all of p_q's neighbors
   * \param k_indices the resultant point indices
   * \param k_distances the resultant point distances
   * \param max_nn if given, bounds the maximum returned neighbors to this value
   */
  bool
    KdTreeFLANN::radiusSearch (const sensor_msgs::PointCloud &points, int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances,
                               int max_nn)
  {
    float* p = (float*)malloc (3 * sizeof (float));
    p[0] = points.points.at (index).x; p[1] = points.points.at (index).y; p[2] = points.points.at (index).z;
    radius *= radius;

    int neighbors_in_radius_ = flann_param_.checks;
//    m_lock_.lock ();
//    int neighbors_in_radius_ = ann_kd_tree_->annkFRSearch (p, radius, 0, NULL, NULL, epsilon_);
//    m_lock_.unlock ();

    if (neighbors_in_radius_  > max_nn) neighbors_in_radius_ = max_nn;
    k_indices.resize (neighbors_in_radius_);
    k_distances.resize (neighbors_in_radius_);

    m_lock_.lock ();
    int neighbors_found = flann_radius_search (index_id_, p, &k_indices[0], &k_distances[0], neighbors_in_radius_,
    		radius, flann_param_.checks, &flann_param_);
    m_lock_.unlock ();
    free (p);

    if (neighbors_found == 0) {
      return (false);
    }

    k_indices.resize(neighbors_found);
    k_distances.resize(neighbors_found);

    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Converts a ROS PointCloud message to the internal ANN point array representation. Returns the number of
   * points.
   * \param ros_cloud the ROS PointCloud message
   */
  int
    KdTreeFLANN::convertCloudToArray (const sensor_msgs::PointCloud &ros_cloud)
  {
    // No point in doing anything if the array is empty
    if (ros_cloud.points.size () == 0)
    {
      m_lock_.lock ();
      points_ = NULL;
      m_lock_.unlock ();
      return (0);
    }

    m_lock_.lock ();
    points_ = (float*)malloc (ros_cloud.points.size () * 3 * sizeof (float));    // default number of dimensions (3 = xyz)

    for (unsigned int cp = 0; cp < ros_cloud.points.size (); cp++)
    {
      points_[cp * 3 + 0] = ros_cloud.points[cp].x;
      points_[cp * 3 + 1] = ros_cloud.points[cp].y;
      points_[cp * 3 + 2] = ros_cloud.points[cp].z;
    }
    m_lock_.unlock ();

    return (ros_cloud.points.size ());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Converts a ROS PointCloud message with a given set of indices to the internal ANN point array
   * representation. Returns the number of points.
   * \note ATTENTION: This method breaks the 1-1 mapping between the indices returned using \a getNeighborsIndices
   * and the ones from the \a ros_cloud message ! When using this method, make sure to get the underlying point data
   * using the \a getPoint method
   * \param ros_cloud the ROS PointCloud message
   * \param indices the point cloud indices
   */
  int
    KdTreeFLANN::convertCloudToArray (const sensor_msgs::PointCloud &ros_cloud, const std::vector<int> &indices)
  {
    // No point in doing anything if the array is empty
    if (ros_cloud.points.size () == 0 || indices.size () > ros_cloud.points.size ())
    {
      m_lock_.lock ();
      points_ = NULL;
      m_lock_.unlock ();
      return (0);
    }

    m_lock_.lock ();
    points_ = (float*)malloc (indices.size () * 3 * sizeof (float));    // default number of dimensions (3 = xyz)

    for (unsigned int cp = 0; cp < indices.size (); cp++)
    {
      points_[cp * 3 + 0] = ros_cloud.points[indices.at (cp)].x;
      points_[cp * 3 + 1] = ros_cloud.points[indices.at (cp)].y;
      points_[cp * 3 + 2] = ros_cloud.points[indices.at (cp)].z;
    }
    m_lock_.unlock ();

    return (indices.size ());
  }
}
