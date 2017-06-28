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
 * $Id: kdtree_ann.cpp 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu */

#include "point_cloud_mapping/kdtree/kdtree_ann.h"

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
    KdTreeANN::nearestKSearch (const geometry_msgs::Point32 &p_q, int k, std::vector<int> &k_indices, std::vector<float> &k_distances)
  {
    k_indices.resize (k);
    k_distances.resize (k);

    ANNpoint p = annAllocPt (3);
    p[0] = p_q.x; p[1] = p_q.y; p[2] = p_q.z;

    m_lock_.lock ();
    ann_kd_tree_->annkSearch (p, k, &k_indices[0], &k_distances[0], epsilon_);
    m_lock_.unlock ();

    annDeallocPt (p);
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
    KdTreeANN::nearestKSearch (const sensor_msgs::PointCloud &points, int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances)
  {
    if (index >= (int)points.points.size ())
      return;

    k_indices.resize (k);
    k_distances.resize (k);

    ANNpoint p = annAllocPt (3);
    p[0] = points.points.at (index).x; p[1] = points.points.at (index).y; p[2] = points.points.at (index).z;

    m_lock_.lock ();
    ann_kd_tree_->annkSearch (p, k, &k_indices[0], &k_distances[0], epsilon_);
    m_lock_.unlock ();

    annDeallocPt (p);
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
    KdTreeANN::radiusSearch (const geometry_msgs::Point32 &p_q, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances,
                             int max_nn)
  {
    ANNpoint p = annAllocPt (3);
    p[0] = p_q.x; p[1] = p_q.y; p[2] = p_q.z;
    radius *= radius;

    m_lock_.lock ();
    int neighbors_in_radius_ = ann_kd_tree_->annkFRSearch (p, radius, 0, NULL, NULL, epsilon_);
    m_lock_.unlock ();

    // No neighbors found ? Return false
    if (neighbors_in_radius_ == 0)
    {
      annDeallocPt (p);
      return (false);
    }

    if (neighbors_in_radius_  > max_nn) neighbors_in_radius_ = max_nn;
    k_indices.resize (neighbors_in_radius_);
    k_distances.resize (neighbors_in_radius_);

    m_lock_.lock ();
    ann_kd_tree_->annkFRSearch (p, radius, neighbors_in_radius_, &k_indices[0], &k_distances[0], epsilon_);
    m_lock_.unlock ();

    annDeallocPt (p);
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
    KdTreeANN::radiusSearch (const sensor_msgs::PointCloud &points, int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances,
                             int max_nn)
  {
    ANNpoint p = annAllocPt (3);
    p[0] = points.points.at (index).x; p[1] = points.points.at (index).y; p[2] = points.points.at (index).z;
    radius *= radius;

    m_lock_.lock ();
    int neighbors_in_radius_ = ann_kd_tree_->annkFRSearch (p, radius, 0, NULL, NULL, epsilon_);
    m_lock_.unlock ();

    // No neighbors found ? Return false
    if (neighbors_in_radius_ == 0)
    {
      annDeallocPt (p);
      return (false);
    }

    if (neighbors_in_radius_  > max_nn) neighbors_in_radius_ = max_nn;
    k_indices.resize (neighbors_in_radius_);
    k_distances.resize (neighbors_in_radius_);

    m_lock_.lock ();
    ann_kd_tree_->annkFRSearch (p, radius, neighbors_in_radius_, &k_indices[0], &k_distances[0], epsilon_);
    m_lock_.unlock ();

    annDeallocPt (p);
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Converts a ROS PointCloud message to the internal ANN point array representation. Returns the number of
   * points.
   * \param ros_cloud the ROS PointCloud message
   */
  int
    KdTreeANN::convertCloudToArray (const sensor_msgs::PointCloud &ros_cloud)
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
    points_ = annAllocPts (ros_cloud.points.size (), 3);       // default number of dimensions (3 = xyz)

    for (unsigned int cp = 0; cp < ros_cloud.points.size (); cp++)
    {
      points_[cp][0] = ros_cloud.points[cp].x;
      points_[cp][1] = ros_cloud.points[cp].y;
      points_[cp][2] = ros_cloud.points[cp].z;
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
    KdTreeANN::convertCloudToArray (const sensor_msgs::PointCloud &ros_cloud, const std::vector<int> &indices)
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
    points_ = annAllocPts (indices.size (), 3);       // default number of dimensions (3 = xyz)

    for (unsigned int cp = 0; cp < indices.size (); cp++)
    {
      points_[cp][0] = ros_cloud.points[indices.at (cp)].x;
      points_[cp][1] = ros_cloud.points[indices.at (cp)].y;
      points_[cp][2] = ros_cloud.points[indices.at (cp)].z;
    }
    m_lock_.unlock ();

    return (indices.size ());
  }
}
