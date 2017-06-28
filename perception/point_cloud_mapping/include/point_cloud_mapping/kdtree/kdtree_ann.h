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
 * $Id: kdtree_ann.h 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_KDTREE_KDTREE_ANN_H_
#define _CLOUD_KDTREE_KDTREE_ANN_H_

#include "point_cloud_mapping/kdtree/kdtree.h"

#include <boost/thread/mutex.hpp>

#include <ANN/ANN.h>

namespace cloud_kdtree
{
  class KdTreeANN : public KdTree
  {
    public:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for KdTree.
        * \param points the ROS point cloud data array
        */
      KdTreeANN (const sensor_msgs::PointCloud &points)
      {
        ann_kd_tree_ = NULL; // To avoid a bad delete in the destructor.
        epsilon_     = 0.0;   // default error bound value
        dim_         = 3;     // default number of dimensions (3 = xyz)
        bucket_size_ = std::min (30, (int)points.points.size ());    // default bucket size value

        // Allocate enough data
        nr_points_ = convertCloudToArray (points);
        if (nr_points_ == 0)
        {
          ROS_ERROR ("[KdTreeANN] Could not create kD-tree for %d points!", nr_points_);
          return;
        }

        // Create the kd_tree representation
        m_lock_.lock ();
        ann_kd_tree_ = new ANNkd_tree (points_, nr_points_, dim_, bucket_size_);
        m_lock_.unlock ();
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for KdTree.
        * \note ATTENTION: This method breaks the 1-1 mapping between the indices returned using \a getNeighborsIndices
        * and the ones from the \a points message ! When using this method, make sure to get the underlying point data
        * using the \a getPoint method
        * \param points the ROS point cloud data array
        * \param indices the point cloud indices
        */
      KdTreeANN (const sensor_msgs::PointCloud &points, const std::vector<int> &indices)
      {
        ann_kd_tree_ = NULL; // To avoid a bad delete in the destructor.
        epsilon_     = 0.0;   // default error bound value
        dim_         = 3;     // default number of dimensions (3 = xyz)
        bucket_size_ = std::min (30, (int)indices.size ());    // default bucket size value

        // Allocate enough data
        nr_points_ = convertCloudToArray (points, indices);
        if (nr_points_ == 0)
        {
          ROS_ERROR ("[KdTreeANN] Could not create kD-tree for %d points!", nr_points_);
          return;
        }

        // Create the kd_tree representation
        m_lock_.lock ();
        ann_kd_tree_ = new ANNkd_tree (points_, nr_points_, dim_, bucket_size_);
        m_lock_.unlock ();
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for KdTree. Deletes all allocated data arrays and destroys the kd-tree structures. */
      virtual ~KdTreeANN ()
      {
        m_lock_.lock ();

        // Data array cleanup
        if (points_ != NULL && nr_points_ != 0)
          annDeallocPts (points_);

        // ANN Cleanup
        if (ann_kd_tree_ != NULL) delete ann_kd_tree_;
        ann_kd_tree_ = NULL;
        annClose ();

        m_lock_.unlock ();
      }

      virtual void nearestKSearch (const geometry_msgs::Point32 &p_q, int k, std::vector<int> &k_indices, std::vector<float> &k_distances);
      virtual void nearestKSearch (const sensor_msgs::PointCloud &points, int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Search for k-nearest neighbors for the given query point.
       * \param index the index in \a points representing the query point
       * \param k the number of neighbors to search for
       * \param k_indices the resultant point indices
       * \param k_distances the resultant point distances
       */
      virtual inline void
        nearestKSearch (int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances)
      {
        if (index >= nr_points_)
          return;

        k_indices.resize (k);
        k_distances.resize (k);

        m_lock_.lock ();
        ann_kd_tree_->annkSearch (points_[index], k, &k_indices[0], &k_distances[0], epsilon_);
        m_lock_.unlock ();
        return;
      }

      virtual bool radiusSearch (const geometry_msgs::Point32 &p_q, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances, int max_nn = INT_MAX);
      virtual bool radiusSearch (const sensor_msgs::PointCloud &points, int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances, int max_nn = INT_MAX);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Search for all the nearest neighbors of the query point in a given radius.
       * \param index the index in \a points representing the query point
       * \param radius the radius of the sphere bounding all of p_q's neighbors
       * \param k_indices the resultant point indices
       * \param k_distances the resultant point distances
       * \param max_nn if given, bounds the maximum returned neighbors to this value
       */
      virtual inline bool
        radiusSearch (int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances,
                      int max_nn = INT_MAX)
      {
        radius *= radius;

        m_lock_.lock ();
        int neighbors_in_radius_ = ann_kd_tree_->annkFRSearch (points_[index], radius, 0, NULL, NULL, epsilon_);
        m_lock_.unlock ();

        // No neighbors found ? Return false
        if (neighbors_in_radius_ == 0)
          return (false);

        if (neighbors_in_radius_  > max_nn) neighbors_in_radius_  = max_nn;
        k_indices.resize (neighbors_in_radius_);
        k_distances.resize (neighbors_in_radius_);

        m_lock_.lock ();
        ann_kd_tree_->annkFRSearch (points_[index], radius, neighbors_in_radius_, &k_indices[0], &k_distances[0], epsilon_);
        m_lock_.unlock ();

        return (true);
      }

    private:

      int convertCloudToArray (const sensor_msgs::PointCloud &ros_cloud);
      int convertCloudToArray (const sensor_msgs::PointCloud &ros_cloud, const std::vector<int> &indices);

    private:

      boost::mutex m_lock_;

      /** \brief The ANN kd tree object */
      ANNkd_tree* ann_kd_tree_;

      /** \brief Internal tree bucket size */
      double bucket_size_;

      /** \brief Internal pointer to data */
      ANNpointArray points_;

      /** \brief Number of points in the tree */
      int nr_points_;
      /** \brief Tree dimensionality (i.e. the number of dimensions per point) */
      int dim_;
  };

}

#endif
