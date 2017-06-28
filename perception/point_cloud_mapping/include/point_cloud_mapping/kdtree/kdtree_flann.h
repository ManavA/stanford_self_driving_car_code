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
 * $Id: kdtree_flann.h 24968 2009-10-14 20:54:15Z wheeler $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_KDTREE_KDTREE_FLANN_H_
#define _CLOUD_KDTREE_KDTREE_FLANN_H_

#include <cstdio>

#include "point_cloud_mapping/kdtree/kdtree.h"

#include <boost/thread/mutex.hpp>

#include <flann.h>

namespace cloud_kdtree
{
  class KdTreeFLANN : public KdTree
  {
    public:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for KdTree.
        * \param points the ROS point cloud data array
        */
      KdTreeFLANN (const sensor_msgs::PointCloud &points)
      {
        epsilon_     = 0.0;   // default error bound value
        dim_         = 3;     // default number of dimensions (3 = xyz)

        // Allocate enough data
        nr_points_ = convertCloudToArray (points);
        if (nr_points_ == 0)
        {
          ROS_ERROR ("[KdTreeFLANN] Could not create kD-tree for %d points!", nr_points_);
          return;
        }

        // Create the kd_tree representation
        float speedup;
        flann_param_.algorithm = KDTREE;
        flann_param_.log_level = LOG_NONE;
        flann_param_.log_destination = NULL;

        flann_param_.trees = 1;
        flann_param_.target_precision = -1;
        flann_param_.checks = 128;

        m_lock_.lock ();
        printf("Building index\n");
        index_id_    = flann_build_index (points_, nr_points_, dim_, &speedup, &flann_param_);
        printf("Index built\n");
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
      KdTreeFLANN (const sensor_msgs::PointCloud &points, const std::vector<int> &indices)
      {
        epsilon_     = 0.0;   // default error bound value
        dim_         = 3;     // default number of dimensions (3 = xyz)

        // Allocate enough data
        nr_points_ = convertCloudToArray (points, indices);
        if (nr_points_ == 0)
        {
          ROS_ERROR ("[KdTreeFLANN] Could not create kD-tree for %d points!", nr_points_);
          return;
        }

        // Create the kd_tree representation
        float speedup;
        flann_param_.algorithm = KDTREE;
        flann_param_.log_level = LOG_NONE;
        flann_param_.log_destination = NULL;

        flann_param_.trees = 1;
        flann_param_.target_precision = -1;
        flann_param_.checks = 128;

        m_lock_.lock ();
        index_id_    = flann_build_index (points_, nr_points_, dim_, &speedup, &flann_param_);
        m_lock_.unlock ();
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for KdTree. Deletes all allocated data arrays and destroys the kd-tree structures. */
      virtual ~KdTreeFLANN ()
      {
        m_lock_.lock ();

        // Data array cleanup
        if (points_ != NULL && nr_points_ != 0)
          free (points_);

        // ANN Cleanup
        flann_free_index (index_id_, &flann_param_);

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

        m_lock_.lock ();
        flann_find_nearest_neighbors_index (index_id_, &points_[index], 1, &k_indices[0], &k_distances[0], k, flann_param_.checks, &flann_param_);
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

        int neighbors_in_radius_ = flann_param_.checks;
//        m_lock_.lock ();
        //int neighbors_in_radius_ = ann_kd_tree_->annkFRSearch (points_[index], radius, 0, NULL, NULL, epsilon_);
//        m_lock_.unlock ();

        if (neighbors_in_radius_ > max_nn) neighbors_in_radius_  = max_nn;
        k_indices.resize (neighbors_in_radius_);
        k_distances.resize (neighbors_in_radius_);

        m_lock_.lock ();
        int neighbors_found = flann_radius_search (index_id_, &points_[index], &k_indices[0], &k_distances[0],
        		neighbors_in_radius_, radius, flann_param_.checks, &flann_param_);
        m_lock_.unlock ();

        if (neighbors_found == 0) {
          return (false);
        }

        k_indices.resize(neighbors_found);
        k_distances.resize(neighbors_found);

        return (true);
      }

    private:

      int convertCloudToArray (const sensor_msgs::PointCloud &ros_cloud);
      int convertCloudToArray (const sensor_msgs::PointCloud &ros_cloud, const std::vector<int> &indices);

    private:

      boost::mutex m_lock_;

      /** \brief A FL-ANN type index reference */
      FLANN_INDEX index_id_;

      /** \brief A pointer to a FL-ANN parameter structure */
      FLANNParameters flann_param_;

      /** \brief Internal pointer to data */
      float* points_;

      /** \brief Number of points in the tree */
      int nr_points_;
      /** \brief Tree dimensionality (i.e. the number of dimensions per point) */
      int dim_;
  };

}

#endif
