/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: kdtree.h 20633 2009-08-04 07:19:09Z tfoote $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_KDTREE_KDTREE_H_
#define _CLOUD_KDTREE_KDTREE_H_

// ROS includes
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

namespace cloud_kdtree
{
  class KdTree
  {
    public:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor for KdTree. Sets some internal values to their defaults.
        */
      KdTree ()
      {
        epsilon_     = 0.0;   // default error bound value
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for KdTree.
        * \param points the ROS point cloud data array
        */
      KdTree (const sensor_msgs::PointCloud &points);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for KdTree.
        * \note ATTENTION: This method breaks the 1-1 mapping between the indices returned using \a getNeighborsIndices
        * and the ones from the \a points message ! When using this method, make sure to get the underlying point data
        * using the \a getPoint method
        * \param points the ROS point cloud data array
        * \param indices the point cloud indices
        */
      KdTree (const sensor_msgs::PointCloud &points, const std::vector<int> &indices);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for KdTree. Deletes all allocated data arrays and destroys the kd-tree structures. */
      virtual ~KdTree () { }

      virtual void nearestKSearch (const geometry_msgs::Point32 &p_q, int k, std::vector<int> &k_indices, std::vector<float> &k_distances) = 0;
      virtual void nearestKSearch (const sensor_msgs::PointCloud &points, int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances) = 0;
      virtual void nearestKSearch (int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances) = 0;

      virtual bool radiusSearch (const geometry_msgs::Point32 &p_q, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances,
                                 int max_nn = INT_MAX) = 0;
      virtual bool radiusSearch (const sensor_msgs::PointCloud &points, int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances,
                                 int max_nn = INT_MAX) = 0;
      virtual bool radiusSearch (int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances,
				 int max_nn = INT_MAX) = 0;


    protected:
      /** \brief Epsilon precision (error bound) for nearest neighbors searches */
      double epsilon_;

  };

}

#endif
