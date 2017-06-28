#ifndef __D3D_BOUNDING_BOX_RAW_H__
#define __D3D_BOUNDING_BOX_RAW_H__
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

#include <vector>

#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.hpp>

#include <descriptors_3d/generic/neighborhood_feature.h>

// --------------------------------------------------------------
/*!
 * \file bounding_box_raw.h
 *
 * \brief A BoundingBoxRaw descriptor computes the dimensions of the
 *        3-D box that encloses a group of points in given xyz space.
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief A BoundingBoxRaw descriptor to compute the dimensions
 *        of the bounding box that encloses a neighborhood of points
 *        within a radius of the interest point/region.
 *
 * The compute features are in order: [dx,dy,dz] where dx is the length
 * along the x dimension, dy is the length along the y dimension, dz is
 * the length along the z dimension.
 *
 */
// --------------------------------------------------------------
class BoundingBoxRaw: public NeighborhoodFeature
{
  public:

    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the bounding box feature with the specified
     *        radius to define the neighborhood
     *
     * When computing the feature for an interest region of points, the
     * bounding box can either be the box that encloses the given region
     * of points (indicated by -negative value), or from the neighboring points
     * within the specified radius from the region's centroid (indicated
     * by positive value).
     *
     * \param bbox_radius The radius from the interest point/region to define
     *                    the neighborhood that defines the bounding box
     */
    // --------------------------------------------------------------
    BoundingBoxRaw(double bbox_radius);

    // --------------------------------------------------------------
    /*!
     * \brief This descriptor uses no shared precomputation, so this method
     *        has no affect
     */
    // --------------------------------------------------------------
    virtual void clearShared();

    // --------------------------------------------------------------
    /*!
     * \brief Returns a name that is unique for any given setting of the parameters.
     *        
     */
    // --------------------------------------------------------------
    std::string getName() const;

  protected:
    // --------------------------------------------------------------
    /*!
     * \brief This descriptor requires no pre-computation, so this method
     *        has no affect
     *
     * \return 0 always
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const std::vector<const geometry_msgs::Point32*>& interest_pts);

    // --------------------------------------------------------------
    /*!
     * \brief This descriptor requires no pre-computation, so this method
     *        has no affect
     *
     * \return 0 always
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const std::vector<const std::vector<int>*>& interest_region_indices);

    // --------------------------------------------------------------
    /*!
     * \brief Computes the bounding box information of the given neighborhood
     *
     * \param data The overall point cloud data
     * \param neighbor_indices The list of indices in data that constitute the neighborhood
     * \param interest_sample_idx The index of the interest point/region that is being
     *                            processed from Descriptor3D::compute()
     * \param result The vector to hold the computed bounding box dimensions
     */
    // --------------------------------------------------------------
    virtual void
    computeNeighborhoodFeature(const sensor_msgs::PointCloud& data,
                               const std::vector<int>& neighbor_indices,
                               const unsigned int interest_sample_idx,
                               std::vector<float>& result) const;
};

#endif
