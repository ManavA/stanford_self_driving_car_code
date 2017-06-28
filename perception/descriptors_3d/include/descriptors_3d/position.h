#ifndef __D3D_POSITION_H__
#define __D3D_POSITION_H__
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

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.hpp>

#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/geometry/nearest.h>

#include <descriptors_3d/descriptor_3d.h>

// --------------------------------------------------------------
/*!
 * \file position.h
 *
 * \brief A Position descriptor uses the 3rd coordinate (z/elevation)
 *        of the interest point/region.
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief A Position descriptor defines a feature from the z-coordinate
 *        of the interest point or the interest region's centroid
 *
 * TODO: use map information such as distance from walls
 */
// --------------------------------------------------------------
class Position: public Descriptor3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the position descriptor such that the z-coordinate
     *        is absolute.  i.e. the feature is the interest point/region's
     *        z-coordinate.
     */
    // --------------------------------------------------------------
    Position();

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
    
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the position descriptor such that the z coordinate
     *        is relative to the given height.  i.e. the feature is the
     *        interest point/region's z-coordinate minus ref_z
     *
     * \param ref_z The reference z (elevation) coordinate.
     */
    // --------------------------------------------------------------
    Position(float ref_z);

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
     * \brief Extract the z-coordinate of the interest points
     *
     * \see Descriptor3D::compute
     */
    // --------------------------------------------------------------
    virtual void doComputation(const sensor_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const std::vector<const geometry_msgs::Point32*>& interest_pts,
                               std::vector<std::vector<float> >& results);

    // --------------------------------------------------------------
    /*!
     * \brief Extract the z-coordinate of the interest regions' centroids
     *
     * \see Descriptor3D::compute
     */
    // --------------------------------------------------------------
    virtual void doComputation(const sensor_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const std::vector<const std::vector<int>*>& interest_region_indices,
                               std::vector<std::vector<float> >& results);

  private:
    float ref_z_;
};

#endif
