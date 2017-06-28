#ifndef __D3D_CHANNEL_H__
#define __D3D_CHANNEL_H__
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
#include <string>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.hpp>

#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>

#include <descriptors_3d/descriptor_3d.h>

// --------------------------------------------------------------
/*!
 * \file channel.h
 *
 * \brief A Channel descriptor uses the value in a specified channel name of
 *        a sensor_msgs::PointCloud for the feature.
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief A Channel descriptor uses the value in a specified channel of
 *        a sensor_msgs::PointCloud as the feature value.
 */
// --------------------------------------------------------------
class Channel: public Descriptor3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the channel descriptor to use the specified
     *        channel as the feature value
     *
     * \param channel_name The name of the channel to use
     */
    // --------------------------------------------------------------
    Channel(const std::string& channel_name);

    // --------------------------------------------------------------
    /*!
     * \brief This descriptor uses no shared precomputation, so this method
     *        has no affect
     */
    // --------------------------------------------------------------
    virtual void clearShared();

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
     * \brief Extracts the value from the channel corresponding to the specified
     *        interest points.
     *
     * If the exact interest point i cannot be found in data, then results[i].size == 0
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
     * \brief Extracts the average value from the channel corresponding to
     *        each point in the specified interest region.
     *
     * \see Descriptor3D::compute
     */
    // --------------------------------------------------------------
    virtual void doComputation(const sensor_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const std::vector<const std::vector<int>*>& interest_region_indices,
                               std::vector<std::vector<float> >& results);

  private:
    /*! \brief The name of the channel to use */
    std::string channel_name_;
};

#endif
