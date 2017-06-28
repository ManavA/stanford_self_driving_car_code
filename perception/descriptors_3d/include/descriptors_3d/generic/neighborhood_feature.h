/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/


#ifndef __D3D_NEIGHBORHOOD_FEATURE_H__
#define __D3D_NEIGHBORHOOD_FEATURE_H__
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

#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/kdtree/kdtree.h>

#include <descriptors_3d/descriptor_3d.h>

// --------------------------------------------------------------
/*!
 * \file neighborhood_feature.h
 *
 * \brief A NeighborhoodFeature descriptor is a generic descriptor
 *        uses a neighborhood of points to compute features
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief A NeighborhoodFeature is an abstract base class for descriptors
 *        that use a local neighborhood of points to compute features.
 *
 * Example: a spin image uses neighboring points to compute a histogram
 */
// --------------------------------------------------------------
class NeighborhoodFeature: public Descriptor3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Abstract constructor
     */
    // --------------------------------------------------------------
    NeighborhoodFeature();

    virtual ~NeighborhoodFeature() = 0;

  protected:
    // --------------------------------------------------------------
    /*!
     * \brief Retrieves the local neighborhood around each interest point
     *        and then computes features
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
     * \brief Retrieves the local neighborhood around each interest region
     *        and then computes features
     *
     * \see Descriptor3D::compute
     */
    // --------------------------------------------------------------
    virtual void doComputation(const sensor_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const std::vector<const std::vector<int>*>& interest_region_indices,
                               std::vector<std::vector<float> >& results);

    // --------------------------------------------------------------
    /*!
     * \brief The prototype of the method that computes the features as
     *        defined in the inheriting class.
     *
     * \param data The overall point cloud data
     * \param neighbor_indices List of indices in data that constitute the neighborhood
     * \param interest_sample_idx The interest point/region that is being processed.
     * \param result The vector to hold the resulting spin image feature vector
     */
    // --------------------------------------------------------------
    virtual void
        computeNeighborhoodFeature(const sensor_msgs::PointCloud& data,
                                   const std::vector<int>& neighbor_indices,
                                   const unsigned int interest_sample_idx,
                                   std::vector<float>& result) const = 0;

    /*! \brief The radius to define the bounding box */
    float neighborhood_radius_;

    /*! \brief Flag if neighborhood_radius_ has been defined */
    bool neighborhood_radius_defined_;
};

#endif
